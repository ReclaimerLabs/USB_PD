/*
  USB_PD.c - USB Power Delivery
  Copyright 2010 The Chromium OS Authors
  Copyright 2017 Jason Cerundolo
  Released under an MIT license. See LICENSE file.  
*/

#include "USB_PD.h"

USB_PD::USB_PD(USB_TCPM *tcpm, int port_id, enum pd_states PD_DEFAULT_STATE) {
	m_tcpm = tcpm;
	port = port_id;

	timeout = 10*MSEC;
	incoming_packet = 0;
	hard_reset_count = 0;
#ifdef CONFIG_USB_PD_DUAL_ROLE
	next_role_swap = PD_T_DRP_SNK;
#ifndef CONFIG_USB_PD_VBUS_DETECT_NONE
	snk_hard_reset_vbus_off = 0;
#endif
#ifdef CONFIG_CHARGE_MANAGER
	typec_curr = 0;
	typec_curr_change = 0;
#endif /* CONFIG_CHARGE_MANAGER */
#endif /* CONFIG_USB_PD_DUAL_ROLE */
	caps_count = 0;
	hard_reset_sent = 0;
	snk_cap_count = 0;

	default_state = PD_DEFAULT_STATE;

#if !defined(CONFIG_USB_PD_COMM_ENABLED) || defined(CONFIG_USB_PD_COMM_LOCKED)
    pd_comm_enabled = 0;
#else
    pd_comm_enabled = 1;
#endif

    drp_state = PD_DRP_TOGGLE_OFF;

    max_request_mv = PD_MAX_VOLTAGE_MV; /* no cap */
}

int USB_PD::init(void) {
	/* Ensure the power supply is in the default state */
	pd_power_supply_reset(port);

	/* Initialize TCPM driver and wait for TCPC to be ready */
	res = m_tcpm->init();
	this_state = (enum pd_states)(res ? PD_STATE_SUSPENDED : default_state);

	/*
	 * If VBUS is high, then initialize flag for VBUS has always been
	 * present. This flag is used to maintain a PD connection after a
	 * reset by sending a soft reset.
	 */
	//flags = pd_is_vbus_present(port) ? PD_FLAGS_VBUS_NEVER_LOW : 0;
	flags = 0;

	// Disable TCPC RX until connection is established
	m_tcpm->set_rx_enable(0);

#ifdef CONFIG_USBC_SS_MUX
	/* Initialize USB mux to its default state */
	usb_mux_init(port);
#endif

	/* Initialize PD protocol state variables for each port. */
	power_role = PD_ROLE_DEFAULT;
	vdm_state = VDM_STATE_DONE;
	set_state(this_state);
	m_tcpm->select_rp_value(CONFIG_USB_PD_PULLUP);
	m_tcpm->set_cc(PD_ROLE_DEFAULT == PD_ROLE_SOURCE ? TYPEC_CC_RP :
							      TYPEC_CC_RD);

#ifdef CONFIG_USB_PD_ALT_MODE_DFP
	/* Initialize PD Policy engine */
	pd_dfp_pe_init(port);
#endif

#ifdef CONFIG_CHARGE_MANAGER
	/* Initialize PD and type-C supplier current limits to 0 */
	pd_set_input_current_limit(port, 0, 0);
	typec_set_input_current_limit(port, 0, 0);
	charge_manager_update_dualrole(port, CAP_UNKNOWN);
#endif

	timeout_deadline = 0;
}

void USB_PD::pd_state_machine(void) {
	// process VDM messages last
	vdm_send_state_machine();

	// Verify board specific health status : current, voltages...
	res = pd_board_checks();
	if (res != EC_SUCCESS) {
		// cut the power
		execute_hard_reset();
		// notify the other side of the issue
		pd_transmit(TCPC_TX_HARD_RESET, 0, NULL);
	}

	// wait for next event/packet or timeout expiration
	evt = task_wait_event(timeout);

	/* if TCPC has reset, then need to initialize it again */
	if (evt & PD_EVENT_TCPC_RESET) {
		/* Ensure CC termination is default */
		m_tcpm->set_cc(PD_ROLE_DEFAULT == PD_ROLE_SOURCE ?
						      TYPEC_CC_RP :
						      TYPEC_CC_RD);

		/*
		 * If we have a stable contract in the default role,
		 * then simply update TCPC with some missing info
		 * so that we can continue without resetting PD comms.
		 * Otherwise, go to the default disconnected state
		 * and force renegotiation.
		 */
		if (vdm_state == VDM_STATE_DONE && (
#ifdef CONFIG_USB_PD_DUAL_ROLE
		    (PD_ROLE_DEFAULT == PD_ROLE_SINK &&
		     task_state == PD_STATE_SNK_READY) ||
#endif
		    (PD_ROLE_DEFAULT == PD_ROLE_SOURCE &&
		     task_state == PD_STATE_SRC_READY))) {
			m_tcpm->set_polarity(polarity);
			m_tcpm->set_msg_header(power_role,
					    data_role);
			m_tcpm->set_rx_enable(1);
		} else {
			/* Ensure state variables are at default */
			power_role = PD_ROLE_DEFAULT;
			vdm_state = VDM_STATE_DONE;
			set_state((enum pd_states)default_state);
		}
	}

	/* process any potential incoming message */
	incoming_packet = evt & PD_EVENT_RX;
	if (incoming_packet) {
		m_tcpm->get_message(payload, &head);
		if (head > 0) {
			handle_request(head, payload);
		}
	}
	/* if nothing to do, verify the state of the world in 500ms */
	this_state = task_state;
	timeout = 500*MSEC;

	switch (this_state) {
		case PD_STATE_DISABLED:
			/* Nothing to do */
			break;
		case PD_STATE_SRC_DISCONNECTED:
			timeout = 10*MSEC;
			m_tcpm->get_cc(&cc1, &cc2);

			/* Vnc monitoring */
			if ((cc1 == TYPEC_CC_VOLT_RD ||
			     cc2 == TYPEC_CC_VOLT_RD) ||
			    (cc1 == TYPEC_CC_VOLT_RA &&
			     cc2 == TYPEC_CC_VOLT_RA)) {
#ifdef CONFIG_USBC_BACKWARDS_COMPATIBLE_DFP
				/* Enable VBUS */
				if (pd_set_power_supply_ready(port))
					break;
#endif
				cc_state = PD_CC_NONE;
				set_state(PD_STATE_SRC_DISCONNECTED_DEBOUNCE);
			}
#ifdef CONFIG_USB_PD_DUAL_ROLE
			/*
			 * Try.SRC state is embedded here. Wait for SNK
			 * detect, or if timer expires, transition to
			 * SNK_DISCONNETED.
			 *
			 * If Try.SRC state is not active, then this block
			 * handles the normal DRP toggle from SRC->SNK
			 */
			else if ((flags & PD_FLAGS_TRY_SRC &&
				 get_time().val >= try_src_marker) ||
				 (!(flags & PD_FLAGS_TRY_SRC) &&
				  drp_state != PD_DRP_FORCE_SOURCE &&
				 get_time().val >= next_role_swap)) {
				power_role = PD_ROLE_SINK;
				set_state(PD_STATE_SNK_DISCONNECTED);
				m_tcpm->set_cc(TYPEC_CC_RD);
				next_role_swap = get_time().val + PD_T_DRP_SNK;
				try_src_marker = get_time().val	+ PD_T_TRY_WAIT;

				/* Swap states quickly */
				timeout = 2*MSEC;
			}
#endif
			break;
		case PD_STATE_SRC_DISCONNECTED_DEBOUNCE:
			timeout = 20*MSEC;
			m_tcpm->get_cc(&cc1, &cc2);

			if (cc1 == TYPEC_CC_VOLT_RD &&
			    cc2 == TYPEC_CC_VOLT_RD) {
				/* Debug accessory */
				new_cc_state = PD_CC_DEBUG_ACC;
			} else if (cc1 == TYPEC_CC_VOLT_RD ||
				   cc2 == TYPEC_CC_VOLT_RD) {
				/* UFP attached */
				new_cc_state = PD_CC_UFP_ATTACHED;
			} else if (cc1 == TYPEC_CC_VOLT_RA &&
				   cc2 == TYPEC_CC_VOLT_RA) {
				/* Audio accessory */
				new_cc_state = PD_CC_AUDIO_ACC;
			} else {
				/* No UFP */
				set_state(PD_STATE_SRC_DISCONNECTED);
				timeout = 5*MSEC;
				break;
			}
			/* If in Try.SRC state, then don't need to debounce */
			if (!(flags & PD_FLAGS_TRY_SRC)) {
				/* Debounce the cc state */
				if (new_cc_state != cc_state) {
					cc_debounce = get_time().val +
						PD_T_CC_DEBOUNCE;
					cc_state = new_cc_state;
					break;
				} else if (get_time().val < cc_debounce) {
					break;
				}
			}

			/* Debounce complete */
			/* UFP is attached */
			if (new_cc_state == PD_CC_UFP_ATTACHED) {
				polarity = (cc2 == TYPEC_CC_VOLT_RD);
				m_tcpm->set_polarity(polarity);

				/* initial data role for source is DFP */
				set_data_role(PD_ROLE_DFP);

#ifndef CONFIG_USBC_BACKWARDS_COMPATIBLE_DFP
				/* Enable VBUS */
				if (pd_set_power_supply_ready(port)) {
#ifdef CONFIG_USBC_SS_MUX
					usb_mux_set(port, TYPEC_MUX_NONE,
						    USB_SWITCH_DISCONNECT,
						    pd[port].polarity);
#endif
					break;
				}
#endif
				/* If PD comm is enabled, enable TCPC RX */
				if (pd_comm_enabled)
					m_tcpm->set_rx_enable(1);

#ifdef CONFIG_USBC_VCONN
				m_tcpm->set_vconn(port, 1);
				flags |= PD_FLAGS_VCONN_ON;
#endif

				flags |= PD_FLAGS_CHECK_PR_ROLE |
						  PD_FLAGS_CHECK_DR_ROLE;
				hard_reset_count = 0;
				timeout = 5*MSEC;
				set_state(PD_STATE_SRC_STARTUP);
			}
			/* Accessory is attached */
			else if (new_cc_state == PD_CC_AUDIO_ACC ||
				 new_cc_state == PD_CC_DEBUG_ACC) {
#ifdef CONFIG_USBC_BACKWARDS_COMPATIBLE_DFP
				/* Remove VBUS */
				pd_power_supply_reset(port);
#endif

				/* Set the USB muxes and the default USB role */
				set_data_role(CONFIG_USB_PD_DEBUG_DR);

				set_state(PD_STATE_SRC_ACCESSORY);
			}
			break;
		case PD_STATE_SRC_HARD_RESET_RECOVER:
			/* Do not continue until hard reset recovery time */
			if (get_time().val < src_recover) {
				timeout = 50*MSEC;
				break;
			}

			/* Enable VBUS */
			timeout = 10*MSEC;
			if (pd_set_power_supply_ready(port)) {
				set_state(PD_STATE_SRC_DISCONNECTED);
				break;
			}
			set_state(PD_STATE_SRC_STARTUP);
			break;
		case PD_STATE_SRC_STARTUP:
			/* Wait for power source to enable */
			if (last_state != task_state) {
				/*
				 * fake set data role swapped flag so we send
				 * discover identity when we enter SRC_READY
				 */
				flags |= PD_FLAGS_DATA_SWAPPED;
				/* reset various counters */
				caps_count = 0;
				msg_id = 0;
				snk_cap_count = 0;
				set_state_timeout(
#ifdef CONFIG_USBC_BACKWARDS_COMPATIBLE_DFP
					/*
					 * delay for power supply to start up.
					 * subtract out debounce time if coming
					 * from debounce state since vbus is
					 * on during debounce.
					 */
					get_time().val +
					PD_POWER_SUPPLY_TURN_ON_DELAY -
					  (last_state ==
					   PD_STATE_SRC_DISCONNECTED_DEBOUNCE
						? PD_T_CC_DEBOUNCE : 0),
#else
					get_time().val +
					PD_POWER_SUPPLY_TURN_ON_DELAY,
#endif
					PD_STATE_SRC_DISCOVERY);
			}
			break;
		case PD_STATE_SRC_DISCOVERY:
			if (last_state != task_state) {
				/*
				 * If we have had PD connection with this port
				 * partner, then start NoResponseTimer.
				 */
				if (flags & PD_FLAGS_PREVIOUS_PD_CONN)
					set_state_timeout(
						get_time().val +
						PD_T_NO_RESPONSE,
						hard_reset_count <
						  PD_HARD_RESET_COUNT ?
						    PD_STATE_HARD_RESET_SEND :
						    PD_STATE_SRC_DISCONNECTED);
			}

			/* Send source cap some minimum number of times */
			if (caps_count < PD_CAPS_COUNT) {
				/* Query capabilites of the other side */
				res = send_source_cap();
				/* packet was acked => PD capable device) */
				if (res >= 0) {
					set_state(PD_STATE_SRC_NEGOCIATE);
					timeout = 10*MSEC;
					hard_reset_count = 0;
					caps_count = 0;
					/* Port partner is PD capable */
					flags |= PD_FLAGS_PREVIOUS_PD_CONN;
				} else { /* failed, retry later */
					timeout = PD_T_SEND_SOURCE_CAP;
					caps_count++;
				}
			}
			break;
		case PD_STATE_SRC_NEGOCIATE:
			/* wait for a "Request" message */
			if (last_state != task_state)
				set_state_timeout(
						  get_time().val +
						  PD_T_SENDER_RESPONSE,
						  PD_STATE_HARD_RESET_SEND);
			break;
		case PD_STATE_SRC_ACCEPTED:
			/* Accept sent, wait for enabling the new voltage */
			if (last_state != task_state)
				set_state_timeout(
					get_time().val +
					PD_T_SINK_TRANSITION,
					PD_STATE_SRC_POWERED);
			break;
		case PD_STATE_SRC_POWERED:
			/* Switch to the new requested voltage */
			if (last_state != task_state) {
				pd_transition_voltage(requested_idx);
				set_state_timeout(
					get_time().val +
					PD_POWER_SUPPLY_TURN_ON_DELAY,
					PD_STATE_SRC_TRANSITION);
			}
			break;
		case PD_STATE_SRC_TRANSITION:
			/* the voltage output is good, notify the source */
			res = send_control(PD_CTRL_PS_RDY);
			if (res >= 0) {
				timeout = 10*MSEC;
				/* it'a time to ping regularly the sink */
				set_state(PD_STATE_SRC_READY);
			} else {
				/* The sink did not ack, cut the power... */
				set_state(PD_STATE_SRC_DISCONNECTED);
			}
			break;
		case PD_STATE_SRC_READY:
			timeout = PD_T_SOURCE_ACTIVITY;

			/*
			 * Don't send any PD traffic if we woke up due to
			 * incoming packet or if VDO response pending to avoid
			 * collisions.
			 */
			if (incoming_packet ||
			    (vdm_state == VDM_STATE_BUSY))
				break;

			/* Send updated source capabilities to our partner */
			if (flags & PD_FLAGS_UPDATE_SRC_CAPS) {
				res = send_source_cap();
				if (res >= 0) {
					set_state(PD_STATE_SRC_NEGOCIATE);
					flags &= ~PD_FLAGS_UPDATE_SRC_CAPS;
				}
				break;
			}

			/* Send get sink cap if haven't received it yet */
			if (last_state != task_state &&
			    !(flags & PD_FLAGS_SNK_CAP_RECVD)) {
				if (++snk_cap_count <= PD_SNK_CAP_RETRIES) {
					/* Get sink cap to know if dual-role device */
					send_control(PD_CTRL_GET_SINK_CAP);
					set_state(PD_STATE_SRC_GET_SINK_CAP);
					break;
				}
			}

			/* Check power role policy, which may trigger a swap */
			if (flags & PD_FLAGS_CHECK_PR_ROLE) {
				pd_check_pr_role(port, PD_ROLE_SOURCE, flags);
				flags &= ~PD_FLAGS_CHECK_PR_ROLE;
				break;
			}

			/* Check data role policy, which may trigger a swap */
			if (flags & PD_FLAGS_CHECK_DR_ROLE) {
				pd_check_dr_role(port, data_role, flags);
				flags &= ~PD_FLAGS_CHECK_DR_ROLE;
				break;
			}

			/* Send discovery SVDMs last */
			if (data_role == PD_ROLE_DFP &&
			    (flags & PD_FLAGS_DATA_SWAPPED)) {
#ifndef CONFIG_USB_PD_SIMPLE_DFP
				send_vdm(USB_SID_PD,
					    CMD_DISCOVER_IDENT, NULL, 0);
#endif
				flags &= ~PD_FLAGS_DATA_SWAPPED;
				break;
			}

			if (!(flags & PD_FLAGS_PING_ENABLED))
				break;

			/* Verify that the sink is alive */
			res = send_control(PD_CTRL_PING);
			if (res >= 0)
				break;

			/* Ping dropped. Try soft reset. */
			set_state(PD_STATE_SOFT_RESET);
			timeout = 10 * MSEC;
			break;
		case PD_STATE_SRC_GET_SINK_CAP:
			if (last_state != task_state)
				set_state_timeout(
						  get_time().val +
						  PD_T_SENDER_RESPONSE,
						  PD_STATE_SRC_READY);
			break;
		case PD_STATE_DR_SWAP:
			if (last_state != task_state) {
				res = send_control(PD_CTRL_DR_SWAP);
				if (res < 0) {
					timeout = 10*MSEC;
					/*
					 * If failed to get goodCRC, send
					 * soft reset, otherwise ignore
					 * failure.
					 */
					set_state(res == -1 ?
						   PD_STATE_SOFT_RESET :
						   READY_RETURN_STATE);
					break;
				}
				/* Wait for accept or reject */
				set_state_timeout(
						  get_time().val +
						  PD_T_SENDER_RESPONSE,
						  READY_RETURN_STATE);
			}
			break;
#ifdef CONFIG_USB_PD_DUAL_ROLE
		case PD_STATE_SRC_SWAP_INIT:
			if (last_state != task_state) {
				res = send_control(PD_CTRL_PR_SWAP);
				if (res < 0) {
					timeout = 10*MSEC;
					/*
					 * If failed to get goodCRC, send
					 * soft reset, otherwise ignore
					 * failure.
					 */
					set_state(res == -1 ?
						   PD_STATE_SOFT_RESET :
						   PD_STATE_SRC_READY);
					break;
				}
				/* Wait for accept or reject */
				set_state_timeout(
						  get_time().val +
						  PD_T_SENDER_RESPONSE,
						  PD_STATE_SRC_READY);
			}
			break;
		case PD_STATE_SRC_SWAP_SNK_DISABLE:
			/* Give time for sink to stop drawing current */
			if (last_state != task_state)
				set_state_timeout(
						  get_time().val +
						  PD_T_SINK_TRANSITION,
						  PD_STATE_SRC_SWAP_SRC_DISABLE);
			break;
		case PD_STATE_SRC_SWAP_SRC_DISABLE:
			/* Turn power off */
			if (last_state != task_state) {
				pd_power_supply_reset(port);
				set_state_timeout(
						  get_time().val +
						  PD_POWER_SUPPLY_TURN_OFF_DELAY,
						  PD_STATE_SRC_SWAP_STANDBY);
			}
			break;
		case PD_STATE_SRC_SWAP_STANDBY:
			/* Send PS_RDY to let sink know our power is off */
			if (last_state != task_state) {
				/* Send PS_RDY */
				res = send_control(PD_CTRL_PS_RDY);
				if (res < 0) {
					timeout = 10*MSEC;
					set_state(PD_STATE_SRC_DISCONNECTED);
					break;
				}
				/* Switch to Rd and swap roles to sink */
				m_tcpm->set_cc(TYPEC_CC_RD);
				power_role = PD_ROLE_SINK;
				/* Wait for PS_RDY from new source */
				set_state_timeout(
						  get_time().val +
						  PD_T_PS_SOURCE_ON,
						  PD_STATE_SNK_DISCONNECTED);
			}
			break;
		case PD_STATE_SUSPENDED:
			/* Wait for resume */
			while (task_state == PD_STATE_SUSPENDED)
				task_wait_event(-1);
			break;
		case PD_STATE_SNK_DISCONNECTED:
#ifdef CONFIG_USB_PD_LOW_POWER
			timeout = drp_state != PD_DRP_TOGGLE_ON ? SECOND
								: 10*MSEC;
#else
			timeout = 10*MSEC;
#endif
			m_tcpm->get_cc(&cc1, &cc2);

			/* Source connection monitoring */
			if (cc1 != TYPEC_CC_VOLT_OPEN ||
			    cc2 != TYPEC_CC_VOLT_OPEN) {

				cc_state = PD_CC_NONE;
				hard_reset_count = 0;
				new_cc_state = PD_CC_NONE;
				cc_debounce = get_time().val +
							PD_T_CC_DEBOUNCE;
				set_state(PD_STATE_SNK_DISCONNECTED_DEBOUNCE);
				timeout = 10*MSEC;
				break;
			}
			/*
			 * If Try.SRC is active and failed to detect a SNK,
			 * then it transitions to TryWait.SNK. Need to prevent
			 * normal dual role toggle until tDRPTryWait timer
			 * expires.
			 */
			if (flags & PD_FLAGS_TRY_SRC) {
				if (get_time().val > try_src_marker) {
					flags &= ~PD_FLAGS_TRY_SRC;
				}
				break;
			}

			/*
			 * If no source detected, check for role toggle.
			 * If VBUS is detected, and we are in the debug
			 * accessory toggle state, then allow toggling.
			 */
			if ((drp_state == PD_DRP_TOGGLE_ON &&
			     get_time().val >= next_role_swap) ||
			    snk_debug_acc_toggle()) {

				/* Swap roles to source */
				power_role = PD_ROLE_SOURCE;
				set_state(PD_STATE_SRC_DISCONNECTED);
				m_tcpm->set_cc(TYPEC_CC_RP);
				next_role_swap = get_time().val + PD_T_DRP_SRC;

				/* Swap states quickly */
				timeout = 2*MSEC;
			}
			break;
		case PD_STATE_SNK_DISCONNECTED_DEBOUNCE:
			m_tcpm->get_cc(&cc1, &cc2);
			if (cc_is_rp(cc1) && cc_is_rp(cc2)) {
				/* Debug accessory */
				new_cc_state = PD_CC_DEBUG_ACC;
			} else if (cc_is_rp(cc1) || cc_is_rp(cc2)) {
				new_cc_state = PD_CC_DFP_ATTACHED;
			} else {
				/* No connection any more */
				set_state(PD_STATE_SNK_DISCONNECTED);
				timeout = 5*MSEC;
				break;
			}

			timeout = 20*MSEC;

			/* Debounce the cc state */
			if (new_cc_state != cc_state) {
				cc_debounce = get_time().val +
					PD_T_CC_DEBOUNCE;
				cc_state = new_cc_state;
				break;
			}

			/* Wait for CC debounce and VBUS present */
			if (get_time().val < cc_debounce ||
			    !pd_is_vbus_present(port)) {
				break;
			}
			if (pd_try_src_enable &&
			    !(flags & PD_FLAGS_TRY_SRC)) {
				/*
				 * If TRY_SRC is enabled, but not active,
				 * then force attempt to connect as source.
				 */
				try_src_marker = get_time().val
					+ PD_T_TRY_SRC;
				/* Swap roles to source */
				power_role = PD_ROLE_SOURCE;
				m_tcpm->set_cc(TYPEC_CC_RP);
				timeout = 2*MSEC;
				set_state(PD_STATE_SRC_DISCONNECTED);
				/* Set flag after the state change */
				flags |= PD_FLAGS_TRY_SRC;
				break;
			}

			/* We are attached */
			polarity = get_snk_polarity(cc1, cc2);
			m_tcpm->set_polarity(polarity);
			/* reset message ID  on connection */
			msg_id = 0;
			/* initial data role for sink is UFP */
			set_data_role(PD_ROLE_UFP);
#ifdef CONFIG_CHARGE_MANAGER
			typec_curr = get_typec_current_limit(pd[port].polarity,
							     cc1, cc2);
			typec_set_input_current_limit(
				port, typec_curr, TYPE_C_VOLTAGE);
#endif
			/* If PD comm is enabled, enable TCPC RX */
			if (pd_comm_enabled)
				m_tcpm->set_rx_enable(1);

			/* DFP is attached */
			if (new_cc_state == PD_CC_DFP_ATTACHED) {
				/*
				 * fake set data role swapped flag so we send
				 * discover identity when we enter SRC_READY
				 */
				flags |= PD_FLAGS_CHECK_PR_ROLE |
						  PD_FLAGS_CHECK_DR_ROLE |
						  PD_FLAGS_DATA_SWAPPED;
				send_control(PD_CTRL_GET_SOURCE_CAP);
				set_state(PD_STATE_SNK_DISCOVERY);
				timeout = 10*MSEC;
				//hook_call_deferred(
				//	&pd_usb_billboard_deferred_data,
				//	PD_T_AME);
			}
#if defined(CONFIG_CASE_CLOSED_DEBUG) || \
defined(CONFIG_CASE_CLOSED_DEBUG_EXTERNAL)
			else if (new_cc_state == PD_CC_DEBUG_ACC) {
#ifdef CONFIG_CASE_CLOSED_DEBUG
				ccd_set_mode(system_is_locked() ?
					     CCD_MODE_PARTIAL :
					     CCD_MODE_ENABLED);
#endif
				set_state(port, PD_STATE_SNK_ACCESSORY);
			}
			break;
		case PD_STATE_SNK_ACCESSORY:
			/* debug accessory state */
			timeout = 100*MSEC;

			m_tcpm->get_cc(&cc1, &cc2);

			/* If accessory becomes detached */
			if (!cc_is_rp(cc1) || !cc_is_rp(cc2)) {
				set_state(PD_STATE_SNK_DISCONNECTED);
#ifdef CONFIG_CASE_CLOSED_DEBUG
				ccd_set_mode(CCD_MODE_DISABLED);
#endif
				timeout = 10*MSEC;
			}
#endif
			break;
		case PD_STATE_SNK_HARD_RESET_RECOVER:
			if (last_state != task_state)
				flags |= PD_FLAGS_DATA_SWAPPED;
#ifdef CONFIG_USB_PD_VBUS_DETECT_NONE
			/*
			 * Can't measure vbus state so this is the maximum
			 * recovery time for the source.
			 */
			if (last_state != task_state)
				set_state_timeout(get_time().val +
						  PD_T_SAFE_0V +
						  PD_T_SRC_RECOVER_MAX +
						  PD_T_SRC_TURN_ON,
						  PD_STATE_SNK_DISCONNECTED);
#else
			/* Wait for VBUS to go low and then high*/
			if (last_state != task_state) {
				snk_hard_reset_vbus_off = 0;
				set_state_timeout(
						  get_time().val +
						  PD_T_SAFE_0V,
						  hard_reset_count <
						    PD_HARD_RESET_COUNT ?
						     PD_STATE_HARD_RESET_SEND :
						     PD_STATE_SNK_DISCOVERY);
			}

			if (!pd_is_vbus_present(port) &&
			    !snk_hard_reset_vbus_off) {
				/* VBUS has gone low, reset timeout */
				snk_hard_reset_vbus_off = 1;
				set_state_timeout(
						  get_time().val +
						  PD_T_SRC_RECOVER_MAX +
						  PD_T_SRC_TURN_ON,
						  PD_STATE_SNK_DISCONNECTED);
			}
			if (pd_is_vbus_present(port) &&
			    snk_hard_reset_vbus_off) {
#ifdef CONFIG_USB_PD_TCPM_TCPCI
				/*
				 * After transmitting hard reset, TCPM writes
				 * to RECEIVE_MESSAGE register to enable
				 * PD message passing.
				 */
				if (pd_comm_enabled)
					tcpm_set_rx_enable(port, 1);
#endif /* CONFIG_USB_PD_TCPM_TCPCI */

				/* VBUS went high again */
				send_control(PD_CTRL_GET_SOURCE_CAP);
				set_state(PD_STATE_SNK_DISCOVERY);
				timeout = 10*MSEC;
			}

			/*
			 * Don't need to set timeout because VBUS changing
			 * will trigger an interrupt and wake us up.
			 */
#endif
			break;
		case PD_STATE_SNK_DISCOVERY:
			/* Wait for source cap expired only if we are enabled */
			if ((last_state != task_state)
			    && pd_comm_enabled) {
				/*
				 * If VBUS has never been low, and we timeout
				 * waiting for source cap, try a soft reset
				 * first, in case we were already in a stable
				 * contract before this boot.
				 */
				if (flags & PD_FLAGS_VBUS_NEVER_LOW) {
					set_state_timeout(
						  get_time().val +
						  PD_T_SINK_WAIT_CAP,
						  PD_STATE_SOFT_RESET);
				/*
				 * If we haven't passed hard reset counter,
				 * start SinkWaitCapTimer, otherwise start
				 * NoResponseTimer.
				 */
				} else if (hard_reset_count < PD_HARD_RESET_COUNT) {
					set_state_timeout(
						  get_time().val +
						  PD_T_SINK_WAIT_CAP,
						  PD_STATE_HARD_RESET_SEND);
				} else if (flags & PD_FLAGS_PREVIOUS_PD_CONN) {
					/* ErrorRecovery */
					set_state_timeout(
						  get_time().val +
						  PD_T_NO_RESPONSE,
						  PD_STATE_SNK_DISCONNECTED);
				}
#ifdef CONFIG_CHARGE_MANAGER
				/*
				 * If we didn't come from disconnected, must
				 * have come from some path that did not set
				 * typec current limit. So, set to 0 so that
				 * we guarantee this is revised below.
				 */
				if (last_state !=
				    PD_STATE_SNK_DISCONNECTED_DEBOUNCE) {
					typec_curr = 0;
				}
#endif
			}

#ifdef CONFIG_CHARGE_MANAGER
			timeout = PD_T_SINK_ADJ - PD_T_DEBOUNCE;

			/* Check if CC pull-up has changed */
			m_tcpm->get_cc(&cc1, &cc2);
			if (typec_curr != get_typec_current_limit(
						polarity, cc1, cc2)) {
				/* debounce signal by requiring two reads */
				if (typec_curr_change) {
					/* set new input current limit */
					typec_curr = get_typec_current_limit(
						polarity, cc1, cc2);
					typec_set_input_current_limit(
					  typec_curr, TYPE_C_VOLTAGE);
				} else {
					/* delay for debounce */
					timeout = PD_T_DEBOUNCE;
				}
				typec_curr_change = !typec_curr_change;
			} else {
				typec_curr_change = 0;
			}
#endif
			break;
		case PD_STATE_SNK_REQUESTED:
			/* Wait for ACCEPT or REJECT */
			if (last_state != task_state) {
				hard_reset_count = 0;
				set_state_timeout(
						  get_time().val +
						  PD_T_SENDER_RESPONSE,
						  PD_STATE_HARD_RESET_SEND);
			}
			break;
		case PD_STATE_SNK_TRANSITION:
			/* Wait for PS_RDY */
			if (last_state != task_state)
				set_state_timeout(
						  get_time().val +
						  PD_T_PS_TRANSITION,
						  PD_STATE_HARD_RESET_SEND);
			break;
		case PD_STATE_SNK_READY:
			timeout = 20*MSEC;

			/*
			 * Don't send any PD traffic if we woke up due to
			 * incoming packet or if VDO response pending to avoid
			 * collisions.
			 */
			if (incoming_packet ||
			    (vdm_state == VDM_STATE_BUSY))
				break;

			/* Check for new power to request */
			if (new_power_request) {
				if (send_request_msg(0) != EC_SUCCESS)
					set_state(PD_STATE_SOFT_RESET);
				break;
			}

			/* Check power role policy, which may trigger a swap */
			if (flags & PD_FLAGS_CHECK_PR_ROLE) {
				pd_check_pr_role(port, PD_ROLE_SINK, flags);
				flags &= ~PD_FLAGS_CHECK_PR_ROLE;
				break;
			}

			/* Check data role policy, which may trigger a swap */
			if (flags & PD_FLAGS_CHECK_DR_ROLE) {
				pd_check_dr_role(port, data_role, flags);
				flags &= ~PD_FLAGS_CHECK_DR_ROLE;
				break;
			}

			/* If DFP, send discovery SVDMs */
			if (data_role == PD_ROLE_DFP &&
			     (flags & PD_FLAGS_DATA_SWAPPED)) {
				send_vdm(USB_SID_PD,
					    CMD_DISCOVER_IDENT, NULL, 0);
				flags &= ~PD_FLAGS_DATA_SWAPPED;
				break;
			}

			/* Sent all messages, don't need to wake very often */
			timeout = 200*MSEC;
			break;
		case PD_STATE_SNK_SWAP_INIT:
			if (last_state != task_state) {
				res = send_control(PD_CTRL_PR_SWAP);
				if (res < 0) {
					timeout = 10*MSEC;
					/*
					 * If failed to get goodCRC, send
					 * soft reset, otherwise ignore
					 * failure.
					 */
					set_state(res == -1 ?
						   PD_STATE_SOFT_RESET :
						   PD_STATE_SNK_READY);
					break;
				}
				/* Wait for accept or reject */
				set_state_timeout(
						  get_time().val +
						  PD_T_SENDER_RESPONSE,
						  PD_STATE_SNK_READY);
			}
			break;
		case PD_STATE_SNK_SWAP_SNK_DISABLE:
			/* Stop drawing power */
			pd_set_input_current_limit(port, 0, 0);
#ifdef CONFIG_CHARGE_MANAGER
			typec_set_input_current_limit(port, 0, 0);
			charge_manager_set_ceil(port,
						CEIL_REQUESTOR_PD,
						CHARGE_CEIL_NONE);
#endif
			set_state(PD_STATE_SNK_SWAP_SRC_DISABLE);
			timeout = 10*MSEC;
			break;
		case PD_STATE_SNK_SWAP_SRC_DISABLE:
			/* Wait for PS_RDY */
			if (last_state != task_state)
				set_state_timeout(
						  get_time().val +
						  PD_T_PS_SOURCE_OFF,
						  PD_STATE_HARD_RESET_SEND);
			break;
		case PD_STATE_SNK_SWAP_STANDBY:
			if (last_state != task_state) {
				/* Switch to Rp and enable power supply */
				m_tcpm->set_cc(TYPEC_CC_RP);
				if (pd_set_power_supply_ready(port)) {
					/* Restore Rd */
					m_tcpm->set_cc(TYPEC_CC_RD);
					timeout = 10*MSEC;
					set_state(PD_STATE_SNK_DISCONNECTED);
					break;
				}
				/* Wait for power supply to turn on */
				set_state_timeout(
					get_time().val +
					PD_POWER_SUPPLY_TURN_ON_DELAY,
					PD_STATE_SNK_SWAP_COMPLETE);
			}
			break;
		case PD_STATE_SNK_SWAP_COMPLETE:
			/* Send PS_RDY and change to source role */
			res = send_control(PD_CTRL_PS_RDY);
			if (res < 0) {
				/* Restore Rd */
				m_tcpm->set_cc(TYPEC_CC_RD);
				pd_power_supply_reset(port);
				timeout = 10 * MSEC;
				set_state(PD_STATE_SNK_DISCONNECTED);
				break;
			}

			/* Don't send GET_SINK_CAP on swap */
			snk_cap_count = PD_SNK_CAP_RETRIES+1;
			caps_count = 0;
			msg_id = 0;
			power_role = PD_ROLE_SOURCE;
			update_roles();
			set_state(PD_STATE_SRC_DISCOVERY);
			timeout = 10*MSEC;
			break;
#ifdef CONFIG_USBC_VCONN_SWAP
		case PD_STATE_VCONN_SWAP_SEND:
			if (pd[port].last_state != pd[port].task_state) {
				res = send_control(port, PD_CTRL_VCONN_SWAP);
				if (res < 0) {
					timeout = 10*MSEC;
					/*
					 * If failed to get goodCRC, send
					 * soft reset, otherwise ignore
					 * failure.
					 */
					set_state(port, res == -1 ?
						   PD_STATE_SOFT_RESET :
						   READY_RETURN_STATE(port));
					break;
				}
				/* Wait for accept or reject */
				set_state_timeout(port,
						  get_time().val +
						  PD_T_SENDER_RESPONSE,
						  READY_RETURN_STATE(port));
			}
			break;
		case PD_STATE_VCONN_SWAP_INIT:
			if (pd[port].last_state != pd[port].task_state) {
				if (!(pd[port].flags & PD_FLAGS_VCONN_ON)) {
					/* Turn VCONN on and wait for it */
					tcpm_set_vconn(port, 1);
					set_state_timeout(port,
					  get_time().val + PD_VCONN_SWAP_DELAY,
					  PD_STATE_VCONN_SWAP_READY);
				} else {
					set_state_timeout(port,
					  get_time().val + PD_T_VCONN_SOURCE_ON,
					  READY_RETURN_STATE(port));
				}
			}
			break;
		case PD_STATE_VCONN_SWAP_READY:
			if (pd[port].last_state != pd[port].task_state) {
				if (!(pd[port].flags & PD_FLAGS_VCONN_ON)) {
					/* VCONN is now on, send PS_RDY */
					pd[port].flags |= PD_FLAGS_VCONN_ON;
					res = send_control(port,
							   PD_CTRL_PS_RDY);
					if (res == -1) {
						timeout = 10*MSEC;
						/*
						 * If failed to get goodCRC,
						 * send soft reset
						 */
						set_state(port,
							  PD_STATE_SOFT_RESET);
						break;
					}
					set_state(port,
						  READY_RETURN_STATE(port));
				} else {
					/* Turn VCONN off and wait for it */
					tcpm_set_vconn(port, 0);
					pd[port].flags &= ~PD_FLAGS_VCONN_ON;
					set_state_timeout(port,
					  get_time().val + PD_VCONN_SWAP_DELAY,
					  READY_RETURN_STATE(port));
				}
			}
			break;
#endif /* CONFIG_USBC_VCONN_SWAP */
#endif /* CONFIG_USB_PD_DUAL_ROLE */
		case PD_STATE_SOFT_RESET:
			if (last_state != task_state) {
				/* Message ID of soft reset is always 0 */
				msg_id = 0;
				res = send_control(PD_CTRL_SOFT_RESET);

				/* if soft reset failed, try hard reset. */
				if (res < 0) {
					set_state(PD_STATE_HARD_RESET_SEND);
					timeout = 5*MSEC;
					break;
				}

				set_state_timeout(
					get_time().val + PD_T_SENDER_RESPONSE,
					PD_STATE_HARD_RESET_SEND);
			}
			break;
		case PD_STATE_HARD_RESET_SEND:
			hard_reset_count++;
			if (last_state != task_state)
				hard_reset_sent = 0;
#ifdef CONFIG_CHARGE_MANAGER
			if (last_state == PD_STATE_SNK_DISCOVERY ||
			    (last_state == PD_STATE_SOFT_RESET &&
			     (flags & PD_FLAGS_VBUS_NEVER_LOW))) {
				flags &= ~PD_FLAGS_VBUS_NEVER_LOW;
				/*
				 * If discovery timed out, assume that we
				 * have a dedicated charger attached. This
				 * may not be a correct assumption according
				 * to the specification, but it generally
				 * works in practice and the harmful
				 * effects of a wrong assumption here
				 * are minimal.
				 */
				charge_manager_update_dualrole(port,
							       CAP_DEDICATED);
			}
#endif

			/* try sending hard reset until it succeeds */
			if (!hard_reset_sent) {
				if (m_tcpm->transmit(TCPC_TX_HARD_RESET,
						0, NULL) < 0) {
					timeout = 10*MSEC;
					break;
				}

				/* successfully sent hard reset */
				hard_reset_sent = 1;
				/*
				 * If we are source, delay before cutting power
				 * to allow sink time to get hard reset.
				 */
				if (power_role == PD_ROLE_SOURCE) {
					set_state_timeout(
					  get_time().val + PD_T_PS_HARD_RESET,
					  PD_STATE_HARD_RESET_EXECUTE);
				} else {
					set_state(PD_STATE_HARD_RESET_EXECUTE);
					timeout = 10*MSEC;
				}
			}
			break;
		case PD_STATE_HARD_RESET_EXECUTE:
#ifdef CONFIG_USB_PD_DUAL_ROLE
			/*
			 * If hard reset while in the last stages of power
			 * swap, then we need to restore our CC resistor.
			 */
			if (last_state == PD_STATE_SNK_SWAP_STANDBY)
				m_tcpm->set_cc(TYPEC_CC_RD);
#endif

			/* reset our own state machine */
			execute_hard_reset();
			timeout = 10*MSEC;
			break;
#ifdef CONFIG_COMMON_RUNTIME
		case PD_STATE_BIST_RX:
			send_bist_cmd(port);
			/* Delay at least enough for partner to finish BIST */
			timeout = PD_T_BIST_RECEIVE + 20*MSEC;
			/* Set to appropriate port disconnected state */
			set_state(port, DUAL_ROLE_IF_ELSE(port,
						PD_STATE_SNK_DISCONNECTED,
						PD_STATE_SRC_DISCONNECTED));
			break;
		case PD_STATE_BIST_TX:
			pd_transmit(port, TCPC_TX_BIST_MODE_2, 0, NULL);
			/* Delay at least enough to finish sending BIST */
			timeout = PD_T_BIST_TRANSMIT + 20*MSEC;
			/* Set to appropriate port disconnected state */
			set_state(port, DUAL_ROLE_IF_ELSE(port,
						PD_STATE_SNK_DISCONNECTED,
						PD_STATE_SRC_DISCONNECTED));
			break;
#endif
		default:
			break;
		};

		last_state = this_state;

		/*
		 * Check for state timeout, and if not check if need to adjust
		 * timeout value to wake up on the next state timeout.
		 */
		now = get_time();
		if (timeout_deadline) {
			if (now.val >= timeout_deadline) {
				set_state(timeout_state);
				/* On a state timeout, run next state soon */
				timeout = timeout < 10*MSEC ? timeout : 10*MSEC;
			} else if (timeout_deadline - now.val < timeout) {
				timeout = timeout_deadline - now.val;
			}
		}

		/* Check for disconnection if we're connected */
		if (!is_connected())
			return;
#ifdef CONFIG_USB_PD_DUAL_ROLE
		if (is_power_swapping())
			return;
#endif
		if (power_role == PD_ROLE_SOURCE) {
			/* Source: detect disconnect by monitoring CC */
			m_tcpm->get_cc(&cc1, &cc2);
			if (polarity)
				cc1 = cc2;
			if (cc1 == TYPEC_CC_VOLT_OPEN) {
				set_state(PD_STATE_SRC_DISCONNECTED);
				/* Debouncing */
				timeout = 10*MSEC;
#ifdef CONFIG_USB_PD_DUAL_ROLE
				/*
				 * If Try.SRC is configured, then ATTACHED_SRC
				 * needs to transition to TryWait.SNK. Change
				 * power role to SNK and start state timer.
				 */
				if (pd_try_src_enable) {
					/* Swap roles to sink */
					power_role = PD_ROLE_SINK;
					m_tcpm->set_cc(TYPEC_CC_RD);
					/* Set timer for TryWait.SNK state */
					try_src_marker = get_time().val
						+ PD_T_TRY_WAIT;
					/* Advance to TryWait.SNK state */
					set_state(PD_STATE_SNK_DISCONNECTED);
					/* Mark state as TryWait.SNK */
					flags |= PD_FLAGS_TRY_SRC;
				}
#endif
			}
		}
#ifdef CONFIG_USB_PD_DUAL_ROLE
		/*
		 * Sink disconnect if VBUS is low and we are not recovering
		 * a hard reset.
		 */
		if (power_role == PD_ROLE_SINK &&
		    !pd_is_vbus_present(port) &&
		    task_state != PD_STATE_SNK_HARD_RESET_RECOVER &&
		    task_state != PD_STATE_HARD_RESET_EXECUTE) {

			/* Sink: detect disconnect by monitoring VBUS */
			set_state(PD_STATE_SNK_DISCONNECTED);
			/* set timeout small to reconnect fast */
			timeout = 5*MSEC;
		}
#endif /* CONFIG_USB_PD_DUAL_ROLE */
}

void USB_PD::set_state(enum pd_states next_state) {
	enum pd_states last_state = task_state;
#ifdef CONFIG_LOW_POWER_IDLE
	int i;
#endif

	set_state_timeout(0, PD_STATE_DISABLED);
	task_state = next_state;

	if (last_state == next_state)
		return;
#ifdef CONFIG_USB_PD_DUAL_ROLE
	/* Ignore dual-role toggling between sink and source */
	if ((last_state == PD_STATE_SNK_DISCONNECTED &&
	     next_state == PD_STATE_SRC_DISCONNECTED) ||
	    (last_state == PD_STATE_SRC_DISCONNECTED &&
	     next_state == PD_STATE_SNK_DISCONNECTED))
		return;
#endif

#ifdef CONFIG_USB_PD_DUAL_ROLE
	if (next_state == PD_STATE_SRC_DISCONNECTED ||
	    next_state == PD_STATE_SNK_DISCONNECTED) {
		/* Clear the input current limit */
		pd_set_input_current_limit(port, 0, 0);
#ifdef CONFIG_USBC_VCONN
		m_tcpm->set_vconn(0);
#endif
#else /* CONFIG_USB_PD_DUAL_ROLE */
	if (next_state == PD_STATE_SRC_DISCONNECTED) {
#endif
		/* If we are source, make sure VBUS is off */
		if (power_role == PD_ROLE_SOURCE)
			pd_power_supply_reset(port);

		//pd[port].dev_id = 0;
		flags &= ~PD_FLAGS_RESET_ON_DISCONNECT_MASK;

#ifdef CONFIG_USB_PD_ALT_MODE_DFP
		pd_dfp_exit_mode(port, 0, 0);
#endif
#ifdef CONFIG_USBC_SS_MUX
		usb_mux_set(port, TYPEC_MUX_NONE, USB_SWITCH_DISCONNECT,
			    pd[port].polarity);
#endif
		/* Disable TCPC RX */
		m_tcpm->set_rx_enable(0);
	}

#ifdef CONFIG_LOW_POWER_IDLE
	// If any PD port is connected, then disable deep sleep
	for (i = 0; i < CONFIG_USB_PD_PORT_COUNT; i++) {
		if (pd_is_connected(i))
			break;
	}
	if (i == CONFIG_USB_PD_PORT_COUNT)
		enable_sleep(SLEEP_MASK_USB_PD);
	else
		disable_sleep(SLEEP_MASK_USB_PD);
#endif
}

void USB_PD::vdm_send_state_machine(void)
{
	int res;
	uint16_t header;

	switch (vdm_state) {
	case VDM_STATE_READY:
		/* Only transmit VDM if connected. */
		if (!is_connected()) {
			vdm_state = VDM_STATE_ERR_BUSY;
			break;
		}

		/*
		 * if there's traffic or we're not in PDO ready state don't send
		 * a VDM.
		 */
		if (pdo_busy())
			break;

		/* Prepare and send VDM */
		header = PD_HEADER(PD_DATA_VENDOR_DEF, power_role,
				   data_role, msg_id,
				   (int)vdo_count);
		res = pd_transmit(TCPC_TX_SOP, header, vdo_data);
		if (res < 0) {
			vdm_state = VDM_STATE_ERR_SEND;
		} else {
			vdm_state = VDM_STATE_BUSY;
			vdm_timeout.val = get_time().val +
				vdm_get_ready_timeout(vdo_data[0]);
		}
		break;
	case VDM_STATE_WAIT_RSP_BUSY:
		/* wait and then initiate request again */
		if (get_time().val > vdm_timeout.val) {
			vdo_data[0] = vdo_retry;
			vdo_count = 1;
			vdm_state = VDM_STATE_READY;
		}
		break;
	case VDM_STATE_BUSY:
		/* Wait for VDM response or timeout */
		if (vdm_timeout.val &&
		    (get_time().val > vdm_timeout.val)) {
			vdm_state = VDM_STATE_ERR_TMOUT;
		}
		break;
	default:
		break;
	}
}

void USB_PD::set_state_timeout(uint64_t timeout_deadline, enum pd_states timeout_state) {
	this->timeout_deadline = timeout_deadline;
	this->timeout_state = timeout_state;
}

// Return flag for pd state is connected
int USB_PD::is_connected(void)
{
	if (task_state == PD_STATE_DISABLED)
		return 0;

	return DUAL_ROLE_IF_ELSE(
		// sink
		task_state != PD_STATE_SNK_DISCONNECTED &&
		task_state != PD_STATE_SNK_DISCONNECTED_DEBOUNCE,
		// source
		task_state != PD_STATE_SRC_DISCONNECTED &&
		task_state != PD_STATE_SRC_DISCONNECTED_DEBOUNCE);
}

int USB_PD::pdo_busy(void)
{
	/*
	 * Note, main PDO state machine (pd_task) uses READY state exclusively
	 * to denote port partners have successfully negociated a contract.  All
	 * other protocol actions force state transitions.
	 */
	int rv = (task_state != PD_STATE_SRC_READY);
#ifdef CONFIG_USB_PD_DUAL_ROLE
	rv &= (task_state != PD_STATE_SNK_READY);
#endif
	return rv;
}

int USB_PD::pd_transmit(enum tcpm_transmit_type type,
		       uint16_t header, const uint32_t *data) {
	int res;

	// If comms are disabled, do not transmit, return error
	if (!pd_comm_enabled)
		return -1;

	m_tcpm->transmit(type, header, data);

	// Wait until TX is complete
	//res = task_wait_event_mask(PD_EVENT_TX, PD_T_TCPC_TX_TIMEOUT);
	//pd_transmit_complete(res);

	if (res != TCPC_TX_COMPLETE_SUCCESS)
		return -1;

	/* TODO: give different error condition for failed vs discarded */
	return tx_status == TCPC_TX_COMPLETE_SUCCESS ? 1 : -1;
}

uint64_t USB_PD::vdm_get_ready_timeout(uint32_t vdm_hdr) {
	uint64_t timeout;
	int cmd = PD_VDO_CMD(vdm_hdr);

	// its not a structured VDM command
	if (!PD_VDO_SVDM(vdm_hdr))
		return 500*MSEC;

	switch (PD_VDO_CMDT(vdm_hdr)) {
	case CMDT_INIT:
		if ((cmd == CMD_ENTER_MODE) || (cmd == CMD_EXIT_MODE))
			timeout = PD_T_VDM_WAIT_MODE_E;
		else
			timeout = PD_T_VDM_SNDR_RSP;
		break;
	default:
		if ((cmd == CMD_ENTER_MODE) || (cmd == CMD_EXIT_MODE))
			timeout = PD_T_VDM_E_MODE;
		else
			timeout = PD_T_VDM_RCVR_RSP;
		break;
	}
	return timeout;
}

void USB_PD::pd_transmit_complete(int status) {
	if (status == TCPC_TX_COMPLETE_SUCCESS)
		inc_id();

	tx_status = status;
	//task_set_event(PD_PORT_TO_TASK_ID(port), PD_EVENT_TX, 0);
}

void USB_PD::inc_id(void) {
	msg_id = (msg_id + 1) & PD_MESSAGE_ID_COUNT;
}

void USB_PD::execute_hard_reset(void) {
	msg_id = 0;
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
	pd_dfp_exit_mode(port, 0, 0);
#endif

	/*
	 * Fake set last state to hard reset to make sure that the next
	 * state to run knows that we just did a hard reset.
	 */
	last_state = PD_STATE_HARD_RESET_EXECUTE;

#ifdef CONFIG_USB_PD_DUAL_ROLE
	/*
	 * If we are swapping to a source and have changed to Rp, restore back
	 * to Rd and turn off vbus to match our power_role.
	 */
	if (task_state == PD_STATE_SNK_SWAP_STANDBY ||
	    task_state == PD_STATE_SNK_SWAP_COMPLETE) {
		m_tcpm->set_cc(TYPEC_CC_RD);
		pd_power_supply_reset(port);
	}

	if (power_role == PD_ROLE_SINK) {
		/* Clear the input current limit */
		pd_set_input_current_limit(port, 0, 0);
#ifdef CONFIG_CHARGE_MANAGER
		charge_manager_set_ceil(port,
					CEIL_REQUESTOR_PD,
					CHARGE_CEIL_NONE);
#endif /* CONFIG_CHARGE_MANAGER */

		set_state(PD_STATE_SNK_HARD_RESET_RECOVER);
		return;
	}
#endif /* CONFIG_USB_PD_DUAL_ROLE */

	/* We are a source, cut power */
	pd_power_supply_reset(port);
	src_recover = get_time().val + PD_T_SRC_RECOVER;
	set_state(PD_STATE_SRC_HARD_RESET_RECOVER);
}

void USB_PD::handle_request(uint16_t head, uint32_t *payload) {
	int cnt = PD_HEADER_CNT(head);
	int p;

	/*
	 * If we are in disconnected state, we shouldn't get a request. Do
	 * a hard reset if we get one.
	 */
	if (!is_connected()) {
		set_state(PD_STATE_HARD_RESET_SEND);
	}

	if (cnt) {
		handle_data_request(head, payload);
	} else {
		handle_ctrl_request(head, payload);
	}
}

void USB_PD::handle_data_request(uint16_t head,
		uint32_t *payload) {
	int type = PD_HEADER_TYPE(head);
	int cnt = PD_HEADER_CNT(head);

	switch (type) {
#ifdef CONFIG_USB_PD_DUAL_ROLE
	case PD_DATA_SOURCE_CAP:
		if ((task_state == PD_STATE_SNK_DISCOVERY)
			|| (task_state == PD_STATE_SNK_TRANSITION)
#ifdef CONFIG_USB_PD_VBUS_DETECT_NONE
			|| (task_state ==
			    PD_STATE_SNK_HARD_RESET_RECOVER)
#endif
			|| (task_state == PD_STATE_SNK_READY)) {
			/* Port partner is now known to be PD capable */
			flags |= PD_FLAGS_PREVIOUS_PD_CONN;

			store_src_cap(cnt, payload);
			/* src cap 0 should be fixed PDO */
			update_pdo_flags(payload[0]);

			process_source_cap(pd_src_cap_cnt, pd_src_caps);
			/* Source will resend source cap on failure */
			int rv;
			rv = send_request_msg(1);
		}
		break;
#endif /* CONFIG_USB_PD_DUAL_ROLE */
	case PD_DATA_REQUEST:
		if ((power_role == PD_ROLE_SOURCE) && (cnt == 1))
			if (!check_requested_voltage(payload[0])) {
				if (send_control(PD_CTRL_ACCEPT) < 0)
					/*
					 * if we fail to send accept, do
					 * nothing and let sink timeout and
					 * send hard reset
					 */
					return;

				/* explicit contract is now in place */
				flags |= PD_FLAGS_EXPLICIT_CONTRACT;
				requested_idx = RDO_POS(payload[0]);
				set_state(PD_STATE_SRC_ACCEPTED);
				return;
			}
		/* the message was incorrect or cannot be satisfied */
		send_control(PD_CTRL_REJECT);
		/* keep last contract in place (whether implicit or explicit) */
		set_state(PD_STATE_SRC_READY);
		break;
	case PD_DATA_BIST:
		/* If not in READY state, then don't start BIST */
		if (DUAL_ROLE_IF_ELSE(
				task_state == PD_STATE_SNK_READY,
				task_state == PD_STATE_SRC_READY)) {
			/* currently only support sending bist carrier mode 2 */
			if ((payload[0] >> 28) == 5) {
				/* bist data object mode is 2 */
				pd_transmit(TCPC_TX_BIST_MODE_2, 0, NULL);
				/* Set to appropriate port disconnected state */
				set_state(DUAL_ROLE_IF_ELSE(
						PD_STATE_SNK_DISCONNECTED,
						PD_STATE_SRC_DISCONNECTED));
			}
		}
		break;
	case PD_DATA_SINK_CAP:
		flags |= PD_FLAGS_SNK_CAP_RECVD;
		/* snk cap 0 should be fixed PDO */
		update_pdo_flags(payload[0]);
		if (task_state == PD_STATE_SRC_GET_SINK_CAP)
			set_state(PD_STATE_SRC_READY);
		break;
	case PD_DATA_VENDOR_DEF:
		handle_vdm_request(cnt, payload);
		break;
	default:
		//CPRINTF("Unhandled data message type %d\n", type);
		break;
	}
}

void USB_PD::handle_ctrl_request(uint16_t head, 
		uint32_t *payload) {
	int type = PD_HEADER_TYPE(head);
	int res;

	switch (type) {
	case PD_CTRL_GOOD_CRC:
		pd_transmit_complete(TCPC_TX_COMPLETE_SUCCESS);
		break;
	case PD_CTRL_PING:
		/* Nothing else to do */
		break;
	case PD_CTRL_GET_SOURCE_CAP:
		res = send_source_cap();
		if ((res >= 0) &&
		    (task_state == PD_STATE_SRC_DISCOVERY))
			set_state(PD_STATE_SRC_NEGOCIATE);
		break;
	case PD_CTRL_GET_SINK_CAP:
#ifdef CONFIG_USB_PD_DUAL_ROLE
		send_sink_cap();
#else
		send_control(PD_CTRL_REJECT);
#endif
		break;
#ifdef CONFIG_USB_PD_DUAL_ROLE
	case PD_CTRL_GOTO_MIN:
		break;
	case PD_CTRL_PS_RDY:
		if (task_state == PD_STATE_SNK_SWAP_SRC_DISABLE) {
			set_state(PD_STATE_SNK_SWAP_STANDBY);
		} else if (task_state == PD_STATE_SRC_SWAP_STANDBY) {
			/* reset message ID and swap roles */
			msg_id = 0;
			power_role = PD_ROLE_SINK;
			update_roles();
			set_state(PD_STATE_SNK_DISCOVERY);
#ifdef CONFIG_USBC_VCONN_SWAP
		} else if (task_state == PD_STATE_VCONN_SWAP_INIT) {
			/*
			 * If VCONN is on, then this PS_RDY tells us it's
			 * ok to turn VCONN off
			 */
			if (flags & PD_FLAGS_VCONN_ON)
				set_state(PD_STATE_VCONN_SWAP_READY);
#endif
		} else if (task_state == PD_STATE_SNK_DISCOVERY) {
			/* Don't know what power source is ready. Reset. */
			set_state(PD_STATE_HARD_RESET_SEND);
		} else if (task_state == PD_STATE_SNK_SWAP_STANDBY) {
			/* Do nothing, assume this is a redundant PD_RDY */
		} else if (power_role == PD_ROLE_SINK) {
			set_state(PD_STATE_SNK_READY);
			pd_set_input_current_limit(port, curr_limit,
						   				supply_voltage);
#ifdef CONFIG_CHARGE_MANAGER
			/* Set ceiling based on what's negotiated */
			charge_manager_set_ceil(
						CEIL_REQUESTOR_PD,
						curr_limit);
#endif
		}
		break;
#endif
	case PD_CTRL_REJECT:
	case PD_CTRL_WAIT:
		if (task_state == PD_STATE_DR_SWAP)
			set_state(READY_RETURN_STATE);
#ifdef CONFIG_USBC_VCONN_SWAP
		else if (task_state == PD_STATE_VCONN_SWAP_SEND)
			set_state(READY_RETURN_STATE);
#endif
#ifdef CONFIG_USB_PD_DUAL_ROLE
		else if (task_state == PD_STATE_SRC_SWAP_INIT)
			set_state(PD_STATE_SRC_READY);
		else if (task_state == PD_STATE_SNK_SWAP_INIT)
			set_state(PD_STATE_SNK_READY);
		else if (task_state == PD_STATE_SNK_REQUESTED)
			/* no explicit contract */
			set_state(PD_STATE_SNK_READY);
#endif
		break;
	case PD_CTRL_ACCEPT:
		if (task_state == PD_STATE_SOFT_RESET) {
			/*
			 * For the case that we sent soft reset in SNK_DISCOVERY
			 * on startup due to VBUS never low, clear the flag.
			 */
			flags &= ~PD_FLAGS_VBUS_NEVER_LOW;
			execute_soft_reset();
		} else if (task_state == PD_STATE_DR_SWAP) {
			/* switch data role */
			pd_dr_swap();
			set_state(READY_RETURN_STATE);
#ifdef CONFIG_USB_PD_DUAL_ROLE
#ifdef CONFIG_USBC_VCONN_SWAP
		} else if (task_state == PD_STATE_VCONN_SWAP_SEND) {
			/* switch vconn */
			set_state(PD_STATE_VCONN_SWAP_INIT);
#endif
		} else if (task_state == PD_STATE_SRC_SWAP_INIT) {
			/* explicit contract goes away for power swap */
			flags &= ~PD_FLAGS_EXPLICIT_CONTRACT;
			set_state(PD_STATE_SRC_SWAP_SNK_DISABLE);
		} else if (task_state == PD_STATE_SNK_SWAP_INIT) {
			/* explicit contract goes away for power swap */
			flags &= ~PD_FLAGS_EXPLICIT_CONTRACT;
			set_state(PD_STATE_SNK_SWAP_SNK_DISABLE);
		} else if (task_state == PD_STATE_SNK_REQUESTED) {
			/* explicit contract is now in place */
			flags |= PD_FLAGS_EXPLICIT_CONTRACT;
			set_state(PD_STATE_SNK_TRANSITION);
#endif
		}
		break;
	case PD_CTRL_SOFT_RESET:
		execute_soft_reset();
		/* We are done, acknowledge with an Accept packet */
		send_control(PD_CTRL_ACCEPT);
		break;
	case PD_CTRL_PR_SWAP:
#ifdef CONFIG_USB_PD_DUAL_ROLE
		if (pd_check_power_swap(port)) {
			send_control(PD_CTRL_ACCEPT);
			/*
			 * Clear flag for checking power role to avoid
			 * immediately requesting another swap.
			 */
			flags &= ~PD_FLAGS_CHECK_PR_ROLE;
			set_state(
				  DUAL_ROLE_IF_ELSE(
					PD_STATE_SNK_SWAP_SNK_DISABLE,
					PD_STATE_SRC_SWAP_SNK_DISABLE));
		} else {
			send_control(PD_CTRL_REJECT);
		}
#else
		send_control(PD_CTRL_REJECT);
#endif
		break;
	case PD_CTRL_DR_SWAP:
		if (pd_check_data_swap(port, data_role)) {
			/*
			 * Accept switch and perform data swap. Clear
			 * flag for checking data role to avoid
			 * immediately requesting another swap.
			 */
			flags &= ~PD_FLAGS_CHECK_DR_ROLE;
			if (send_control(PD_CTRL_ACCEPT) >= 0)
				pd_dr_swap();
		} else {
			send_control(PD_CTRL_REJECT);
		}
		break;
	case PD_CTRL_VCONN_SWAP:
#ifdef CONFIG_USBC_VCONN_SWAP
		if (task_state == PD_STATE_SRC_READY ||
		    task_state == PD_STATE_SNK_READY) {
			if (pd_check_vconn_swap()) {
				if (send_control(PD_CTRL_ACCEPT) > 0)
					set_state(PD_STATE_VCONN_SWAP_INIT);
			} else {
				send_control(PD_CTRL_REJECT);
			}
		}
#else
		send_control(PD_CTRL_REJECT);
#endif
		break;
	default:
		//CPRINTF("Unhandled ctrl message type %d\n", type);
		break;
	}
}

int USB_PD::check_requested_voltage(uint32_t rdo) {
	int max_ma = rdo & 0x3FF;
	int op_ma = (rdo >> 10) & 0x3FF;
	int idx = RDO_POS(rdo);
	uint32_t pdo;
	uint32_t pdo_ma;

	/* Board specific check for this request */
	if (pd_board_check_request(rdo))
		return EC_ERROR_INVAL;

	/* check current ... */
	pdo = pd_src_pdo[idx - 1];
	pdo_ma = (pdo & 0x3ff);
	if (op_ma > pdo_ma)
		return EC_ERROR_INVAL; /* too much op current */
	if (max_ma > pdo_ma && !(rdo & RDO_CAP_MISMATCH))
		return EC_ERROR_INVAL; /* too much max current */

	/* Accept the requested voltage */
	return EC_SUCCESS;
}

int USB_PD::send_control(int type) {
	int bit_len;
	uint16_t header = PD_HEADER(type, power_role,
			data_role, msg_id, 0);

	bit_len = pd_transmit(TCPC_TX_SOP, header, NULL);

	return bit_len;
}

void USB_PD::update_pdo_flags(uint32_t pdo) {
#ifdef CONFIG_CHARGE_MANAGER
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
	int charge_whitelisted =
		(power_role == PD_ROLE_SINK &&
		 pd_charge_from_device(pd_get_identity_vid(),
				       pd_get_identity_pid()));
#else
	const int charge_whitelisted = 0;
#endif
#endif

	/* can only parse PDO flags if type is fixed */
	if ((pdo & PDO_TYPE_MASK) != PDO_TYPE_FIXED)
		return;

#ifdef CONFIG_USB_PD_DUAL_ROLE
	if (pdo & PDO_FIXED_DUAL_ROLE)
		flags |= PD_FLAGS_PARTNER_DR_POWER;
	else
		flags &= ~PD_FLAGS_PARTNER_DR_POWER;

	if (pdo & PDO_FIXED_EXTERNAL)
		flags |= PD_FLAGS_PARTNER_EXTPOWER;
	else
		flags &= ~PD_FLAGS_PARTNER_EXTPOWER;

	if (pdo & PDO_FIXED_COMM_CAP)
		flags |= PD_FLAGS_PARTNER_USB_COMM;
	else
		flags &= ~PD_FLAGS_PARTNER_USB_COMM;
#endif

	if (pdo & PDO_FIXED_DATA_SWAP)
		flags |= PD_FLAGS_PARTNER_DR_DATA;
	else
		flags &= ~PD_FLAGS_PARTNER_DR_DATA;

#ifdef CONFIG_CHARGE_MANAGER
	/*
	 * Treat device as a dedicated charger (meaning we should charge
	 * from it) if it does not support power swap, or if it is externally
	 * powered, or if we are a sink and the device identity matches a
	 * charging white-list.
	 */
	if (!(flags & PD_FLAGS_PARTNER_DR_POWER) ||
	    (flags & PD_FLAGS_PARTNER_EXTPOWER) ||
	    charge_whitelisted)
		charge_manager_update_dualrole(CAP_DEDICATED);
	else
		charge_manager_update_dualrole(CAP_DUALROLE);
#endif
}

void USB_PD::handle_vdm_request(int cnt, uint32_t *payload) {
	int rlen = 0;
	uint32_t *rdata;

	if (vdm_state == VDM_STATE_BUSY) {
		/* If UFP responded busy retry after timeout */
		if (PD_VDO_CMDT(payload[0]) == CMDT_RSP_BUSY) {
			vdm_timeout.val = get_time().val +
				PD_T_VDM_BUSY;
			vdm_state = VDM_STATE_WAIT_RSP_BUSY;
			vdo_retry = (payload[0] & ~VDO_CMDT_MASK) |
				CMDT_INIT;
			return;
		} else {
			vdm_state = VDM_STATE_DONE;
		}
	}

	if (PD_VDO_SVDM(payload[0]))
		rlen = pd_svdm(cnt, payload, &rdata);
	else
		rlen = pd_custom_vdm(cnt, payload, &rdata);

	if (rlen > 0) {
		queue_vdm(rdata, &rdata[1], rlen - 1);
		return;
	}
}

int USB_PD::send_source_cap(void) {
	int bit_len;
#if defined(CONFIG_USB_PD_DYNAMIC_SRC_CAP) || \
		defined(CONFIG_USB_PD_MAX_SINGLE_SOURCE_CURRENT)
	const uint32_t *src_pdo;
	const int src_pdo_cnt = charge_manager_get_source_pdo(&src_pdo);
#else
	const uint32_t *src_pdo = pd_src_pdo;
	const int src_pdo_cnt = pd_src_pdo_cnt;
#endif
	uint16_t header;

	if (src_pdo_cnt == 0)
		/* No source capabilities defined, sink only */
		header = PD_HEADER(PD_CTRL_REJECT, power_role,
			data_role, msg_id, 0);
	else
		header = PD_HEADER(PD_DATA_SOURCE_CAP, power_role,
			data_role, msg_id, src_pdo_cnt);

	bit_len = pd_transmit(TCPC_TX_SOP, header, src_pdo);

	return bit_len;
}

void USB_PD::execute_soft_reset(void) {
	msg_id = 0;
	set_state(DUAL_ROLE_IF_ELSE(PD_STATE_SNK_DISCOVERY,
						PD_STATE_SRC_DISCOVERY));
}

void USB_PD::pd_dr_swap(void) {
	set_data_role(!data_role);
	flags |= PD_FLAGS_DATA_SWAPPED;
}

void USB_PD::set_data_role(int role) {
	data_role = role;
	pd_execute_data_swap(port, role);

#ifdef CONFIG_USBC_SS_MUX
#ifdef CONFIG_USBC_SS_MUX_DFP_ONLY
	/*
	 * Need to connect SS mux for if new data role is DFP.
	 * If new data role is UFP, then disconnect the SS mux.
	 */
	if (role == PD_ROLE_DFP)
		usb_mux_set(port, TYPEC_MUX_USB, USB_SWITCH_CONNECT,
			    pd[port].polarity);
	else
		usb_mux_set(port, TYPEC_MUX_NONE, USB_SWITCH_DISCONNECT,
			    pd[port].polarity);
#else
	usb_mux_set(port, TYPEC_MUX_USB, USB_SWITCH_CONNECT,
		    pd[port].polarity);
#endif
#endif
	update_roles();
}

void USB_PD::update_roles(void) {
	/* Notify TCPC of role update */
	m_tcpm->set_msg_header(power_role, data_role);
}

#ifdef CONFIG_USB_PD_ALT_MODE

#ifdef CONFIG_USB_PD_ALT_MODE_DFP

static struct pd_policy pe[CONFIG_USB_PD_PORT_COUNT];

void pd_dfp_pe_init(int port)
{
	memset(&pe[port], 0, sizeof(struct pd_policy));
}

static void dfp_consume_identity(int port, int cnt, uint32_t *payload)
{
	int ptype = PD_IDH_PTYPE(payload[VDO_I(IDH)]);
	size_t identity_size = MIN(sizeof(pe[port].identity),
				   (cnt - 1) * sizeof(uint32_t));
	pd_dfp_pe_init(port);
	memcpy(&pe[port].identity, payload + 1, identity_size);
	switch (ptype) {
	case IDH_PTYPE_AMA:
		/* TODO(tbroch) do I disable VBUS here if power contract
		 * requested it
		 */
		if (!PD_VDO_AMA_VBUS_REQ(payload[VDO_I(AMA)]))
			pd_power_supply_reset(port);
		break;
		/* TODO(crosbug.com/p/30645) provide vconn support here */
	default:
		break;
	}
}

static int dfp_discover_svids(int port, uint32_t *payload)
{
	payload[0] = VDO(USB_SID_PD, 1, CMD_DISCOVER_SVID);
	return 1;
}

static void dfp_consume_svids(int port, uint32_t *payload)
{
	int i;
	uint32_t *ptr = payload + 1;
	uint16_t svid0, svid1;

	for (i = pe[port].svid_cnt; i < pe[port].svid_cnt + 12; i += 2) {
		if (i == SVID_DISCOVERY_MAX) {
			CPRINTF("ERR:SVIDCNT\n");
			break;
		}

		svid0 = PD_VDO_SVID_SVID0(*ptr);
		if (!svid0)
			break;
		pe[port].svids[i].svid = svid0;
		pe[port].svid_cnt++;

		svid1 = PD_VDO_SVID_SVID1(*ptr);
		if (!svid1)
			break;
		pe[port].svids[i + 1].svid = svid1;
		pe[port].svid_cnt++;
		ptr++;
	}
	/* TODO(tbroch) need to re-issue discover svids if > 12 */
	if (i && ((i % 12) == 0))
		CPRINTF("ERR:SVID+12\n");
}

static int dfp_discover_modes(int port, uint32_t *payload)
{
	uint16_t svid = pe[port].svids[pe[port].svid_idx].svid;
	if (pe[port].svid_idx >= pe[port].svid_cnt)
		return 0;
	payload[0] = VDO(svid, 1, CMD_DISCOVER_MODES);
	return 1;
}

static void dfp_consume_modes(int port, int cnt, uint32_t *payload)
{
	int idx = pe[port].svid_idx;
	pe[port].svids[idx].mode_cnt = cnt - 1;
	if (pe[port].svids[idx].mode_cnt < 0) {
		CPRINTF("ERR:NOMODE\n");
	} else {
		memcpy(pe[port].svids[pe[port].svid_idx].mode_vdo, &payload[1],
		       sizeof(uint32_t) * pe[port].svids[idx].mode_cnt);
	}

	pe[port].svid_idx++;
}

static int get_mode_idx(int port, uint16_t svid)
{
	int i;

	for (i = 0; i < PD_AMODE_COUNT; i++) {
		if (pe[port].amodes[i].fx->svid == svid)
			return i;
	}
	return -1;
}

static struct svdm_amode_data *get_modep(int port, uint16_t svid)
{
	int idx = get_mode_idx(port, svid);

	return (idx == -1) ? NULL : &pe[port].amodes[idx];
}

int pd_alt_mode(int port, uint16_t svid)
{
	struct svdm_amode_data *modep = get_modep(port, svid);

	return (modep) ? modep->opos : -1;
}

int allocate_mode(int port, uint16_t svid)
{
	int i, j;
	struct svdm_amode_data *modep;
	int mode_idx = get_mode_idx(port, svid);

	if (mode_idx != -1)
		return mode_idx;

	/* There's no space to enter another mode */
	if (pe[port].amode_idx == PD_AMODE_COUNT) {
		CPRINTF("ERR:NO AMODE SPACE\n");
		return -1;
	}

	/* Allocate ...  if SVID == 0 enter default supported policy */
	for (i = 0; i < supported_modes_cnt; i++) {
		if (!&supported_modes[i])
			continue;

		for (j = 0; j < pe[port].svid_cnt; j++) {
			struct svdm_svid_data *svidp = &pe[port].svids[j];
			if ((svidp->svid != supported_modes[i].svid) ||
			    (svid && (svidp->svid != svid)))
				continue;

			modep = &pe[port].amodes[pe[port].amode_idx];
			modep->fx = &supported_modes[i];
			modep->data = &pe[port].svids[j];
			pe[port].amode_idx++;
			return pe[port].amode_idx - 1;
		}
	}
	return -1;
}

/*
 * Enter default mode ( payload[0] == 0 ) or attempt to enter mode via svid &
 * opos
*/
uint32_t pd_dfp_enter_mode(int port, uint16_t svid, int opos)
{
	int mode_idx = allocate_mode(port, svid);
	struct svdm_amode_data *modep;
	uint32_t mode_caps;

	if (mode_idx == -1)
		return 0;
	modep = &pe[port].amodes[mode_idx];

	if (!opos) {
		/* choose the lowest as default */
		modep->opos = 1;
	} else if (opos <= modep->data->mode_cnt) {
		modep->opos = opos;
	} else {
		CPRINTF("opos error\n");
		return 0;
	}

	mode_caps = modep->data->mode_vdo[modep->opos - 1];
	if (modep->fx->enter(port, mode_caps) == -1)
		return 0;

	/* SVDM to send to UFP for mode entry */
	return VDO(modep->fx->svid, 1, CMD_ENTER_MODE | VDO_OPOS(modep->opos));
}

static int validate_mode_request(struct svdm_amode_data *modep,
				 uint16_t svid, int opos)
{
	if (!modep->fx)
		return 0;

	if (svid != modep->fx->svid) {
		CPRINTF("ERR:svid r:0x%04x != c:0x%04x\n",
			svid, modep->fx->svid);
		return 0;
	}

	if (opos != modep->opos) {
		CPRINTF("ERR:opos r:%d != c:%d\n",
			opos, modep->opos);
		return 0;
	}

	return 1;
}

static void dfp_consume_attention(int port, uint32_t *payload)
{
	uint16_t svid = PD_VDO_VID(payload[0]);
	int opos = PD_VDO_OPOS(payload[0]);
	struct svdm_amode_data *modep = get_modep(port, svid);

	if (!modep || !validate_mode_request(modep, svid, opos))
		return;

	if (modep->fx->attention)
		modep->fx->attention(port, payload);
}

/*
 * This algorithm defaults to choosing higher pin config over lower ones.  Pin
 * configs are organized in pairs with the following breakdown.
 *
 *  NAME | SIGNALING | OUTPUT TYPE | MULTI-FUNCTION | PIN CONFIG
 * -------------------------------------------------------------
 *  A    |  USB G2   |  ?          | no             | 00_0001
 *  B    |  USB G2   |  ?          | yes            | 00_0010
 *  C    |  DP       |  CONVERTED  | no             | 00_0100
 *  D    |  PD       |  CONVERTED  | yes            | 00_1000
 *  E    |  DP       |  DP         | no             | 01_0000
 *  F    |  PD       |  DP         | yes            | 10_0000
 *
 * if UFP has NOT asserted multi-function preferred code masks away B/D/F
 * leaving only A/C/E.  For single-output dongles that should leave only one
 * possible pin config depending on whether its a converter DP->(VGA|HDMI) or DP
 * output.  If someone creates a multi-output dongle presumably they would need
 * to either offer different mode capabilities depending upon connection type or
 * the DFP would need additional system policy to expose those options.
 */
int pd_dfp_dp_get_pin_mode(int port, uint32_t status)
{
	struct svdm_amode_data *modep = get_modep(port, USB_SID_DISPLAYPORT);
	uint32_t mode_caps;
	uint32_t pin_caps;
	if (!modep)
		return 0;

	mode_caps = modep->data->mode_vdo[modep->opos - 1];

	/* TODO(crosbug.com/p/39656) revisit with DFP that can be a sink */
	pin_caps = PD_DP_PIN_CAPS(mode_caps);

	/* if don't want multi-function then ignore those pin configs */
	if (!PD_VDO_DPSTS_MF_PREF(status))
		pin_caps &= ~MODE_DP_PIN_MF_MASK;

	/* TODO(crosbug.com/p/39656) revisit if DFP drives USB Gen 2 signals */
	pin_caps &= ~MODE_DP_PIN_BR2_MASK;

	/* get_next_bit returns undefined for zero */
	if (!pin_caps)
		return 0;

	return 1 << get_next_bit(&pin_caps);
}

int pd_dfp_exit_mode(int port, uint16_t svid, int opos)
{
	struct svdm_amode_data *modep;
	int idx;

	/*
	 * Empty svid signals we should reset DFP VDM state by exiting all
	 * entered modes then clearing state.  This occurs when we've
	 * disconnected or for hard reset.
	 */
	if (!svid) {
		for (idx = 0; idx < PD_AMODE_COUNT; idx++)
			if (pe[port].amodes[idx].fx)
				pe[port].amodes[idx].fx->exit(port);

		pd_dfp_pe_init(port);
		return 0;
	}

	/*
	 * TODO(crosbug.com/p/33946) : below needs revisited to allow multiple
	 * mode exit.  Additionally it should honor OPOS == 7 as DFP's request
	 * to exit all modes.  We currently don't have any UFPs that support
	 * multiple modes on one SVID.
	 */
	modep = get_modep(port, svid);
	if (!modep || !validate_mode_request(modep, svid, opos))
		return 0;

	/* call DFPs exit function */
	modep->fx->exit(port);
	/* exit the mode */
	modep->opos = 0;
	return 1;
}

uint16_t pd_get_identity_vid(int port)
{
	return PD_IDH_VID(pe[port].identity[0]);
}

uint16_t pd_get_identity_pid(int port)
{
	return PD_PRODUCT_PID(pe[port].identity[2]);
}

#ifdef CONFIG_CMD_USB_PD_PE
static void dump_pe(int port)
{
	const char * const idh_ptype_names[]  = {
		"UNDEF", "Hub", "Periph", "PCable", "ACable", "AMA",
		"RSV6", "RSV7"};

	int i, j, idh_ptype;
	struct svdm_amode_data *modep;
	uint32_t mode_caps;

	if (pe[port].identity[0] == 0) {
		ccprintf("No identity discovered yet.\n");
		return;
	}
	idh_ptype = PD_IDH_PTYPE(pe[port].identity[0]);
	ccprintf("IDENT:\n");
	ccprintf("\t[ID Header] %08x :: %s, VID:%04x\n", pe[port].identity[0],
		 idh_ptype_names[idh_ptype], pd_get_identity_vid(port));
	ccprintf("\t[Cert Stat] %08x\n", pe[port].identity[1]);
	for (i = 2; i < ARRAY_SIZE(pe[port].identity); i++) {
		ccprintf("\t");
		if (pe[port].identity[i])
			ccprintf("[%d] %08x ", i, pe[port].identity[i]);
	}
	ccprintf("\n");

	if (pe[port].svid_cnt < 1) {
		ccprintf("No SVIDS discovered yet.\n");
		return;
	}

	for (i = 0; i < pe[port].svid_cnt; i++) {
		ccprintf("SVID[%d]: %04x MODES:", i, pe[port].svids[i].svid);
		for (j = 0; j < pe[port].svids[j].mode_cnt; j++)
			ccprintf(" [%d] %08x", j + 1,
				 pe[port].svids[i].mode_vdo[j]);
		ccprintf("\n");
		modep = get_modep(port, pe[port].svids[i].svid);
		if (modep) {
			mode_caps = modep->data->mode_vdo[modep->opos - 1];
			ccprintf("MODE[%d]: svid:%04x caps:%08x\n", modep->opos,
				 modep->fx->svid, mode_caps);
		}
	}
}

static int command_pe(int argc, char **argv)
{
	int port;
	char *e;
	if (argc < 3)
		return EC_ERROR_PARAM_COUNT;
	/* command: pe <port> <subcmd> <args> */
	port = strtoi(argv[1], &e, 10);
	if (*e || port >= CONFIG_USB_PD_PORT_COUNT)
		return EC_ERROR_PARAM2;
	if (!strncasecmp(argv[2], "dump", 4))
		dump_pe(port);

	return EC_SUCCESS;
}

#endif /* CONFIG_CMD_USB_PD_PE */

#endif /* CONFIG_USB_PD_ALT_MODE_DFP */

int pd_svdm(int cnt, uint32_t *payload, uint32_t **rpayload)
{
	int cmd = PD_VDO_CMD(payload[0]);
	int cmd_type = PD_VDO_CMDT(payload[0]);
	int (*func)(uint32_t *payload) = NULL;

	int rsize = 1; /* VDM header at a minimum */

	payload[0] &= ~VDO_CMDT_MASK;
	*rpayload = payload;

	if (cmd_type == CMDT_INIT) {
		switch (cmd) {
		case CMD_DISCOVER_IDENT:
			func = svdm_rsp.identity;
			break;
		case CMD_DISCOVER_SVID:
			func = svdm_rsp.svids;
			break;
		case CMD_DISCOVER_MODES:
			func = svdm_rsp.modes;
			break;
		case CMD_ENTER_MODE:
			func = svdm_rsp.enter_mode;
			break;
		case CMD_DP_STATUS:
			func = svdm_rsp.amode->status;
			break;
		case CMD_DP_CONFIG:
			func = svdm_rsp.amode->config;
			break;
		case CMD_EXIT_MODE:
			func = svdm_rsp.exit_mode;
			break;
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
		case CMD_ATTENTION:
			/*
			 * attention is only SVDM with no response
			 * (just goodCRC) return zero here.
			 */
			dfp_consume_attention(payload);
			return 0;
#endif
		default:
			rsize = 0;
		}
		if (func)
			rsize = func(payload);
		else /* not supported : NACK it */
			rsize = 0;
		if (rsize >= 1)
			payload[0] |= VDO_CMDT(CMDT_RSP_ACK);
		else if (!rsize) {
			payload[0] |= VDO_CMDT(CMDT_RSP_NAK);
			rsize = 1;
		} else {
			payload[0] |= VDO_CMDT(CMDT_RSP_BUSY);
			rsize = 1;
		}
	} else if (cmd_type == CMDT_RSP_ACK) {
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
		struct svdm_amode_data *modep;

		modep = get_modep(PD_VDO_VID(payload[0]));
#endif
		switch (cmd) {
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
		case CMD_DISCOVER_IDENT:
			dfp_consume_identity(cnt, payload);
			rsize = dfp_discover_svids(payload);
			break;
		case CMD_DISCOVER_SVID:
			dfp_consume_svids(payload);
			rsize = dfp_discover_modes(payload);
			break;
		case CMD_DISCOVER_MODES:
			dfp_consume_modes(cnt, payload);
			rsize = dfp_discover_modes(payload);
			/* enter the default mode for DFP */
			if (!rsize) {
				payload[0] = pd_dfp_enter_mode(0, 0);
				if (payload[0])
					rsize = 1;
			}
			break;
		case CMD_ENTER_MODE:
			if (!modep) {
				rsize = 0;
			} else {
				if (!modep->opos)
					pd_dfp_enter_mode(0, 0);

				if (modep->opos) {
					rsize = modep->fx->status(payload);
					payload[0] |= PD_VDO_OPOS(modep->opos);
				}
			}
			break;
		case CMD_DP_STATUS:
			/* DP status response & UFP's DP attention have same
			   payload */
			dfp_consume_attention(payload);
			if (modep && modep->opos)
				rsize = modep->fx->config(payload);
			else
				rsize = 0;
			break;
		case CMD_DP_CONFIG:
			if (modep && modep->opos && modep->fx->post_config)
				modep->fx->post_config();
			/* no response after DFPs ack */
			rsize = 0;
			break;
		case CMD_EXIT_MODE:
			/* no response after DFPs ack */
			rsize = 0;
			break;
#endif
		case CMD_ATTENTION:
			/* no response after DFPs ack */
			rsize = 0;
			break;
		default:
			rsize = 0;
		}

		payload[0] |= VDO_CMDT(CMDT_INIT);
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
	} else if (cmd_type == CMDT_RSP_BUSY) {
		switch (cmd) {
		case CMD_DISCOVER_IDENT:
		case CMD_DISCOVER_SVID:
		case CMD_DISCOVER_MODES:
			/* resend if its discovery */
			rsize = 1;
			break;
		case CMD_ENTER_MODE:
			/* Error */
			rsize = 0;
			break;
		case CMD_EXIT_MODE:
			rsize = 0;
			break;
		default:
			rsize = 0;
		}
	} else if (cmd_type == CMDT_RSP_NAK) {
		/* nothing to do */
		rsize = 0;
#endif /* CONFIG_USB_PD_ALT_MODE_DFP */
	} else {
		/* do not answer */
		rsize = 0;
	}
	return rsize;
}

#else

int USB_PD::pd_svdm(int cnt, uint32_t *payload, uint32_t **rpayload) {
	return 0;
}

#endif /* CONFIG_USB_PD_ALT_MODE */

int USB_PD::pd_custom_vdm(int cnt, uint32_t *payload,
		  uint32_t **rpayload) {
	return 0;
}

void USB_PD::queue_vdm(uint32_t *header, const uint32_t *data,
			     int data_cnt)
{
	vdo_count = data_cnt + 1;
	vdo_data[0] = header[0];
	memcpy(&vdo_data[1], data, sizeof(uint32_t) * data_cnt);
	/* Set ready, pd task will actually send */
	vdm_state = VDM_STATE_READY;
}

void USB_PD::request_data_swap(void) {
	if (DUAL_ROLE_IF_ELSE(
				task_state == PD_STATE_SNK_READY,
				task_state == PD_STATE_SRC_READY))
		set_state(PD_STATE_DR_SWAP);
	//task_wake(PD_PORT_TO_TASK_ID(port));
}

void USB_PD::send_vdm(uint32_t vid, int cmd, const uint32_t *data,
		 int count)
{
	if (count > VDO_MAX_SIZE - 1) {
		return;
	}

	/* set VDM header with VID & CMD */
	vdo_data[0] = VDO(vid, ((vid & USB_SID_PD) == USB_SID_PD) ?
				   1 : (PD_VDO_CMD(cmd) <= CMD_ATTENTION), cmd);
	queue_vdm(vdo_data, data, count);
}

int USB_PD::snk_debug_acc_toggle(void) {
#if defined(CONFIG_CASE_CLOSED_DEBUG) || \
defined(CONFIG_CASE_CLOSED_DEBUG_EXTERNAL)
#ifdef CONFIG_USB_PD_QUIRK_SLOW_CC_STATUS
	static int possible_debug_acc[CONFIG_USB_PD_PORT_COUNT];
	int vbus = pd_is_vbus_present(port);
	int result;

	/* reset debouncing of Rd/Rd debug accessory presence */
	if ((pd[port].last_state != PD_STATE_SNK_DISCONNECTED) || !vbus)
		possible_debug_acc[port] = 0;
	/* returns if it was possibly present in the previous iteration */
	result = possible_debug_acc[port];
	possible_debug_acc[port] = vbus;
	return result;
#else /* !CONFIG_USB_PD_QUIRK_SLOW_CC_STATUS */
	/*
	 * when we are in SNK_DISCONNECTED and we see VBUS appearing
	 * (without having seen Rp before), that might be a powered debug
	 * accessory, let's toggle to source to try to detect it.
	 */
	return pd_is_vbus_present(port);
#endif /* !CONFIG_USB_PD_QUIRK_SLOW_CC_STATUS */
#else
	/* Debug accessories not supported, never toggle */
	return 0;
#endif
}

int USB_PD::cc_is_rp(int cc) {
	return (cc == TYPEC_CC_VOLT_SNK_DEF) || (cc == TYPEC_CC_VOLT_SNK_1_5) ||
	       (cc == TYPEC_CC_VOLT_SNK_3_0);
}

/**
 * Returns the polarity of a Sink.
 */
int USB_PD::get_snk_polarity(int cc1, int cc2) {
	/* the following assumes:
	 * TYPEC_CC_VOLT_SNK_3_0 > TYPEC_CC_VOLT_SNK_1_5
	 * TYPEC_CC_VOLT_SNK_1_5 > TYPEC_CC_VOLT_SNK_DEF
	 * TYPEC_CC_VOLT_SNK_DEF > TYPEC_CC_VOLT_OPEN
	 */
	return (cc2 > cc1);
}

#ifdef CONFIG_USB_PD_DUAL_ROLE
void USB_PD::store_src_cap(int cnt, uint32_t *src_caps) {
	int i;

	pd_src_cap_cnt = cnt;
	for (i = 0; i < cnt; i++)
		pd_src_caps[i] = *src_caps++;
}

/*
 * Request desired charge voltage from source.
 * Returns EC_SUCCESS on success or non-zero on failure.
 */
int USB_PD::send_request_msg(int always_send_request) {
	uint32_t rdo, curr_limit, supply_voltage;
	int res;

#ifdef CONFIG_CHARGE_MANAGER
	int charging = (charge_manager_get_active_charge_port() == port);
#else
	const int charging = 1;
#endif

#ifdef CONFIG_USB_PD_CHECK_MAX_REQUEST_ALLOWED
	int max_request_allowed = pd_is_max_request_allowed();
#else
	const int max_request_allowed = 1;
#endif

	/* Clear new power request */
	new_power_request = 0;

	/* Build and send request RDO */
	/*
	 * If this port is not actively charging or we are not allowed to
	 * request the max voltage, then select vSafe5V
	 */
	res = build_request(pd_src_cap_cnt, pd_src_caps,
			       &rdo, &curr_limit, &supply_voltage,
			       charging && max_request_allowed ?
					PD_REQUEST_MAX : PD_REQUEST_VSAFE5V);

	if (res != EC_SUCCESS)
		/*
		 * If fail to choose voltage, do nothing, let source re-send
		 * source cap
		 */
		return -1;

	/* Don't re-request the same voltage */
	if (!always_send_request && prev_request_mv == supply_voltage)
		return EC_SUCCESS;

	this->curr_limit = curr_limit;
	this->supply_voltage = supply_voltage;
	this->prev_request_mv = supply_voltage;
	res = send_request(rdo);
	if (res < 0)
		return res;
	set_state(PD_STATE_SNK_REQUESTED);
	return EC_SUCCESS;
}
#endif

int USB_PD::build_request(int cnt, uint32_t *src_caps, uint32_t *rdo,
		     uint32_t *ma, uint32_t *mv, enum pd_request_type req_type)
{
	int pdo_index, flags = 0;
	uint32_t uw;

	if (req_type == PD_REQUEST_VSAFE5V)
		/* src cap 0 should be vSafe5V */
		pdo_index = 0;
	else
		/* find pdo index for max voltage we can request */
		pdo_index = find_pdo_index(cnt, src_caps, max_request_mv);

	/* If could not find desired pdo_index, then return error */
	if (pdo_index == -1)
		return -EC_ERROR_UNKNOWN;

	extract_pdo_power(src_caps[pdo_index], ma, mv);

	uw = *ma * *mv;
	/* Mismatch bit set if less power offered than the operating power */
	if (uw < (1000 * PD_OPERATING_POWER_MW))
		flags |= RDO_CAP_MISMATCH;

	if ((src_caps[pdo_index] & PDO_TYPE_MASK) == PDO_TYPE_BATTERY) {
		uint32_t mw = uw / 1000;
		*rdo = RDO_BATT(pdo_index + 1, mw, mw, flags);
	} else {
		*rdo = RDO_FIXED(pdo_index + 1, *ma, *ma, flags);
	}
	return EC_SUCCESS;
}

void USB_PD::process_source_cap(int cnt, uint32_t *src_caps) {
#ifdef CONFIG_CHARGE_MANAGER
	uint32_t ma, mv;
	int pdo_index;
	/* Get max power info that we could request */
	pdo_index = pd_find_pdo_index(cnt, src_caps, PD_MAX_VOLTAGE_MV);
	if (pdo_index < 0)
		pdo_index = 0;
	pd_extract_pdo_power(src_caps[pdo_index], &ma, &mv);

	/* Set max. limit, but apply 500mA ceiling */
	charge_manager_set_ceil(port, CEIL_REQUESTOR_PD, PD_MIN_MA);
	pd_set_input_current_limit(port, ma, mv);
#endif
}

#ifdef CONFIG_USB_PD_DUAL_ROLE
void USB_PD::send_sink_cap(void) {
	int bit_len;
	uint16_t header = PD_HEADER(PD_DATA_SINK_CAP, power_role,
			data_role, msg_id, pd_snk_pdo_cnt);

	bit_len = pd_transmit(TCPC_TX_SOP, header, pd_snk_pdo);
}

int USB_PD::send_request(uint32_t rdo) {
	int bit_len;
	uint16_t header = PD_HEADER(PD_DATA_REQUEST, power_role,
			data_role, msg_id, 1);

	bit_len = pd_transmit(TCPC_TX_SOP, header, &rdo);

	return bit_len;
}
#endif /* CONFIG_USB_PD_DUAL_ROLE */

/**
 * Find PDO index that offers the most amount of power and stays within
 * max_mv voltage.
 *
 * @param cnt  the number of Power Data Objects.
 * @param src_caps Power Data Objects representing the source capabilities.
 * @param max_mv maximum voltage (or -1 if no limit)
 * @return index of PDO within source cap packet
 */
int USB_PD::find_pdo_index(int cnt, uint32_t *src_caps, int max_mv) {
	uint32_t i, uw, max_uw = 0, mv, ma;
	int ret = -1;
#ifdef PD_PREFER_LOW_VOLTAGE
	uint32_t cur_mv = 0;
#endif
	/* max voltage is always limited by this boards max request */
	max_mv = MIN(max_mv, PD_MAX_VOLTAGE_MV);

	/* Get max power that is under our max voltage input */
	for (i = 0; i < cnt; i++) {
		mv = ((src_caps[i] >> 10) & 0x3FF) * 50;
		/* Skip any voltage not supported by this board */
		if (!pd_is_valid_input_voltage(mv))
			continue;

		if ((src_caps[i] & PDO_TYPE_MASK) == PDO_TYPE_BATTERY) {
			uw = 250000 * (src_caps[i] & 0x3FF);
		} else {
			ma = (src_caps[i] & 0x3FF) * 10;
			ma = MIN(ma, PD_MAX_CURRENT_MA);
			uw = ma * mv;
		}
#ifdef PD_PREFER_LOW_VOLTAGE
		if (mv > max_mv)
			continue;
		uw = MIN(uw, PD_MAX_POWER_MW * 1000);
		if ((uw > max_uw) || ((uw == max_uw) && mv < cur_mv)) {
			ret = i;
			max_uw = uw;
			cur_mv = mv;
		}
#else

		if ((uw > max_uw) && (mv <= max_mv)) {
			ret = i;
			max_uw = uw;
		}
#endif
	}
	return ret;
}

/**
 * Extract power information out of a Power Data Object (PDO)
 *
 * @pdo Power data object
 * @ma Current we can request from that PDO
 * @mv Voltage of the PDO
 */
void USB_PD::extract_pdo_power(uint32_t pdo, uint32_t *ma, uint32_t *mv) {
	uint32_t max_ma, uw;
	*mv = ((pdo >> 10) & 0x3FF) * 50;

	if ((pdo & PDO_TYPE_MASK) == PDO_TYPE_BATTERY) {
		uw = 250000 * (pdo & 0x3FF);
		max_ma = 1000 * MIN(1000 * uw, PD_MAX_POWER_MW) / *mv;
	} else {
		max_ma = 10 * (pdo & 0x3FF);
		max_ma = MIN(max_ma, PD_MAX_POWER_MW * 1000 / *mv);
	}

	*ma = MIN(max_ma, PD_MAX_CURRENT_MA);
}

int USB_PD::is_power_swapping(void) {
	/* return true if in the act of swapping power roles */
	return  task_state == PD_STATE_SNK_SWAP_SNK_DISABLE ||
		task_state == PD_STATE_SNK_SWAP_SRC_DISABLE ||
		task_state == PD_STATE_SNK_SWAP_STANDBY ||
		task_state == PD_STATE_SNK_SWAP_COMPLETE ||
		task_state == PD_STATE_SRC_SWAP_SNK_DISABLE ||
		task_state == PD_STATE_SRC_SWAP_SRC_DISABLE ||
		task_state == PD_STATE_SRC_SWAP_STANDBY;
}
