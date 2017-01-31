#define CONFIG_USB_PD_DUAL_ROLE
#define CONFIG_USB_PD_PULLUP TYPEC_RP_1A5

#include <FUSB302.h>
#include <USB_PD.h>
#include <util/atomic.h>

FUSB302 usbc;
const int pin_fusb302b_int = A0;
USB_TCPM *usb_tcpm = &usbc;
USB_PD  usb_pd(usb_tcpm, 0, PD_STATE_SNK_DISCONNECTED);
enum pd_states last_state = PD_STATE_SNK_DISCONNECTED;

void setup() {
  Serial.begin(115200);
  Serial.println("*****");
  usb_pd.init();
}

void loop() {
  usb_pd.pd_state_machine();
}

void pd_power_supply_reset(int port) {
  return;
}

int pd_is_vbus_present(int port) {
  return 1; //usbc.get_vbus_level();
}

void pd_set_input_current_limit(int port, uint32_t max_ma,
        uint32_t supply_voltage) {
  return;
}

int task_clear_event(int port) {
  usbc.clear_int_pin();
}

int task_wait_event_mask(uint32_t event_mask, uint64_t timeout_us) {
  // wait up to timeout_us for INT line to go low
  unsigned long time_start_us = micros();
  while(1) {
    int reg;
    usbc.tcpc_read(TCPC_REG_STATUS1, &reg);
    if (!(reg & TCPC_REG_STATUS1_RX_EMPTY)) {
      return PD_EVENT_RX;
    }
    /*
    if (digitalRead(pin_fusb302b_int) == LOW) {
      uint32_t reason;
      reason = usbc.get_interrupt_reason();

      //#define PD_EVENT_RX         (1<<2) // Incoming packet event
      //#define PD_EVENT_TX         (1<<3) // Outgoing packet event
      //#define PD_EVENT_CC         (1<<4) // CC line change event
      //#define PD_EVENT_TCPC_RESET (1<<5) // TCPC has reset

      if (reason & (1<<8)) { // INTERRUPTB, bit 0 is set (Rx packet)
        return PD_EVENT_RX;
      }
    }
    */
    if ((micros() - time_start_us) >= timeout_us) {
      break;
    }
    //delay(1);
  }
  return 0;
}

int task_wait_event(uint64_t timeout_us) {
  // wait up to timeout_us for INT line to go low
  unsigned long time_start_us = micros();
  while(1) {
    int reg;
    usbc.tcpc_read(TCPC_REG_STATUS1, &reg);
    if (!(reg & TCPC_REG_STATUS1_RX_EMPTY)) {
      return PD_EVENT_RX;
    }
    /*
    if (digitalRead(pin_fusb302b_int) == LOW) {
      uint32_t reason;
      reason = usbc.get_interrupt_reason();

      //#define PD_EVENT_RX         (1<<2) // Incoming packet event
      //#define PD_EVENT_TX         (1<<3) // Outgoing packet event
      //#define PD_EVENT_CC         (1<<4) // CC line change event
      //#define PD_EVENT_TCPC_RESET (1<<5) // TCPC has reset

      if (reason & (1<<8)) { // INTERRUPTB, bit 0 is set (Rx packet)
        return PD_EVENT_RX;
      }
    }
    */
    if ((micros() - time_start_us) >= timeout_us) {
      break;
    }
    //delay(1);
  }
  return 0;
}

timestamp_t get_time(void) {
  timestamp_t t;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    t.val = (1000*millis()) + (micros()%1000);
  }
  return t;
}

int pd_board_checks(void) {
  return EC_SUCCESS;
}

int pd_board_check_request(uint32_t rdo) {
  return EC_SUCCESS; // = 0
}

int pd_check_data_swap(int port, int data_role) {
  // Always refuse data swap
  return 0;
}

int pd_check_power_swap(int port) {
  // Always refuse
  return 0;
}

void pd_execute_data_swap(int port, int data_role) {
  return;
}

int pd_set_power_supply_ready(int port) {
  /* Disable charging */
  //gpio_set_level(port ? GPIO_USB_C1_CHARGE_EN_L :
  //          GPIO_USB_C0_CHARGE_EN_L, 1);
  /* Provide VBUS */
  //gpio_set_level(port ? GPIO_USB_C1_5V_EN :
  //          GPIO_USB_C0_5V_EN, 1);

  /* notify host of power info change */
  //pd_send_host_event(PD_EVENT_POWER_CHANGE);

  return EC_SUCCESS; /* we are ready */
}

void pd_transition_voltage(int idx) {
  return;
}

void pd_check_pr_role(int port, int pr_role, int flags) {
  /*
   * If partner is dual-role power and dualrole toggling is on, consider
   * if a power swap is necessary.
   */
  //if ((flags & PD_FLAGS_PARTNER_DR_POWER) &&
  //    pd_get_dual_role() == PD_DRP_TOGGLE_ON) {
    /*
     * If we are a sink and partner is not externally powered, then
     * swap to become a source. If we are source and partner is
     * externally powered, swap to become a sink.
     */
    //int partner_extpower = flags & PD_FLAGS_PARTNER_EXTPOWER;

    //if ((!partner_extpower && pr_role == PD_ROLE_SINK) ||
    //     (partner_extpower && pr_role == PD_ROLE_SOURCE))
    //  pd_request_power_swap(port);
  //}
}

void pd_check_dr_role(int port, int dr_role, int flags) {
  /* If UFP, try to switch to DFP */
  if ((flags & PD_FLAGS_PARTNER_DR_DATA) && dr_role == PD_ROLE_UFP)
    usb_pd.request_data_swap();
}

int pd_is_valid_input_voltage(uint32_t mv) {
  return 1;
}

