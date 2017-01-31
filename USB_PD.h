/*
  USB_PD.h - USB Power Delivery
  Created by Jason Cerundolo, 2017.
  Released under an MIT license. See LICENSE.md. 
*/

#ifndef USB_PD_H
#define USB_PD_H

#include <USB_TCPM.h>

#define CONFIG_USB_PD_DUAL_ROLE

/* Standard macros / definitions */
#ifndef MAX
#define MAX(a, b)                   \
    ({                      \
        __typeof__(a) temp_a = (a);     \
        __typeof__(b) temp_b = (b);     \
                            \
        temp_a > temp_b ? temp_a : temp_b;  \
    })
#endif
#ifndef MIN
#define MIN(a, b)                   \
    ({                      \
        __typeof__(a) temp_a = (a);     \
        __typeof__(b) temp_b = (b);     \
                            \
        temp_a < temp_b ? temp_a : temp_b;  \
    })
#endif
#ifndef NULL
#define NULL ((void *)0)
#endif

#define MSEC         1000ull
#define SECOND    1000000ull
#define MINUTE   60000000ull
#define HOUR   3600000000ull  /* Too big to fit in a signed int */

/* Control Message type */
enum pd_ctrl_msg_type {
    /* 0 Reserved */
    PD_CTRL_GOOD_CRC = 1,
    PD_CTRL_GOTO_MIN = 2,
    PD_CTRL_ACCEPT = 3,
    PD_CTRL_REJECT = 4,
    PD_CTRL_PING = 5,
    PD_CTRL_PS_RDY = 6,
    PD_CTRL_GET_SOURCE_CAP = 7,
    PD_CTRL_GET_SINK_CAP = 8,
    PD_CTRL_DR_SWAP = 9,
    PD_CTRL_PR_SWAP = 10,
    PD_CTRL_VCONN_SWAP = 11,
    PD_CTRL_WAIT = 12,
    PD_CTRL_SOFT_RESET = 13,
    /* 14-15 Reserved */
};

/* Data message type */
enum pd_data_msg_type {
    /* 0 Reserved */
    PD_DATA_SOURCE_CAP = 1,
    PD_DATA_REQUEST = 2,
    PD_DATA_BIST = 3,
    PD_DATA_SINK_CAP = 4,
    /* 5-14 Reserved */
    PD_DATA_VENDOR_DEF = 15,
};

/* Protocol revision */
#define PD_REV10 0
#define PD_REV20 1

/* Power role */
#define PD_ROLE_SINK   0
#define PD_ROLE_SOURCE 1
/* Data role */
#define PD_ROLE_UFP    0
#define PD_ROLE_DFP    1
/* Vconn role */
#define PD_ROLE_VCONN_OFF 0
#define PD_ROLE_VCONN_ON  1

/* Port role at startup */
#ifdef CONFIG_USB_PD_DUAL_ROLE
#define PD_ROLE_DEFAULT PD_ROLE_SINK
#else
#define PD_ROLE_DEFAULT PD_ROLE_SOURCE
#endif

#ifdef CONFIG_USB_PD_DUAL_ROLE
#define DUAL_ROLE_IF_ELSE(sink_clause, src_clause) \
    (this->power_role == PD_ROLE_SINK ? (sink_clause) : (src_clause))
#else
#define DUAL_ROLE_IF_ELSE(sink_clause, src_clause) (src_clause)
#endif

#define READY_RETURN_STATE DUAL_ROLE_IF_ELSE(PD_STATE_SNK_READY, \
                             PD_STATE_SRC_READY)

/* Type C supply voltage (mV) */
#define TYPE_C_VOLTAGE  5000 /* mV */

/* PD counter definitions */
#define PD_MESSAGE_ID_COUNT 7
#define PD_HARD_RESET_COUNT 2
#define PD_CAPS_COUNT 50
#define PD_SNK_CAP_RETRIES 3

/* build message header */
#define PD_HEADER(type, prole, drole, id, cnt) \
    ((type) | (PD_REV20 << 6) | \
     ((drole) << 5) | ((prole) << 8) | \
     ((id) << 9) | ((cnt) << 12))

#define PD_HEADER_CNT(header)  (((header) >> 12) & 7)
#define PD_HEADER_TYPE(header) ((header) & 0xF)
#define PD_HEADER_ID(header)   (((header) >> 9) & 7)

/* K-codes for special symbols */
#define PD_SYNC1 0x18
#define PD_SYNC2 0x11
#define PD_SYNC3 0x06
#define PD_RST1  0x07
#define PD_RST2  0x19
#define PD_EOP   0x0D

/* Request types for pd_build_request() */
enum pd_request_type {
    PD_REQUEST_VSAFE5V,
    PD_REQUEST_MAX,
};

/* Timers */

/* Time to wait for TCPC to complete transmit */
#define PD_T_TCPC_TX_TIMEOUT  (100*MSEC)

#define PD_T_SEND_SOURCE_CAP  (100*MSEC) /* between 100ms and 200ms */
#define PD_T_SINK_WAIT_CAP    (240*MSEC) /* between 210ms and 250ms */
#define PD_T_SINK_TRANSITION   (35*MSEC) /* between 20ms and 35ms */
#define PD_T_SOURCE_ACTIVITY   (45*MSEC) /* between 40ms and 50ms */
#define PD_T_SENDER_RESPONSE   (30*MSEC) /* between 24ms and 30ms */
#define PD_T_PS_TRANSITION    (500*MSEC) /* between 450ms and 550ms */
#define PD_T_PS_SOURCE_ON     (480*MSEC) /* between 390ms and 480ms */
#define PD_T_PS_SOURCE_OFF    (920*MSEC) /* between 750ms and 920ms */
#define PD_T_PS_HARD_RESET     (15*MSEC) /* between 10ms and 20ms */
#define PD_T_ERROR_RECOVERY    (25*MSEC) /* 25ms */
#define PD_T_CC_DEBOUNCE       (100*MSEC) /* between 100ms and 200ms */
/* DRP_SNK + DRP_SRC must be between 50ms and 100ms with 30%-70% duty cycle */
#define PD_T_DRP_SNK           (40*MSEC) /* toggle time for sink DRP */
#define PD_T_DRP_SRC           (30*MSEC) /* toggle time for source DRP */
#define PD_T_DEBOUNCE          (15*MSEC) /* between 10ms and 20ms */
#define PD_T_SINK_ADJ          (55*MSEC) /* between PD_T_DEBOUNCE and 60ms */
#define PD_T_SRC_RECOVER      (760*MSEC) /* between 660ms and 1000ms */
#define PD_T_SRC_RECOVER_MAX (1000*MSEC) /* 1000ms */
#define PD_T_SRC_TURN_ON      (275*MSEC) /* 275ms */
#define PD_T_SAFE_0V          (650*MSEC) /* 650ms */
#define PD_T_NO_RESPONSE     (5500*MSEC) /* between 4.5s and 5.5s */
#define PD_T_BIST_TRANSMIT     (50*MSEC) /* 50ms (used for task_wait arg) */
#define PD_T_BIST_RECEIVE      (60*MSEC) /* 60ms (max time to process bist) */
#define PD_T_VCONN_SOURCE_ON  (100*MSEC) /* 100ms */
#define PD_T_TRY_SRC          (125*MSEC) /* Max time for Try.SRC state */
#define PD_T_TRY_WAIT         (600*MSEC) /* Max time for TryWait.SNK state */

#define PD_FLAGS_PING_ENABLED      (1 << 0) /* SRC_READY pings enabled */
#define PD_FLAGS_PARTNER_DR_POWER  (1 << 1) /* port partner is dualrole power */
#define PD_FLAGS_PARTNER_DR_DATA   (1 << 2) /* port partner is dualrole data */
#define PD_FLAGS_DATA_SWAPPED      (1 << 3) /* data swap complete */
#define PD_FLAGS_SNK_CAP_RECVD     (1 << 4) /* sink capabilities received */
#define PD_FLAGS_EXPLICIT_CONTRACT (1 << 6) /* explicit pwr contract in place */
#define PD_FLAGS_VBUS_NEVER_LOW    (1 << 7) /* VBUS input has never been low */
#define PD_FLAGS_PREVIOUS_PD_CONN  (1 << 8) /* previously PD connected */
#define PD_FLAGS_CHECK_PR_ROLE     (1 << 9) /* check power role in READY */
#define PD_FLAGS_CHECK_DR_ROLE     (1 << 10)/* check data role in READY */
#define PD_FLAGS_PARTNER_EXTPOWER  (1 << 11)/* port partner has external pwr */
#define PD_FLAGS_VCONN_ON          (1 << 12)/* vconn is being sourced */
#define PD_FLAGS_TRY_SRC           (1 << 13)/* Try.SRC states are active */
#define PD_FLAGS_PARTNER_USB_COMM  (1 << 14)/* port partner is USB comms */
#define PD_FLAGS_UPDATE_SRC_CAPS   (1 << 15)/* send new source capabilities */
/* Flags to clear on a disconnect */
#define PD_FLAGS_RESET_ON_DISCONNECT_MASK (PD_FLAGS_PARTNER_DR_POWER | \
                       PD_FLAGS_PARTNER_DR_DATA | \
                       PD_FLAGS_DATA_SWAPPED | \
                       PD_FLAGS_SNK_CAP_RECVD | \
                       PD_FLAGS_EXPLICIT_CONTRACT | \
                       PD_FLAGS_PREVIOUS_PD_CONN | \
                       PD_FLAGS_CHECK_PR_ROLE | \
                       PD_FLAGS_CHECK_DR_ROLE | \
                       PD_FLAGS_PARTNER_EXTPOWER | \
                       PD_FLAGS_VCONN_ON | \
                       PD_FLAGS_TRY_SRC | \
                       PD_FLAGS_PARTNER_USB_COMM | \
                       PD_FLAGS_UPDATE_SRC_CAPS)

#define PDO_FIXED_FLAGS (PDO_FIXED_DUAL_ROLE | PDO_FIXED_DATA_SWAP |\
             PDO_FIXED_COMM_CAP)

#define RX_BUFFER_SIZE 1

enum pd_rx_errors {
    PD_RX_ERR_INVAL = -1,           /* Invalid packet */
    PD_RX_ERR_HARD_RESET = -2,      /* Got a Hard-Reset packet */
    PD_RX_ERR_CRC = -3,             /* CRC mismatch */
    PD_RX_ERR_ID = -4,              /* Invalid ID number */
    PD_RX_ERR_UNSUPPORTED_SOP = -5, /* Unsupported SOP */
    PD_RX_ERR_CABLE_RESET = -6      /* Got a Cable-Reset packet */
};

/* Events for USB PD task */
#define PD_EVENT_RX         (1<<2) /* Incoming packet event */
#define PD_EVENT_TX         (1<<3) /* Outgoing packet event */
#define PD_EVENT_CC         (1<<4) /* CC line change event */
#define PD_EVENT_TCPC_RESET (1<<5) /* TCPC has reset */

/* --- PD data message helpers --- */
#define PDO_MAX_OBJECTS   7
#define PDO_MODES (PDO_MAX_OBJECTS - 1)

/* PDO : Power Data Object */
/*
 * 1. The vSafe5V Fixed Supply Object shall always be the first object.
 * 2. The remaining Fixed Supply Objects,
 *    if present, shall be sent in voltage order; lowest to highest.
 * 3. The Battery Supply Objects,
 *    if present shall be sent in Minimum Voltage order; lowest to highest.
 * 4. The Variable Supply (non battery) Objects,
 *    if present, shall be sent in Minimum Voltage order; lowest to highest.
 */
#define PDO_TYPE_FIXED    (0l << 30)
#define PDO_TYPE_BATTERY  (1l << 30)
#define PDO_TYPE_VARIABLE (2l << 30)
#define PDO_TYPE_MASK     (3l << 30)

#define PDO_FIXED_DUAL_ROLE (1l << 29) /* Dual role device */
#define PDO_FIXED_SUSPEND   (1l << 28) /* USB Suspend supported */
#define PDO_FIXED_EXTERNAL  (1l << 27) /* Externally powered */
#define PDO_FIXED_COMM_CAP  (1l << 26) /* USB Communications Capable */
#define PDO_FIXED_DATA_SWAP (1l << 25) /* Data role swap command supported */
#define PDO_FIXED_PEAK_CURR () /* [21..20] Peak current */
#define PDO_FIXED_VOLT(mv)  (((mv)/50l) << 10) /* Voltage in 50mV units */
#define PDO_FIXED_CURR(ma)  (((ma)/10l) << 0)  /* Max current in 10mA units */

#define PDO_FIXED(mv, ma, flags) (PDO_FIXED_VOLT(mv) |\
                  PDO_FIXED_CURR(ma) | (flags))

#define PDO_VAR_MAX_VOLT(mv) ((((mv) / 50) & 0x3FFl) << 20)
#define PDO_VAR_MIN_VOLT(mv) ((((mv) / 50) & 0x3FFl) << 10)
#define PDO_VAR_OP_CURR(ma)  ((((ma) / 10) & 0x3FFl) << 0)

#define PDO_VAR(min_mv, max_mv, op_ma) \
                (PDO_VAR_MIN_VOLT(min_mv) | \
                 PDO_VAR_MAX_VOLT(max_mv) | \
                 PDO_VAR_OP_CURR(op_ma)   | \
                 PDO_TYPE_VARIABLE)

#define PDO_BATT_MAX_VOLT(mv) ((((mv) / 50) & 0x3FF) << 20)
#define PDO_BATT_MIN_VOLT(mv) ((((mv) / 50) & 0x3FF) << 10)
#define PDO_BATT_OP_POWER(mw) ((((mw) / 250) & 0x3FF) << 0)

#define PDO_BATT(min_mv, max_mv, op_mw) \
                (PDO_BATT_MIN_VOLT(min_mv) | \
                 PDO_BATT_MAX_VOLT(max_mv) | \
                 PDO_BATT_OP_POWER(op_mw) | \
                 PDO_TYPE_BATTERY)

/* RDO : Request Data Object */
#define RDO_OBJ_POS(n)             (((n) & 0x7l) << 28)
#define RDO_POS(rdo)               (((rdo) >> 28l) & 0x7)
#define RDO_GIVE_BACK              (1l << 27)
#define RDO_CAP_MISMATCH           (1l << 26)
#define RDO_COMM_CAP               (1l << 25)
#define RDO_NO_SUSPEND             (1l << 24)
#define RDO_FIXED_VAR_OP_CURR(ma)  ((((ma) / 10) & 0x3FFl) << 10)
#define RDO_FIXED_VAR_MAX_CURR(ma) ((((ma) / 10) & 0x3FFl) << 0)

#define RDO_BATT_OP_POWER(mw)      ((((mw) / 250) & 0x3FFl) << 10)
#define RDO_BATT_MAX_POWER(mw)     ((((mw) / 250) & 0x3FFl) << 10)

#define RDO_FIXED(n, op_ma, max_ma, flags) \
                (RDO_OBJ_POS(n) | (flags) | \
                RDO_FIXED_VAR_OP_CURR(op_ma) | \
                RDO_FIXED_VAR_MAX_CURR(max_ma))


#define RDO_BATT(n, op_mw, max_mw, flags) \
                (RDO_OBJ_POS(n) | (flags) | \
                RDO_BATT_OP_POWER(op_mw) | \
                RDO_BATT_MAX_POWER(max_mw))

/* BDO : BIST Data Object */
#define BDO_MODE_RECV       (0 << 28)
#define BDO_MODE_TRANSMIT   (1 << 28)
#define BDO_MODE_COUNTERS   (2 << 28)
#define BDO_MODE_CARRIER0   (3 << 28)
#define BDO_MODE_CARRIER1   (4 << 28)
#define BDO_MODE_CARRIER2   (5 << 28)
#define BDO_MODE_CARRIER3   (6 << 28)
#define BDO_MODE_EYE        (7 << 28)

#define BDO(mode, cnt)      ((mode) | ((cnt) & 0xFFFF))

#define SVID_DISCOVERY_MAX 16

/* number of edges and time window to detect CC line is not idle */
#define PD_RX_TRANSITION_COUNT  3
#define PD_RX_TRANSITION_WINDOW 20 /* between 12us and 20us */

/* from USB Type-C Specification Table 5-1 */
#define PD_T_AME (1*SECOND) /* timeout from UFP attach to Alt Mode Entry */

/* VDM Timers ( USB PD Spec Rev2.0 Table 6-30 )*/
#define PD_T_VDM_BUSY         (100*MSEC) /* at least 100ms */
#define PD_T_VDM_E_MODE        (25*MSEC) /* enter/exit the same max */
#define PD_T_VDM_RCVR_RSP      (15*MSEC) /* max of 15ms */
#define PD_T_VDM_SNDR_RSP      (30*MSEC) /* max of 30ms */
#define PD_T_VDM_WAIT_MODE_E  (100*MSEC) /* enter/exit the same max */

/*
 * VDO : Vendor Defined Message Object
 * VDM object is minimum of VDM header + 6 additional data objects.
 */

/*
 * VDM header
 * ----------
 * <31:16>  :: SVID
 * <15>     :: VDM type ( 1b == structured, 0b == unstructured )
 * <14:13>  :: Structured VDM version (can only be 00 == 1.0 currently)
 * <12:11>  :: reserved
 * <10:8>   :: object position (1-7 valid ... used for enter/exit mode only)
 * <7:6>    :: command type (SVDM only?)
 * <5>      :: reserved (SVDM), command type (UVDM)
 * <4:0>    :: command
 */
#define VDO_MAX_SIZE 7
#define VDO(vid, type, custom)              \
    (((vid) << 16) |                \
     ((type) << 15) |               \
     ((custom) & 0x7FFF))

#define VDO_SVDM_TYPE     (1 << 15)
#define VDO_SVDM_VERS(x)  (x << 13)
#define VDO_OPOS(x)       (x << 8)
#define VDO_CMDT(x)       (x << 6)
#define VDO_OPOS_MASK     VDO_OPOS(0x7)
#define VDO_CMDT_MASK     VDO_CMDT(0x3)

#define CMDT_INIT     0
#define CMDT_RSP_ACK  1
#define CMDT_RSP_NAK  2
#define CMDT_RSP_BUSY 3


/* reserved for SVDM ... for Google UVDM */
#define VDO_SRC_INITIATOR (0 << 5)
#define VDO_SRC_RESPONDER (1 << 5)

#define CMD_DISCOVER_IDENT  1
#define CMD_DISCOVER_SVID   2
#define CMD_DISCOVER_MODES  3
#define CMD_ENTER_MODE      4
#define CMD_EXIT_MODE       5
#define CMD_ATTENTION       6
#define CMD_DP_STATUS      16
#define CMD_DP_CONFIG      17

#define VDO_CMD_VENDOR(x)    (((10 + (x)) & 0x1f))

#define PD_VDO_VID(vdo)  ((vdo) >> 16)
#define PD_VDO_SVDM(vdo) (((vdo) >> 15) & 1)
#define PD_VDO_OPOS(vdo) (((vdo) >> 8) & 0x7)
#define PD_VDO_CMD(vdo)  ((vdo) & 0x1f)
#define PD_VDO_CMDT(vdo) (((vdo) >> 6) & 0x3)

#define ARRAY_SIZE(A) (sizeof(A) / sizeof(A[0]))

/* USB-IF SIDs */
#define USB_SID_PD          0xff00 /* power delivery */
#define USB_SID_DISPLAYPORT 0xff01

#define USB_GOOGLE_TYPEC_URL "http://www.google.com/chrome/devices/typec"
/* USB Vendor ID assigned to Google Inc. */
#define USB_VID_GOOGLE 0x18d1

/* Other Vendor IDs */
#define USB_VID_APPLE  0x05ac

/* Timeout for message receive in microseconds */
#define USB_PD_RX_TMOUT_US 1800

enum pd_states {
    PD_STATE_DISABLED,                      //  0
    PD_STATE_SUSPENDED,                     //  1
//#ifdef CONFIG_USB_PD_DUAL_ROLE
    PD_STATE_SNK_DISCONNECTED,              //  2
    PD_STATE_SNK_DISCONNECTED_DEBOUNCE,     //  3
    PD_STATE_SNK_ACCESSORY,                 //  4
    PD_STATE_SNK_HARD_RESET_RECOVER,        //  5
    PD_STATE_SNK_DISCOVERY,                 //  6
    PD_STATE_SNK_REQUESTED,                 //  7
    PD_STATE_SNK_TRANSITION,                //  8
    PD_STATE_SNK_READY,                     //  9

    PD_STATE_SNK_SWAP_INIT,                 // 10
    PD_STATE_SNK_SWAP_SNK_DISABLE,          // 11
    PD_STATE_SNK_SWAP_SRC_DISABLE,          // 12
    PD_STATE_SNK_SWAP_STANDBY,              // 13
    PD_STATE_SNK_SWAP_COMPLETE,             // 14
//#endif /* CONFIG_USB_PD_DUAL_ROLE */

    PD_STATE_SRC_DISCONNECTED,              // 15
    PD_STATE_SRC_DISCONNECTED_DEBOUNCE,     // 16
    PD_STATE_SRC_ACCESSORY,                 // 17
    PD_STATE_SRC_HARD_RESET_RECOVER,        // 18
    PD_STATE_SRC_STARTUP,                   // 19
    PD_STATE_SRC_DISCOVERY,                 // 20
    PD_STATE_SRC_NEGOCIATE,                 // 21
    PD_STATE_SRC_ACCEPTED,                  // 22
    PD_STATE_SRC_POWERED,                   // 23
    PD_STATE_SRC_TRANSITION,                // 24
    PD_STATE_SRC_READY,                     // 25
    PD_STATE_SRC_GET_SINK_CAP,              // 26
    PD_STATE_DR_SWAP,                       // 27

#ifdef CONFIG_USB_PD_DUAL_ROLE
    PD_STATE_SRC_SWAP_INIT,                 // 28
    PD_STATE_SRC_SWAP_SNK_DISABLE,          // 29
    PD_STATE_SRC_SWAP_SRC_DISABLE,          // 30
    PD_STATE_SRC_SWAP_STANDBY,              // 31

#ifdef CONFIG_USBC_VCONN_SWAP
    PD_STATE_VCONN_SWAP_SEND,
    PD_STATE_VCONN_SWAP_INIT,
    PD_STATE_VCONN_SWAP_READY,
#endif /* CONFIG_USBC_VCONN_SWAP */
#endif /* CONFIG_USB_PD_DUAL_ROLE */

    PD_STATE_SOFT_RESET,                    // 32
    PD_STATE_HARD_RESET_SEND,               // 33
    PD_STATE_HARD_RESET_EXECUTE,            // 34
#ifdef CONFIG_COMMON_RUNTIME
    PD_STATE_BIST_RX,                       // 35
    PD_STATE_BIST_TX,                       // 36
#endif

    /* Number of states. Not an actual state. */
    PD_STATE_COUNT,
};

enum pd_cc_states {
    PD_CC_NONE,

    /* From DFP perspective */
    PD_CC_NO_UFP,
    PD_CC_AUDIO_ACC,
    PD_CC_DEBUG_ACC,
    PD_CC_UFP_ATTACHED,

    /* From UFP perspective */
    PD_CC_DFP_ATTACHED
};

enum vdm_states {
    VDM_STATE_ERR_BUSY = -3,
    VDM_STATE_ERR_SEND = -2,
    VDM_STATE_ERR_TMOUT = -1,
    VDM_STATE_DONE = 0,
    /* Anything >0 represents an active state */
    VDM_STATE_READY = 1,
    VDM_STATE_BUSY = 2,
    VDM_STATE_WAIT_RSP_BUSY = 3,
};

enum pd_dual_role_states {
    PD_DRP_TOGGLE_ON,
    PD_DRP_TOGGLE_OFF,
    PD_DRP_FORCE_SINK,
    PD_DRP_FORCE_SOURCE,
};

/* Microsecond timestamp. */
typedef union {
    uint64_t val;
    struct {
        uint32_t lo;
        uint32_t hi;
    } le /* little endian words */;
} timestamp_t;

/*
 * This is a group of misciallanous functions and defines
 * that are needed to interface with implementation-specific
 * hardware or underlying OS. 
 * 
 * As this library gets more organized, these should go 
 * somwhere specific. 
 */
#define CONFIG_USB_PD_PULLUP TYPEC_RP_1A5
/* Define typical operating power and max power */
#define PD_OPERATING_POWER_MW 25000ul
#define PD_MAX_POWER_MW       25000ul
#define PD_MAX_CURRENT_MA     5000ul
#define PD_MAX_VOLTAGE_MV     20000ul
void pd_power_supply_reset(int port);
int  pd_set_power_supply_ready(int port);
int  pd_is_vbus_present(int port);
#ifdef CONFIG_USBC_SS_MUX
// Initialize USB mux to its default state
usb_mux_init(port);
#endif
void pd_set_input_current_limit(int port, uint32_t max_ma,
        uint32_t supply_voltage);
// Default state of PD communication enabled flag
#define CONFIG_USB_PD_COMM_ENABLED
int task_clear_event(int port);
int task_wait_event_mask(uint32_t event_mask, uint64_t timeout_us);
int task_wait_event(uint64_t timeout_us);
timestamp_t get_time(void);
int pd_board_checks(void);
int pd_board_check_request(uint32_t rdo);
int pd_check_data_swap(int port, int data_role);
int pd_check_power_swap(int port);
void pd_execute_data_swap(int port, int data_role);
/* Default USB data role when a USB PD debug accessory is seen */
#define CONFIG_USB_PD_DEBUG_DR PD_ROLE_DFP
/*
 * delay to turn on the power supply max is ~16ms.
 * delay to turn off the power supply max is about ~180ms.
 */
#define PD_POWER_SUPPLY_TURN_ON_DELAY  30000  /* us */
#define PD_POWER_SUPPLY_TURN_OFF_DELAY 250000 /* us */
void pd_transition_voltage(int idx);
void pd_check_pr_role(int port, int pr_role, int flags);
void pd_check_dr_role(int port, int pr_role, int flags);
/* TODO: fill in correct source and sink capabilities */
const uint32_t pd_src_pdo[] = {
        PDO_FIXED(5000ul, 1500ul, PDO_FIXED_FLAGS),
};
const int pd_src_pdo_cnt = ARRAY_SIZE(pd_src_pdo);

const uint32_t pd_snk_pdo[] = {
        PDO_FIXED(5000ul, 500ul, PDO_FIXED_FLAGS),
        PDO_BATT(4750ul, 21000ul, 15000ul),
        PDO_VAR(4750ul, 21000ul, 3000ul),
};
const int pd_snk_pdo_cnt = ARRAY_SIZE(pd_snk_pdo);
int pd_is_valid_input_voltage(uint32_t mv);

class USB_PD {
    public:
        USB_PD(USB_TCPM *tcpm, int port_id, enum pd_states default_state);
        int     init(void);
        void    pd_state_machine(void);
        void    set_state(enum pd_states next_state);
        void    vdm_send_state_machine(void);
        void    request_data_swap(void);

    //private:
        void    set_state_timeout(uint64_t timeout, enum pd_states timeout_state);
        int     is_connected(void);
        int     pdo_busy(void);
        int     pd_transmit(enum tcpm_transmit_type type,
                    uint16_t header, const uint32_t *data);
        uint64_t vdm_get_ready_timeout(uint32_t vdm_hdr);
        void    pd_transmit_complete(int status);
        void    inc_id(void);
        void    execute_hard_reset(void);
        void    handle_request(uint16_t head, uint32_t *payload);
        void    handle_data_request(uint16_t head,
                    uint32_t *payload);
        void    handle_ctrl_request(uint16_t head, 
                    uint32_t *payload);
        int     check_requested_voltage(uint32_t rdo);
        int     send_control(int type);
        void    update_pdo_flags(uint32_t pdo);
        void    handle_vdm_request(int cnt, uint32_t *payload);
        int     send_source_cap(void);
        void    execute_soft_reset(void);
        void    pd_dr_swap(void);
        void    set_data_role(int role);
        void    update_roles(void);

#ifdef CONFIG_USB_PD_ALT_MODE_DFP
#ifdef CONFIG_CMD_USB_PD_PE
#endif /* CONFIG_CMD_USB_PD_PE */
#endif /* CONFIG_USB_PD_ALT_MODE_DFP */
        int     pd_svdm(int cnt, uint32_t *payload, uint32_t **rpayload);
        int     pd_custom_vdm(int cnt, uint32_t *payload, uint32_t **rpayload);
        void    queue_vdm(uint32_t *header, const uint32_t *data,
                    int data_cnt);
        void    send_vdm(uint32_t vid, int cmd, const uint32_t *data,
                    int count);
#ifdef CONFIG_USB_PD_DUAL_ROLE
        void    store_src_cap(int cnt, uint32_t *src_caps);
        int     send_request_msg(int always_send_request);
        void    process_source_cap(int cnt, uint32_t *src_caps);
        void    send_sink_cap(void);
        int     send_request(uint32_t rdo);
        int     is_power_swapping(void);

        /**
         * Get dual role state
         *
         * @return Current dual-role state, from enum pd_dual_role_states
         */
        //enum pd_dual_role_states pd_get_dual_role(void);
        /**
         * Set dual role state, from among enum pd_dual_role_states
         *
         * @param state New state of dual-role port, selected from
         *              enum pd_dual_role_states
         */
        //void pd_set_dual_role(enum pd_dual_role_states state);

        /**
         * Get role, from among PD_ROLE_SINK and PD_ROLE_SOURCE
         *
         * @param port Port number from which to get role
         */
        //int pd_get_role(int port);
#endif
        int     snk_debug_acc_toggle(void);
        static int cc_is_rp(int cc);
        static int get_snk_polarity(int cc1, int cc2);
        int     build_request(int cnt, uint32_t *src_caps, uint32_t *rdo,
             uint32_t *ma, uint32_t *mv, enum pd_request_type req_type);
        int     find_pdo_index(int cnt, uint32_t *src_caps, int max_mv);
        void    extract_pdo_power(uint32_t pdo, uint32_t *ma, uint32_t *mv);


        USB_TCPM *m_tcpm;

        int head;
        int port;
        uint32_t payload[7];
        uint64_t timeout_deadline;
        int cc1, cc2;
        int res, incoming_packet;
        int hard_reset_count;
#ifdef CONFIG_USB_PD_DUAL_ROLE
        uint64_t next_role_swap;
#ifndef CONFIG_USB_PD_VBUS_DETECT_NONE
        int snk_hard_reset_vbus_off;
#endif
#ifdef CONFIG_CHARGE_MANAGER
        int typec_curr, typec_curr_change;
#endif /* CONFIG_CHARGE_MANAGER */
#endif /* CONFIG_USB_PD_DUAL_ROLE */
        int default_state;
        enum pd_states this_state;
        enum pd_cc_states new_cc_state;
        timestamp_t now;
        int caps_count, hard_reset_sent;
        int snk_cap_count;
        int evt;
        /* Cap on the max voltage requested as a sink (in millivolts) */
        unsigned int max_request_mv;

        uint8_t pd_comm_enable;
        int pd_comm_enabled;

        /* current port power role (SOURCE or SINK) */
        uint8_t power_role;
        /* current port data role (DFP or UFP) */
        uint8_t data_role;
        /* port flags, see PD_FLAGS_* */
        uint16_t flags;
        /* 3-bit rolling message ID counter */
        uint8_t msg_id;
        /* Port polarity : 0 => CC1 is CC line, 1 => CC2 is CC line */
        uint8_t polarity;
        /* PD state for port */
        enum pd_states task_state;
        /* PD state when we run state handler the last time */
        enum pd_states last_state;
        /* The state to go to after timeout */
        enum pd_states timeout_state;
        /* Timeout for the current state. Set to 0 for no timeout. */
        uint64_t timeout;
        /* Time for source recovery after hard reset */
        uint64_t src_recover;
        /* Time for CC debounce end */
        uint64_t cc_debounce;
        /* The cc state */
        enum pd_cc_states cc_state;
        /* status of last transmit */
        uint8_t tx_status;

        /* last requested voltage PDO index */
        int requested_idx;
        /* Current limit / voltage based on the last request message */
        uint32_t curr_limit;
        uint32_t supply_voltage;
        /* Signal charging update that affects the port */
        int new_power_request;
        /* Store previously requested voltage request */
        int prev_request_mv;
        /* Time for Try.SRC states */
        uint64_t try_src_marker;

        /* PD state for Vendor Defined Messages */
        enum vdm_states vdm_state;
        /* Timeout for the current vdm state.  Set to 0 for no timeout. */
        timestamp_t vdm_timeout;
        /* next Vendor Defined Message to send */
        uint32_t vdo_data[VDO_MAX_SIZE];
        uint8_t vdo_count;
        /* VDO to retry if UFP responder replied busy. */
        uint32_t vdo_retry;

#ifdef CONFIG_USB_PD_DUAL_ROLE
        /* Port dual-role state */
        enum pd_dual_role_states drp_state;

        /* Last received source cap */
        uint32_t pd_src_caps[PDO_MAX_OBJECTS];
        int pd_src_cap_cnt;

        /* Enable varible for Try.SRC states */
        uint8_t pd_try_src_enable;
#endif
};

#endif // USB_PD_H
