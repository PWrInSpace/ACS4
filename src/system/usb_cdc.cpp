/*
 * ACS4 Flight Computer — USB CDC Implementation
 *
 * USB OTG_HS in FS mode, CDC ACM class (Virtual COM Port).
 * Based on ChibiOS testhal/STM32/multi/USB_CDC example.
 *
 * STM32H725 has only OTG_HS (mapped as USBD2 in ChibiOS).
 * OTG_HS runs in FS mode via internal FS PHY on PA11/PA12.
 * USB clock: HSI48 (48 MHz).
 */

#include "system/usb_cdc.h"

extern "C" {
#include "ch.h"

#include "hal.h"
}

/* Only compile for custom PCB target. */
#if defined(STM32H725xx) && HAL_USE_USB && HAL_USE_SERIAL_USB

/* ── Endpoint numbers ─────────────────────────────────────────────────── */

static constexpr usbep_t USB_CDC_DATA_EP      = 1; /* Bulk IN+OUT       */
static constexpr usbep_t USB_CDC_INTERRUPT_EP  = 2; /* Interrupt IN      */

/* ── SDU1 driver instance ─────────────────────────────────────────────── */

static SerialUSBDriver SDU1;

/* ═══════════════════════════════════════════════════════════════════════
 * USB Descriptors
 * ═══════════════════════════════════════════════════════════════════════ */

/* ── Device Descriptor ────────────────────────────────────────────────── */

static const uint8_t vcom_device_descriptor_data[18] = {
    USB_DESC_DEVICE(
        0x0110, /* bcdUSB (1.1)               */
        0x02,   /* bDeviceClass (CDC)          */
        0x00,   /* bDeviceSubClass             */
        0x00,   /* bDeviceProtocol             */
        0x40,   /* bMaxPacketSize (64 bytes)   */
        0x0483, /* idVendor (ST)               */
        0x5740, /* idProduct (CDC VCP)         */
        0x0200, /* bcdDevice                   */
        1,      /* iManufacturer               */
        2,      /* iProduct                    */
        3,      /* iSerialNumber               */
        1)      /* bNumConfigurations          */
};

static const USBDescriptor vcom_device_descriptor = {
    sizeof vcom_device_descriptor_data,
    vcom_device_descriptor_data};

/* ── Configuration Descriptor (CDC ACM, 2 interfaces) ────────────────── */

static const uint8_t vcom_configuration_descriptor_data[67] = {
    /* Configuration Descriptor. */
    USB_DESC_CONFIGURATION(
        67,   /* wTotalLength                */
        0x02, /* bNumInterfaces              */
        0x01, /* bConfigurationValue         */
        0,    /* iConfiguration              */
        0xC0, /* bmAttributes (self-powered) */
        50),  /* bMaxPower (100 mA)          */

    /* Interface 0: CDC Communication (control). */
    USB_DESC_INTERFACE(
        0x00, /* bInterfaceNumber            */
        0x00, /* bAlternateSetting           */
        0x01, /* bNumEndpoints               */
        0x02, /* bInterfaceClass (CDC)       */
        0x02, /* bInterfaceSubClass (ACM)    */
        0x01, /* bInterfaceProtocol (AT cmd) */
        0),   /* iInterface                  */

    /* Header Functional Descriptor (CDC 5.2.3). */
    USB_DESC_BYTE(5),      /* bLength                     */
    USB_DESC_BYTE(0x24),   /* bDescriptorType (CS_IFACE)  */
    USB_DESC_BYTE(0x00),   /* bDescriptorSubtype (Header) */
    USB_DESC_BCD(0x0110),  /* bcdCDC                      */

    /* Call Management Functional Descriptor. */
    USB_DESC_BYTE(5),    /* bFunctionLength               */
    USB_DESC_BYTE(0x24), /* bDescriptorType (CS_IFACE)    */
    USB_DESC_BYTE(0x01), /* bDescriptorSubtype (Call Mgmt)*/
    USB_DESC_BYTE(0x00), /* bmCapabilities                */
    USB_DESC_BYTE(0x01), /* bDataInterface                */

    /* ACM Functional Descriptor. */
    USB_DESC_BYTE(4),    /* bFunctionLength               */
    USB_DESC_BYTE(0x24), /* bDescriptorType (CS_IFACE)    */
    USB_DESC_BYTE(0x02), /* bDescriptorSubtype (ACM)      */
    USB_DESC_BYTE(0x02), /* bmCapabilities                */

    /* Union Functional Descriptor. */
    USB_DESC_BYTE(5),    /* bFunctionLength               */
    USB_DESC_BYTE(0x24), /* bDescriptorType (CS_IFACE)    */
    USB_DESC_BYTE(0x06), /* bDescriptorSubtype (Union)    */
    USB_DESC_BYTE(0x00), /* bMasterInterface              */
    USB_DESC_BYTE(0x01), /* bSlaveInterface0              */

    /* Endpoint 2 IN: Interrupt (CDC notifications). */
    USB_DESC_ENDPOINT(
        USB_CDC_INTERRUPT_EP | 0x80, /* bEndpointAddress (IN)   */
        0x03,                        /* bmAttributes (Interrupt)*/
        0x0008,                      /* wMaxPacketSize (8)      */
        0xFF),                       /* bInterval               */

    /* Interface 1: CDC Data (bulk transfers). */
    USB_DESC_INTERFACE(
        0x01, /* bInterfaceNumber            */
        0x00, /* bAlternateSetting           */
        0x02, /* bNumEndpoints               */
        0x0A, /* bInterfaceClass (CDC Data)  */
        0x00, /* bInterfaceSubClass          */
        0x00, /* bInterfaceProtocol          */
        0x00),/* iInterface                  */

    /* Endpoint 1 OUT: Bulk (host -> device). */
    USB_DESC_ENDPOINT(
        USB_CDC_DATA_EP,        /* bEndpointAddress (OUT)   */
        0x02,                   /* bmAttributes (Bulk)      */
        0x0040,                 /* wMaxPacketSize (64)      */
        0x00),                  /* bInterval                */

    /* Endpoint 1 IN: Bulk (device -> host). */
    USB_DESC_ENDPOINT(
        USB_CDC_DATA_EP | 0x80, /* bEndpointAddress (IN)    */
        0x02,                   /* bmAttributes (Bulk)      */
        0x0040,                 /* wMaxPacketSize (64)      */
        0x00)                   /* bInterval                */
};

static const USBDescriptor vcom_configuration_descriptor = {
    sizeof vcom_configuration_descriptor_data,
    vcom_configuration_descriptor_data};

/* ── String Descriptors ───────────────────────────────────────────────── */

/* Language ID (US English). */
static const uint8_t vcom_string0[] = {
    USB_DESC_BYTE(4),
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    USB_DESC_WORD(0x0409)};

/* Manufacturer. */
static const uint8_t vcom_string1[] = {
    USB_DESC_BYTE(22),
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    'A', 0, 'C', 0, 'S', 0, '4', 0, ' ', 0,
    'T', 0, 'e', 0, 'a', 0, 'm', 0, 0, 0};

/* Product. */
static const uint8_t vcom_string2[] = {
    USB_DESC_BYTE(48),
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    'A', 0, 'C', 0, 'S', 0, '4', 0, ' ', 0,
    'F', 0, 'l', 0, 'i', 0, 'g', 0, 'h', 0, 't', 0, ' ', 0,
    'C', 0, 'o', 0, 'm', 0, 'p', 0, 'u', 0, 't', 0, 'e', 0, 'r', 0,
    ' ', 0, 'C', 0, 'D', 0, 'C', 0};

/* Serial number. */
static const uint8_t vcom_string3[] = {
    USB_DESC_BYTE(8),
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    '0' + CH_KERNEL_MAJOR, 0,
    '0' + CH_KERNEL_MINOR, 0,
    '0' + CH_KERNEL_PATCH, 0};

static const USBDescriptor vcom_strings[] = {
    {sizeof vcom_string0, vcom_string0},
    {sizeof vcom_string1, vcom_string1},
    {sizeof vcom_string2, vcom_string2},
    {sizeof vcom_string3, vcom_string3}};

/* ═══════════════════════════════════════════════════════════════════════
 * USB Callbacks
 * ═══════════════════════════════════════════════════════════════════════ */

static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t    dtype,
                                           uint8_t    dindex,
                                           uint16_t   lang)
{
    (void)usbp;
    (void)lang;
    switch (dtype)
    {
        case USB_DESCRIPTOR_DEVICE:
            return &vcom_device_descriptor;
        case USB_DESCRIPTOR_CONFIGURATION:
            return &vcom_configuration_descriptor;
        case USB_DESCRIPTOR_STRING:
            if (dindex < 4)
            {
                return &vcom_strings[dindex];
            }
            break;
        default:
            break;
    }
    return nullptr;
}

/* ── Endpoint state & config ──────────────────────────────────────────── */

static USBInEndpointState  ep1instate;
static USBOutEndpointState ep1outstate;

static const USBEndpointConfig ep1config = {
    USB_EP_MODE_TYPE_BULK,  /* ep_mode          */
    nullptr,                /* setup_cb         */
    sduDataTransmitted,     /* in_cb            */
    sduDataReceived,        /* out_cb           */
    0x0040,                 /* in_maxsize  (64) */
    0x0040,                 /* out_maxsize (64) */
    &ep1instate,            /* in_state         */
    &ep1outstate,           /* out_state        */
    2,                      /* ep_buffers       */
    nullptr                 /* setup_buf        */
};

static USBInEndpointState ep2instate;

static const USBEndpointConfig ep2config = {
    USB_EP_MODE_TYPE_INTR,      /* ep_mode          */
    nullptr,                    /* setup_cb         */
    sduInterruptTransmitted,    /* in_cb            */
    nullptr,                    /* out_cb           */
    0x0010,                     /* in_maxsize  (16) */
    0x0000,                     /* out_maxsize      */
    &ep2instate,                /* in_state         */
    nullptr,                    /* out_state        */
    1,                          /* ep_buffers       */
    nullptr                     /* setup_buf        */
};

/* ── USB event handler ────────────────────────────────────────────────── */

static void usb_event(USBDriver *usbp, usbevent_t event)
{
    switch (event)
    {
        case USB_EVENT_ADDRESS:
            return;

        case USB_EVENT_CONFIGURED:
            chSysLockFromISR();
            usbInitEndpointI(usbp, USB_CDC_DATA_EP, &ep1config);
            usbInitEndpointI(usbp, USB_CDC_INTERRUPT_EP, &ep2config);
            sduConfigureHookI(&SDU1);
            chSysUnlockFromISR();
            return;

        case USB_EVENT_RESET:
            /* Falls through. */
        case USB_EVENT_UNCONFIGURED:
            /* Falls through. */
        case USB_EVENT_SUSPEND:
            chSysLockFromISR();
            sduSuspendHookI(&SDU1);
            chSysUnlockFromISR();
            return;

        case USB_EVENT_WAKEUP:
            chSysLockFromISR();
            sduWakeupHookI(&SDU1);
            chSysUnlockFromISR();
            return;

        case USB_EVENT_STALLED:
            return;
    }
}

/* ── SOF handler (required for CDC timing) ────────────────────────────── */

static void sof_handler(USBDriver *usbp)
{
    (void)usbp;
    osalSysLockFromISR();
    sduSOFHookI(&SDU1);
    osalSysUnlockFromISR();
}

/* ── USB driver config ────────────────────────────────────────────────── */

static const USBConfig usbcfg = {
    usb_event,        /* event_cb          */
    get_descriptor,   /* get_descriptor_cb */
    sduRequestsHook,  /* requests_hook_cb  */
    sof_handler       /* sof_cb            */
};

/* ── Serial-over-USB config ───────────────────────────────────────────── */
/*
 * H725 has only OTG_HS, which is USBD2 in ChibiOS.
 * OTG_HS runs in FS mode via internal FS PHY.
 */
static const SerialUSBConfig serusbcfg = {
    &USBD2,              /* usbp     (OTG_HS = USBD2)    */
    USB_CDC_DATA_EP,     /* bulk_in  endpoint             */
    USB_CDC_DATA_EP,     /* bulk_out endpoint             */
    USB_CDC_INTERRUPT_EP /* int_in   endpoint             */
};

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

namespace acs
{

void usb_cdc_init()
{
    /* Initialize the SerialUSBDriver object. */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Force USB re-enumeration by disconnecting/reconnecting.
     * This ensures the host recognizes a fresh device after MCU reset.
     * The 1500 ms delay allows the host OS to complete enumeration.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
}

BaseSequentialStream *usb_cdc_stream()
{
    return reinterpret_cast<BaseSequentialStream *>(&SDU1);
}

}  // namespace acs

#endif /* STM32H725xx && HAL_USE_USB && HAL_USE_SERIAL_USB */

