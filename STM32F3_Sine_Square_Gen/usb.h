//------------------------------------------------------------------------------
// Simple USB implementation for STM32F103C8T6 Bluepill board
// Based on CH340 USB to serial converter and derived from STM virtual com port
// but uses way less code. Lacks error handling.
//
// To use the USB device the system clock needs to be either 48MHz or 72MHz
// Select the right SUB clock divide setting to have a 48MHz USB clock
// RCC->CFGR USB_PRE bit
//   0 for 72MHz system clock
//   1 for 48MHZ system clock
//------------------------------------------------------------------------------

#ifndef USB_H
#define USB_H

#ifdef __cplusplus
extern "C" {
#endif

void usbInit(void);        //Needs to be called before the USB device can be used
void usbIrqHandler(void);  //Needs to be set in the interrupt vector table
void usbSend(uint8_t c);   //Can be called to send a character
int16_t usbRead(void);    //Can be called to get a character. Returns -1 when nothing available

//Default USB endpoint settings
#define EP0RX_BASE        0x40006080  //Endpoint 0 receive buffer base
#define EP0TX_BASE        0x40006100  //Endpoint 0 transmit buffer base

#define EP2RX_BASE        0x40006200  //Endpoint 2 receive buffer base
#define EP2TX_BASE        0x40006180  //Endpoint 2 transmit buffer base

#define BTABLE_ADDRESS    0x00

//Overlay on PMA endpoint 0 receive buffer to get to setup data
typedef struct
{
  __IO uint8_t  bmRequestType;
  __IO uint8_t  bRequest;
       uint16_t RESERVED1;
  __IO wbcombi  wValue;
       uint16_t RESERVED2;
  __IO wbcombi  wIndex;
       uint16_t RESERVED3;
  __IO uint16_t wLength;
       uint16_t RESERVED4;
} SETUP_PCKT_TypeDef;

#define SETUPPACKET ((SETUP_PCKT_TypeDef *) EP0RX_BASE)


//Receive count has a special format. The 10 least significant bits represent the number of actually received bytes
//The upper 6 bits use a block count system where the buffer size is either multiples of 2 or 32
//So for an 8 byte buffer the setting is msb low and a block count of 4
//So for an 32 byte buffer the setting is msb low and a block count of 16
#define USB_8_BYTE_RX_BUF    0x1000U
#define USB_32_BYTE_RX_BUF   0x4000U


//bmRequestType.Type
#define REQUEST_STANDARD  0
#define REQUEST_CLASS     1
#define REQUEST_VENDOR    2
#define REQUEST_RESERVED  3

//USB Standard Request Codes
#define USB_REQUEST_GET_STATUS          0
#define USB_REQUEST_CLEAR_FEATURE       1
#define USB_REQUEST_SET_FEATURE         3
#define USB_REQUEST_SET_ADDRESS         5
#define USB_REQUEST_GET_DESCRIPTOR      6
#define USB_REQUEST_SET_DESCRIPTOR      7
#define USB_REQUEST_GET_CONFIGURATION   8
#define USB_REQUEST_SET_CONFIGURATION   9
#define USB_REQUEST_GET_INTERFACE      10
#define USB_REQUEST_SET_INTERFACE      11
#define USB_REQUEST_SYNC_FRAME         12

// USB Descriptor Types
#define USB_DEVICE_DESC_TYPE      1
#define USB_CFG_DESC_TYPE         2
#define USB_STR_DESC_TYPE         3
#define USB_IFACE_DESC_TYPE       4
#define USB_EP_DESC_TYPE          5
#define USB_DEVICE_QR_DESC_TYPE   6
#define USB_OSPEED_CFG_DESC_TYPE  7
#define USB_IFACE_PWR_DESC_TYPE   8


#define USB_CH340_REQ_VENDOR_VERSION   0x5F
#define USB_CH340_REQ_READ_REG         0x95
#define USB_CH340_REQ_WRITE_REG        0x9A
#define USB_CH340_REQ_SERIAL_INIT      0xA1
#define USB_CH340_REQ_MODEM_CTRL       0xA4

#define USB_CH340_READ_ATTACH          0x2518  //For reading the line control setting
#define USB_CH340_READ_STATUS          0x0706  //For reading the modem line status

#define USB_CH340_REG_BAUDRATE         0x1312
#define USB_CH340_REG_BAUDFACTOR       0x0F2C  //Sends the low byte of the factor for more precise baud setting
#define USB_CH340_REG_LINECONTROL      0x2518  //For writing the line control settings when used
                                               //Some linux variants have a different driver where this is not used
                                               //Is always 0x50 which should be 0xC3

#define CH340_LCR_ENABLE_RX       0x80
#define CH340_LCR_ENABLE_TX       0x40
#define CH340_LCR_MARK_SPACE      0x20
#define CH340_LCR_PAR_EVEN        0x10
#define CH340_LCR_ENABLE_PAR      0x08
#define CH340_LCR_STOP_BITS_2     0x04
#define CH340_LCR_CS8             0x03
#define CH340_LCR_CS7             0x02
#define CH340_LCR_CS6             0x01
#define CH340_LCR_CS5             0x00

#define CH340_LCR_NOF_BITS_MASK   0x03

#define CH340_BIT_CTS             0x01
#define CH340_BIT_DSR             0x02
#define CH340_BIT_RI              0x04
#define CH340_BIT_DCD             0x08

#define CH340_BAUDRATE_FACTOR     1532620800
#define CH340_BAUDRATE_DIVMAX     3

#ifdef __cplusplus
}
#endif

#endif /* USB_H */

