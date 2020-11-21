//------------------------------------------------------------------------------
// Simple USB implementation for STM32F103C8T6 Bluepill board
// Based on CH340 USB to serial converter and derived from STM virtual com port
// but uses way less code. Lacks error and suspend handling.
//
// To use the USB device the system clock needs to be either 48MHz or 72MHz
// Select the right SUB clock divide setting to have a 48MHz USB clock
// RCC->CFGR USB_PRE bit
//   0 for 72MHz system clock
//   1 for 48MHZ system clock
//------------------------------------------------------------------------------

#include "stm32f303_db.h"
#include "usb.h"

//Global memory assignment for USB device
uint8_t usb_rx[64];                       //Buffer for USB receive data
uint8_t volatile usb_rx_in_idx = 0;       //Index for putting data into the USB receive buffer
uint8_t volatile usb_rx_out_idx = 0;      //Index for taking data from the USB receive buffer. Set volatile since it changes in interrupt routine.

uint8_t usb_tx[64];                       //Buffer for USB transmit data
uint8_t volatile usb_tx_in_idx = 0;       //Index for putting data into the USB transmit buffer
uint8_t volatile usb_tx_out_idx = 0;      //Index for taking data from the USB transmit buffer. Set volatile since it changes in interrupt routine.

volatile uint16_t *EP0TxPtr;              //Endpoint 0 transmit pointer. Data needs to be send in 8 byte blocks. This pointer keeps track of where to get the data from
volatile uint8_t   EP0TxLen;              //Endpoint 0 transmit length. This length counter keeps track of how many bytes still need to be send.
volatile uint8_t   DeviceAddress = 0;     //Flag to signal host has send a "set address" command, which needs to be set after confirmation transmission.
volatile uint8_t   DeviceConfigured = 0;  //Flag to signal device is up and running.
volatile uint8_t   EP2DisableTX = 0;      //Flag to disable the endpoint 2 transmission function.

//USB Standard Device Descriptor
const uint8_t DeviceDescriptor[] =
{
  0x12,   // bLength
  0x01,   // bDescriptorType
  0x10,
  0x01,   // bcdUSB = 1.1
  0xFF,   // bDeviceClass: Vendor
  0x00,   // bDeviceSubClass
  0x00,   // bDeviceProtocol
  0x08,   // bMaxPacketSize0 = 8
  0x86,
  0x1A,   // idVendor = 0x1A86
  0x23,
  0x75,   // idProduct = 0x7523
  0x62,
  0x02,   // bcdDevice = 2.62
  0x00,   // Index of string descriptor describing manufacturer
  0x02,   // Index of string descriptor describing product
  0x00,   // Index of string descriptor describing the device's serial number
  0x01    // bNumConfigurations
};

//USB Configuration Descriptor
const uint8_t ConfigDescriptor[] =
{
  0x09,   // bLength: Configuration Descriptor size
  0x02,   // bDescriptorType: Configuration
  0x27,   // wTotalLength:no of returned bytes
  0x00,
  0x01,   // bNumInterfaces: 1 interface
  0x01,   // bConfigurationValue: Configuration value
  0x00,   // iConfiguration: Index of string descriptor describing the configuration
  0x80,   // bmAttributes: self powered
  0x31,   // MaxPower 98 mA

  //Interface Descriptor
  0x09,   // bLength: Interface Descriptor size
  0x04,   // bDescriptorType: Interface
  // Interface descriptor type
  0x00,   // bInterfaceNumber: Number of Interface
  0x00,   // bAlternateSetting: Alternate setting
  0x03,   // bNumEndpoints: 3 endpoints used
  0xFF,   // bInterfaceClass: Vendor Interface Class
  0x01,   // bInterfaceSubClass
  0x02,   // bInterfaceProtocol
  0x00,   // iInterface:

  //Endpoint 2 IN Descriptor
  0x07,   // bLength: Endpoint Descriptor size
  0x05,   // bDescriptorType: Endpoint
  0x82,   // bEndpointAddress: (IN2)
  0x02,   // bmAttributes: Bulk
  0x20,   // wMaxPacketSize:
  0x00,
  0x00,   // bInterval:

  //Endpoint 2 OUT Descriptor
  0x07,   // bLength: Endpoint Descriptor size
  0x05,   // bDescriptorType: Endpoint
  0x02,   // bEndpointAddress: (OUT2)
  0x02,   // bmAttributes: Bulk
  0x20,   // wMaxPacketSize:
  0x00,
  0x00,   // bInterval: ignore for Bulk transfer

  //Endpoint 1 IN Descriptor
  0x07,   // bLength: Endpoint Descriptor size
  0x05,   // bDescriptorType: Endpoint
  0x81,   // bEndpointAddress: (IN1)
  0x03,   // bmAttributes: Interrupt
  0x08,   // wMaxPacketSize:
  0x00,
  0x01    // bInterval
};

//USB String Descriptors
const uint8_t StringLangID[] =
{
  0x04,
  0x03,
  0x09,
  0x04    // LangID = 0x0409: U.S. English
};

const uint8_t StringVendor[] =
{
  0x26,  // Size of Vendor string
  0x03,  // bDescriptorType
         // Manufacturer: "STMicroelectronics"
  'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
  'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
  'c', 0, 's', 0
};

const uint8_t StringProduct[] =
{
  0x1C,  // bLength
  0x03,  // bDescriptorType
         // Serial name: "USB 2.0-Serial"
  'U', 0, 'S', 0, 'B', 0, '2', 0, '.', 0, '0', 0, '-', 0,
  'S', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0
};

const uint8_t StringSerial[] =
{
  0x1C,  // bLength
  0x03,  // bDescriptorType
         // Serial name: "USB 2.0-Serial"
  'U', 0, 'S', 0, 'B', 0, '2', 0, '.', 0, '0', 0, '-', 0,
  'S', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0
};

//Vendor specifics
const uint8_t vendorVersion[] = { 0x31, 0x00 };
const uint8_t vendorAttach[]  = { 0xC3, 0x00 };
const uint8_t vendorStatus[]  = { 0xFF, 0xEE };

//Function for getting a character from the USB receive buffer
int16_t usbRead(void)
{
  uint8_t c;

  //See if there is any data in the buffer
  if(usb_rx_out_idx == usb_rx_in_idx)
    return -1;

  //Get available character
  c = usb_rx[usb_rx_out_idx++];

  //Keep index in valid range
  usb_rx_out_idx %= sizeof(usb_rx);

  return c;
}

//Function for putting a character in the USB transmit buffer
void usbSend(uint8_t c)
{
  uint8_t charsfree;

  //Wait until there is room in the transmit buffer
  do
  {
    //Calculate the number of free bytes minus one. Need one byte free because indexes being the same means empty buffer.
    charsfree = usb_tx_out_idx - usb_tx_in_idx - 1;

    //When the in index is higher than the out index the result is negative so add the size of the buffer to get the
    //number of free bytes. Otherwise the result is already positive
    if(usb_tx_in_idx >= usb_tx_out_idx)
      charsfree += sizeof(usb_tx);

    //When charsfree is 0 the buffer is full so calculate again till space comes available
  } while(charsfree == 0);

  //Disable transmission while putting character in the buffer
  EP2DisableTX = 1;

  //Put the character in the transmit buffer and move to next free location
  usb_tx[usb_tx_in_idx++] = c;

  //Keep index in range of buffer size
  usb_tx_in_idx %= sizeof(usb_tx);

  //Enable transmission when done
  EP2DisableTX = 0;
}

void usbInit(void)
{
  //Setup the USB peripheral
  //Enable USB
  RCC->APB1ENR |= RCC_APB1ENR_USBEN;

  //Configure EXTI line 18 which is internally connected to the USB interrupt
  EXTI->PR    = EXTI_Line18; //Clear possible pending interrupt first
  EXTI->RTSR |= EXTI_Line18; //Set to trigger on rising edge
  EXTI->IMR  |= EXTI_Line18; //Enable the interrupt

  //Configure the Nested Vector Interrupt Controller
  //Set 2 bit for preemption (group) priority and 2 bits for sub priority
  SCB->AIRCR = AIRCR_VECTKEY_MASK | NVIC_PriorityGroup_2;

  //Set priority for USB interrupt to be lower then the other interrupts
  //This is an array of 8 bit registers, of which only the upper 4 bits are used for the priority allowing for 16 levels
  //By grouping this is separated to allow for having sub priorities within a single group.
  //The higher the number the lower the priority
  NVIC->IP[USB_LP_CAN1_RX0_IRQn] = 0xC0;  //(1100b) Group priority 3, sub priority 0

  //Enable the USB interrupt
  //This is an array of 32 bit registers, only used to enable an interrupt. To disable the ICER registers need to be used
  //Each register serves 32 interrupts, so to get the register for the interrupt, shift the IRQ number right 5 times (divide by 32) and to get
  //the right interrupt enable bit, shift a unsigned 32 bit integer 1 the IRQ number anded with 31 (modulo 32) times to the right
  NVIC->ISER[USB_LP_CAN1_RX0_IRQn >> 0x05] = (uint32_t)0x01 << (USB_LP_CAN1_RX0_IRQn & 0x1F);

  //Setup the Endpoints needed for this USB device
  //Endpoint buffer descriptor table is located in the PMA, pointed to by the BTABLE register
  USB->BTABLE = BTABLE_ADDRESS;

  //Clear all pending USB interrupts
  USB->ISTR = 0;

  //Enable the needed interrupts to handle the USB tasks and clear the reset and power down bit to start the device
  USB->CNTR = USB_CNTR_SOFM | USB_CNTR_RESETM | USB_CNTR_CTRM;
}

void EP0SendData(uint16_t *sptr, uint16_t len, uint16_t maxlen)
{
  uint32_t *dptr = (uint32_t *)EP0TX_BASE;

  if(len > maxlen)
    len = maxlen;

  EP0TxPtr = sptr;
  EP0TxLen = len;

  //Check if more bytes to send than buffer space available
  if(len > 8)
    len = 8;

  //Take of the number of bytes being send now
  EP0TxLen -= len;

  //Signal to the USB device how many bytes need to be send
  EPBTABLE->EPD[0].TX_COUNT.DATA = len;

  //Number of bytes needs to be divided by two because transfer is done two bytes at a time
  //Add one extra to allow for odd number of bytes to be handled
  len = (len + 1) / 2;

  //Copy the bytes to the PMA
  while(len--)
    *dptr++ = (uint32_t)*EP0TxPtr++;

  //Signal ready to transmit by preparing for toggling the needed TX status bits
  //Keep the other bits unaltered, by using the previous data or the invariant value
  USB->EP0R = ((USB->EP0R ^ USB_EP_TX_VALID) & (USB_EPREG_MASK | USB_EPTX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX;
}

void EP2SendData(void)
{
  uint32_t *dptr = (uint32_t *)EP2TX_BASE;
  uint16_t usbdata;
  uint16_t cnt;

  //Check if transmission has been disabled
  if(EP2DisableTX == 1)
    return;

  //Check if there is data to send to the host
  //Calculate the number of bytes available in the buffer
  if((cnt = usb_tx_in_idx - usb_tx_out_idx) != 0)
  {
    //When out index bigger then in index data has a rollover in the buffer so need add the size of the buffer to get the right number
    if(usb_tx_out_idx > usb_tx_in_idx)
      cnt += sizeof(usb_tx);

    //Check if available number of characters more then the USB buffer size. If so limit the number to be copied
    if(cnt > 32)
      cnt = 32;

    //Signal to the USB device how many bytes need to be send
    EPBTABLE->EPD[2].TX_COUNT.DATA = cnt;

    //Copy all the needed bytes to the PMA two bytes at a time
    while(cnt--)
    {
      //Put first byte in low part of the temporary storage for making 16 bits data
      usbdata = (uint16_t)(usb_tx[usb_tx_out_idx++]);

      //Check if out index needs to rollover back to start of buffer
      usb_tx_out_idx %= sizeof(usb_tx);

      //See if next byte available
      if(cnt)
      {
        //Put second byte in high part of the temporary storage for making 16 bits data
        usbdata |= (uint16_t)(usb_tx[usb_tx_out_idx++] << 8);

        //Check if out index needs to rollover back to start of buffer
        usb_tx_out_idx %= sizeof(usb_tx);

        //Did the second byte
        cnt--;
      }

      //Store the data in the PMA and point to next 16 bits data field
      *dptr++ = usbdata;
    }

    //Signal ready to transmit by preparing for toggling the needed TX status bits
    //Keep the other bits unaltered, by using the previous data or the invariant value
    USB->EP2R = ((USB->EP2R ^ USB_EP_TX_VALID) & (USB_EPREG_MASK | USB_EPTX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX;
  }
}

void usbIrqHandler(void)
{
  //Check if we received an USB start of frame
  if(USB->ISTR & USB_ISTR_SOF)
  {
    //Check if device is configured to send data
    if(DeviceConfigured == 1)
    {
      //Send data through endpoint 2
      EP2SendData();
    }

    //Reset the interrupt flag
    USB->ISTR = (uint16_t)(~USB_ISTR_SOF);
  }

  //Check if we received an USB expected start of frame
  if(USB->ISTR & USB_ISTR_ESOF)
  {
    //Reset the interrupt flag
    USB->ISTR = (uint16_t)(~USB_ISTR_ESOF);
  }

  //Check if we received an USB correct transfer
  if(USB->ISTR & USB_ISTR_CTR)
  {
    uint8_t epid = USB->ISTR & USB_ISTR_EP_ID;

    //Check if the control end point is addressed
    if(epid == 0)
    {
      //Check if data received
      if(USB->EP0R & USB_EP_CTR_RX)
      {
        //For the control endpoint the type is kept on control and the address stays on 0
        //The EP_KIND bit is not used so kept on zero
        //Reset the CTR RX bit by writing a zero to it
        USB->EP0R = USB_EP_CONTROL | USB_EP_CTR_TX;

        //Check if setup data received
        if(USB->EP0R & USB_EP_SETUP)
        {
          //Process the setup data
          switch(SETUPPACKET->bRequest)
          {
            case USB_REQUEST_SET_ADDRESS:
              //Send zero length packet
              EP0SendData(0, 0, 8);

              //Prepare the device address for being set after zero length data has been transmitted
              DeviceAddress = SETUPPACKET->wValue.bytes.low;
              break;

            case USB_REQUEST_GET_DESCRIPTOR:
              switch(SETUPPACKET->wValue.bytes.high)
              {
                case USB_DEVICE_DESC_TYPE:
                  EP0SendData((uint16_t *)DeviceDescriptor, sizeof(DeviceDescriptor), SETUPPACKET->wLength);
                  break;

                case USB_CFG_DESC_TYPE:
                  EP0SendData((uint16_t *)ConfigDescriptor, sizeof(ConfigDescriptor), SETUPPACKET->wLength);
                  break;

                case USB_STR_DESC_TYPE:
                  switch(SETUPPACKET->wValue.bytes.low)
                  {
                    case 0:
                      EP0SendData((uint16_t *)StringLangID, sizeof(StringLangID), SETUPPACKET->wLength);
                      break;

                    case 1:
                      EP0SendData((uint16_t *)StringVendor, sizeof(StringVendor), SETUPPACKET->wLength);
                      break;

                    case 2:
                      EP0SendData((uint16_t *)StringProduct, sizeof(StringProduct), SETUPPACKET->wLength);
                      break;

                    case 3:
                      EP0SendData((uint16_t *)StringSerial, sizeof(StringSerial), SETUPPACKET->wLength);
                      break;
                  }
                  break;

                default:
                  EP0SendData(0, 0, 8);
                  break;
              }
              break;

            case USB_REQUEST_SET_CONFIGURATION:
            case USB_CH340_REQ_SERIAL_INIT:
            case USB_CH340_REQ_WRITE_REG:
            case USB_CH340_REQ_MODEM_CTRL:
              //Respond with 0 length packet
              EP0SendData(0, 0, 8);
              break;

            case USB_CH340_REQ_VENDOR_VERSION:
              EP0SendData((uint16_t *)vendorVersion, sizeof(vendorVersion), SETUPPACKET->wLength);
              break;

            case USB_CH340_REQ_READ_REG:
              if(SETUPPACKET->wValue.word == USB_CH340_READ_ATTACH)
                EP0SendData((uint16_t *)vendorAttach, sizeof(vendorAttach), SETUPPACKET->wLength);
              else if(SETUPPACKET->wValue.word == USB_CH340_READ_STATUS)
                EP0SendData((uint16_t *)vendorStatus, sizeof(vendorStatus), SETUPPACKET->wLength);
              break;
          }
        }

        //Signal we are ready to receive more data by preparing for toggling the needed RX status bits
        //Keep the other bits unaltered, by using the previous data or the invariant value
        USB->EP0R = ((USB->EP0R ^ USB_EP_RX_VALID) & (USB_EPREG_MASK | USB_EPRX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX;
      }

      if(USB->EP0R & USB_EP_CTR_TX)
      {
        //Reset the CTR TX bit by writing a zero to it
        USB->EP0R = USB_EP_CONTROL | USB_EP_CTR_RX;

        //Check if a set device address has been received
        if(DeviceAddress)
        {
          //Set the received address and keep the device enabled
          USB->DADDR = DeviceAddress  | USB_DADDR_EF;

          //Clear flag so address not set again
          DeviceAddress = 0;

          //Signal device is active
          DeviceConfigured = 1;
        }

        //Check if we need to send something
        if(EP0TxLen)
        {
          //TX means data send to the host
          //So check if more data needs to be send.
          uint32_t *dptr = (uint32_t *)EP0TX_BASE;
          uint8_t   len = EP0TxLen;

          //Check if more bytes to send then space available
          if(len > 8)
            len = 8;

          //Take of the number of bytes being send now
          EP0TxLen -= len;

          //Signal to the USB device how many bytes need to be send
          EPBTABLE->EPD[0].TX_COUNT.DATA = len;

          //Number of bytes needs to be divided by two because transfer is done two bytes at a time
          //Add one extra to allow for odd number of bytes to be handled
          len = (len + 1) / 2;

          //Copy the bytes to the PMA
          while(len--)
            *dptr++ = (uint32_t)*EP0TxPtr++;

          //Signal ready to transmit by preparing for toggling the needed TX status bits
          //Keep the other bits unaltered, by using the previous data or the invariant value
          USB->EP0R = ((USB->EP0R ^ USB_EP_TX_VALID) & (USB_EPREG_MASK | USB_EPTX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX;
        }
      }
    }
    else if(epid == 2)
    {
      //Endpoint 2 is used to handle the serial communication data

      //Check if data received
      if(USB->EP2R & USB_EP_CTR_RX)
      {
        //Reset the CTR RX bit by writing a zero to it
        USB->EP2R = USB_EP_BULK | USB_EP_CTR_TX | 0x0002;

        //Get the number of bytes to process
        uint8_t   cnt = EPBTABLE->EPD[2].RX_COUNT.DATA & 0x7F;
        uint32_t *dptr = (uint32_t *)EP2RX_BASE;
        uint16_t  data;

        //Copy the data from the PMA to the receive buffer
        while(cnt--)
        {
          //Get two bytes from the PMA
          data = *dptr++;

          //Put the first byte in the receive buffer
          usb_rx[usb_rx_in_idx++] = data & 0xFF;
          usb_rx_in_idx %= sizeof(usb_rx);

          //Check if more bytes need to be processed
          if(cnt)
          {
            //Put the second byte in the receive buffer
            usb_rx[usb_rx_in_idx++] = (data >> 8) & 0xFF;
            usb_rx_in_idx %= sizeof(usb_rx);
            cnt--;
          }
        }

        //Signal we are ready to receive more data by preparing for toggling the needed RX status bits
        //Keep the other bits unaltered, by using the previous data or the invariant value
        USB->EP2R = ((USB->EP2R ^ USB_EP_RX_VALID) & (USB_EPREG_MASK | USB_EPRX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX;
      }

      //Data has been send so check if we need to send some more
      if(USB->EP2R & USB_EP_CTR_TX)
      {
        //Reset the CTR TX bit by writing a zero to it
        USB->EP2R = USB_EP_BULK | USB_EP_CTR_RX | 0x0002;

        EP2SendData();
      }
    }
  }

  //Check if we received an USB reset
  if(USB->ISTR & USB_ISTR_RESET)
  {
    //Endpoint 0 is the control endpoint
    //Endpoint registers have toggle bits. These are toggled by writing a 1. It concerns the status bits and the dtog bits
    //They also have write 0 clear bits. These are the correct transfer bits. So cleared by writing a 0. Writing 1 has no effect
    //On receipt of an USB reset all endpoint registers are cleared, except for the two CTR bits.

    //Set the type of this endpoint to control, receive valid and stall transmission. Endpoint address is 0.
    USB->EP0R = USB_EP_CONTROL | USB_EP_RX_VALID | USB_EP_TX_NAK | USB_EP_CTR_RX | USB_EP_CTR_TX;

    //Setup send and receive buffer
    EPBTABLE->EPD[0].TX_ADDRESS.DATA = 0x0080;
    EPBTABLE->EPD[0].TX_COUNT.DATA = 0;
    EPBTABLE->EPD[0].RX_ADDRESS.DATA = 0x0040;
    EPBTABLE->EPD[0].RX_COUNT.DATA = USB_8_BYTE_RX_BUF;

    //Set endpoint 1 to interrupt and transmit nak and receive disabled. Endpoint address is 1;
    USB->EP1R = USB_EP_INTERRUPT | USB_EP_RX_DIS | USB_EP_TX_NAK | USB_EP_CTR_RX | USB_EP_CTR_TX | 0x0001;

    //Setup send and receive buffer. Receive is disabled so 0 byte buffer
    EPBTABLE->EPD[1].TX_ADDRESS.DATA = 0x0110;
    EPBTABLE->EPD[1].TX_COUNT.DATA = 0;
    EPBTABLE->EPD[1].RX_ADDRESS.DATA = 0x0000;
    EPBTABLE->EPD[1].RX_COUNT.DATA = 0;

    //Set endpoint 2 to bulk and transmit nak and receive valid. Endpoint address is 2
    USB->EP2R = USB_EP_BULK | USB_EP_RX_VALID | USB_EP_TX_NAK | USB_EP_CTR_RX | USB_EP_CTR_TX | 0x0002;

    //Setup send and receive buffer
    EPBTABLE->EPD[2].TX_ADDRESS.DATA = 0x00C0;
    EPBTABLE->EPD[2].TX_COUNT.DATA = 0;
    EPBTABLE->EPD[2].RX_ADDRESS.DATA = 0x0100;
    EPBTABLE->EPD[2].RX_COUNT.DATA = USB_32_BYTE_RX_BUF;

    //Allow some USB events to generate interrupts
    USB->CNTR = USB_CNTR_SOFM | USB_CNTR_RESETM | USB_CNTR_CTRM;

    //The endpoint buffer table is at the start of the PMA
    USB->BTABLE = BTABLE_ADDRESS;

    //Reset the device address and keep the device enabled
    USB->DADDR = USB_DADDR_EF;

    //Signal device is not active
    DeviceConfigured = 0;

    //Reset all interrupt flags
    USB->ISTR = 0;
  }
}

