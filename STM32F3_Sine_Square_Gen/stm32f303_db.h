#ifndef __STM32F303_H_
#define __STM32F303_H_

#include <stdint.h>

#define     __IO    volatile

#define     __IM     volatile const      //Defines 'read only' structure member permissions
#define     __OM     volatile            //Defines 'write only' structure member permissions
#define     __IOM    volatile            //Defines 'read / write' structure member permissions

//Power Control
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CSR;
} PWR_TypeDef;


//Reset and Clock Control
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
  __IO uint32_t AHBRSTR;
  __IO uint32_t CFGR2;
  __IO uint32_t CFGR3;
} RCC_TypeDef;

//FLASH Registers
typedef struct
{
  __IO uint32_t ACR;
  __IO uint32_t KEYR;
  __IO uint32_t OPTKEYR;
  __IO uint32_t SR;
  __IO uint32_t CR;
  __IO uint32_t AR;
  __IO uint32_t RESERVED;
  __IO uint32_t OBR;
  __IO uint32_t WRPR;
} FLASH_TypeDef;

//Real-Time Clock
typedef struct
{
  __IO uint32_t TR;
  __IO uint32_t DR;
  __IO uint32_t CR;
  __IO uint32_t ISR;
  __IO uint32_t PRER;
  __IO uint32_t WUTR;
       uint32_t RESERVED0;
  __IO uint32_t ALRMAR;
  __IO uint32_t ALRMBR;
  __IO uint32_t WPR;
  __IO uint32_t SSR;
  __IO uint32_t SHIFTR;
  __IO uint32_t TSTR;
  __IO uint32_t TSDR;
  __IO uint32_t TSSSR;
  __IO uint32_t CALR;
  __IO uint32_t TAFCR;
  __IO uint32_t ALRMASSR;
  __IO uint32_t ALRMBSSR;
       uint32_t RESERVED7;
  __IO uint32_t BKP0R;
  __IO uint32_t BKP1R;
  __IO uint32_t BKP2R;
  __IO uint32_t BKP3R;
  __IO uint32_t BKP4R;
  __IO uint32_t BKP5R;
  __IO uint32_t BKP6R;
  __IO uint32_t BKP7R;
  __IO uint32_t BKP8R;
  __IO uint32_t BKP9R;
  __IO uint32_t BKP10R;
  __IO uint32_t BKP11R;
  __IO uint32_t BKP12R;
  __IO uint32_t BKP13R;
  __IO uint32_t BKP14R;
  __IO uint32_t BKP15R;
} RTC_TypeDef;

//General Purpose I/O
typedef struct
{
  __IO uint32_t MODER;        //GPIO port mode register,               Address offset: 0x00
  __IO uint32_t OTYPER;       //GPIO port output type register,        Address offset: 0x04
  __IO uint32_t OSPEEDR;      //GPIO port output speed register,       Address offset: 0x08
  __IO uint32_t PUPDR;        //GPIO port pull-up/pull-down register,  Address offset: 0x0C
  __IO uint32_t IDR;          //GPIO port input data register,         Address offset: 0x10
  __IO uint32_t ODR;          //GPIO port output data register,        Address offset: 0x14
  __IO uint32_t BSRR;         //GPIO port bit set/reset register,      Address offset: 0x1A
  __IO uint32_t LCKR;         //GPIO port configuration lock register, Address offset: 0x1C
  __IO uint32_t AFR[2];       //GPIO alternate function registers,     Address offset: 0x20-0x24
  __IO uint32_t BRR;          //GPIO bit reset register,               Address offset: 0x28
} GPIO_TypeDef;

//TIM Timers
typedef struct
{
  __IO uint32_t CR1;             //TIM control register 1,                      Address offset: 0x00
  __IO uint32_t CR2;             //TIM control register 2,                      Address offset: 0x04
  __IO uint32_t SMCR;            //TIM slave Mode Control register,             Address offset: 0x08
  __IO uint32_t DIER;            //TIM DMA/interrupt enable register,           Address offset: 0x0C
  __IO uint32_t SR;              //TIM status register,                         Address offset: 0x10
  __IO uint32_t EGR;             //TIM event generation register,               Address offset: 0x14
  __IO uint32_t CCMR1;           //TIM  capture/compare mode register 1,        Address offset: 0x18
  __IO uint32_t CCMR2;           //TIM  capture/compare mode register 2,        Address offset: 0x1C
  __IO uint32_t CCER;            //TIM capture/compare enable register,         Address offset: 0x20
  __IO uint32_t CNT;             //TIM counter register,                        Address offset: 0x24
  __IO uint32_t PSC;             //TIM prescaler register,                      Address offset: 0x28
  __IO uint32_t ARR;             //TIM auto-reload register,                    Address offset: 0x2C
  __IO uint32_t RCR;             //TIM  repetition counter register,            Address offset: 0x30
  __IO uint32_t CCR1;            //TIM capture/compare register 1,              Address offset: 0x34
  __IO uint32_t CCR2;            //TIM capture/compare register 2,              Address offset: 0x38
  __IO uint32_t CCR3;            //TIM capture/compare register 3,              Address offset: 0x3C
  __IO uint32_t CCR4;            //TIM capture/compare register 4,              Address offset: 0x40
  __IO uint32_t BDTR;            //TIM break and dead-time register,            Address offset: 0x44
  __IO uint32_t DCR;             //TIM DMA control register,                    Address offset: 0x48
  __IO uint32_t DMAR;            //TIM DMA address for full transfer register,  Address offset: 0x4C
  __IO uint32_t OR;              //TIM option register,                         Address offset: 0x50
  __IO uint32_t CCMR3;           //TIM capture/compare mode register 3,         Address offset: 0x54
  __IO uint32_t CCR5;            //TIM capture/compare register5,               Address offset: 0x58
  __IO uint32_t CCR6;            //TIM capture/compare register 4,              Address offset: 0x5C
}TIM_TypeDef;

//External Interrupt/Event Controller
typedef struct
{
  __IO uint32_t IMR;
  __IO uint32_t EMR;
  __IO uint32_t RTSR;
  __IO uint32_t FTSR;
  __IO uint32_t SWIER;
  __IO uint32_t PR;
       uint32_t RESERVED1;
       uint32_t RESERVED2;
  __IO uint32_t IMR2;
  __IO uint32_t EMR2;
  __IO uint32_t RTSR2;
  __IO uint32_t FTSR2;
  __IO uint32_t SWIER2;
  __IO uint32_t PR2;
} EXTI_TypeDef;

//Structure type to access the System Control Block (SCB).
typedef struct
{
  __IM  uint32_t CPUID;                  //Offset: 0x000 (R/ )  CPUID Base Register
  __IOM uint32_t ICSR;                   //Offset: 0x004 (R/W)  Interrupt Control and State Register
  __IOM uint32_t VTOR;                   //Offset: 0x008 (R/W)  Vector Table Offset Register
  __IOM uint32_t AIRCR;                  //Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register
  __IOM uint32_t SCR;                    //Offset: 0x010 (R/W)  System Control Register
  __IOM uint32_t CCR;                    //Offset: 0x014 (R/W)  Configuration Control Register
  __IOM uint8_t  SHP[12U];               //Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15)
  __IOM uint32_t SHCSR;                  //Offset: 0x024 (R/W)  System Handler Control and State Register
  __IOM uint32_t CFSR;                   //Offset: 0x028 (R/W)  Configurable Fault Status Register
  __IOM uint32_t HFSR;                   //Offset: 0x02C (R/W)  HardFault Status Register
  __IOM uint32_t DFSR;                   //Offset: 0x030 (R/W)  Debug Fault Status Register
  __IOM uint32_t MMFAR;                  //Offset: 0x034 (R/W)  MemManage Fault Address Register
  __IOM uint32_t BFAR;                   //Offset: 0x038 (R/W)  BusFault Address Register
  __IOM uint32_t AFSR;                   //Offset: 0x03C (R/W)  Auxiliary Fault Status Register
  __IM  uint32_t PFR[2U];                //Offset: 0x040 (R/ )  Processor Feature Register
  __IM  uint32_t DFR;                    //Offset: 0x048 (R/ )  Debug Feature Register
  __IM  uint32_t ADR;                    //Offset: 0x04C (R/ )  Auxiliary Feature Register
  __IM  uint32_t MMFR[4U];               //Offset: 0x050 (R/ )  Memory Model Feature Register
  __IM  uint32_t ISAR[5U];               //Offset: 0x060 (R/ )  Instruction Set Attributes Register
        uint32_t RESERVED0[5U];
  __IOM uint32_t CPACR;                  //Offset: 0x088 (R/W)  Coprocessor Access Control Register
} SCB_Type;

//Structure type to access the Nested Vectored Interrupt Controller (NVIC)
typedef struct
{
  __IOM uint32_t ISER[8U];               //Offset: 0x000 (R/W)  Interrupt Set Enable Register
        uint32_t RESERVED0[24U];
  __IOM uint32_t ICER[8U];               //Offset: 0x080 (R/W)  Interrupt Clear Enable Register
        uint32_t RSERVED1[24U];
  __IOM uint32_t ISPR[8U];               //Offset: 0x100 (R/W)  Interrupt Set Pending Register
        uint32_t RESERVED2[24U];
  __IOM uint32_t ICPR[8U];               //Offset: 0x180 (R/W)  Interrupt Clear Pending Register
        uint32_t RESERVED3[24U];
  __IOM uint32_t IABR[8U];               //Offset: 0x200 (R/W)  Interrupt Active bit Register
        uint32_t RESERVED4[56U];
  __IOM uint8_t  IP[240U];               //Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide)
        uint32_t RESERVED5[644U];
  __OM  uint32_t STIR;                   //Offset: 0xE00 ( /W)  Software Trigger Interrupt Register
}  NVIC_Type;

//Structure typedef to access the System Tick Timer
typedef struct
{
  __IOM uint32_t CTRL;                   //Offset: 0x000 (R/W)  SysTick Control and Status Register
  __IOM uint32_t LOAD;                   //Offset: 0x004 (R/W)  SysTick Reload Value Register
  __IOM uint32_t VAL;                    //Offset: 0x008 (R/W)  SysTick Current Value Register
  __IM  uint32_t CALIB;                  //Offset: 0x00C (R/ )  SysTick Calibration Register
} STK_Type;

//DMA channel
typedef struct
{
  __IO uint32_t CCR;          //DMA channel x configuration register            
  __IO uint32_t CNDTR;        //DMA channel x number of data register           
  __IO uint32_t CPAR;         //DMA channel x peripheral address register       
  __IO uint32_t CMAR;         //DMA channel x memory address register           
} DMA_Channel_TypeDef;

//DMA
typedef struct
{
  __IO uint32_t ISR;          //DMA interrupt status register,                            Address offset: 0x00
  __IO uint32_t IFCR;         //DMA interrupt flag clear register,                        Address offset: 0x04
} DMA_TypeDef;

//Digital to Analog Converter
typedef struct
{
  __IO uint32_t CR;       //DAC control register,                                    Address offset: 0x00
  __IO uint32_t SWTRIGR;  //DAC software trigger register,                           Address offset: 0x04
  __IO uint32_t DHR12R1;  //DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08
  __IO uint32_t DHR12L1;  //DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C
  __IO uint32_t DHR8R1;   //DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10
  __IO uint32_t DHR12R2;  //DAC channel2 12-bit right aligned data holding register, Address offset: 0x14
  __IO uint32_t DHR12L2;  //DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18
  __IO uint32_t DHR8R2;   //DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C
  __IO uint32_t DHR12RD;  //Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20
  __IO uint32_t DHR12LD;  //DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24
  __IO uint32_t DHR8RD;   //DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28
  __IO uint32_t DOR1;     //DAC channel1 data output register,                       Address offset: 0x2C
  __IO uint32_t DOR2;     //DAC channel2 data output register,                       Address offset: 0x30
  __IO uint32_t SR;       //DAC status register,                                     Address offset: 0x34
} DAC_TypeDef;

//Analog to digital converter
typedef struct
{
  __IO uint32_t ISR;              //ADC Interrupt and Status Register,                 Address offset: 0x00
  __IO uint32_t IER;              //ADC Interrupt Enable Register,                     Address offset: 0x04
  __IO uint32_t CR;               //ADC control register,                              Address offset: 0x08
  __IO uint32_t CFGR;             //ADC Configuration register,                        Address offset: 0x0C
  uint32_t      RESERVED0;        //Reserved, 0x010                                                        
  __IO uint32_t SMPR1;            //ADC sample time register 1,                        Address offset: 0x14
  __IO uint32_t SMPR2;            //ADC sample time register 2,                        Address offset: 0x18
  uint32_t      RESERVED1;        //Reserved, 0x01C                                                        
  __IO uint32_t TR1;              //ADC watchdog threshold register 1,                 Address offset: 0x20
  __IO uint32_t TR2;              //ADC watchdog threshold register 2,                 Address offset: 0x24
  __IO uint32_t TR3;              //ADC watchdog threshold register 3,                 Address offset: 0x28
  uint32_t      RESERVED2;        //Reserved, 0x02C                                                        
  __IO uint32_t SQR1;             //ADC regular sequence register 1,                   Address offset: 0x30
  __IO uint32_t SQR2;             //ADC regular sequence register 2,                   Address offset: 0x34
  __IO uint32_t SQR3;             //ADC regular sequence register 3,                   Address offset: 0x38
  __IO uint32_t SQR4;             //ADC regular sequence register 4,                   Address offset: 0x3C
  __IO uint32_t DR;               //ADC regular data register,                         Address offset: 0x40
  uint32_t      RESERVED3;        //Reserved, 0x044                                                        
  uint32_t      RESERVED4;        //Reserved, 0x048                                                        
  __IO uint32_t JSQR;             //ADC injected sequence register,                    Address offset: 0x4C
  uint32_t      RESERVED5[4];     //Reserved, 0x050 - 0x05C                                                
  __IO uint32_t OFR1;             //ADC offset register 1,                             Address offset: 0x60
  __IO uint32_t OFR2;             //ADC offset register 2,                             Address offset: 0x64
  __IO uint32_t OFR3;             //ADC offset register 3,                             Address offset: 0x68
  __IO uint32_t OFR4;             //ADC offset register 4,                             Address offset: 0x6C
  uint32_t      RESERVED6[4];     //Reserved, 0x070 - 0x07C                                                
  __IO uint32_t JDR1;             //ADC injected data register 1,                      Address offset: 0x80
  __IO uint32_t JDR2;             //ADC injected data register 2,                      Address offset: 0x84
  __IO uint32_t JDR3;             //ADC injected data register 3,                      Address offset: 0x88
  __IO uint32_t JDR4;             //ADC injected data register 4,                      Address offset: 0x8C
  uint32_t      RESERVED7[4];     //Reserved, 0x090 - 0x09C                                                
  __IO uint32_t AWD2CR;           //ADC  Analog Watchdog 2 Configuration Register,     Address offset: 0xA0
  __IO uint32_t AWD3CR;           //ADC  Analog Watchdog 3 Configuration Register,     Address offset: 0xA4
  uint32_t      RESERVED8;        //Reserved, 0x0A8                                                        
  uint32_t      RESERVED9;        //Reserved, 0x0AC                                                        
  __IO uint32_t DIFSEL;           //ADC  Differential Mode Selection Register,         Address offset: 0xB0
  __IO uint32_t CALFACT;          //ADC  Calibration Factors,                          Address offset: 0xB4
} ADC_TypeDef;

//Common registers of an ADC pair
typedef struct
{
  __IO uint32_t CSR;              //ADC Common status register,                                  Address offset: ADC1/3 base address + 0x300
  uint32_t      RESERVED;         //Reserved,                                                    Address offset: ADC1/3 base address + 0x304                                                   
  __IO uint32_t CCR;              //ADC common control register,                                 Address offset: ADC1/3 base address + 0x308
  __IO uint32_t CDR;              //ADC common regular data register for dual and triple modes,  Address offset: ADC1/3 base address + 0x30C
} ADC_Common_TypeDef;

//Universal Serial Bus Full Speed Device
typedef struct
{
  __IO uint16_t EP0R;                 //USB Endpoint 0 register,         Address offset: 0x00
  __IO uint16_t RESERVED0;            //Reserved
  __IO uint16_t EP1R;                 //USB Endpoint 1 register,         Address offset: 0x04
  __IO uint16_t RESERVED1;            //Reserved
  __IO uint16_t EP2R;                 //USB Endpoint 2 register,         Address offset: 0x08
  __IO uint16_t RESERVED2;            //Reserved
  __IO uint16_t EP3R;                 //USB Endpoint 3 register,         Address offset: 0x0C
  __IO uint16_t RESERVED3;            //Reserved
  __IO uint16_t EP4R;                 //USB Endpoint 4 register,         Address offset: 0x10
  __IO uint16_t RESERVED4;            //Reserved
  __IO uint16_t EP5R;                 //USB Endpoint 5 register,         Address offset: 0x14
  __IO uint16_t RESERVED5;            //Reserved
  __IO uint16_t EP6R;                 //USB Endpoint 6 register,         Address offset: 0x18
  __IO uint16_t RESERVED6;            //Reserved
  __IO uint16_t EP7R;                 //USB Endpoint 7 register,         Address offset: 0x1C
  __IO uint16_t RESERVED7[17];        //Reserved
  __IO uint16_t CNTR;                 //Control register,                Address offset: 0x40
  __IO uint16_t RESERVED8;            //Reserved
  __IO uint16_t ISTR;                 //Interrupt status register,       Address offset: 0x44
  __IO uint16_t RESERVED9;            //Reserved
  __IO uint16_t FNR;                  //Frame number register,           Address offset: 0x48
  __IO uint16_t RESERVEDA;            //Reserved
  __IO uint16_t DADDR;                //Device address register,         Address offset: 0x4C
  __IO uint16_t RESERVEDB;            //Reserved
  __IO uint16_t BTABLE;               //Buffer Table address register,   Address offset: 0x50
  __IO uint16_t RESERVEDC;            //Reserved
} USB_TypeDef;

//Packet memory entry
typedef struct
{
  __IO uint16_t DATA;
       uint16_t RESERVED;
}PMA_ENTRY;

//Structure for PMA
typedef struct
{
  PMA_ENTRY MEMORY[256];
} PMA_TypeDef;

//Structure for Buffer Descriptor Table
typedef struct
{
  PMA_ENTRY TX_ADDRESS;
  PMA_ENTRY TX_COUNT;
  PMA_ENTRY RX_ADDRESS;
  PMA_ENTRY RX_COUNT;
} BTABLE_ENTRY;

//Structure for endpoint descriptors
typedef struct
{
  BTABLE_ENTRY EPD[8];
} BTABLE_TypeDef;

typedef union
{
  uint16_t word;
  struct BYTES
  {
    uint8_t low;
    uint8_t high;
  } bytes;
} wbcombi;


#define PMA_BASE    (0x40006000L)  //USB_IP Packet Memory Area base address

#define PMA         ((PMA_TypeDef *) PMA_BASE)
#define EPBTABLE    ((BTABLE_TypeDef *) PMA_BASE)


#define RCC_BASE    0x40021000
#define PWR_BASE    0x40007000
#define RTC_BASE    0x40002800
#define EXTI_BASE   0x40010400

#define USB_BASE    0x40005C00

#define GPIOA_BASE  0x48000000
#define GPIOB_BASE  0x48000400
#define GPIOC_BASE  0x48000800

#define DMA1_BASE             0x40020000
#define DMA1_Channel1_BASE    0x40020008
#define DMA1_Channel2_BASE    0x4002001C
#define DMA1_Channel3_BASE    0x40020030
#define DMA1_Channel4_BASE    0x40020044
#define DMA1_Channel5_BASE    0x40020058
#define DMA1_Channel6_BASE    0x4002006C
#define DMA1_Channel7_BASE    0x40020080
#define DMA2_BASE             0x40020400
#define DMA2_Channel1_BASE    0x40020408
#define DMA2_Channel2_BASE    0x4002041C
#define DMA2_Channel3_BASE    0x40020430
#define DMA2_Channel4_BASE    0x40020444
#define DMA2_Channel5_BASE    0x40020458

#define ADC1_BASE              0x50000000
#define ADC2_BASE              0x50000100
#define ADC1_2_COMMON_BASE     0x50000300
#define ADC3_BASE              0x50000400
#define ADC4_BASE              0x50000500
#define ADC3_4_COMMON_BASE     0x50000700

#define DAC1_BASE   0x40007400

#define TIM1_BASE   0x40012C00

#define TIM2_BASE   0x40000000
#define TIM3_BASE   0x40000400
#define TIM4_BASE   0x40000800

#define USART_BASE  0x40013800

#define NVIC_BASE   0xE000E100
#define SCB_BASE    0xE000ED00
#define STK_BASE    0xE000E010

#define FLASH_BASE  0x40022000

#define FLASH_START 0x08000000





#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE)

#define TIM1                ((TIM_TypeDef *)TIM1_BASE)
#define TIM2                ((TIM_TypeDef *)TIM2_BASE)
#define TIM3                ((TIM_TypeDef *)TIM3_BASE)
#define TIM4                ((TIM_TypeDef *)TIM4_BASE)

#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Channel1       ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Channel1       ((DMA_Channel_TypeDef *) DMA2_Channel1_BASE)
#define DMA2_Channel2       ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE)
#define DMA2_Channel3       ((DMA_Channel_TypeDef *) DMA2_Channel3_BASE)
#define DMA2_Channel4       ((DMA_Channel_TypeDef *) DMA2_Channel4_BASE)
#define DMA2_Channel5       ((DMA_Channel_TypeDef *) DMA2_Channel5_BASE)

#define DAC1                ((DAC_TypeDef *)DAC1_BASE)

#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define ADC3                ((ADC_TypeDef *) ADC3_BASE)
#define ADC4                ((ADC_TypeDef *) ADC4_BASE)
#define ADC12_COMMON        ((ADC_Common_TypeDef *) ADC1_2_COMMON_BASE)
#define ADC34_COMMON        ((ADC_Common_TypeDef *) ADC3_4_COMMON_BASE)

#define PWR                 ((PWR_TypeDef *)PWR_BASE)
#define RCC                 ((RCC_TypeDef *)RCC_BASE)
#define USB                 ((USB_TypeDef *)USB_BASE)
#define FLASH               ((FLASH_TypeDef *)FLASH_BASE)
#define RTC                 ((RTC_TypeDef *)RTC_BASE)
#define EXTI                ((EXTI_TypeDef *)EXTI_BASE)
#define SCB                 ((SCB_Type *)SCB_BASE)
#define NVIC                ((NVIC_Type *)NVIC_BASE)
#define STK                 ((STK_Type *)STK_BASE)


#define DMA1_CH3_IRQn         13
#define USB_LP_CAN1_RX0_IRQn  20
#define TIM1_UP_IRQn          25
#define TIM2_IRQn             28
#define TIM3_IRQn             29


#define FLASH_ACR_LATENCY_2   0x00000004
#define FLASH_ACR_PRFTBE      0x00000010               //Prefetch Buffer Enable

#define RCC_CFGR_PLLMULL9     0x001C0000               //PLL input clock*9
#define RCC_CFGR_PLLSRC       0x00010000               //PLL entry clock source
#define RCC_CFGR_PPRE1_DIV2   0x00000400               //HCLK divided by 2
#define RCC_CFGR_PPRE2_DIV2   0x00002000               //HCLK divided by 2

#define RCC_CR_HSEON          0x00010000               //External High Speed clock enable
#define RCC_CR_HSERDY         0x00020000               //External High Speed clock ready flag

#define RCC_CR_PLLON          0x01000000               //PLL enable
#define RCC_CR_PLLRDY         0x02000000               //PLL clock ready flag

#define RCC_CFGR_SW_PLL       0x00000002               //PLL selected as system clock
#define RCC_CFGR_SWS_PLL      0x00000008               //PLL used as system clock

// Bit definition for RCC_AHBENR register
#define RCC_AHBENR_DMA1EN                        0x00000001         //DMA1 clock enable
#define RCC_AHBENR_DMA2EN                        0x00000002         //DMA2 clock enable
#define RCC_AHBENR_SRAMEN                        0x00000004         //SRAM interface clock enable
#define RCC_AHBENR_FLITFEN                       0x00000010         //FLITF clock enable
#define RCC_AHBENR_CRCEN                         0x00000040         //CRC clock enable
#define RCC_AHBENR_GPIOAEN                       0x00020000         //GPIOA clock enable
#define RCC_AHBENR_GPIOBEN                       0x00040000         //GPIOB clock enable
#define RCC_AHBENR_GPIOCEN                       0x00080000         //GPIOC clock enable
#define RCC_AHBENR_GPIODEN                       0x00100000         //GPIOD clock enable
#define RCC_AHBENR_GPIOEEN                       0x00200000         //GPIOE clock enable
#define RCC_AHBENR_GPIOFEN                       0x00400000         //GPIOF clock enable
#define RCC_AHBENR_TSCEN                         0x01000000         //TS clock enable
#define RCC_AHBENR_ADC12EN                       0x10000000         //ADC1 & ADC2 clock enable
#define RCC_AHBENR_ADC34EN                       0x20000000         //ADC3 & ADC4 clock enable

// Bit definition for RCC_APB2ENR register
#define RCC_APB2ENR_SYSCFGEN                     0x00000001        //SYSCFG clock enable
#define RCC_APB2ENR_TIM1EN                       0x00000800        //TIM1 clock enable
#define RCC_APB2ENR_SPI1EN                       0x00001000        //SPI1 clock enable
#define RCC_APB2ENR_TIM8EN                       0x00002000        //TIM8 clock enable
#define RCC_APB2ENR_USART1EN                     0x00004000        //USART1 clock enable
#define RCC_APB2ENR_TIM15EN                      0x00010000        //TIM15 clock enable
#define RCC_APB2ENR_TIM16EN                      0x00020000        //TIM16 clock enable
#define RCC_APB2ENR_TIM17EN                      0x00040000        //TIM17 clock enable

//Bit definition for RCC_APB1ENR register
#define RCC_APB1ENR_TIM2EN                       0x00000001        //Timer 2 clock enable
#define RCC_APB1ENR_TIM3EN                       0x00000002        //Timer 3 clock enable
#define RCC_APB1ENR_TIM4EN                       0x00000004        //Timer 4 clock enable
#define RCC_APB1ENR_TIM6EN                       0x00000010        //Timer 6 clock enable
#define RCC_APB1ENR_TIM7EN                       0x00000020        //Timer 7 clock enable
#define RCC_APB1ENR_WWDGEN                       0x00000800        //Window Watchdog clock enable
#define RCC_APB1ENR_SPI2EN                       0x00004000        //SPI2 clock enable
#define RCC_APB1ENR_SPI3EN                       0x00008000        //SPI3 clock enable
#define RCC_APB1ENR_USART2EN                     0x00020000        //USART 2 clock enable
#define RCC_APB1ENR_USART3EN                     0x00040000        //USART 3 clock enable
#define RCC_APB1ENR_UART4EN                      0x00080000        //UART 4 clock enable
#define RCC_APB1ENR_UART5EN                      0x00100000        //UART 5 clock enable
#define RCC_APB1ENR_I2C1EN                       0x00200000        //I2C 1 clock enable
#define RCC_APB1ENR_I2C2EN                       0x00400000        //I2C 2 clock enable
#define RCC_APB1ENR_USBEN                        0x00800000        //USB clock enable
#define RCC_APB1ENR_CANEN                        0x02000000        //CAN clock enable
#define RCC_APB1ENR_PWREN                        0x10000000        //PWR clock enable
#define RCC_APB1ENR_DAC1EN                       0x20000000        //DAC 1 clock enable


#define PWR_CR_DBP                           0x00000100              //Disable Backup Domain write protection

//Bit definition for RCC_CSR register
#define RCC_CSR_LSION                        0x00000001              //Internal Low Speed oscillator enable
#define RCC_CSR_LSIRDY                       0x00000002              //Internal Low Speed oscillator Ready
#define RCC_CSR_RMVF                         0x01000000              //Remove reset flag
#define RCC_CSR_PINRSTF                      0x04000000              //PIN reset flag
#define RCC_CSR_PORRSTF                      0x08000000              //POR/PDR reset flag
#define RCC_CSR_SFTRSTF                      0x10000000              //Software Reset flag
#define RCC_CSR_IWDGRSTF                     0x20000000              //Independent Watchdog reset flag
#define RCC_CSR_WWDGRSTF                     0x40000000              //Window watchdog reset flag
#define RCC_CSR_LPWRRSTF                     0x80000000              //Low-Power reset flag

//Bit definition for RCC_BDCR register
#define RCC_BDCR_LSEON                       0x00000001              //External Low Speed oscillator enable
#define RCC_BDCR_LSERDY                      0x00000002              //External Low Speed oscillator Ready
#define RCC_BDCR_LSEBYP                      0x00000004              //External Low Speed oscillator Bypass

//RTC configuration
#define RCC_BDCR_RTCSEL                      0x00000300              //RTCSEL[1:0] bits (RTC clock source selection)
#define RCC_BDCR_RTCSEL_NOCLOCK              0x00000000              //No clock
#define RCC_BDCR_RTCSEL_LSE                  0x00000100              //LSE oscillator clock used as RTC clock
#define RCC_BDCR_RTCSEL_LSI                  0x00000200              //LSI oscillator clock used as RTC clock
#define RCC_BDCR_RTCSEL_HSE                  0x00000300              //HSE oscillator clock divided by 128 used as RTC clock

#define RCC_BDCR_RTCEN                       0x00008000              //RTC clock enable
#define RCC_BDCR_BDRST                       0x00010000              //Backup domain software reset

#define RTC_CRL_RSF                          0x00000008               //Registers Synchronized Flag
#define RTC_CRL_CNF                          0x00000010               //Configuration Flag
#define RTC_CRL_RTOFF                        0x00000020               //RTC operation OFF


#define TIM_CCER_CC1E                        0x00000001               //Capture/Compare 1 output enable
#define TIM_CCER_CC1P                        0x00000002               //Capture/Compare 1 output Polarity

#define TIM_CCER_CC2E                        0x00000010               //Capture/Compare 2 output enable
#define TIM_CCER_CC2P                        0x00000020               //Capture/Compare 2 output Polarity

#define TIM_CCER_CC3E                        0x00000100               //Capture/Compare 3 output enable
#define TIM_CCER_CC3P                        0x00000200               //Capture/Compare 3 output Polarity

#define TIM_CCER_CC4E                        0x00001000               //Capture/Compare 4 output enable
#define TIM_CCER_CC4P                        0x00002000               //Capture/Compare 4 output Polarity


#define TIM_CR2_MMS                          0x00000070               //MMS[2:0] bits (Master Mode Selection)
#define TIM_CR2_MMS_0                        0x00000010
#define TIM_CR2_MMS_1                        0x00000020
#define TIM_CR2_MMS_2                        0x00000040

#define TIM_CR1_CEN                          0x00000001                //Counter enable

#define TIM_CR1_ARPE                         0x00000080                //Auto-reload preload enable

#define TIM_CR1_CMS                          0x00000060                //CMS[1:0] bits (Center-aligned mode selection)
#define TIM_CR1_CMS_0                        0x00000020
#define TIM_CR1_CMS_1                        0x00000040



#define TIM_SMCR_SMS                         0x00010007                //SMS[2:0] bits (Slave mode selection)
#define TIM_SMCR_SMS_0                       0x00000001
#define TIM_SMCR_SMS_1                       0x00000002
#define TIM_SMCR_SMS_2                       0x00000004
#define TIM_SMCR_SMS_3                       0x00010000

#define TIM_CCMR1_OC1PE                      0x00000008               //Output Compare 1 Preload enable

#define TIM_CCMR1_OC1M                       0x00010070                //OC1M[2:0] bits (Output Compare 1 Mode)
#define TIM_CCMR1_OC1M_0                     0x00000010
#define TIM_CCMR1_OC1M_1                     0x00000020
#define TIM_CCMR1_OC1M_2                     0x00000040
#define TIM_CCMR1_OC1M_3                     0x00010000


#define TIM_CCMR1_OC2M                       0x01007000                //OC2M[2:0] bits (Output Compare 2 Mode)
#define TIM_CCMR1_OC2M_0                     0x00001000
#define TIM_CCMR1_OC2M_1                     0x00002000
#define TIM_CCMR1_OC2M_2                     0x00004000
#define TIM_CCMR1_OC2M_3                     0x01000000


#define TIM_CCMR2_OC3M                       0x00010070                //OC1M[2:0] bits (Output Compare 3 Mode)
#define TIM_CCMR2_OC3M_0                     0x00000010
#define TIM_CCMR2_OC3M_1                     0x00000020
#define TIM_CCMR2_OC3M_2                     0x00000040
#define TIM_CCMR2_OC3M_3                     0x00010000


#define TIM_CCMR2_OC4M                       0x01007000                //OC2M[2:0] bits (Output Compare 4 Mode)
#define TIM_CCMR2_OC4M_0                     0x00001000
#define TIM_CCMR2_OC4M_1                     0x00002000
#define TIM_CCMR2_OC4M_2                     0x00004000
#define TIM_CCMR2_OC4M_3                     0x01000000


#define TIM_BDTR_MOE                         0x00008000                //Main Output enable

#define TIM_DIER_UIE                         0x00000001                //Update interrupt enable

#define TIM_DIER_UDE                         0x00000100                //Update DMA request enable

#define TIM_DIER_CC1IE                       0x00000002                //Capture/Compare 1 interrupt enable


#define TIM_SR_CC1IF                         0x00000002                //Capture/Compare 1 interrupt Flag


#define AFIO_MAPR_TIM3_REMAP_1               0x00000800                //Timer3 pins on alternate function pins

#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE        0x02000000                //JTAG-DP Disabled and SW-DP Enabled


#define AIRCR_VECTKEY_MASK      ((uint32_t)0x05FA0000)

#define EXTI_Line18             ((uint32_t)0x40000)  //External interrupt line 18 Connected to the USB Device/USB OTG FS

#define NVIC_PriorityGroup_2    ((uint32_t)0x500)    //2 bits for pre-emption priority, 2 bits for subpriority

#define STK_CTRL_ENABLE       0x00000001               //Enable the system timer
#define STK_CTRL_INT_ENABLE   0x00000002               //Enable the system timer interrupt
#define STK_CTRL_CLK_CPU      0x00000004               //Select the cpu clock as system timer input clock
#define STK_CTRL_OVERFLOW     0x00010000               //Overflow flag


#define USB_CNTR_SOFM                           0x00000200              //Start Of Frame Interrupt Mask
#define USB_CNTR_RESETM                         0x00000400              //RESET Interrupt Mask
#define USB_CNTR_SUSPM                          0x00000800              //Suspend mode Interrupt Mask
#define USB_CNTR_CTRM                           0x00008000              //Correct Transfer Interrupt Mask

#define USB_EP_TX_VALID                         0x00000030              //EndPoint TX VALID
#define USB_EP_TX_NAK                           0x00000020              //EndPoint TX NAKed

#define USB_EP_CTR_RX                           0x00008000              //EndPoint Correct TRansfer RX
#define USB_EPRX_STAT                           0x00003000              //EndPoint RX STATus bit field
#define USB_EP_SETUP                            0x00000800              //EndPoint SETUP
#define USB_EP_T_FIELD                          0x00000600              //EndPoint TYPE
#define USB_EP_KIND                             0x00000100              //EndPoint KIND
#define USB_EP_CTR_TX                           0x00000080              //EndPoint Correct TRansfer TX
#define USB_EPTX_STAT                           0x00000030              //EndPoint TX STATus bit field
#define USB_EPADDR_FIELD                        0x0000000F              //EndPoint ADDRess FIELD

#define  USB_EPREG_MASK                      (USB_EP_CTR_RX | USB_EP_SETUP | USB_EP_T_FIELD | USB_EP_KIND | USB_EP_CTR_TX | USB_EPADDR_FIELD)

#define USB_ISTR_EP_ID                          0x0000000F               //Endpoint Identifier
#define USB_ISTR_ESOF                           0x00000100               //Expected Start Of Frame
#define USB_ISTR_SOF                            0x00000200               //Start Of Frame
#define USB_ISTR_RESET                          0x00000400               //USB RESET request
#define USB_ISTR_SUSP                           0x00000800               //Suspend mode request
#define USB_ISTR_WKUP                           0x00001000               //Wake up
#define USB_ISTR_ERR                            0x00002000               //Error
#define USB_ISTR_PMAOVR                         0x00004000               //Packet Memory Area Over / Underrun
#define USB_ISTR_CTR                            0x00008000               //Correct Transfer

#define USB_EP_BULK                             0x00000000               //EndPoint BULK
#define USB_EP_CONTROL                          0x00000200               //EndPoint CONTROL
#define USB_EP_INTERRUPT                        0x00000600               //EndPoint INTERRUPT

#define USB_EP_RX_DIS                           0x00000000               //EndPoint RX DISabled
#define USB_EP_RX_VALID                         0x00003000               //EndPoint RX VALID

#define USB_DADDR_EF                            0x00000080               //Enable Function






//Bit definition for DAC_CR register
#define DAC_CR_EN1                  0x00000001                            //DAC channel1 enable
#define DAC_CR_BOFF1                0x00000002                            //DAC channel1 output buffer disable
#define DAC_CR_TEN1                 0x00000004                            //DAC channel1 Trigger enable

#define DAC_CR_TSEL1                0x00000038                            //TSEL1[2:0] (DAC channel1 Trigger selection)
#define DAC_CR_TSEL1_0              0x00000008
#define DAC_CR_TSEL1_1              0x00000010
#define DAC_CR_TSEL1_2              0x00000020

#define DAC_CR_TSEL1_SOFTWARE       0x00000038


#define DAC_CR_WAVE1                0x000000C0                            //WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable)
#define DAC_CR_WAVE1_0              0x00000040
#define DAC_CR_WAVE1_1              0x00000080

#define DAC_CR_MAMP1                0x00000F00                            //MAMP1[3:0] (DAC channel1 Mask/Amplitude selector)
#define DAC_CR_MAMP1_0              0x00000100
#define DAC_CR_MAMP1_1              0x00000200
#define DAC_CR_MAMP1_2              0x00000400
#define DAC_CR_MAMP1_3              0x00000800

#define DAC_CR_DMAEN1               0x00001000                            //DAC channel1 DMA enable
#define DAC_CR_DMAUDRIE1            0x00002000                            //DAC channel1 DMA underrun IT enable 
#define DAC_CR_EN2                  0x00010000                            //DAC channel2 enable
#define DAC_CR_BOFF2                0x00020000                            //DAC channel2 output buffer disable
#define DAC_CR_TEN2                 0x00040000                            //DAC channel2 Trigger enable

#define DAC_CR_TSEL2                0x00380000                            //TSEL2[2:0] (DAC channel2 Trigger selection)
#define DAC_CR_TSEL2_0              0x00080000
#define DAC_CR_TSEL2_1              0x00100000
#define DAC_CR_TSEL2_2              0x00200000

#define DAC_CR_WAVE2                0x00C00000                            //WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable)
#define DAC_CR_WAVE2_0              0x00400000
#define DAC_CR_WAVE2_1              0x00800000

#define DAC_CR_MAMP2                0x0F000000                            //MAMP2[3:0] (DAC channel2 Mask/Amplitude selector)
#define DAC_CR_MAMP2_0              0x01000000
#define DAC_CR_MAMP2_1              0x02000000
#define DAC_CR_MAMP2_2              0x04000000
#define DAC_CR_MAMP2_3              0x08000000

#define DAC_CR_DMAEN2               0x10000000                            //DAC channel2 DMA enabled
#define DAC_CR_DMAUDRIE2            0x20000000                            //DAC channel2 DMA underrun IT enable 

//Bit definition for DAC_SWTRIGR register
#define DAC_SWTRIGR_SWTRIG1         0x00000001                            //DAC channel1 software trigger
#define DAC_SWTRIGR_SWTRIG2         0x00000002                            //DAC channel2 software trigger

//Bit definition for DAC_DHR12R1 register
#define DAC_DHR12R1_DACC1DHR        0x00000FFF                            //DAC channel1 12-bit Right aligned data

//Bit definition for DAC_DHR12L1 register
#define DAC_DHR12L1_DACC1DHR        0x0000FFF0                            //DAC channel1 12-bit Left aligned data

//Bit definition for DAC_DHR8R1 register
#define DAC_DHR8R1_DACC1DHR         0x000000FF                            //DAC channel1 8-bit Right aligned data

//Bit definition for DAC_DHR12R2 register
#define DAC_DHR12R2_DACC2DHR        0x00000FFF                            //DAC channel2 12-bit Right aligned data

//Bit definition for DAC_DHR12L2 register
#define DAC_DHR12L2_DACC2DHR        0x0000FFF0                            //DAC channel2 12-bit Left aligned data

//Bit definition for DAC_DHR8R2 register
#define DAC_DHR8R2_DACC2DHR         0x000000FF                            //DAC channel2 8-bit Right aligned data

//Bit definition for DAC_DHR12RD register
#define DAC_DHR12RD_DACC1DHR        0x00000FFF                            //DAC channel1 12-bit Right aligned data
#define DAC_DHR12RD_DACC2DHR        0x0FFF0000                            //DAC channel2 12-bit Right aligned data

//Bit definition for DAC_DHR12LD register
#define DAC_DHR12LD_DACC1DHR        0x0000FFF0                            //DAC channel1 12-bit Left aligned data
#define DAC_DHR12LD_DACC2DHR        0xFFF00000                            //DAC channel2 12-bit Left aligned data

//Bit definition for DAC_DHR8RD register
#define DAC_DHR8RD_DACC1DHR         0x000000FF                            //DAC channel1 8-bit Right aligned data
#define DAC_DHR8RD_DACC2DHR         0x0000FF00                            //DAC channel2 8-bit Right aligned data

//Bit definition for DAC_DOR1 register
#define DAC_DOR1_DACC1DOR           0x00000FFF                            //DAC channel1 data output

//Bit definition for DAC_DOR2 register
#define DAC_DOR2_DACC2DOR           0x00000FFF                            //DAC channel2 data output

//Bit definition for DAC_SR register
#define DAC_SR_DMAUDR1              0x00002000                            //DAC channel1 DMA underrun flag
#define DAC_SR_DMAUDR2              0x20000000                            //DAC channel2 DMA underrun flag

//ADC defines

#define ADC_ISR_ADRDY               0x00000001                            //ADC ready flag



#define ADC_CR_ADEN                 0x00000001                            //ADC enable

#define ADC_CR_ADCAL                0x80000000                            //ADC calibration

#define ADC_CR_ADVREG_INTERMEDIATE  0x00000000                            //Set ADC voltage regulator to intermediate state
#define ADC_CR_ADVREG_ON            0x10000000                            //Set ADC voltage regulator to on state
#define ADC_CR_ADVREG_OFF           0x20000000                            //Set ADC voltage regulator to off state

#define ADC_SQR1_SQ1_POS            6                                     //Position of lsb for first serquence channel selector
#define ADC_SQR1_SQ1_MSK            0x000007C0                            //Bit mask for first serquence channel selector


//Sample times are in adc clock cycles plus one half cycle. (So ADC_SAMPLE_TIME_7 means 7.5 adc clock cycles)
#define ADC_SAMPLE_TIME_1           0
#define ADC_SAMPLE_TIME_2           1
#define ADC_SAMPLE_TIME_4           2
#define ADC_SAMPLE_TIME_7           3
#define ADC_SAMPLE_TIME_19          4
#define ADC_SAMPLE_TIME_61          5
#define ADC_SAMPLE_TIME_181         6
#define ADC_SAMPLE_TIME_601         7

#define ADC_SMPR1_CH3_POS           9                                     //Position of lsb for third channel sample time setting
#define ADC_SMPR1_CH3_MSK           0x00000E00                            //Bit mask for third channel sample time setting


#define ADC_CCR_CKMODE_ASYNC        0x00000000                            //Asynchronous clock mode
#define ADC_CCR_CKMODE_HCLK_1       0x00010000                            //Synchronous clock mode, HCLK / 1
#define ADC_CCR_CKMODE_HCLK_2       0x00020000                            //Synchronous clock mode, HCLK / 2
#define ADC_CCR_CKMODE_HCLK_4       0x00030000                            //Synchronous clock mode, HCLK / 4

#define ADC_CCR_MDMA_OFF            0x00000000                            //DMA in dual mode disabled
#define ADC_CCR_MDMA_10_12_BIT      0x00008000                            //DMA in dual mode set for 10 or 12 bits
#define ADC_CCR_MDMA_6_8_BIT        0x0000C000                            //DMA in dual mode set for 6 or 8 bits

#define ADC_CCR_DMACFG_CIRCULAIR    0x00002000                            //DMA in circulair mode

#define ADC_CCR_MULTI_RS            0x00000006                            //Multi mode regulair simultaneous only


//DMA definitions
#define DMA_CCR_EN                  0x00000001                            //Channel enable
#define DMA_CCR_TCIE                0x00000002                            //Transfer complete interrupt enable
#define DMA_CCR_HTIE                0x00000004                            //Half Transfer interrupt enable
#define DMA_CCR_TEIE                0x00000008                            //Transfer error interrupt enable
#define DMA_CCR_DIR                 0x00000010                            //Data transfer direction
#define DMA_CCR_CIRC                0x00000020                            //Circular mode
#define DMA_CCR_PINC                0x00000040                            //Peripheral increment mode
#define DMA_CCR_MINC                0x00000080                            //Memory increment mode

#define DMA_CCR_PSIZE               0x00000300                            //PSIZE[1:0] bits (Peripheral size)
#define DMA_CCR_PSIZE_0             0x00000100
#define DMA_CCR_PSIZE_1             0x00000200

#define DMA_CCR_MSIZE               0x00000C00                            //MSIZE[1:0] bits (Memory size)
#define DMA_CCR_MSIZE_0             0x00000400
#define DMA_CCR_MSIZE_1             0x00000800

#define DMA_CCR_PL                  0x00003000                            //PL[1:0] bits(Channel Priority level)
#define DMA_CCR_PL_0                0x00001000
#define DMA_CCR_PL_1                0x00002000

#define DMA_CCR_MEM2MEM             0x00004000                            //Memory to memory mode




//Bitmap for the gpio settings
//Bit 0:1   Mode
//Bit 4     Output type
//Bit 8:9   Output speed
//Bit 12:13 Pull-up pull-down settings

#define GPIO_MODE_MASK                    0x00000003
#define GPIO_TYPE_MASK                    0x00000010
#define GPIO_SPEED_MASK                   0x00000300
#define GPIO_PUPD_MASK                    0x00003000

//General purpose output options
//Push pull
#define GPIO_OUTPUT_PP_LOW_SPEED          0x00000001
#define GPIO_OUTPUT_PP_MEDIUM_SPEED       0x00000101
#define GPIO_OUTPUT_PP_HIGH_SPEED         0x00000301

//Push pull with pull up
#define GPIO_OUTPUT_PP_PU_LOW_SPEED       0x00001001
#define GPIO_OUTPUT_PP_PU_MEDIUM_SPEED    0x00001101
#define GPIO_OUTPUT_PP_PU_HIGH_SPEED      0x00001301

//Push pull with pull down
#define GPIO_OUTPUT_PP_PD_LOW_SPEED       0x00002001
#define GPIO_OUTPUT_PP_PD_MEDIUM_SPEED    0x00002101
#define GPIO_OUTPUT_PP_PD_HIGH_SPEED      0x00002301

//Open drain
#define GPIO_OUTPUT_OD_LOW_SPEED          0x00000011
#define GPIO_OUTPUT_OD_MEDIUM_SPEED       0x00000111
#define GPIO_OUTPUT_OD_HIGH_SPEED         0x00000311

//Open drain with pull up
#define GPIO_OUTPUT_OD_PU_LOW_SPEED       0x00001011
#define GPIO_OUTPUT_OD_PU_MEDIUM_SPEED    0x00001111
#define GPIO_OUTPUT_OD_PU_HIGH_SPEED      0x00001311

//Open drain with pull down
#define GPIO_OUTPUT_OD_PD_LOW_SPEED       0x00002011
#define GPIO_OUTPUT_OD_PD_MEDIUM_SPEED    0x00002111
#define GPIO_OUTPUT_OD_PD_HIGH_SPEED      0x00002311

//Alternate function options
//Push pull
#define GPIO_AF_PP_LOW_SPEED              0x00000002
#define GPIO_AF_PP_MEDIUM_SPEED           0x00000102
#define GPIO_AF_PP_HIGH_SPEED             0x00000302

//Push pull with pull up
#define GPIO_AF_PP_PU_LOW_SPEED           0x00001002
#define GPIO_AF_PP_PU_MEDIUM_SPEED        0x00001102
#define GPIO_AF_PP_PU_HIGH_SPEED          0x00001302

//Push pull with pull down
#define GPIO_AF_PP_PD_LOW_SPEED           0x00002002
#define GPIO_AF_PP_PD_MEDIUM_SPEED        0x00002102
#define GPIO_AF_PP_PD_HIGH_SPEED          0x00002302

//Open drain
#define GPIO_AF_OD_LOW_SPEED              0x00000012
#define GPIO_AF_OD_MEDIUM_SPEED           0x00000112
#define GPIO_AF_OD_HIGH_SPEED             0x00000312

//Open drain with pull up
#define GPIO_AF_OD_PU_LOW_SPEED           0x00001012
#define GPIO_AF_OD_PU_MEDIUM_SPEED        0x00001112
#define GPIO_AF_OD_PU_HIGH_SPEED          0x00001312

//Open drain with pull down
#define GPIO_AF_OD_PD_LOW_SPEED           0x00002012
#define GPIO_AF_OD_PD_MEDIUM_SPEED        0x00002112
#define GPIO_AF_OD_PD_HIGH_SPEED          0x00002312

//General purpose input options
#define GPIO_INPUT_FLOAT                  0x00000000
#define GPIO_INPUT_PULLUP                 0x00001000
#define GPIO_INPUT_PULLDOWN               0x00002000

//Analog option
#define GPIO_ANALOG                       0x00000003


#endif


