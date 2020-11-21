//Run openocd for debugging
//openocd -f ~/NetBeansProjects/STM32/F303/STM_Sine_Square_Gen/STM32F303CBT6.cfg
//openocd -f ~/Data/NetbeansProjects/STM32/F303/STM32F3_Sine_Square_Gen/STM32F303CBT6.cfg

#include "stm32f303_db.h"
#include "usb.h"
#include "sintab.h"

//STM3F303 has:
//  a different clock configuration and other AHB and ABP bus setup
//  enabling of peripherals is done in the same way, but differ in what is connected to which bus
//  a different GPIO setup
//  a different way of moving alternate functions for the pins. AFIO is SYSCFG and is used to remap functions not pins
//  the ADC's are different in configuration and usage
//  the xB device has 32KB of standard RAM and 8KB of core coupled memory
//  the xC device has 40KB of standard RAM and 8KB of core coupled memory

//The top of stack is set to the address right after the end of the internal RAM
//For the xB version this is at 0x20007D00 (32KB)
//For the xC version this is at 0x20009C40 (40KB)
#define STACK_TOP 0x20007D00

//Simple embedded systems can do without a separate heap.
//This means not using standard libraries
//Memory is statically declared and used.
//To allow for a proper working leave memory for the stack to grow so do not use more then RAM size - 1024 bytes

//Programming the STM32 in C starting from the reset vector needs the following steps in given order
// 1) Setup the interrupt and exception vector table
// 2) Configure C variables (resetHandler)
// 3) Configure the clock system and enable the needed peripheral clocks
// 4) Configure the input / output pins
// 5) Configure the peripherals
// 6) Enable the interrupts
// 7) Loop through the main code

extern unsigned char  INIT_DATA_VALUES;
extern unsigned char  INIT_DATA_START;
extern unsigned char  INIT_DATA_END;
extern unsigned char  BSS_START;
extern unsigned char  BSS_END;

int main(void);
void resetHandler(void);
void tim2IrqHandler(void);
void tim3IrqHandler(void);

//Vector table is setup in flash at the ".vectors" location, which is defined in the linker script file stm32f303-128k.ld
//See ST manual PM0214 chapter 2.3.4
const void * intVectors[76] __attribute__((section(".vectors"))) =
{
    (void*) STACK_TOP,
    resetHandler,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    usbIrqHandler,
    0,0,0,0,
    0, //tim1IrqHandler,
    0,0,
    tim2IrqHandler,
    tim3IrqHandler,
    0,0,0,0,0,0,0,0,0
};

//On reset this handler is called to setup the initial memory state
void resetHandler(void)
{
  unsigned char volatile *src;
  unsigned char volatile *dst;
  unsigned len;

  //Setup pointers and counter for copying initial values from flash to normal memory
  //This concerns all variables that are declared globally with an initial value
  src= &INIT_DATA_VALUES;
  dst= &INIT_DATA_START;
  len= &INIT_DATA_END - &INIT_DATA_START;

  while(len--)
    *dst++ = *src++;

  //Setup pointer and counter for erasing all the other variables
  dst = &BSS_START;
  len = &BSS_END - &BSS_START;

  while(len--)
    *dst++=0;

  //Go and start the main process
  main();
}

//The number of microseconds must be less then 1864135. (just over 1.8 second)
//Multiplied by 9 gives the number of ticks.
//With this it can only lead to a single timer overflow, which this function can handle
void usdelay(int32_t usec)
{
  int32_t end = STK->VAL - (usec * 9);

  //Check if there is the need to wait for an timer overflow
  if(end <= 0)
  {
    //Wait for the overflow to occur
    while((STK->CTRL & STK_CTRL_OVERFLOW) == 0);

    //calculate the new end value
    end += 0x00FFFFFF;
  }

  //Wait till the timer reaches the intended value for the given delay
  while(STK->VAL >= end);
}

//Simple function for setup of an IO pin
void InitIOPin(GPIO_TypeDef *port, uint32_t pin, uint32_t setting, uint32_t alternatefunction)
{
  //Each pin uses two bits in the mode register for four modes (MODER)
  // 00 input
  // 01 general purpose output
  // 10 alternate function
  // 11 analog
  
  //For an output there are two types (OTYPER)
  // 0 Pull up / down
  // 1 Open drain
  
  //There are three output speed settings (OSPEEDR)
  // x0 Low speed
  // 01 Medium speed
  // 11 High speed
  
  //There are three pull up and pull down possibilities (PUPDR)
  // 00 No pull up or pull down
  // 01 Pull up
  // 10 Pull down

  //Create a base pointer for either the lower or the higher alternate function register
  __IO uint32_t *reg;
  
  //2 bit settings need to be shifted twice the distance
  uint32_t shifter = pin * 2;

  //Set the requested configuration
  port->MODER |= (setting & GPIO_MODE_MASK) << shifter;
  port->OTYPER |= ((setting & GPIO_TYPE_MASK)  >> 4) << pin;
  port->OSPEEDR |= ((setting & GPIO_SPEED_MASK) >> 8) << shifter;
  port->PUPDR |= ((setting & GPIO_PUPD_MASK) >> 12) << shifter;

  //See if the lower alternate function register or the higher alternate function register needs to be used
  if(pin < 8)
  {
    //Low control register used for first 8 pins
    reg = &port->AFR[0];
  }
  else
  {
    //Force pin into 8 pins per register range
    pin -= 8;

    //High control register used for upper 8 pins
    reg = &port->AFR[1];
  }

  //4 control bits used per pin
  pin *= 4;

  //Reset bits first and set new mode and configuration.
  *reg &= ~(0x0F << pin);
  *reg |=  ((alternatefunction & 0x0F) << pin);
}

void writedisplay(uint8_t data, uint8_t type)
{
  //Based on type set the RS line (low for commands, high for data)
  if(type == 0)
    GPIOC->ODR &= ~(1 << 14);
  else
    GPIOC->ODR |= 1 << 14;

  //Put upper four bits on data bus and allow for some data setup time
  GPIOA->ODR &= 0xFFF0;
  GPIOA->ODR |= data >> 4;
  usdelay(2);

  //pulse display enable high for at least a microsecond
  GPIOC->ODR |= 1 << 15;
  usdelay(2);
  GPIOC->ODR &= ~(1 << 15);
  usdelay(20);

  //Put lower four bits on data bus and allow for some data setup time
  GPIOA->ODR &= 0xFFF0;
  GPIOA->ODR |= data & 0x0F;
  usdelay(2);

  //pulse display enable high for at least a microsecond
  GPIOC->ODR |= 1 << 15;
  usdelay(2);
  GPIOC->ODR &= ~(1 << 15);

  //Wait for data to be processed (>37us)
  usdelay(100);
}

void initdisplay(void)
{
  //Make sure enable is low to start with
  GPIOC->ODR &= ~(1 << 15);

  //Wait at least 50ms after reset
  usdelay(50000);

  //Reset display by setting it to 8 bit mode three times in a row with long delays in between
  GPIOA->ODR &= 0xFFF0;
  GPIOA->ODR |= 3;
  usdelay(2);

  //pulse display enable high for at least a microsecond and wait 4.5ms
  GPIOC->ODR |= 1 << 15;
  usdelay(2);
  GPIOC->ODR &= ~(1 << 15);
  usdelay(4500);

  //pulse display enable high again for at least a microsecond and wait 4.5ms
  GPIOC->ODR |= 1 << 15;
  usdelay(2);
  GPIOC->ODR &= ~(1 << 15);
  usdelay(4500);

  //pulse display enable high for third and last time for at least a microsecond and wait 150us
  GPIOC->ODR |= 1 << 15;
  usdelay(2);
  GPIOC->ODR &= ~(1 << 15);
  usdelay(150);

  //Set display in 4 bits mode by resetting D4 and allow for some data setup time
  GPIOA->ODR &= 0xFFFE;
  usdelay(2);

  //pulse display enable high for at least a microsecond and wait 100us
  GPIOC->ODR |= 1 << 15;
  usdelay(2);
  GPIOC->ODR &= ~(1 << 15);
  usdelay(100);

  //Keep display in 4 bits mode and specify 2 lines with 5x8 font
  writedisplay(0x28, 0);

  //Turn display on without blinking cursor
  writedisplay(0x0C, 0);

  //Clear the display
  writedisplay(0x01, 0);

  //Wait till cleared
  usdelay(2000);

  //Set entry mode to no shift and increment address
  writedisplay(0x06, 0);

  //Wait till entry mode is set
  //Some displays need a long time
  usdelay(20000);
}

void displaystring(char *string, uint8_t position)
{
  uint8_t pos;

  while(*string)
  {
    //Translate input position to display position
    pos = position++;

    if((pos >= 20) && (pos < 40))
      pos += 44;
    else if((pos >= 40) && (pos < 60))
      pos -= 20;
    else if((pos >= 60) && (pos < 80))
      pos += 24;

    writedisplay(0x80 + pos,0);  //Output position command
    writedisplay(*string++,1);   //Write the actual character
  }
}

void displayint(int32_t value, uint8_t position)
{
  //Decimal
  //Hexadecimal
  //Left justify
  //Right justify (default)
  //Left pad with 0
  //left pad with spaces
  char      b[12] = { ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'};
  uint32_t  u = value;
  uint8_t   n = 0;
  uint8_t   i = 11;

  if(value == 0)
  {
    //Put a 0 in the for last index
    b[--i] = '0';
  }
  else
  {
    //Check if negative value
    if(value < 0)
    {
      //Negate if so and signal negative sign needed
      u = -value;
      n = 1;
    }

    //Process the digits
    while(u)
    {
      //Add current digit to decreased index
      b[--i] = (u % 10) + '0';

      //Take of the current digit
      u /= 10;
    }
  }

  //Check if negative number and if so put a minus in front of it
  if(n == 1)
    b[--i] = '-';

  //Display the string from the current index
  displaystring(&b[i], position);
}

void displaydegree(int32_t value, uint8_t position)
{
  char      b[12] = { ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '0', '.', '0', '\0'};
  uint32_t  u = value / 2;
  uint8_t   i = 9;

  if(u == 0)
  {
    b[--i] = '0';
  }
  else
  {
    while(u)
    {
      b[--i] = (u % 10) + '0';

      u /= 10;
    }
  }

  //Check if half a degree
  if(value & 1)
    b[10] = '5';

  displaystring(&b[i], position);
}

//Table for decoding states of a rotary encoder
const int8_t encoderstates[] = { 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, 0 };

//Table for square wave states
//Write to BSRR register. A 1 sets or resets a pin based on upper (reset) or lower (set) 16 bits.
//Output on pin B6, B7, B8, B9
//Initial state needs to be B6 low, B7 high, B8 low, B9 high (0000 0010 1000 0000) 0x0280
//0000 0010 0100 0000 => 0000 0000 1000 0000 0000 0000 0100 0000
//0000 0001 0100 0000 => 0000 0010 0000 0000 0000 0001 0000 0000
//0000 0001 1000 0000 => 0000 0000 0100 0000 0000 0000 1000 0000
//0000 0010 1000 0000 => 0000 0001 0000 0000 0000 0010 0000 0000
//const int16_t squarewavestates[] = {0x0240, 0x0140, 0x0180, 0x0280};
const int32_t squarewavestates[] = {0x00800040, 0x02000100, 0x00400080, 0x01000200};

//Variables now global for reading in interrupt routine
volatile int16_t phase = 0;
volatile int16_t prevphase = 0;
volatile int16_t phasediff;
volatile int16_t sinephase;
volatile int16_t squarephase;
volatile int16_t currentphase;
volatile int16_t timcnt;

//Main program part
int main(void)
{
  //Setup flash to work with 72MHz clock
  //Enable the Prefetch Buffer and Set to 2 wait states
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

  //Configure system clock
  //External oscillator: 8MHz
  //PLL multiplier: x9
  //SYSCLK: 72MHz
  //AHB: SYSCLK = 72MHz
  //APB1: SYSCLK/2 = 36MHz  //Timer 2,3 and 4 run on 72MHz since APB1 divider is not 1
  //APB2: SYSCLK/2 = 36MHz  //Timer 1 also runs on 72MHz since APB2 divider is not 1
  //ADC: SYSCLK/6 = 12MHz
  //USB: SYSCLK/1.5 = 48MHz
  RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2;

  //Enable external oscillator
  RCC->CR |= RCC_CR_HSEON;

  //Wait for the clock to become stable
  while((RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY);

  //Enable the PLL
  RCC->CR |= RCC_CR_PLLON;

  //Wait for the PLL to become stable
  while((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);

  //Switch to the PLL clock as system clock source. Since on reset these bits are set to 0 no need to clear first.
  RCC->CFGR |= RCC_CFGR_SW_PLL;

  //Wait for the PLL to become the clock source
  while((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

  //From this point on it is not possible to change the clock configuration without switching back to HSI

  //Enable the used peripherals. PORTA, PORTB, PORTC and DMA1
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_DMA1EN; //RCC_AHBENR_ADC12EN |  | RCC_AHBENR_DMA2EN
  
  //Enable TIM1
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  //Enable TIM2, TIM3 and DAC1
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_DAC1EN;
  
  //Enable the system timer for delay function
  //In default setting with 72MHz AHB clock this timer runs on 9MHz, so 111ns per tick. Takes 9 ticks for a microsecond
  STK->LOAD = 0x00FFFFFF;
  STK->CTRL = STK_CTRL_ENABLE;
  
  //Decide which clock is best to be used for the ADC
  //There are two possibilities. AHB clock divided by 1,2 of 4 to be synchronous with the AHB clock
  //or the pll clock divided by a prescaler. With this clock the conversions can be asynchronous to timer triggers and lead to jitter.
  
  //PA2 is ADC1_IN3  (PA3 is ADC1_IN4 and negative input for IN3 in differential mode)
  //PA6 is ADC2_IN3  (PA7 is ADC2_IN4 and negative input for IN3 in differential mode)
  
  //Before using the ADC it is needed to turn on the internal voltage regulator
  //It is also needed to calibrate the ADC. When both single ended and differential inputs are used one needs
  //to do separate calibrations for each type. Otherwise only the one for the type in use needs to be done
  //Only after these steps the ADC can be enabled
  //After enabling the ADC a wait until ADCRDY is set needs to be done before converting any channels
  
  //For simultaneous getting the voltage and the current it is possible to use the master slave mode of two connected ADC's
  //Select the regular simultaneous mode
  
  //To reduce noise the ADC clock can be lowered to sysclk/4 when synchronous mode is used.
  //With a sample time of 7.5 clock cycles the conversion is done in 20 clock cycles. This leads to a max rate
  //of 900KHz when the sysclk is 72MHz
  
  //ADC1 uses DMA1 channel1
  //ADC2 uses DMA2 either channel1 or channel3 (only when remapped)
  
  //On the blue pill the VDDA pin is connected to VDD, so DAC output can be routed back into the ADC.
  
  //Init the USB device
  usbInit();

  //IO pins need to be configured first
  //Pin with LED to show activity
  InitIOPin(GPIOC, 13, GPIO_OUTPUT_PP_LOW_SPEED, 0);
  
  //Time measurement pin irq tim 3
  InitIOPin(GPIOA, 6, GPIO_OUTPUT_PP_MEDIUM_SPEED, 0);

  //SPI Display connections
  //InitIOPin(GPIOA, 2, GPIO_OUTPUT_PP_MEDIUM_SPEED, 0);  //Reset                 (RES) (RESET)
  //InitIOPin(GPIOA, 3, GPIO_OUTPUT_PP_MEDIUM_SPEED, 0);  //Register select       (RS) (A0) (DS/RS)
  //InitIOPin(GPIOA, 15, GPIO_AF_PP_HIGH_SPEED, 5);       //Chip select  SPI/NSS  (CS)
  //InitIOPin(GPIOB, 3, GPIO_AF_PP_HIGH_SPEED, 6);        //Clock        SPI/SCK  (SCK) (SCL)
  //InitIOPin(GPIOB, 5, GPIO_AF_PP_HIGH_SPEED, 6);        //Data         SPI/MOSI (SDA) (SDI)
  
  //LCD 1602 display connection
  //PC14 for RS, PC15 for E, PA0 - PA3 for databus
  InitIOPin(GPIOA, 0, GPIO_OUTPUT_PP_LOW_SPEED, 0);
  InitIOPin(GPIOA, 1, GPIO_OUTPUT_PP_LOW_SPEED, 0);
  InitIOPin(GPIOA, 2, GPIO_OUTPUT_PP_LOW_SPEED, 0);
  InitIOPin(GPIOA, 3, GPIO_OUTPUT_PP_LOW_SPEED, 0);
  InitIOPin(GPIOC, 14, GPIO_OUTPUT_PP_LOW_SPEED, 0);
  InitIOPin(GPIOC, 15, GPIO_OUTPUT_PP_LOW_SPEED, 0);
  
  //Rotary encoder pins. External pullups are used. Debouncing filter might be needed
  InitIOPin(GPIOB, 0, GPIO_INPUT_FLOAT, 0);
  InitIOPin(GPIOB, 1, GPIO_INPUT_FLOAT, 0);
  
  //USB pins need to be set for correct alternate function
  InitIOPin(GPIOA, 11, GPIO_AF_PP_HIGH_SPEED, 14);
  InitIOPin(GPIOA, 12, GPIO_AF_PP_HIGH_SPEED, 14);

  //Square wave output pins
  InitIOPin(GPIOB, 6, GPIO_OUTPUT_PP_LOW_SPEED, 0); //90 degree
  InitIOPin(GPIOB, 7, GPIO_OUTPUT_PP_LOW_SPEED, 0); //270 degree
  InitIOPin(GPIOB, 8, GPIO_OUTPUT_PP_LOW_SPEED, 0); //180 degree
  InitIOPin(GPIOB, 9, GPIO_OUTPUT_PP_LOW_SPEED, 0); //0 degree
  
  //DAC pin need to be set for analog
  InitIOPin(GPIOA, 4, GPIO_ANALOG, 0);
  
  //ADC input pins need to be set for analog
//  InitIOPin(GPIOA, 2, GPIO_ANALOG, 0);   //ADC1_IN3
//  InitIOPin(GPIOA, 6, GPIO_ANALOG, 0);   //ADC2_IN3
  
  //Set initial state on square wave pins
  GPIOB->ODR |= 0x0280;
  
  //Setup the DMA channel for timer 1 overflow to write data into the dac
  DMA1_Channel5->CPAR = (uint32_t)&DAC1->DHR12R1;
  DMA1_Channel5->CMAR = (uint32_t)dmasinetable_1900;
  DMA1_Channel5->CNDTR = STEPS_360_DEGREES;
  DMA1_Channel5->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_EN;
  
  //Setup the DMA channel for timer 3 overflow to write data to port b
  DMA1_Channel3->CPAR = (uint32_t)&GPIOB->BSRR;
  DMA1_Channel3->CMAR = (uint32_t)squarewavestates;
  DMA1_Channel3->CNDTR = 4;
  DMA1_Channel3->CCR = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_EN;
  
  //Setup the dac. Only first channel for now
  //Output buffer is enabled, no wave generation. Data is automatically transferred to the output.
  //Since DMA is used based on a timer overflow the DAC will not be triggered by an event.
  DAC1->CR = DAC_CR_EN1 | DAC_CR_BOFF1;
  
  //Initialize on half voltage
  DAC1->DHR12R1 = 2048;
  
  //Output the value
  DAC1->SWTRIGR = 1;
  
  //Timer 1 is used to trigger the dma that outputs the sine data to the dac. It runs at a frequency of 720KHz
  TIM1->CNT = 0;    //No initial count
  TIM1->PSC = 0;    //No prescaler
  TIM1->ARR = 99;   //Divide master clock by 100. Counts from 0 to 99, which is 100 ticks.

  //Timer 1 up counting. Preload and dma on update enabled.
  TIM1->DIER = TIM_DIER_UDE;
  TIM1->CR2 = TIM_CR2_MMS_1;
  
  //Timer 2 is used for blinking the led on PC13
  TIM2->CNT = 0;
  TIM2->PSC = 15999;  //72MHz / 16000 = 4500Hz
  TIM2->ARR = 1124;   //4500Hz / 1125 = 4Hz;

  //Timer 2 generates an interrupt at a 4Hz rate.
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->CR1 = TIM_CR1_CEN;
    
  //Timer 3 generates the 4KHz clock to trigger the dma for the creation of the 4 square wave signals
  TIM3->CNT  = 0;                       //Initial phase on 0 degrees
  TIM3->PSC  = 0;                       //No prescaler
  TIM3->ARR  = MAX_COUNT_90_DEGREES;    //720KHz / 180 = 4KHz;
  TIM3->CCR1 = HALF_COUNT_90_DEGREES;   //For compare interrupt halfway the timer count

  //Enable the DMA trigger
  TIM3->DIER = TIM_DIER_UDE;
  TIM3->SMCR = TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;  //Clock source for timer 3 is timer 1 (TS = 000)

  //Start the timers used for sine and square wave generation  
  TIM1->CR1 = TIM_CR1_CEN;
  TIM3->CR1 = TIM_CR1_CEN;
  
  
  //ADC is for future features
  /*
  
  //Setup ADC 1 and 2 for converting simultaneous on inputs PA2 and PA6
  //Put converters in intermediate state before turning on the voltage regulator
  ADC1->CR = ADC_CR_ADVREG_INTERMEDIATE;
  ADC2->CR = ADC_CR_ADVREG_INTERMEDIATE;
  
  //Wait for 10us to make sure things are settled
  usdelay(10);
  
  //Turn on the voltage regulators
  ADC1->CR = ADC_CR_ADVREG_ON;
  ADC2->CR = ADC_CR_ADVREG_ON;
  
  //Wait for 20us to make sure things are settled
  usdelay(20);
  
  //Start calibration for single ended conversions
  ADC1->CR |= ADC_CR_ADCAL;
  ADC2->CR |= ADC_CR_ADCAL;
  
  //Wait till calibration of both converters is done
  while((ADC1->CR & ADC_CR_ADCAL) || (ADC2->CR & ADC_CR_ADCAL));

  //Setup the channels to be converted. Only a single channel per ADC
  //Channel 0 is not mapped so range is from 1 to 18.
  //Sequence count is 1 on reset
  ADC1->SQR1 = 3 << ADC_SQR1_SQ1_POS;
  ADC2->SQR1 = 3 << ADC_SQR1_SQ1_POS;

  //Setup the sample time for the channels in use. 7,5 adc clock cycles is used
  ADC1->SMPR1 = ADC_SAMPLE_TIME_7 << ADC_SMPR1_CH3_POS;
  ADC2->SMPR1 = ADC_SAMPLE_TIME_7 << ADC_SMPR1_CH3_POS;

  //Setup dual mode for simultaneous conversions 
  //Clock is synchronous HCLK divided by 4, DMA in 12bit mode
  ADC12_COMMON->CCR = ADC_CCR_CKMODE_HCLK_4 | ADC_CCR_MDMA_10_12_BIT | ADC_CCR_DMACFG_CIRCULAIR | ADC_CCR_MULTI_RS;
  
  //Setup DMA for the ADC's
  
  //Enable both converters
  ADC1->CR |= ADC_CR_ADEN;
  ADC2->CR |= ADC_CR_ADEN;
  
  //Wait till both converters are ready
  while((ADC1->ISR & ADC_ISR_ADRDY) || (ADC2->CR & ADC_ISR_ADRDY));
  
  */ 
  
  //Set priority for timer 1, 2 interrupt to be higher then the other interrupts
  //This is an array of 8 bit registers, of which only the upper 4 bits are used for the priority allowing for 16 levels
  //By grouping this is separated to allow for having sub priorities within a single group.
  //In the usb init this is set for 4 group priorities with each 4 sub priorities.
  //The higher the number the lower the priority
  //NVIC->IP[TIM1_UP_IRQn] = 0x80;  //(1000b) Group priority 2, sub priority 0
  NVIC->IP[TIM2_IRQn]    = 0x90;  //(1001b) Group priority 2, sub priority 1
  NVIC->IP[TIM3_IRQn]    = 0x80;  //(1000b) Group priority 2, sub priority 0

  //Enable the timer 1 and 2 interrupt
  //This is an array of 32 bit registers, only used to enable an interrupt. To disable the ICER registers need to be used
  //Each register serves 32 interrupts, so to get the register for the interrupt, shift the IRQ number right 5 times (divide by 32) and to get
  //the right interrupt enable bit, shift a unsigned 32 bit integer 1 the IRQ number anded with 31 (modulo 32) times to the right
  //NVIC->ISER[TIM1_UP_IRQn >> 0x05] = (uint32_t)0x01 << (TIM1_UP_IRQn & 0x1F);
  NVIC->ISER[TIM2_IRQn >> 0x05] = (uint32_t)0x01 << (TIM2_IRQn & 0x1F);
  NVIC->ISER[TIM3_IRQn >> 0x05] = (uint32_t)0x01 << (TIM3_IRQn & 0x1F);
  
  
  //Initialize the attached display
  initdisplay();
  
  //Show the initial phase shift
  displaystring("FASE: 0.0   GRD", 0);
  
  //Variable for rotary encoder handling
  uint8_t h = 0;
  
  while(1)
  {
    //Get character from USB receive buffer
    int16_t c = usbRead();

    //Echo it when valid character
    if(c != -1)
      usbSend(c);
    
    //Rotary encoder part
    //Hardware debouncing door 10Komh pullup en 10Kohm serie weerstand en 10nf condensator naar ground.

    //2ms delay
    usdelay(2000);

    //Read the encoder pins
    h <<= 2;
    h |= (GPIOB->IDR & 0x03);

    //Modify the count accordingly
    phase += encoderstates[h & 0x0F];

    if(phase < 0)                         //Check on underrun and fall back to maximum
      phase = MAX_PHASE;
    else if(phase > MAX_PHASE)            //Check on overrun and fall back to minimum
      phase = 0;

    //Check if a change in phase
    if(phase != prevphase)
    {
      //Update previous phase for filtering
      prevphase = phase;

      //Display the new phase
      displaystring("FASE:       GRD", 0);
      displaydegree(phase, 6);
      
      //Clear any pending interrupts to make sure the event takes place on the actual moment
      TIM3->SR = 0;
      
      //And enable the compare interrupt to make the adjustment
      TIM3->DIER |= TIM_DIER_CC1IE;
    }
  }
}

//Handler for timer 2 interrupt. Only used to blink the led on the board
void tim2IrqHandler(void)
{
  //Clear the interrupt flags
  TIM2->SR = 0;

  //Toggle the led output pin
  GPIOC->ODR ^= (1 << 13);
}

//Handler for timer 3 interrupt. Used to shift the phase within certain limits
void tim3IrqHandler(void)
{
  //Set pin for time measurement PA6
  GPIOA->ODR |= 0x0040;

  //Make sure it is the compare interrupt in use
  if(TIM3->SR & TIM_SR_CC1IF)
  {
    //Determine the current phase difference
    sinephase = STEPS_360_DEGREES - DMA1_Channel5->CNDTR;         //dma counter counts down, so subtraction from max is needed
    squarephase = ((4 - DMA1_Channel3->CNDTR) * 180) + TIM3->CNT; //Each square wave dma step indicates 180 half degrees
    currentphase = sinephase - squarephase;

    //Phase needs to be positive
    if(currentphase < 0)
      currentphase += STEPS_360_DEGREES;

    //Calculate the needed phase difference
    phasediff = currentphase - phase;

    //Check if timer needs to be updated
    if(phasediff)
    {
      //Check if difference out of quarter count range and limit if so
      if(phasediff > QUARTER_COUNT_90_DEGREES)
        phasediff = QUARTER_COUNT_90_DEGREES;
      else if(phasediff < -QUARTER_COUNT_90_DEGREES)
        phasediff = -QUARTER_COUNT_90_DEGREES;
      else
        TIM3->DIER &= ~TIM_DIER_CC1IE;  //Within the limits then one correction is enough so disable the interrupt

      //Adjust timer 3 count for the new phase
      TIM3->CNT += phasediff;
    }
    else
      TIM3->DIER &= ~TIM_DIER_CC1IE;  //No correction needed so disable the interrupt
  }
  
  //Clear the interrupt flags
  TIM3->SR = 0;
  
  //Clear pin for time measurement PA6
  GPIOA->ODR &= 0xFFBF;
}
