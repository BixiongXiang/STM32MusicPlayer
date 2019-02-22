/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V1.0
* Date               : 10/08/2007
* Description        : Main program body
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"


#include "stm32f10x_lib.h" //for exti/tim

#include "math.h"  //sound frq calculate

#include "i2c_ee.h"
#include "string.h"
#include "stdio.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((u32)0x4001244C)
#define ADC2_DR_Address    ((u32)0x4001284C)
#define ADC3_DR_Address    ((u32)0x40013C4C)//maybe

#define shuma1  0x6f 
#define shanshuo  0x0f  //四个全闪
#define shanshcmd  0x70 //闪烁控制字
#define shuma1  0x6f

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
vu32 ADC_DR_Value;
vu16 ADC_ConvertedValue;//change to 16 latter
vu16 ADC2_ConvertedValue;


USART_InitTypeDef USART_InitStructure;

ErrorStatus HSEStartUpStatus;

EXTI_InitTypeDef EXTI_InitStructure;  //for key EXTI 
    
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//for tim
TIM_OCInitTypeDef  TIM_OCInitStructure;//for TIM4 pwm

char name[]={0xcf,0xf2,0xb1,0xd8,0xd0,0xdb,' ',0xd5,0xc5,0xd0,0xde,0xb4,0xcf,0};

u8 ledstate=0;
u8 soundindex = 0;

enum digit{dgt1=0x60,dgt2=0xda,dgt3=0xf2,dgt4=0x66,dgt5=0xb6,dgt6=0xbe,dgt7=0xe0,dgt8=0xfe,dgt9=0xe6,dgt0=0xfc,dgta=0xee,dgtb=0x3e,dgtc=0x9c,dgtd=0x7a,dgte=0x9e,dgtf=0x8e,dgth=0x2e};

//C381 D340 E302 F285 G254 A226 B202  c190(+1)
enum note{c1=381,d1=340,e1=302,f1=285,g1=254,a1=226,b1=202,c2=190,d2=170,e2=150,f2=142,g2=127};
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

void Sound(void);//pwm key
void SoundReverse(void);// press wakeup down sound
void Sound2(void);//interrupt wakeup
void Music(void);
void btnKey(void);
void dgtDisplay(void);
void dgtDisable(void);
void btnJS(void);
void ledADC(void);
void ADCValueConvert(void);
void Delay(vu32 nCount);
/* Private functions ---------------------------------------------------------*/

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
#ifdef DEBUG
  debug();
#endif

  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration();

  /* NVIC configuration ------------------------------------------------------*/
  NVIC_Configuration();

  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();

/*[TIM Initiation]*/
	//tim2 sound frq: C261.6HZ D293.6HZ E329.6HZ F349.2HZ G392HZ A440HZ B493.8HZ
	//C382 D341 E303 F286 G255 A227 B203 (-1)
	  TIM_TimeBaseStructure.TIM_Period = 226;       //100khz/(226+1)=440.5hz
	  TIM_TimeBaseStructure.TIM_Prescaler =359;  //36mhz/(359+1)=100khz
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //normally set 0
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  //TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	  //TIM_Cmd(TIM2, ENABLE);  //set enable in Sound()
	  
	  /*tim4_ch3 for PWM Audio*/
	  TIM_TimeBaseStructure.TIM_Period = 226;       //10khz/(9999+1)=1hz
	  TIM_TimeBaseStructure.TIM_Prescaler =359;  //36mhz/(3599+1)=10khz
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	  
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//pwm mode 1		
	  TIM_OCInitStructure.TIM_Channel = TIM_Channel_3;			     
	  TIM_OCInitStructure.TIM_Pulse = 200;//duty: 200/227 = 0.88
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		
	  TIM_OCInit(TIM4, &TIM_OCInitStructure);		//channel 3 -> OC3		
	  // 閰嶇疆閫氶亾3鐨勮緭鍑烘瘮杈冮瑁呰浇鏈夋晥锛屽畾鏃跺櫒4鐨勬椂鍩鸿繘琛岄噸瑁呰浇 
	  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);		//channel 3 -> OC3	
	  // 閰嶇疆TIM4鐨勯噸瑁呰浇鏈夋晥 
	  TIM_ARRPreloadConfig(TIM4, ENABLE);

	  //TIM_Cmd(TIM4, ENABLE);
          
          /* 启动定时器TIM3 */
          //TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 
	  
	  /* TIM1 Main Output Enable */ //extra add
	  //TIM_CtrlPWMOutputs(TIM4, ENABLE);
	  
	  //enable timer interrupt TIM2/3
	  //TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);  
	  
	  //TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //enable tim interrupt as update
/*[TIM Initiation END]*/

/*[EXTI Initiation]*/
	  //??? Connect EXTI Line9 to PB.09 
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9); //bound exit & bus
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);  
          
	  //Configure EXTI Line9 to generate an interrupt on falling edge   
	  EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
	  
          //GPIOA PA0
          EXTI_InitStructure.EXTI_Line = EXTI_Line0;
          EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
          EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
          EXTI_InitStructure.EXTI_LineCmd = ENABLE;
          EXTI_Init(&EXTI_InitStructure);
	  // Generate software interrupt: simulate a falling edge applied on EXTI line 9 
	  EXTI_GenerateSWInterrupt(EXTI_Line9);
          EXTI_GenerateSWInterrupt(EXTI_Line0);
	  
/*[EXTI Initiation end]*/

/*[USART Initiation]*/
	  USART_InitStructure.USART_BaudRate = 115200;//exti9 triggered on this scentence
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No ;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_InitStructure.USART_Clock = USART_Clock_Disable;
	  USART_InitStructure.USART_CPOL = USART_CPOL_Low;
	  USART_InitStructure.USART_CPHA = USART_CPHA_2Edge;
	  USART_InitStructure.USART_LastBit = USART_LastBit_Disable;
	  
	  // Configure the USARTx  
	  USART_Init(USARTx, &USART_InitStructure);
	  // Enable the USARTx 
	  USART_Cmd(USARTx, ENABLE);

	  printf("\n\n\rUSART READY!\n\r");
/*[USART Initiation end]*/

/*[DMA configuration]*/
	  // DMA channel1 configuration ----------------------------------------------

	  DMA_DeInit(DMA_Channel1);
	  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_DR_Value;
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	  DMA_InitStructure.DMA_BufferSize = 1;//
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;//WORD 32bit
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	  DMA_Init(DMA_Channel1, &DMA_InitStructure);
	  
	  // Enable DMA channel1 
	  DMA_Cmd(DMA_Channel1, ENABLE);//put this in exti function?
		 
	  printf("DMA READY!\n\r");
/*[DMA configuration end]*/

/*[ADC configuration]*/
	  // ADC1 configuration ------------------------------------------------------

	  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;  //ADC_Mode_Independen
	  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	  ADC_InitStructure.ADC_NbrOfChannel = 1;
	  ADC_Init(ADC1, &ADC_InitStructure);
          
          ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;  //ADC_Mode_Independen
	  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	  ADC_InitStructure.ADC_NbrOfChannel = 1;
	  ADC_Init(ADC2, &ADC_InitStructure);

	  // ADC1 regular channel14 configuration ->potentiometer   ch15->mic  ch10->temperature
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_55Cycles5);
	  ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 1, ADC_SampleTime_55Cycles5);

          /* Enable ADC2 external trigger conversion */
          ADC_ExternalTrigConvCmd(ADC2, ENABLE);//
  
	  // Enable ADC1 DMA 
	  ADC_DMACmd(ADC1, ENABLE);
	  //ADC_DMACmd(ADC2, ENABLE);
	  
	  // Enable ADC1 
	  ADC_Cmd(ADC1, ENABLE);
	  ADC_Cmd(ADC2, ENABLE);

	  // Enable ADC1 reset calibaration register    
	  ADC_ResetCalibration(ADC1);
	  ADC_ResetCalibration(ADC2);
	  // Check the end of ADC1 reset calibration register 
	  while(ADC_GetResetCalibrationStatus(ADC1));
	  while(ADC_GetResetCalibrationStatus(ADC2));

	  // Start ADC1 calibaration 
	  ADC_StartCalibration(ADC1);
	  ADC_StartCalibration(ADC2);
	  // Check the end of ADC1 calibration 
	  while(ADC_GetCalibrationStatus(ADC1));
	  while(ADC_GetCalibrationStatus(ADC2));
		 
	  // Start ADC1 Software Conversion 
	  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	  //ADC_SoftwareStartConvCmd(ADC2, ENABLE);
          
          printf("ADC READY!\n\r");

	  //ADC_Cmd(ADC2, DISABLE);
/*[ADC configuration end]*/
  printf("*** All Green, System Started! ***\n\r");
  printf("Designed by: %s \n\n\r",name);
  printf("Instructions: \n\r 1,[key]enable ADC [wakeup]disable ADC [tamper]paly music \n\r 2,knob for led3\n\r");
  printf(" 3,[JOYSTICK]:[U&D] Select LED React To Potentiometer or Mic\n\r  [L&R]Select Two Music  \n\r");
  printf(" 4,[Music]:The pitch of every sound will vary with potentiometer for 4 level  \n\r");
  printf(" 5,[Speak]:Try to say 'Music'to mic loudly.(suprise!)  \n\n\r");
  //sprintf(name,"sprintf!");
  
  /* Initialize the I2C EEPROM driver */
  I2C_EE_Init(); 
  dgtDisable();
  I2C_shuma_ByteWrite(dgth, 0x11);//点亮第一个数码管
    
  I2C_shuma_ByteWrite(dgt1, 0x10);//点亮第二个数码管
   
  while (1)
  {
     //LCD_SetTextColor(Yellow);
	//separate adc1_dr value to adc1 and adc2
	ADCValueConvert();
	  
    //joyStick Control
	btnJS();
	
    //turn on led3 when ADC>2000
	ledADC();
    
	//print value of adc, because the end is \r so every time will refresh this line
    printf("[ADC DR Value]=0x%x [ADC1&2 Value]=[0x%x] [0x%x] \r",ADC_DR_Value,ADC_ConvertedValue,ADC2_ConvertedValue);
	
    //generate sound when key is pressed
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) == 0)//KEY
    {
      GPIO_SetBits(GPIOC, GPIO_Pin_9);
      Sound();//try put this in exti function ? 
      GPIO_ResetBits(GPIOB, GPIO_Pin_9);
      
    }
    
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) == 0)//TEMPER
    {
      GPIO_SetBits(GPIOC, GPIO_Pin_9);
      Music();
      GPIO_ResetBits(GPIOB, GPIO_Pin_9);
      //GPIO_SetBits(GPIOC, GPIO_Pin_8);
      //Sound2();
    }
    
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1)//wakeup
    {
       GPIO_SetBits(GPIOC, GPIO_Pin_9);
      SoundReverse();//try put this in exti function ? 
      GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    }
	
  }
}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
  
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  
    /* PLLCLK = 8MHz * 7 = 56 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

/* Enable peripheral clocks --------------------------------------------------*/
  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA, ENABLE);

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOC, ENABLE);


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx | RCC_APB2Periph_AFIO, ENABLE);

/* Enable USARTx clocks */
#ifdef USE_USART1  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#else
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USARTx, ENABLE);
#endif


/*[EXTI] ???Enable GPIOB, GPIOC and AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
	
	
  /*[TIM] TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);/* TIM select2: TIM  clock */
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);/* TIM select2: TIM  clock */
	
  /* I2C1 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
   //GPIO_PinRemapConfig(GPIO_FullRemap_TIM4,ENABLE);

  /*[ADC] Configure PC.04 (ADC Channel14) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 |GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
  /*[LED] Configure PC.06, PC.07, PC.08 and PC.09 as Output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /*[key] Configure PB.09 as input floating (EXTI Line 9) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /*[TAMPER] Configure PC.13 as input floating (EXTI Line 9) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /*[TAMPER] Configure PC.13 as input floating (EXTI Line 9) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*[SOUND] Configure PB.08 as Output push-pull [SOUND]*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //SET AS SECOND DEVICE OUTPUT ON GPIO
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /*[Digit Tube] Configure PD.0-7 as Output push-pull 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  */
  
  /*[Joystick] D8-UP D14-DOWN E1-L E0-R D12-SET */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_14 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  
  
#ifdef USE_USART2//婊¤冻鏉′欢锛堝畾涔変簡xxx锛夋墠鎵ц涓嬮潰鐨勮鍙?  /* Enable the USART2 Pins Software Remapping */
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
#endif

  /*[USART] Configure USARTx_Tx as alternate function push-pull PA9 PA10*/
  GPIO_InitStructure.GPIO_Pin = GPIO_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOx, &GPIO_InitStructure);

  /* Configure USARTx_Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOx, &GPIO_InitStructure);

}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;  //for NVIC initiation
	
#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

	/* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* Enable the EXTI9_5 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the EXTI0 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* 缂栫▼NVIC锛屼娇鑳絋IM2/3 涓柇閫氶亾锛屾寚瀹氫紭鍏堢骇 */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /*
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	*/
}

/*******************************************************************************
* Function Name  : PUTCHAR_PROTOTYPE
* Description    : Retargets the C library printf function to the USART.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
PUTCHAR_PROTOTYPE
{
  /* Write a character to the USART */
  USART_SendData(USARTx, (u8) ch);

  /* Loop until the end of transmission */
  while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
  {
  }

  return ch;
}


#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*******************************************************************************
* Function Name  : Sound
* Description    : Play do mi so do.
* Input          : nCount: None
* Output         : None
* Return         : None
*******************************************************************************/
void Sound(void)
{
	
          GPIO_InitTypeDef GPIO_InitStructure;
          /*[SOUND] Configure PB.08 as Output push-pull [SOUND]*/
          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //SET AS SECOND DEVICE OUTPUT ON GPIO
          GPIO_Init(GPIOB, &GPIO_InitStructure);
		  // above is unnecessary
  
  /*tim4_ch3 for PWM Audio*/
	  TIM_TimeBaseStructure.TIM_Period = 381/(pow(2,ADC_ConvertedValue>>10));       //10khz/(9999+1)=1hz
	  TIM_TimeBaseStructure.TIM_Prescaler =359;  //36mhz/(3599+1)=10khz
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	  
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//pwm mode 1		
	  TIM_OCInitStructure.TIM_Channel = TIM_Channel_3;			     
	  TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period * 0.8;//duty: 200/227 = 0.88
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		
	  TIM_OCInit(TIM4, &TIM_OCInitStructure);		//channel 3 -> OC3		
	  // 閰嶇疆閫氶亾3鐨勮緭鍑烘瘮杈冮瑁呰浇鏈夋晥锛屽畾鏃跺櫒4鐨勬椂鍩鸿繘琛岄噸瑁呰浇 
	  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);		//channel 3 -> OC3	
	  // 閰嶇疆TIM4鐨勯噸瑁呰浇鏈夋晥 
	  TIM_ARRPreloadConfig(TIM4, ENABLE);

	  TIM_Cmd(TIM4, ENABLE);
          Delay(0x15FFFF);
          
          
          TIM_TimeBaseStructure.TIM_Period = 302/(pow(2,ADC_ConvertedValue>>10));       //10khz/(9999+1)=1hz
          TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
          TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period * 0.8;
          TIM_OCInit(TIM4, &TIM_OCInitStructure);	
          TIM_Cmd(TIM4, ENABLE);
          Delay(0x15FFFF);
          
          TIM_TimeBaseStructure.TIM_Period = 254/(pow(2,ADC_ConvertedValue>>10));       //10khz/(9999+1)=1hz
          TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
          TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period * 0.8;
          TIM_OCInit(TIM4, &TIM_OCInitStructure);	
          TIM_Cmd(TIM4, ENABLE);
          Delay(0x15FFFF);
          
          TIM_TimeBaseStructure.TIM_Period = 190/(pow(2,ADC_ConvertedValue>>10));       //10khz/(9999+1)=1hz
          TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
          TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period * 0.8;
          TIM_OCInit(TIM4, &TIM_OCInitStructure);	
          TIM_Cmd(TIM4, ENABLE);
          Delay(0x15FFFF);
          
          TIM_Cmd(TIM4, DISABLE);
  
          GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  
	  
}

void SoundReverse(void)
{
	
          GPIO_InitTypeDef GPIO_InitStructure;
          /*[SOUND] Configure PB.08 as Output push-pull [SOUND]*/
          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //SET AS SECOND DEVICE OUTPUT ON GPIO
          GPIO_Init(GPIOB, &GPIO_InitStructure);
		  // above is unnecessary
  
  /*tim4_ch3 for PWM Audio*/
	  TIM_TimeBaseStructure.TIM_Period = 190/(pow(2,ADC_ConvertedValue>>10));       //10khz/(9999+1)=1hz
	  TIM_TimeBaseStructure.TIM_Prescaler =359;  //36mhz/(3599+1)=10khz
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	  
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//pwm mode 1		
	  TIM_OCInitStructure.TIM_Channel = TIM_Channel_3;			     
	  TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period * 0.8;//duty: 200/227 = 0.88
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		
	  TIM_OCInit(TIM4, &TIM_OCInitStructure);		//channel 3 -> OC3		
	  // 閰嶇疆閫氶亾3鐨勮緭鍑烘瘮杈冮瑁呰浇鏈夋晥锛屽畾鏃跺櫒4鐨勬椂鍩鸿繘琛岄噸瑁呰浇 
	  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);		//channel 3 -> OC3	
	  // 閰嶇疆TIM4鐨勯噸瑁呰浇鏈夋晥 
	  TIM_ARRPreloadConfig(TIM4, ENABLE);

	  TIM_Cmd(TIM4, ENABLE);
          Delay(0x15FFFF);
          
          
          TIM_TimeBaseStructure.TIM_Period = 254/(pow(2,ADC_ConvertedValue>>10));       //10khz/(9999+1)=1hz
          TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
          TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period * 0.8;
          TIM_OCInit(TIM4, &TIM_OCInitStructure);	
          TIM_Cmd(TIM4, ENABLE);
          Delay(0x15FFFF);
          
          TIM_TimeBaseStructure.TIM_Period = 302/(pow(2,ADC_ConvertedValue>>10));       //10khz/(9999+1)=1hz
          TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
          TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period * 0.8;
          TIM_OCInit(TIM4, &TIM_OCInitStructure);	
          TIM_Cmd(TIM4, ENABLE);
          Delay(0x15FFFF);
          
          TIM_TimeBaseStructure.TIM_Period = 381/(pow(2,ADC_ConvertedValue>>10));       //10khz/(9999+1)=1hz
          TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
          TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period * 0.8;
          TIM_OCInit(TIM4, &TIM_OCInitStructure);	
          TIM_Cmd(TIM4, ENABLE);
          Delay(0x15FFFF);
          
          TIM_Cmd(TIM4, DISABLE);
  
          GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  
	  
}

void Sound2(void)
{
          GPIO_InitTypeDef GPIO_InitStructure;
          TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure2;
          
          /*[SOUND] Configure PB.08 as Output push-pull [SOUND]*/
          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //SET AS SECOND DEVICE OUTPUT ON GPIO
          GPIO_Init(GPIOB, &GPIO_InitStructure);
  
          
          
          GPIO_SetBits(GPIOC, GPIO_Pin_9);
        
        //C382 D341 E303 F286 G255 A227 B203 (-1) C2-191
	  TIM_TimeBaseStructure2.TIM_Period = 381/(pow(2,ADC_ConvertedValue>>10));       //100khz/(226+1)=440.5hz   381/(2*pow(ADC_ConvertedValue>>11))
	  TIM_TimeBaseStructure2.TIM_Prescaler = 359;  //36mhz/(359+1)=100khz
	  TIM_TimeBaseStructure2.TIM_ClockDivision = 0; //normally set 0
	  TIM_TimeBaseStructure2.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure2);
	  TIM_Cmd(TIM2, ENABLE);  
	  Delay(0x15FFFF);
	  
	  TIM_TimeBaseStructure2.TIM_Period = 302/(pow(2,ADC_ConvertedValue>>10));       //100khz/(226+1)=440.5hz
	  TIM_TimeBaseStructure2.TIM_Prescaler = 359;  //36mhz/(359+1)=100khz
	  TIM_TimeBaseStructure2.TIM_ClockDivision = 0; //normally set 0
	  TIM_TimeBaseStructure2.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure2);
	  TIM_Cmd(TIM2, ENABLE);  
	  Delay(0x15FFFF);
	  
	  TIM_TimeBaseStructure2.TIM_Period = 254/(pow(2,ADC_ConvertedValue>>10));       //100khz/(226+1)=440.5hz
	  TIM_TimeBaseStructure2.TIM_Prescaler = 359;  //36mhz/(359+1)=100khz
	  TIM_TimeBaseStructure2.TIM_ClockDivision = 0; //normally set 0
	  TIM_TimeBaseStructure2.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure2);
	  TIM_Cmd(TIM2, ENABLE);  
	  Delay(0x15FFFF);
	  
	  TIM_TimeBaseStructure2.TIM_Period = 190/(pow(2,ADC_ConvertedValue>>10));       //100khz/(226+1)=440.5hz
	  TIM_TimeBaseStructure2.TIM_Prescaler = 359;  //36mhz/(359+1)=100khz
	  TIM_TimeBaseStructure2.TIM_ClockDivision = 0; //normally set 0
	  TIM_TimeBaseStructure2.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure2);
	  TIM_Cmd(TIM2, ENABLE);  
	  Delay(0x15FFFF);
	  
	  TIM_Cmd(TIM2, DISABLE);
          
  
          GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  
	  
}

void Music(void)
{
		 int i = 0;
  
		//tim2 sound frq: C261.6HZ D293.6HZ E329.6HZ F349.2HZ G392HZ A440HZ B493.8HZ
						//C381 D340 E302 F285 G254 A226 B202  c190(+1)
              
		/*int note[] = {e1,e1,f1,g1,g1,f1,e1,d1,
                              c1,c1,d1,e1,e1,d1,d1,
                              e1,e1,f1,g1,g1,f1,e1,d1,
                               c1,c1,d1,e1, d1,c1,c1,0};//zero for end of song
        */
		int note[256]=0;//automaticly end with 0
		//ode an die freude(Choral)
		int song1[] = {e1,e1,f1,g1,g1,f1,e1,d1,
					   c1,c1,d1,e1,e1,d1,d1,
					   //e1,e1,f1,g1,g1,f1,e1,d1,
					   //c1,c1,d1,e1, d1,c1,c1,
					   0};//zero for end of song
		//canon in D
		int song2[] = {g2,g2,e2,f2,g2,g2,e2,f2,
					   g2,g1,a1,b1,c2,d2,e2,f2,
					   e2,e2,c2,d2,e2,e2,e1,f1,
					   g1,a1,g1,f1,g1,c2,b1,c2,
					   a1,a1,c2,b1,a1,a1,g1,f1,
					   g1,f1,e1,f1,g1,a1,b1,c2,
					   a1,a1,c2,b1,c2,c2,b1,a1,
					   b1,c2,d2,c2,b1,c2,a1,b1,
					   c2,c2,c2,c2,
					   0};//zero for end of song
		if(soundindex == 0){
			for(i=0;song1[i] != 0 && i<255;i++){
				note[i] = song1[i];
			}			   
		}
		if(soundindex == 1){
			for(i=0;song2[i] != 0 && i<255;i++){
				note[i] = song2[i];
			}			   
		}
		 

		  /*tim4_ch3 for PWM Audio*/
		  TIM_TimeBaseStructure.TIM_Period = note[0]/(pow(2,ADC_ConvertedValue>>10));       //10khz/(9999+1)=1hz
		  TIM_TimeBaseStructure.TIM_Prescaler =359;  //36mhz/(3599+1)=10khz
		  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		  
		  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//pwm mode 1		
		  TIM_OCInitStructure.TIM_Channel = TIM_Channel_3;			     
		  TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period * 0.8;//duty: 200/227 = 0.88
		  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		
		  TIM_OCInit(TIM4, &TIM_OCInitStructure);		//channel 3 -> OC3		
		  // tim4_ch3 oc reload enable 
		  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);		//channel 3 -> OC3	
		  // tim4_ch3 reload enable
		  TIM_ARRPreloadConfig(TIM4, ENABLE);

		  TIM_Cmd(TIM4, ENABLE);
          Delay(0x15FFFF);
          
          for(i = 1;i <= (sizeof(note) / sizeof(note[0]))-1 && note[i] != 0;i++){
			  TIM_TimeBaseStructure.TIM_Period = note[i]/(pow(2,ADC_ConvertedValue>>10));       //10khz/(9999+1)=1hz
			  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
			  TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period * 0.8;
			  TIM_OCInit(TIM4, &TIM_OCInitStructure);	
			  TIM_Cmd(TIM4, ENABLE);
			  Delay(0x15FFFF);
          }
          
          TIM_Cmd(TIM4, DISABLE);
  
          GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  
	  
}
/*******************************************************************************
* Function Name  : btnKey
* Description    : response when key is pressed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void btnKey(void)
{

}

/*******************************************************************************
* Function Name  : digitron
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void dgtDisplay(void)
{
  if(ledstate ==0){
  
  }
  
  if(ledstate ==1){
  
  }
    I2C_shuma_ByteWrite(dgt5, 0x10);//点亮第一个数码管
   Delay(0xBFFFF);
    
   I2C_shuma_ByteWrite(dgt6, 0x11);//点亮第二个数码管
   Delay(0xBFFFF);
  
   I2C_shuma_ByteWrite(dgt7, 0x12);//点亮第三个数码管
   Delay(0xBFFFF);
   
   I2C_shuma_ByteWrite(dgt8, 0x13);//点亮第4个数码管
   Delay(0xBFFFF);
   
   I2C_shuma_ByteWrite(dgt9, 0x10);//点亮第一个数码管
   Delay(0xBFFFF);
    
   I2C_shuma_ByteWrite(dgt0, 0x11);//点亮第二个数码管
   Delay(0xBFFFF);
  
   I2C_shuma_ByteWrite(dgta, 0x12);//点亮第三个数码管
   Delay(0xBFFFF);
   
   I2C_shuma_ByteWrite(dgtb, 0x13);//点亮第4个数码管
   Delay(0xBFFFF);
   
   I2C_shuma_ByteWrite(dgtc, 0x10);//点亮第一个数码管
   Delay(0xBFFFF);
    
   I2C_shuma_ByteWrite(dgtd, 0x11);//点亮第二个数码管
   Delay(0xBFFFF);
  
   I2C_shuma_ByteWrite(dgte, 0x12);//点亮第三个数码管
   Delay(0xBFFFF);
   
   I2C_shuma_ByteWrite(dgtf, 0x13);//点亮第4个数码管
   Delay(0xBFFFF);
}

void dgtDisable(void)
{
   I2C_shuma_ByteWrite(0, 0x10);
    
   I2C_shuma_ByteWrite(0, 0x11);
  
   I2C_shuma_ByteWrite(0, 0x12);
   
   I2C_shuma_ByteWrite(0, 0x13);
}
/*******************************************************************************
* Function Name  : btnJS
* Description    : response when key is pressed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void btnJS(void)
{
  
  if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8) == 0)//UP
    {
      ledstate = 0;
      
      dgtDisable();
      I2C_shuma_ByteWrite(dgta, 0x13);
      I2C_shuma_ByteWrite(dgtd, 0x12);
      I2C_shuma_ByteWrite(dgtc, 0x11);
      I2C_shuma_ByteWrite(dgt1, 0x10);
      GPIO_SetBits(GPIOC, GPIO_Pin_9);
      Delay(0x15FFFF);
      GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    }
  
  if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14) == 0)//DOWN
    {
      ledstate = 1;
      
      dgtDisable();
      I2C_shuma_ByteWrite(dgta, 0x13);
      I2C_shuma_ByteWrite(dgtd, 0x12);
      I2C_shuma_ByteWrite(dgtc, 0x11);
      I2C_shuma_ByteWrite(dgt2, 0x10);
      GPIO_SetBits(GPIOC, GPIO_Pin_9);
      Delay(0x15FFFF);
      GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    }
  
  if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1) == 0)//LEFT
    {
      soundindex = 0;
      
      dgtDisable();
      I2C_shuma_ByteWrite(dgt5, 0x11);
      I2C_shuma_ByteWrite(dgt1, 0x10);
      GPIO_SetBits(GPIOC, GPIO_Pin_9);
      Delay(0x15FFFF);
      GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    }
  
  if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) == 0)//RIGHT
    {
      soundindex = 1;
      
      dgtDisable();
      I2C_shuma_ByteWrite(dgt5, 0x11);
      I2C_shuma_ByteWrite(dgt2, 0x10);
       GPIO_SetBits(GPIOC, GPIO_Pin_9);
      Delay(0x15FFFF);
      GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    }
  
  if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12) == 0)//SET
    {
      GPIO_SetBits(GPIOC, GPIO_Pin_9);
      Delay(0x15FFFF);
      GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    }
}

/*******************************************************************************
* Function Name  : ledADC
* Description    : turn on/off led when ADC is changed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ledADC(void)
{
	
	
	if(ledstate == 0){  //potentiometer to led
	  if(ADC_ConvertedValue>=1000){
		   GPIO_SetBits(GPIOC, GPIO_Pin_6);
		}else{
		  GPIO_ResetBits(GPIOC, GPIO_Pin_6);
		}
	  
		if(ADC_ConvertedValue>=2000){
		   GPIO_SetBits(GPIOC, GPIO_Pin_7);
		}else{
		  GPIO_ResetBits(GPIOC, GPIO_Pin_7);
		}
	  
	  if(ADC_ConvertedValue>=3000){
		   GPIO_SetBits(GPIOC, GPIO_Pin_8);
		}else{
		  GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		}
	  
	  if(ADC_ConvertedValue>=4000){
		   GPIO_SetBits(GPIOC, GPIO_Pin_9);
		}else{
		  GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		}
	}
	
	
	if(ledstate == 1){				//mic to led
	  if(ADC2_ConvertedValue>=1900){
		   GPIO_SetBits(GPIOC, GPIO_Pin_6);
		}else{
		  GPIO_ResetBits(GPIOC, GPIO_Pin_6);
		}
	  
		if(ADC2_ConvertedValue>=2100){
		   GPIO_SetBits(GPIOC, GPIO_Pin_7);
		}else{
		  GPIO_ResetBits(GPIOC, GPIO_Pin_7);
		}
	  
	  if(ADC2_ConvertedValue>=2200){
		   GPIO_SetBits(GPIOC, GPIO_Pin_8);
		}else{
		  GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		}
	  
	  if(ADC2_ConvertedValue>=2300){
		   GPIO_SetBits(GPIOC, GPIO_Pin_9);
		}else{
		  GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		}
          if(ADC2_ConvertedValue>=3500){
                   GPIO_SetBits(GPIOC, GPIO_Pin_7);
		   GPIO_SetBits(GPIOC, GPIO_Pin_9);
                   GPIO_ResetBits(GPIOC, GPIO_Pin_8);
                   GPIO_ResetBits(GPIOC, GPIO_Pin_6);
		   Delay(0x9FFFFF);
		   Music();//if reached max volum sing a song
		}
	}
}

/*******************************************************************************
* Function Name  : ADCValueConvert
* Description    : separate adc1_dr value to adc1 and adc2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADCValueConvert(void)
{
	ADC_ConvertedValue = (u16)ADC_DR_Value;
	ADC2_ConvertedValue = (u16)(ADC_DR_Value>>16);
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
