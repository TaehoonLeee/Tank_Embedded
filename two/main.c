//flash load "C:\Users\Team03\Documents\DS-5 Workspace\two\flashclear.axf"
//flash load "C:\Users\Team03\Documents\DS-5 Workspace\two\Debug\two.axf"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_dac.h"
#include "misc.h"
#include "core_cm3.h"
#include "lcd.h"
#include "touch.h"

#define ADC1_DR ((u32) 0x4001244C)
#define ADC2_DR ((u32) 0x4001284C)

void selectionMode(unsigned char d);

int upInput = 0, downInput = 0, leftInput = 0, rightInput = 0, attackInput = 0, stopInput = 0, scanInput = 0;

DMA_InitTypeDef DMA_InitStructure;
ADC_InitTypeDef ADC_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure2;
TIM_OCInitTypeDef TIM_OCInitStructure2;

__IO uint32_t ADC_value;
int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};

unsigned int infraValue = 0, lightValue = 0;
unsigned char d = 0;

void delay() {
   int i = 10000000;
   while(i--);
}

void USART1_Init(void)
{
  USART_InitTypeDef usart1_init_struct;
  GPIO_InitTypeDef gpioa_init_struct;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO |
        RCC_APB2Periph_GPIOA, ENABLE);

  gpioa_init_struct.GPIO_Pin = GPIO_Pin_9;
  gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
  gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &gpioa_init_struct);
  gpioa_init_struct.GPIO_Pin = GPIO_Pin_10;
  gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
  gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpioa_init_struct);

  usart1_init_struct.USART_BaudRate = 57600;
  usart1_init_struct.USART_WordLength = USART_WordLength_8b;
  usart1_init_struct.USART_StopBits = USART_StopBits_1;
  usart1_init_struct.USART_Parity = USART_Parity_No;
  usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1, &usart1_init_struct);
  USART_Cmd(USART1, ENABLE);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void USART2_Init(void)
{
  USART_InitTypeDef usart2_init_struct;
  GPIO_InitTypeDef gpioa_init_struct;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  // tx, rx 설정
  gpioa_init_struct.GPIO_Pin = GPIO_Pin_2;
  gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
  gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &gpioa_init_struct);

  gpioa_init_struct.GPIO_Pin = GPIO_Pin_3;
  gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
  gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpioa_init_struct);

  usart2_init_struct.USART_BaudRate = 9600;
  usart2_init_struct.USART_WordLength = USART_WordLength_8b;
  usart2_init_struct.USART_StopBits = USART_StopBits_1;
  usart2_init_struct.USART_Parity = USART_Parity_No;
  usart2_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  usart2_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART2, &usart2_init_struct);
  USART_Cmd(USART2, ENABLE);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void light_setting() {
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void infrared_setting() {
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void led_setting() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void wheel_setting() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void attacker_setting() {
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void servo_setting() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

//    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) 72 - 1;
}

static void timer_init() {
TIM_TimeBaseInitTypeDef timer2;
timer2.TIM_Prescaler = 1000 / 36; //우리 system clock이 20MHZ라서 만약 40이면 1000/18로
timer2.TIM_Period = 14399;
timer2.TIM_ClockDivision = TIM_CKD_DIV1;
timer2.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM2, &timer2);
TIM_Cmd(TIM2, ENABLE);
}

static void timer_init2(){
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;

TIM_TimeBaseStructure.TIM_Period = 10000;
TIM_TimeBaseStructure.TIM_Prescaler =(72-1);
TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

 TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
 TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
 TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
 TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
 TIM_ICInitStructure.TIM_ICFilter = 0x03;
 TIM_ICInit(TIM5, &TIM_ICInitStructure);

 TIM_Cmd(TIM5,ENABLE );
 TIM_ITConfig( TIM5,TIM_IT_Update|TIM_IT_CC2,ENABLE);
}

static void PWM_Init() {
TIM_OCInitTypeDef TIM_OCInitStructure;

TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_Pulse = 0;
TIM_OC1Init(TIM2, &TIM_OCInitStructure);
TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
TIM_OC2Init(TIM2, &TIM_OCInitStructure);
TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
TIM_ARRPreloadConfig(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // Clear the interrupt flag
    }

    if(TIM_GetITStatus(TIM2,TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIM2,TIM_IT_CC1);
    }
}

void Touch_LCD_setting() {
   LCD_Init();
   Touch_Configuration();
   Touch_Adjust();
   LCD_Clear(WHITE);

   LCD_ShowString(50, 50, "TANK_TERM!!", BLUE, WHITE);
   LCD_ShowString(20, 80, "infrared", BLUE, WHITE);
   LCD_ShowString(100, 80, "Light", BLUE, WHITE);
}

//DMA 설정
void DMA_setting(){
   //DMA1 channel1 configuration

   DMA_DeInit(DMA1_Channel1);
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
   DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ADC_value;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
   DMA_InitStructure.DMA_BufferSize = 1;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
   DMA_Init(DMA1_Channel1, &DMA_InitStructure);

   //Enable DMA1 Channel1
   DMA_Cmd(DMA1_Channel1, ENABLE);
}

//적외선 값 측정
int infraCheck(){
   if(!(GPIOD->IDR & GPIO_Pin_12)) {
     LCD_ShowString(70, 170, "Detect something", BLACK, WHITE);
     return 1;
   }

   //전방에 물체 없을때
   LCD_ShowString(70, 170, "Detect nothing!!", BLACK, WHITE);
   return 0;
}

//ADC 설정
void ADC_setting(){
   //ADC1 configuration
   ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
   ADC_InitStructure.ADC_ScanConvMode = ENABLE;
   ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   ADC_InitStructure.ADC_NbrOfChannel = 1;
   ADC_Init(ADC1, &ADC_InitStructure);

   //ADC1 regular channel14 configuration
   ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_239Cycles5);

   //Enable ADC1 DMA
   ADC_DMACmd(ADC1, ENABLE);

   //Enable ADC1
   ADC_Cmd(ADC1, ENABLE);

   //Enable ADC1 reset calibaration register
   ADC_ResetCalibration(ADC1);
   while(ADC_GetResetCalibrationStatus(ADC1))
      ;
   ADC_StartCalibration(ADC1);
   while(ADC_GetCalibrationStatus(ADC1))
      ;

   //Start ADC1 Software Conversion
   ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void DAC_setting() {
   DAC_InitTypeDef DAC_InitStructure;

   RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
   DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Noise;
   DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits9_0;
   DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
   DAC_Init(DAC_Channel_1, &DAC_InitStructure);

   DAC_Cmd(DAC_Channel_1, ENABLE);

   DAC_SetChannel1Data(DAC_Align_12b_L, 0x7ff0);

   DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
}

void scan(int num){
	if (num == 0) { //
		TIM2->CCR1 = 6500; //90도 반시계방향
	}
	else if (num == 1) { //
		TIM2->CCR1 = 4000; //180도 시계방향
	}
}

//감시기 컨트롤
void head_control() {
	int i;
	for(i = 4000; i <= 6500; i++) {
		TIM2->CCR1 = i;
		if(infraCheck() == 1) {
			scanInput = 0;
		}
	}
	for(i = 6500; i >= 4000; i--) {
		TIM2->CCR1 = i;
		if(infraCheck() == 1) {
			scanInput = 0;
		}
	}
//	int i;
//	for(i = 0;i<5000000;i++);
//	scan(0);
//	for(i = 0;i<5000000;i++);
//	scan(1);
}

//공격기 컨트롤
void attack_control(){

}

void show_adc_value(){
	int i = 0;
   lightValue = ADC_value; //조도
//   infraValue = ADC_value[1];

   LCD_ShowNum(20, 110, infraValue, 4, BLACK, WHITE);
   LCD_ShowNum(100, 110, lightValue, 4, BLACK, WHITE);

   selectionMode(d);

   scan(1);
   for(i = 0; i < 5000000; i++);
   scan(0);
   for(i = 0; i < 5000000; i++);

   //공격모드
   if (attackInput == 1){
      LCD_ShowString(100, 240, "ATACK", BLACK, WHITE);
      GPIOE->BSRR &= (uint32_t) (~(GPIO_BSRR_BS9)) | (~(GPIO_BSRR_BS11))
                  | (~(GPIO_BSRR_BS10)) | (~(GPIO_BSRR_BS12));
      GPIOE->BRR &= (uint32_t) (~(GPIO_BSRR_BS9)) | (~(GPIO_BSRR_BS11))
                  | (~(GPIO_BSRR_BS10)) | (~(GPIO_BSRR_BS12));
      GPIOE->BRR |= (uint32_t) (GPIO_BSRR_BS9 | GPIO_BSRR_BS11
            | GPIO_BSRR_BS10 | GPIO_BSRR_BS12);
      attack_control();
      attackInput = 0;
   }

   //전진
   else if (upInput == 1){
      LCD_ShowString(100, 240, "UP!!!", BLACK, WHITE);
      GPIO_SetBits(GPIOC, GPIO_Pin_9);
      GPIO_ResetBits(GPIOC, GPIO_Pin_8);
      GPIO_SetBits(GPIOC, GPIO_Pin_11);
      GPIO_ResetBits(GPIOC, GPIO_Pin_10);
   }

   //후진 //  오른쪽 바퀴가 전진으로감
   else if (downInput == 1){
      LCD_ShowString(100, 240, "DOWN!", BLACK, WHITE);
      GPIO_SetBits(GPIOC, GPIO_Pin_8);
      GPIO_ResetBits(GPIOC, GPIO_Pin_9);
      GPIO_SetBits(GPIOC, GPIO_Pin_10);
      GPIO_ResetBits(GPIOC, GPIO_Pin_11);
   }

   //좌회전 // 오른쪽 바퀴 정방향
   else if (leftInput == 1){
      LCD_ShowString(100, 240, "LEFT!", BLACK, WHITE);
      GPIO_SetBits(GPIOC, GPIO_Pin_9);
      GPIO_ResetBits(GPIOC, GPIO_Pin_8);
      GPIO_ResetBits(GPIOC, GPIO_Pin_11);
      GPIO_ResetBits(GPIOC, GPIO_Pin_10);
   }

   //우회전
   else if (rightInput == 1){
      LCD_ShowString(100, 240, "RIGHT", BLACK, WHITE);
      GPIO_ResetBits(GPIOC, GPIO_Pin_8);
      GPIO_ResetBits(GPIOC, GPIO_Pin_9);
      GPIO_SetBits(GPIOC, GPIO_Pin_11);
      GPIO_ResetBits(GPIOC, GPIO_Pin_10);
   }

   //정지
   else if (stopInput == 1){
      LCD_ShowString(100, 240, "STOP!", BLACK, WHITE);
      GPIO_ResetBits(GPIOC, GPIO_Pin_8);
      GPIO_ResetBits(GPIOC, GPIO_Pin_9);
      GPIO_ResetBits(GPIOC, GPIO_Pin_10);
      GPIO_ResetBits(GPIOC, GPIO_Pin_11);
   }

   //스캔모드
   else if (scanInput == 1){
      //물체 스캔
      head_control();
   }

   delay();
}

void USART1_IRQHandler(void) {
   while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
      ;

   d = (unsigned char) USART_ReceiveData(USART1);
   USART_SendData(USART2, d);
   USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}

void USART2_IRQHandler(void) {
   while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
      ;

   d = (unsigned char) USART_ReceiveData(USART2);
   USART_SendData(USART1, d);
   USART_ClearITPendingBit(USART2, USART_IT_RXNE);
}
//탱크 플레이
void play(){
   GPIO_SetBits(GPIOC, GPIO_Pin_6);
   GPIO_SetBits(GPIOC, GPIO_Pin_7);

   while(1){
      show_adc_value();
   }
}

void selectionMode(unsigned char d) {
   switch(d){
       //up
   case 'u':
         upInput=1;
         downInput=0;
         leftInput=0;
         rightInput=0;
         attackInput=0;
         stopInput=0;
         scanInput=0;
         break;
       //right
      case 'r':
         upInput=0;
         downInput=0;
         leftInput=0;
         rightInput=1;
         attackInput=0;
         stopInput=0;
         scanInput=0;
         break;
       //left
      case 'l':
         upInput=0;
         downInput=0;
         leftInput=1;
         rightInput=0;
         attackInput=0;
         stopInput=0;
         scanInput=0;
         break;
       //down
      case 'd':
         upInput=0;
         downInput=1;
         leftInput=0;
         rightInput=0;
         attackInput=0;
         stopInput=0;
         scanInput=0;
         break;
       //attack
      case 'f':
         upInput=0;
         downInput=0;
         leftInput=0;
         rightInput=0;
         attackInput=1;
         stopInput=0;
         scanInput=0;
         break;
          //stop
         case 'p':
            upInput=0;
            downInput=0;
            leftInput=0;
            rightInput=0;
            attackInput=0;
            stopInput=1;
            scanInput=0;
            break;
         //scan
      case 's':
            upInput=0;
            downInput=0;
            leftInput=0;
            rightInput=0;
            attackInput=0;
            stopInput=0;
            scanInput=1;
            break;
      default:
            break;
      }
}

int main(){
   SystemInit();
   infrared_setting();
   light_setting();
   USART1_Init();
   USART2_Init();
   led_setting();
   attacker_setting();
   servo_setting();
   wheel_setting();
   Touch_LCD_setting();

   timer_init();
   timer_init2();
   PWM_Init();

   DMA_setting();
   ADC_setting();
   DAC_setting();

   play();

   return 0;
}
