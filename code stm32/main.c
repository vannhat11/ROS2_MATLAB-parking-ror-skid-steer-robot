#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define T    0.03f
#define CPR 506.0f

// PID & SETPOINT
float Kp = 2.2f, Ki = 21.03f, Kd = 0.0018f;
float setL = 0.0f, setR = 0.0f;
float tocdoL = 0, tocdoR = 0;
float eL = 0, e1L = 0, e2L = 0;
float eR = 0, e1R = 0, e2R = 0;
float alpha, beta, gamma;
float outL = 0, outR = 0, lastOutL = 0, lastOutR = 0;

#define BOOST_L    8
#define OFFSET_R   0

// UART Buffer
#define RX_BUF_SIZE 40
char rx_buf[RX_BUF_SIZE];
uint8_t rx_index = 0;
volatile uint8_t data_ready_flag = 0;

// Bi?n chia t?n s? g?i d? li?u trong SysTick
uint8_t feedback_divider = 0;

void Clock72MHz(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;
    RCC->CFGR |= RCC_CFGR_PLLSRC;
    RCC->CFGR |= RCC_CFGR_PLLMULL9;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void USART2_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
    GPIOA->CRL |= (GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1) | GPIO_CRL_CNF2_1;
    GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);
    GPIOA->CRL |= GPIO_CRL_CNF3_0;
    USART2->BRR = (0x13 << 4) | 0x9;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    USART2->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void)
{
    if(USART2->SR & USART_SR_RXNE)
    {
        char c = (char)(USART2->DR & 0xFF);
        if(c != '\n' && c != '\r' && rx_index < (RX_BUF_SIZE - 1)) { rx_buf[rx_index++] = c; }
        else if(c == '\n' || c == '\r') { if(rx_index > 0) { rx_buf[rx_index] = '\0'; data_ready_flag = 1; rx_index = 0; } }
    }
}

void Send_Feedback_To_Pi(void)
{
    char tx_buf[40];
    sprintf(tx_buf, "%.2f,%.2f\n", tocdoL, tocdoR);
    for(int i = 0; tx_buf[i] != '\0'; i++)
    {
        while(!(USART2->SR & USART_SR_TXE));
        USART2->DR = tx_buf[i];
    }
}

void Parse_Pi_Commands(void)
{
    if(data_ready_flag)
    {
        char *token = strtok(rx_buf, ",");
        if(token != NULL) { setL = atof(token); token = strtok(NULL, ","); if(token != NULL) setR = atof(token); }
        data_ready_flag = 0;
    }
}

void Motor_GPIO_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOB->CRL |= GPIO_CRL_MODE0 | GPIO_CRL_MODE1;
    GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10 | GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
    GPIOB->CRH |= GPIO_CRH_MODE10 | GPIO_CRH_MODE11;
    GPIOB->BRR = GPIO_BRR_BR0 | GPIO_BRR_BR1 | GPIO_BRR_BR10 | GPIO_BRR_BR11;
}

void PWM_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOA->CRL |= GPIO_CRL_MODE6 | GPIO_CRL_CNF6_1 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7_1;
    GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8 | GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOA->CRH |= GPIO_CRH_MODE8 | GPIO_CRH_CNF8_1 | GPIO_CRH_MODE10 | GPIO_CRH_CNF10_1;
    TIM3->PSC = 72 - 1; TIM3->ARR = 255;
    TIM3->CCR1 = 0; TIM3->CCR2 = 0;
    TIM3->CCMR1 |= (6 << 4) | (6 << 12);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->CR1 |= TIM_CR1_ARPE; TIM3->EGR |= TIM_EGR_UG; TIM3->CR1 |= TIM_CR1_CEN;
    TIM1->PSC = 72 - 1; TIM1->ARR = 255;
    TIM1->CCR1 = 0; TIM1->CCR3 = 0;
    TIM1->CCMR1 |= (6 << 4) | TIM_CCMR1_OC1PE;
    TIM1->CCMR2 |= (6 << 4) | TIM_CCMR2_OC3PE;
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_ARPE; TIM1->EGR |= TIM_EGR_UG; TIM1->CR1 |= TIM_CR1_CEN;
}

void Encoder_Hardware_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOA->CRL |= GPIO_CRL_CNF0_1 | GPIO_CRL_CNF1_1;
    GPIOA->ODR |= (1 << 0) | (1 << 1);
    GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOB->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1;
    GPIOB->ODR |= (1 << 6) | (1 << 7);
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    TIM2->SMCR |= (3 << 0);
    TIM2->ARR = 0xFFFF; TIM2->CNT = 0; TIM2->CR1 |= TIM_CR1_CEN;
    TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    TIM4->SMCR |= (3 << 0);
    TIM4->ARR = 0xFFFF; TIM4->CNT = 0; TIM4->CR1 |= TIM_CR1_CEN;
}

void Drive_L298N_Left(float pwm)
{
    uint16_t rearPWM; uint16_t frontPWM;
    if(pwm > 0.0f) { GPIOB->BSRR = GPIO_BSRR_BS0; GPIOB->BRR = GPIO_BRR_BR1; rearPWM = (uint16_t)((int16_t)pwm); frontPWM = rearPWM + BOOST_L; if(frontPWM > 255) frontPWM = 255; TIM3->CCR1 = rearPWM; TIM1->CCR1 = frontPWM; }
    else if(pwm < 0.0f) { GPIOB->BRR = GPIO_BRR_BR0; GPIOB->BSRR = GPIO_BSRR_BS1; rearPWM = (uint16_t)((int16_t)(-pwm)); frontPWM = rearPWM + BOOST_L; if(frontPWM > 255) frontPWM = 255; TIM3->CCR1 = rearPWM; TIM1->CCR1 = frontPWM; }
    else { GPIOB->BRR = GPIO_BRR_BR0 | GPIO_BRR_BR1; TIM3->CCR1 = 0; TIM1->CCR1 = 0; }
}

void Drive_L298N_Right(float pwm)
{
    uint16_t rearPWM; uint16_t frontPWM;
    if(pwm > 0.0f) { GPIOB->BRR = GPIO_BRR_BR10; GPIOB->BSRR = GPIO_BSRR_BS11; rearPWM = (uint16_t)((int16_t)pwm); if(rearPWM > OFFSET_R) frontPWM = rearPWM - OFFSET_R; else frontPWM = 0; TIM3->CCR2 = rearPWM; TIM1->CCR3 = frontPWM; }
    else if(pwm < 0.0f) { GPIOB->BSRR = GPIO_BSRR_BS10; GPIOB->BRR = GPIO_BRR_BR11; rearPWM = (uint16_t)((int16_t)(-pwm)); if(rearPWM > OFFSET_R) frontPWM = rearPWM - OFFSET_R; else frontPWM = 0; TIM3->CCR2 = rearPWM; TIM1->CCR3 = frontPWM; }
    else { GPIOB->BRR = GPIO_BRR_BR10 | GPIO_BRR_BR11; TIM3->CCR2 = 0; TIM1->CCR3 = 0; }
}

void PID_Control(void)
{
    int16_t countL = -((int16_t)TIM4->CNT);
    int16_t countR = (int16_t)TIM2->CNT;
    TIM4->CNT = 0; TIM2->CNT = 0;
    tocdoL = ((float)countL * 60.0f) / ((CPR * 4.0f) * T);
    tocdoR = ((float)countR * 60.0f) / ((CPR * 4.0f) * T);
    alpha = 2 * T * Kp + Ki * T * T + 2 * Kd;
    beta  = Ki * T * T - 4 * Kd - 2 * T * Kp;
    gamma = 2 * Kd;
    outL = (alpha * (setL - tocdoL) + beta * e1L + gamma * e2L + 2 * T * lastOutL) / (2 * T);
    outR = (alpha * (setR - tocdoR) + beta * e1R + gamma * e2R + 2 * T * lastOutR) / (2 * T);
    if(outL > 255.0f) outL = 255.0f; if(outL < -255.0f) outL = -255.0f;
    if(outR > 255.0f) outR = 255.0f; if(outR < -255.0f) outR = -255.0f;
    lastOutL = outL; e2L = e1L; e1L = (setL - tocdoL);
    lastOutR = outR; e2R = e1R; e1R = (setR - tocdoR);
    Drive_L298N_Left(outL); Drive_L298N_Right(outR);
}

// Ng?t d?nh th?i ch?y v?i t?n s? 33Hz d? l?y m?u PID
void SysTick_Handler(void) 
{ 
    PID_Control(); 
    
    // S?a d?i b? chia: M?i chu k? ng?t (33Hz ~ 30.3ms) thì g?i d? li?u feedback lên Pi luôn
    if(++feedback_divider >= 1) 
    {
        Send_Feedback_To_Pi();
        feedback_divider = 0;
    }
}

int main(void)
{
    Clock72MHz();
    USART2_Init();
    Motor_GPIO_Init();
    PWM_Init();
    Encoder_Hardware_Init();
    
    // C?u hình ng?t th?i gian th?c d?nh th?i 33Hz
    SysTick_Config(SystemCoreClock / 33);
    
    while(1)
    {
        // Vòng l?p chính hoàn toàn r?nh r?i d? b?t chu?i kí t? nh?n t? Raspberry Pi qua ng?t UART
        Parse_Pi_Commands();
    }
}