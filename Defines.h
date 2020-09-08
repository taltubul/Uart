#include <stdint.h>


#define Base 0x40000000

#define RCC_Base (Base + 0x21000)
#define USART1_Base (Base + 0x13800)
#define GPIOA_Base (Base + 0x10800)
#define GPIOB_Base (Base + 0x10C00)
#define TIM_4_Base (Base + 0x00800)
#define TIM_3_Base (Base + 0x00400)
#define Counter_period 3750


typedef struct TIM_4
	{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SMCR;
	uint32_t DIER;
	uint32_t SR;
	uint32_t EGR;
	uint32_t CCMR1;
	uint32_t CCMR2_;	
	uint32_t CCER;
	uint32_t CNT;
	uint32_t PSC;
	uint32_t ARR;
	uint32_t RES1;
	uint32_t CCR1;
	uint32_t CCR2;
	uint32_t CCR3;
	uint32_t CCR4;
	uint32_t RES2;
	uint32_t DCR;
	uint32_t DMAR;
} TIM_4;
	typedef struct TIM_3
	{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SMCR;
	uint32_t DIER;
	uint32_t SR;
	uint32_t EGR;
	uint32_t CCMR1;
	uint32_t CCMR2_;	
	uint32_t CCER;
	uint32_t CNT;
	uint32_t PSC;
	uint32_t ARR;
	uint32_t RES1;
	uint32_t CCR1;
	uint32_t CCR2;
	uint32_t CCR3;
	uint32_t CCR4;
	uint32_t RES2;
	uint32_t DCR;
	uint32_t DMAR;
} TIM_3;

typedef struct GPIO_A
	{
	uint32_t CRL;
	uint32_t CRH;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t BRR;
	uint32_t LCKR;
} GPIO_A;
	
typedef struct GPIO_B
	{
	uint32_t CRL;
	uint32_t CRH;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t BRR;
	uint32_t LCKR;
} GPIO_B;
	
