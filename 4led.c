#include "main.h"
void delay(int n){
	for(int i=0;i<=n;i++){

	}
}
int main(void){
	RCC->APB2ENR |= 0x1<<2;
	RCC->APB2ENR |= 0x1<<3;
	//--------------------------------------
	GPIOA->CRH &= ~(0x1<<18);
	GPIOA->CRH |= 0x1<<16;//pa5 output
	GPIOA->CRH &= ~(0x1<<14);
	GPIOA->CRH |= 0x1<<12;//pa5 output
	//--------------------------------------
	GPIOB->CRH &= ~(0x1<<18);
	GPIOB->CRH |= 0x1<<16;//pa5 output
	GPIOB->CRH &= ~(0x1<<14);
	GPIOB->CRH |= 0x1<<12;//pa5 output
	//---------------------------------------
	GPIOA->ODR |= 0x1<<11;
	GPIOB->ODR |= 0x1<<11;
	GPIOB->ODR |= 0x1<<12;
	while(1){
		GPIOA->BSRR |= 0x1<<12;//LED reset
		delay(100000);
		GPIOA->BSRR |= 0x1<<11;//LED reset
		delay(100000);
		GPIOA->BSRR |= 0x1<<28;//LED reset
		delay(100000);
		GPIOA->BSRR |= 0x1<<27;//LED reset
		delay(100000);
		//------------------------------------------
		GPIOB->BSRR |= 0x1<<12;//LED reset
		delay(100000);
		GPIOB->BSRR |= 0x1<<11;//LED reset
		delay(100000);
		GPIOB->BSRR |= 0x1<<28;//LED reset
		delay(100000);
		GPIOB->BSRR |= 0x1<<27;//LED reset
		delay(100000);
	}

}
