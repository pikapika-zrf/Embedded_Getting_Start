/*lighting PB5 led*/
#include "Mystm32f103.h"
typedef unsigned int uint;

void SystemInit(void)
{
    
}

void delay(uint i)
{
    while(i--);
}

int main()
{
    RCC_APB2ENR |= 1<<3;
    GPIOB_CRL &= ~(0xF<<4*5);   //20~23Î»ÇåÁã
    GPIOB_CRL |= (3<<4*5);
    GPIOB_BSRR = (1<<(16+5));
    
    while(1)
    {
        GPIOB_BSRR = (1<<(16+5));   //PB5ÖÃ0
        delay(0xFFFF);
        
        GPIOB_BSRR = (1<<(5));  //PB5ÖÃ1
        delay(0xFFFF);
    }
}
