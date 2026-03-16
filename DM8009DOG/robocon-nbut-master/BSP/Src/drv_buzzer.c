#include "drv_buzzer.h"

void Buzzer_Note(Buzzer_Operation_t* opt,uint32_t Freq,uint32_t ticks)
{
    if(Freq==0) 
    {
        opt->Buzzer_Silent();
    }
    else
    {
        opt->Buzzer_Set_Freq(Freq);
    }
    opt->Buzzer_Delay(ticks);   
}
