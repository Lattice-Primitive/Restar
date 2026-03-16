#ifndef REMOCTRL_TASK_H
#define REMOCTRL_TASK_H

#define STEPLENTH_MAX 12.0f                     //步长最大值
#define RC_CPLTCTRL_FREQ_MAX 11.0f               //遥控器控制步频最大值
#define RC_law_FREQ_MAX 7.0f               //遥控器控制步频最大值
#define RC_lawer_FREQ_MAX 5.0f               //遥控器控制步频最大值

#define RC_CPLTCTRL_STEPLENTH_MAX 11.0f         //遥控器控制步幅最大值
#define RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX 5.0f //遥控器微调步幅最大值
#define RC_CPLTCTRL_FLIGHTPERCENT_MAX 0.2f       //遥控器控制摆动相占比最大值

void RemoCtrl_Task(void *argument);

#endif /*REMOCTRL_TASK_H*/
