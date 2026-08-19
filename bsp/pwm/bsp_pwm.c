#include "bsp_pwm.h"
#include "stdlib.h"
#include "memory.h"
#include "omni_led.h"

// 配合中断以及初始化
static uint8_t idx;
static PWMInstance *pwm_instance[PWM_DEVICE_CNT] = {NULL}; // 所有的pwm instance保存于此,用于callback时判断中断来源
static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim );
/**
 * @brief pwm dma传输完成回调函数
 *
 * @param htim 发生中断的定时器句柄
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    OmniLedPulseFinishedCallback(htim);
    for (uint8_t i = 0; i < idx; i++)
    { // 来自同一个定时器的中断且通道相同
        if (pwm_instance[i]->htim == htim && htim->Channel == (1<<(pwm_instance[i]->channel/4)))
        {
            if (pwm_instance[i]->callback) // 如果有回调函数
                pwm_instance[i]->callback(pwm_instance[i]);
            return; // 一次只能有一个通道的中断,所以直接返回
        }
    }
}

PWMInstance *PWMRegister(PWM_Init_Config_s *config)
{
    if (idx >= PWM_DEVICE_CNT) // 超过最大实例数,考虑增加或查看是否有内存泄漏
        while (1)
            ;
    PWMInstance *pwm = (PWMInstance *)malloc(sizeof(PWMInstance));
    memset(pwm, 0, sizeof(PWMInstance));

    pwm->htim = config->htim;
    pwm->channel = config->channel;
    pwm->period = config->period;
    pwm->dutyratio = config->dutyratio;
    pwm->callback = config->callback;
    pwm->id = config->id;
    pwm->tclk = PWMSelectTclk(pwm->htim);
    // 启动PWM
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    PWMSetPeriod(pwm, pwm->period);
    PWMSetDutyRatio(pwm, pwm->dutyratio);
    pwm_instance[idx++] = pwm;
    return pwm;
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWMStart(PWMInstance *pwm)
{
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWMStop(PWMInstance *pwm)
{
    HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
}

/*
 * @brief 设置pwm周期
 *
 * @param pwm pwm实例
 * @param period 周期 单位 s
 */
void PWMSetPeriod(PWMInstance *pwm, float period)
{
    __HAL_TIM_SetAutoreload(pwm->htim, period*((pwm->tclk)/(pwm->htim->Init.Prescaler+1)));
}
/*
    * @brief 设置pwm占空比
    *
    * @param pwm pwm实例
    * @param dutyratio 占空比 0~1
*/
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio)
{
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, dutyratio * (pwm->htim->Instance->ARR));
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size)
{
    HAL_TIM_PWM_Start_DMA(pwm->htim, pwm->channel, pData, Size);
}

// 设置pwm对应定时器时钟源频率
//tim2~7,12~14:APB1  tim1,8~11:APB2
static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim )
{
    RCC_ClkInitTypeDef clocks;
    uint32_t flash_latency;
    uint32_t pclk;
    uint32_t divider;

    HAL_RCC_GetClockConfig(&clocks, &flash_latency);

    if (htim->Instance == TIM1
#if defined(TIM8)
        || htim->Instance == TIM8
#endif
#if defined(TIM15)
        || htim->Instance == TIM15 || htim->Instance == TIM16 || htim->Instance == TIM17
#endif
    )
    {
        pclk = HAL_RCC_GetPCLK2Freq();
        divider = clocks.APB2CLKDivider;
    }
    else
    {
        pclk = HAL_RCC_GetPCLK1Freq();
        divider = clocks.APB1CLKDivider;
    }

    return divider == RCC_APB1_DIV1 ? pclk : (pclk * 2U);
}
