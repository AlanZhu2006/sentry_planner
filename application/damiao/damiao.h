#ifndef DAMIAO_APP_H
#define DAMIAO_APP_H

/**
 * @brief 达妙应用初始化，固定创建一个达妙电机实例并切到速度模式。
 */
void DamiaoInit(void);

/**
 * @brief 达妙任务，维持一个小速度指令并周期打印电机状态。
 */
void DamiaoTask(void);

#endif // DAMIAO_APP_H
