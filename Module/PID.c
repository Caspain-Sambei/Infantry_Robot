//created by Caspian Sambei in 2026/3/1
#include <math.h>
#include "reg.h"
#include "PID.h"

/**
 * @brief 积分：变速积分，积分限幅，死区冻结/清零积分
 *			微分：一阶低通滤波,前馈
 * @param p	详参PID.h
 * @param mode gimbal_mode和chassis_mode两种
 */
void PID_Update(pid *p,uint8_t mode)
{
	p->LastError = p->PreError;
	p->PreError = p->Target - p->Actual;
	// 在开头保存上一次的微分结果，因为下面会用到
	p->LAST_D_OUT = p->D_OUT;
	/***************************************************************************
	 *								积分项计算
	 **************************************************************************/
	// ========================死区冻结积分/死区积分清零======================
	// 死区积分冻结
	if (mode == 1)
	{
		if(fabs(p->PreError) <= p->DEADZONE)
			p->PreError = 0.0f;
	}
	// 死区积分清零
	else if (mode == 2)
		if(fabs(p->PreError) <= p->DEADZONE){p->SunError = 0.0f;}

	// =======================变速积分======================
	if(p->ki != 0.0f)
	{
		if (fabs(p->PreError) <= p->I_L)
		{
			p->SunError += (p->PreError + p->LastError) / 2.0f;
		}
		else if (fabs(p->PreError) <= p->I_M)
		{
			p->SunError += (p->PreError + p->LastError) / 2.0f * (p->I_M - p->PreError) / (p->I_M - p->I_L);
		}
	}
	else
		p->SunError = 0.0f;

	// =======================积分限幅======================
	if(p->SunError >= p->IMAX){p->SunError = p->IMAX;}
	if(p->SunError <= -p->IMAX){p->SunError = -p->IMAX;}

	/***************************************************************************
	 *								微分项计算
	 **************************************************************************/
	// =======================不完全微分===================
	// 在纯微分项后面串联一个一阶低通滤波器，来抑制微分项的噪声放大效应
	float temp_Out = p->kp * p->PreError + p->ki * p->SunError;

	p->D_OUT = (1 - p->RC_DF) * (temp_Out - p->LAST_OUT)
				+ p->RC_DF * p->LAST_D_OUT;

	/***************************************************************************
	 *								前馈计算
	 **************************************************************************/
	float FeedForward = p->k1 * p->Target
						+ (1 - p->k1) * (p->Target - p->last_Target);

	/***************************************************************************
	 *								增量式PID计算
	 **************************************************************************/
	p->Out = p->kp * p->PreError
			+ p->ki * p->SunError
			+ p->D_OUT
			+ FeedForward;
	//		+ p->kd * (p->PreError - p->LastError);

	/***************************************************************************
	 *								数据继承
	 **************************************************************************/
	p->LAST_OUT = p->Out;
	p->last_Target = p->Target;

	/***************************************************************************
	 *								输出限幅
	 **************************************************************************/
	if(p->Out >= p->OUTMAX){p->Out = p->OUTMAX;}
	if(p->Out <= -p->OUTMAX){p->Out = -p->OUTMAX;}
}

/**
 * @brief PID参数初始化
 * @param p PID结构体类型指针
 * @param kp P
 * @param ki I
 * @param kd D
 * @param OUTMAX 输出限幅
 * @param IMAX 积分限幅
 * @param DEADZONE 积分死区
 * @param I_L 变速积分中档范围
 * @param I_M 变速积分高档范围
 * @param RC_DF 一阶低通滤波增益
 * @param k1 前馈增益
 */
void PID_Init(pid *p,
              float kp,float ki,float kd,
              float OUTMAX,float IMAX,float DEADZONE,
              float I_L,float I_M,
              float RC_DF,float k1)
{
	p->kp = kp;p->ki = ki;p->kd = kd;
	p->OUTMAX = OUTMAX;p->IMAX = IMAX;p->DEADZONE = DEADZONE;
	p->I_L = I_L;p->I_M = I_M;
	p->RC_DF = RC_DF;p->k1 = k1;
}

void PID_Clear(pid *p)
{
	p->PreError = 0;
	p->LastError = 0;
	p->SunError = 0;
	p->Out = 0;
	p->LAST_OUT = 0;
	p->LAST_D_OUT = 0;
	p->D_OUT = 0;
}
