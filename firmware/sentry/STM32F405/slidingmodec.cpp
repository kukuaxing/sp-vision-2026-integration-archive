#include"slidingmodec.h"
#include<cmath>

// 参数说明: SMC(C, K, ref, error_eps, u_max, J, epsilon)
// C=30       : 滑模面斜率，决定极点位置与收敛速度
// K=28       : 切换增益，过高超调振荡，过低响应慢 (原42→28)
// error_eps=0.2: 近零误差平滑区(°)，减小过大死区导致的“发软”
// epsilon=1.0  : 饱和函数边界层，值越大切换越平滑 (原0.5→1.0)
SMC YawSMC(30, 28, 0, 0.2, 25000, 0.45, 1.0);

void SMC::SMC_Tick(float angle_now,float angle_vel) //anlge为当前位置(°),ang_vel为角速度(°/s)
{
	//读取参数
	angle = angle_now;
    ang_vel = angle_vel;
	error = angle - ref;
	ddref = (ref - refl) - dref; //这里对前馈进行了处理，没有严格单位统一
	dref = (ref - refl);
    // 近零误差平滑处理：不直接清零输出，避免云台“松”
	float error_eff = error;
	if (fabs(error) < error_eps)
	{
		const float scale = fabs(error) / error_eps;
		error_eff = error * scale;
	}
	//smc surface
   s = C * error_eff + (ang_vel - dref);
	u = J * (ddref - C * (ang_vel - dref) - epsilon * Sat(s) - K * s);
	//控制量限幅
	if (u > u_max)
		u = u_max;
	if (u < -u_max)
		u = -u_max;
	//参数更新
	refl = ref;
}