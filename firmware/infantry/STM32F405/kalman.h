#pragma once
class Kalman
{
public:
	float X_last; //上一时刻的最优结果  X(k-|k-1)
	float X_mid;  //当前时刻的预测结果  X(k|k-1)
	float X_now;  //当前时刻的最优结果  X(k|k)
	float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
	float P_now;  //当前时刻最优结果的协方差  P(k|k)
	float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
	float kg;     //kalman增益
	float A;      //系统参数
	float B;
	float Q;
	float R;
	float H;
	/**
	  * @name   kalmanCreate
	  * @brief  创建一个卡尔曼滤波器
	  * @param  p:  滤波器
	  *         T_Q:系统噪声协方差
	  *         T_R:测量噪声协方差
	  *
	  * @retval none
	  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
	  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
	  */
	Kalman(float T_Q, float T_R)
	{
		X_last = (float)0;
		P_last = 0;
		Q = T_Q;
		R = T_R;
		A = 1;
		B = 0;
		H = 1;
		X_mid = X_last;
	}

	float Filter(float dat)
	{
		X_mid = A * X_last;                   //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
		P_mid = A * P_last + Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
		kg = P_mid / (P_mid + R);           //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
		X_now = X_mid + kg * (dat - X_mid);   //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
		P_now = (1 - kg) * P_mid;             //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
		P_last = P_now;                     //状态更新
		X_last = X_now;
		return X_now;						//输出预测结果x(k|k)
	}
};