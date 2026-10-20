#include "imu.h"
#ifndef SENTINEL_IMU_HOST_TEST
#include "label.h"
#endif
void IMU::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type)
{
	huart->Init(Instance, BaudRate).DMARxInit();
	m_uart = huart;
	this->type = type;
	queueHandler = &huart->UartQueueHandler;
}

void IMU::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) return;

	/* 🔴 队列空时必须立刻返回。
	 * 原来缺这一句：xQueueReceive 失败时 rxData【保持上一次的内容】，
	 * 于是本函数会拿旧帧再解析一遍 —— 表现为「同一帧被重复处理 N 次」
	 * （N = 后续空轮询次数），而 last_rx_tick / init_done 会被反复刷新。
	 * ⇒ 采样有效性判据失真，且 A 实验统计的样本数完全不可信。
	 * （第三个参数是"最长等待 tick 数"，传 0 = 立即返回，不阻塞。）*/
    if (!m_uart->ReceiveSnapshot(rxData, rx_length, rx_arrival_us)) return;
    if (rx_length < 6u) return;

	if (type == IMU601)        DecodeIMU601();
	else if (type == CH010)    Decode0x91();
	else if (type == HI226)    Decode0x91();
}

/* ======================================================================
 * IMU601（0x55 0x55 家族，帧头 [0..1]，[2]=type，[3]=len）
 *
 * ⚠️ 本机用的是 CH010，这个分支不接设备。保留它是因为它在 MCU 上白占代码，
 *    且历史上出过两个 bug（越界读 + 校验长度算错），顺手修掉以免以后接回来又踩。
 *    修法依据：0x55 家族手册口径为「len = 数据域字节数，校验和 = 帧头 4 字节 +
 *    数据区全部字节的算术和，校验字节在其后」。不再按"数据组数 × 6"推算。
 *
 *   [0..1] 55 55   [2] type   [3] len(数据域字节数)   [4..4+len) 数据   [4+len] 校验和
 *
 * 校验和 = 帧头 4 字节 + 数据区 len 字节 的字节和（uint8_t 累加）。
 * ====================================================================== */
void IMU::DecodeIMU601()
{
	if (rxData[0] != 0x55 || rxData[1] != 0x55) return;

	const uint32_t ndata = rxData[3];                        /* 数据域字节数 */
	const uint32_t need  = 5u + ndata;                       /* 4 + 数据域 + 1 校验 */

	if (ndata == 0u || need > UART_MAX_LEN)
	{
		++len_bad;
		return;
	}

	uint8_t sum = 0;
	for (uint32_t i = 0; i < (need - 1u); ++i) sum = static_cast<uint8_t>(sum + rxData[i]);
	if (sum != rxData[need - 1u])
	{
		++crc_fail;
		return;
	}
	++rx_ok;

	const uint8_t* d = rxData + 4;
	const uint8_t subtype = rxData[2];   /* 不叫 type：会遮蔽成员 type(IMU_TYPE) */

	if (subtype == 0x01)                                        /* 角度：3 组 × 6B = 18B */
	{
		if (ndata < 18u) { ++short_payload; return; }
		angle.roll  = static_cast<float>(getword(d[1], d[0])) * 180.f / 32768.f;
		angle.pitch = static_cast<float>(getword(d[3], d[2])) * 180.f / 32768.f;
		angle.yaw   = static_cast<float>(getword(d[5], d[4])) * 180.f / 32768.f;
	}
	else if (subtype == 0x02 || subtype == 0x03)                /* 角速度 / 加速度：6 组 × 6B = 36B */
	{
		if (ndata < 36u) { ++short_payload; return; }
		const float scale = (subtype == 0x03) ? (ACC_FSR / 32768.f) : (GYRO_FSR / 32768.f);
		const int16_t Ax = getword(d[1], d[0]);
		const int16_t Ay = getword(d[3], d[2]);
		const int16_t Az = getword(d[5], d[4]);
		if (subtype == 0x03)
		{
			acceleration.x = static_cast<float>(Ax) * scale;
			acceleration.y = static_cast<float>(Ay) * scale;
			acceleration.z = static_cast<float>(Az) * scale;
		}
		else
		{
			angularvelocity.roll  = static_cast<float>(Ax) * scale;
			angularvelocity.pitch = static_cast<float>(Ay) * scale;
			angularvelocity.yaw   = static_cast<float>(Az) * scale;
		}
	}
	else
	{
		return;                                             /* 其他 type 不认，但帧本身有效 */
	}

	/* 🔴 原 IMU601 分支【完全没碰】init_done / last_rx_tick / data_ok，
	 * 而 IsValid() 判的就是这三个 ⇒ 一旦切到 IMU601，IsValid() 永远 false。
	 * CH010 分支有、这里没有，是明显的分支不对称（复制粘贴漏了）。*/
	MarkSampleValid();
}

/* ======================================================================
 * CH010 / HI226（0x5A 0xA5 家族，两个型号帧结构一致）
 *
 *   [0]     帧头 0x5A
 *   [1]     帧类型 0xA5
 *   [2..3]  LEN  (u16, LSB 低字节在前)  数据域长度，不含帧头/帧类型/LEN/CRC
 *   [4..5]  CRC  (u16, LSB 低字节在前)
 *   [6..]   数据域（由若干子数据包组成：1B 标签 + 0~128B 数据）
 *
 * 🔴🔴 CRC 覆盖范围（官方手册 + 官方样例帧双向确认，2026-10-15 查 CH10X 手册）：
 *
 *   手册表：「CRC校验 = 除 CRC 本身外其余所有字段(帧头, 帧类型, 长度, 数据域)
 *            的 16 位 CRC 校验和。LSB（低字节在前）」
 *   手册校验代码：
 *          crc = 0;
 *          crc16_update(&crc, buf, 4);                // 帧头 2B + 长度 2B
 *          crc16_update(&crc, buf + 6, payload_len);  // 数据域
 *
 *   ⚠️ 是【两段拼接】，不是一整段！中间的 CRC 字段 [4..5] 被跳过、不参与计算。
 *      「除 CRC 本身外」= CRC 字段不算进去（不是把它算进去凑自同步，那种理解错）。
 *
 *   算法 = CRC-16/CCITT-FALSE：poly 0x1021、init 0x0000、MSB-first、
 *   不反射、无 final xor。与手册 crc16_update() 逐位等价。
 *
 *   样例帧实测（手册原文样例，82 字节）：
 *      len=76、帧内 CRC=0x516C、两段拼接算得 0x516C ⇒ 匹配。
 *
 *   帧总长 = 2 + 2 + 2 + len = len + 6。本机实测 len=76 ⇒ 82，与 rx_len 吻合。
 *
 * ⚠️ 之前这里写的是 `Check(rxData + 6, len, crc)`（只盖数据域、漏了帧头+长度 4 字节）
 *    ⇒ 实测 rx_ok 恒 0、crc_fail 持续涨、len_bad 恒 0。算法/init/字节序都是对的。
 * ====================================================================== */
void IMU::Decode0x91()
{
	if (rxData[0] != 0x5A || rxData[1] != 0xA5) return;

	const uint32_t len_raw = (uint32_t)rxData[2] | ((uint32_t)rxData[3] << 8);
	const uint32_t crc_raw = (uint32_t)rxData[4] | ((uint32_t)rxData[5] << 8);

	/* 🔴 CRC 覆盖长度 = len + 6（帧头2 + 帧类型1 + LEN2 + CRC占位1）。
	 *    这里直接算好「覆盖字节数」再判界，避免后面 Check() 再加一次而溢出。
	 *
	 * ⚠️ 用 UART_MAX_LEN 显式常量而不是 sizeof(rxData)：
	 *    sizeof 作用在"指针别名"上时只有 8 字节（探针阶段真踩过这个）。*/
	const uint32_t frame_len = len_raw + 6u;      /* = 帧总长 */
	if (len_raw == 0u || frame_len > UART_MAX_LEN || frame_len != rx_length)
	{
		++len_bad;
		return;
	}

	/* 🔴🔴 CRC 是【两段拼接】，中间的 CRC 字段 [4..5] 跳过不参与计算。
	 * 手册原文：「除 CRC 本身外其余所有字段(帧头, 帧类型, 长度, 数据域)」
	 * 手册代码：crc16_update(&crc, buf, 4); crc16_update(&crc, buf+6, len);
	 * ⚠️ 不是"一整段 len+6"（那是把 CRC 字段也算进去凑自同步，理解错）。
	 *    官方样例帧实测：两段拼接得 0x516C == 帧内携带值，一整段则对不上。
	 *
	 * ⚠️⚠️ 两段【只能累加，最后才比较】。
	 *    第一段算完的中间值（样例帧里是 0x035D）不是最终 CRC，
	 *    拿它去和 crc_raw 比必然不等 —— 那样会在第一段就 return，
	 *    第二段根本不跑，angle/acceleration 永远不更新。
	 *    （这个 bug 就是这么来的：现象是「rxData 有数据但 angle 读不到」，
	 *      而 crc_fail 在涨、rx_ok 恒 0，和最初的覆盖范围错误症状一模一样，
	 *      极易误判成"还没修对"。）
	 *    所以这里用 CrcAccum() 只做累加、不做比较。*/
	CrcReset();                                 /* 每帧从 0x0000 起算，杜绝跨帧残留 */
	CrcAccum(rxData, 4u);                       /* buf[0..3]  帧头 + 长度 */
	CrcAccum(rxData + 6, len_raw);              /* buf[6..6+len] 数据域 */
	if (CrcResult() != crc_raw) { ++crc_fail; return; }
	++rx_ok;

	if (rxData[6] != 0x91) return;                           /* 帧有效但不是姿态包 */

	// A HI91 packet is exactly 76 bytes. The quaternion at 60..75 is required;
	// a valid CRC on a truncated Euler-only packet does not make a usable pose.
	if (len_raw < 76u)
	{
		++short_payload;
		return;
	}
	if (len_raw != 76u) { ++len_bad; return; }

	/* ---------------------------------------------------------------------
	 * 官方手册 0x91 字段表（偏移相对 p = rxData + 6 = 数据域首字节）：
	 *    0      uint8_t   1     -      数据包标签 0x91
	 *    1      uint16_t  2     -      main_status; offset 3 is int8 temperature
	 *    4      float     4     Pa     气压（部分型号支持，CH010 无气压计⇒该值无意义）
	 *    8      uint32_t  4     ms     本地时间戳，从开机累加，每毫秒 +1  ← A 实验的基准
	 *    12     float     12    1G     加速度 XYZ（已出厂校准）
	 *    24     float     12    deg/s  角速度 XYZ
	 *    36     float     12    uT     磁强度 XYZ
	 *    48     float     12    deg    Device Euler roll/pitch/yaw (default ENU/312)
	 *    60     float     16    -      Quaternion WXYZ, body-to-navigation rotation
	 *
	 * ⚠️ 原实现把 gyro 的 pitch/roll 位置对调了（24→pitch、28→roll）。
	 *   这是【无依据的历史错误】，与"补偿 90° 安装"无关 —— 手册字段表明确是
	 *   24=X、28=Y、32=Z 的顺序。轴互换应通过 B 实验【实测】决定，
	 *   不能靠猜一个"看起来像在补偿安装"的错位。本次按手册恢复。
	 * --------------------------------------------------------------------- */
	const uint8_t* p = rxData + 6;
	// Manufacturer HI91 (legacy manual appendix 9.29/9.30 and official SDK):
	// https://download.hipnuc.com/legacy/imu_cum_cn_150_170.pdf
	// https://github.com/hipnuc/products/blob/2676ffb1214fc5eb80e1410c5b217adda1569035/c/hipnuc/hipnuc_dec.h
	// Device Euler labels depend on its ENU/312 vs NED/321 configuration. Decode
	// the same body-to-navigation quaternion as R = Rz(yaw)*Ry(pitch)*Rx(roll),
	// matching host AngleAxis Z/Y/X reconstruction. Do not swap gyro/body axes.
	double qw = R4(p + 60), qx = R4(p + 64), qy = R4(p + 68), qz = R4(p + 72);
	const double norm2 = qw*qw + qx*qx + qy*qy + qz*qz;
	if (!std::isfinite(norm2) || norm2 < 0.98 || norm2 > 1.02) {
		++quaternion_bad;
		return;
	}
	const double inv_norm = 1.0 / std::sqrt(norm2);
	qw *= inv_norm; qx *= inv_norm; qy *= inv_norm; qz *= inv_norm;
	const double sin_pitch = 2.0 * (qw*qy - qz*qx);
	const double bounded_sin_pitch = sin_pitch < -1.0 ? -1.0 : (sin_pitch > 1.0 ? 1.0 : sin_pitch);
	const double rad2deg = 57.2957795130823208768;
	Angle normalized{};
	normalized.pitch = static_cast<float>(std::asin(bounded_sin_pitch) * rad2deg);
	if (std::fabs(bounded_sin_pitch) >= 1.0 - 1e-12) {
		// At the ZYX singularity roll/yaw are not independently identifiable.
		// Choose roll=0 and the equivalent yaw so the reconstructed rotation agrees.
		normalized.roll = 0.f;
		normalized.yaw = static_cast<float>(std::atan2(2.0*(qw*qz-qx*qy), 1.0-2.0*(qx*qx+qz*qz)) * rad2deg);
	} else {
		normalized.roll = static_cast<float>(std::atan2(2.0*(qw*qx+qy*qz), 1.0-2.0*(qx*qx+qy*qy)) * rad2deg);
		normalized.yaw = static_cast<float>(std::atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz)) * rad2deg);
	}
	// Validate everything used for orientation before committing its epoch/time.
	if (!std::isfinite(normalized.roll) || !std::isfinite(normalized.pitch) ||
	    !std::isfinite(normalized.yaw)) { ++quaternion_bad; return; }
    const uint32_t device_time = U4(p + 8);
    if (have_device_timestamp && device_time == last_device_timestamp) return;
    if (have_device_timestamp && uint32_t(device_time-last_device_timestamp) >= 0x80000000u) {
        if (++sample_epoch == 0) sample_epoch = 1; // sensor reference restarted, not normal u32 wrap.
    }
    timestamp = device_time;
    last_device_timestamp = device_time;
    have_device_timestamp = true;

	acceleration.x  = R4(p + 12);
	acceleration.y  = R4(p + 16);
	acceleration.z  = R4(p + 20);

	angularvelocity.roll  = R4(p + 24);
	angularvelocity.pitch = R4(p + 28);
	angularvelocity.yaw   = R4(p + 32);

	device_angle.roll  = R4(p + 48);
	device_angle.pitch = R4(p + 52);
	device_angle.yaw   = R4(p + 56);
	angle = normalized;

	++sample_ok;
	MarkSampleValid();
}

/* 统一收口：三个分支的"这帧数据可用"标记都走这里，避免再漏。*/
void IMU::MarkSampleValid()
{
	data_ok = fabsf(angle.roll) <= 180.f
	       && fabsf(angle.pitch) <= 90.f
	       && fabsf(angle.yaw) <= 180.f;   /* NaN/Inf 比较恒 false，天然拦掉 */
    sample_arrival_us = rx_arrival_us;
    last_rx_tick = xTaskGetTickCount();
	if (data_ok) init_done = true;             /* 数据离谱就不认"已初始化" */
}

float IMU::GetAngleYaw()
{
	return angle.yaw;
}
uint32_t IMU::GetTimestamp() { return timestamp; }

bool IMU::IsValid()
{
	return init_done
		&& (xTaskGetTickCount() - last_rx_tick) <= pdMS_TO_TICKS(50)
		&& data_ok;
}


float IMU::GetGyroRoll() { return angularvelocity.roll; }
float IMU::GetGyroPitch() { return angularvelocity.pitch; }
float IMU::GetGyroYaw() { return angularvelocity.yaw; }


float IMU::GetAnglePitch()
{
	return angle.pitch;
}

float IMU::GetAngleRoll()
{
	return angle.roll;
}

float IMU::GetAccX() { return acceleration.x; }
float IMU::GetAccY() { return acceleration.y; }
float IMU::GetAccZ() { return acceleration.z; }

/* ======================================================================
 * CRC 累加器（CH010 / HI226）
 *
 * 为什么单独做成"只累加、不比较"的两段接口：
 *   CH010 的 CRC 覆盖范围是【两段拼接】—— buf[0..3]（帧头+长度）
 *   + buf[6..6+len]（数据域），中间的 CRC 字段 buf[4..5] 不参与计算。
 *   所以运算必须跨两段延续，但【只能在最后比较一次】。
 *
 * 🔴 曾在这里踩过：写成 `if (!Check(seg1)) return; if (!Check(seg2)) return;`
 *    而 Check() 每段都拿自己的结果和 com 比 ⇒ 第一段的中间值必然不等 ⇒
 *    提前 return，第二段没跑，angle/acceleration 永远不更新。
 *    症状（crc_fail 涨、rx_ok=0、rxData 有数据但 angle 读不到）
 *    与"覆盖范围写错"完全一样 ⇒ 极易误判成没修对。接口语义必须防住这种误用。
 *
 * 算法 = CRC-16/CCITT-FALSE：poly 0x1021、init 0x0000、MSB-first、
 *   不反射、无 final xor。与 CH10X 手册的 crc16_update() 逐位等价。
 *   （与 V2 PC 帧用的 CRC-16/MCRF4XX 是两个不同算法，别混用。）
 * ====================================================================== */
void IMU::CrcReset()      { crc_acc = 0x0000u; }
void IMU::CrcAccum(const uint8_t* p, uint16_t n)
{
	uint16_t c = crc_acc;
	for (uint16_t j = 0; j < n; ++j)
	{
		c ^= (uint16_t)((uint16_t)p[j] << 8);
		for (uint8_t i = 0; i < 8; ++i)
		{
			const uint16_t shifted = (uint16_t)(c << 1);
			c = (c & 0x8000u) ? (uint16_t)(shifted ^ 0x1021u) : shifted;
		}
	}
	crc_acc = c;
}
uint16_t IMU::CrcResult() const { return crc_acc; }

bool IMU::Check(uint8_t* pdata, uint8_t len, uint32_t com)
{
	if (type == IMU601)
	{
		uint8_t t = 0;
		for (int i = 0; i < len; i++)
		{
			t += pdata[i];
		}
		return t == com;
	}
	else if (type == CH010 || type == HI226)
	{
		/* 单段版本（CH010 实际走 CrcReset/CrcAccum/CrcResult 三段接口，
		 * 因为它的覆盖范围是"两段拼接、末尾比较"）。这里保留单段实现
		 * 供将来其他型号复用，语义就是"算这一段并与 com 比较"。*/
		CrcReset();
		CrcAccum(pdata, len);
		return (uint32_t)CrcResult() == com;
	}
	return false;
}


int16_t IMU::getword(uint8_t HighBit, uint8_t LowBits)
{
	/* ⚠️ 原式 `HighBit << 8 | LowBits`：HighBit 是 uint8_t，
	 *    整型提升为 int 后左移 8 位，值域 0x00~0xFF00 恰好在 int 内，
	 *    所以【没有】UB，但依赖提升规则，隐晦。改成显式 uint16 组合。*/
	return (int16_t)((uint16_t)HighBit << 8 | (uint16_t)LowBits);
}
