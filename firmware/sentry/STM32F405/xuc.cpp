#include "xuc.h"
#ifndef SENTINEL_HOST_TEST
#include "label.h"
#include "imu.h"
#include "CRC.h"
#include "control.h"
#include "tim.h"
#include "math.h"
#endif

XUC xuc;

/* ==========================================================================
 * 小端序读写（逐字节装载，规避 STM32 未对齐 float 访问，V2 §10.5）
 * ========================================================================== */
static inline void W1(uint8_t* p, uint8_t v) { p[0] = v; }
static inline void W2(uint8_t* p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
}
static inline void W4(uint8_t* p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
	p[2] = (uint8_t)((v >> 16) & 0xFFu);
	p[3] = (uint8_t)((v >> 24) & 0xFFu);
}
static inline void WF(uint8_t* p, float v) { memcpy(p, &v, 4); }

static inline uint16_t R2(const uint8_t* p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t R4u(const uint8_t* p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
	     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline float RF(const uint8_t* p)
{
	float f = 0.f;
	memcpy(&f, p, 4);
	return f;
}

static inline float Deg2Rad(float d) { return d * (XUC_PI / 180.f); }

/* ==========================================================================
 * 初始化
 * ========================================================================== */
void XUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr).DMATxInit();
	m_uart = huart;
	queue_handler = &huart->UartQueueHandler;

    // Hardware randomness avoids the old deterministic tick/address boot identity.
    // Zero denotes RNG failure and inhibits AIM/NAV control, with telemetry retained.
#ifdef SENTINEL_HOST_TEST
    boot_id = 0x11223344u;
#else
    __HAL_RCC_RNG_CLK_ENABLE();
    RNG->CR |= RNG_CR_RNGEN;
    boot_id = 0;
    for (uint32_t n=0; n<100000u; ++n) {
        if (RNG->SR & (RNG_SR_CECS | RNG_SR_SECS)) break;
        if (RNG->SR & RNG_SR_DRDY) { boot_id = RNG->DR; break; }
    }
    RNG->CR &= ~RNG_CR_RNGEN;
#endif

	rx_len = 0;
	pending_ms = 0;
	ref_head = 0;
    memset(ref_cache, 0, sizeof(ref_cache));
	rx_total = rx_count = crc_err = resync_cnt = 0;
	last_reject = XUC_OK;
	seq_gimbal = seq_chassis = seq_link = 0;
    seq_feed=0; feed_diagnostic={};
	aim_seq_init = false;
	nav_seq_init = false;
	nav_last_seq = 0;
	prespeedx = prespeedy = 0.f;
	frame_time_us = 0;
	frame_len = 0;

	RevokeAim();
	nav_accepted = false;
	nav_enable = false;
}

/* ==========================================================================
 * 下行：接收 + 流式解析
 * ========================================================================== */
void XUC::Decode()
{
	if (!m_uart || !queue_handler || !*queue_handler) return;

	/* ① 抽干队列。⚠️ 底层队列是「深度 1 + 覆盖写」（usart.cpp 的 ISR），
	 *    两次调用之间到达的帧会互相覆盖 ⇒ 视觉下发率必须 ≤ DecodeTask
	 *    的轮询率（5 ms ⇒ 200 Hz）。这里用 0 超时循环把当前所有可用快照取干净。
	 *
	 * ⚠️ 每次只取【一帧】的元信息：深度 1 的队列在一轮里若收到多帧，最终留下
	 *    的那一帧才是队列里真实的那个。取多次会把计数重复累加，得不到正确配对。*/
    if (m_uart->ReceiveSnapshot(m_frame, frame_len, frame_time_us)) {
        ++rx_total;
        AppendRx(m_frame, frame_len);
    }

	/* ② 流式解析 */
	ParseRx();
}

void XUC::AppendRx(const uint8_t* p, uint16_t n)
{
	if (n > XUC_RX_BUF) n = XUC_RX_BUF;

	if ((uint32_t)rx_len + (uint32_t)n > XUC_RX_BUF)
	{
		/* 缓冲溢出 ⇒ 清空解析状态并失效控制新鲜度（V2 §10.3） */
		rx_len = 0;
		pending_ms = 0;
		++resync_cnt;
	}

	memcpy(rx_buf + rx_len, p, n);
	rx_len = (uint16_t)(rx_len + n);
}

bool XUC::PayloadLenOk(uint8_t type, uint16_t plen)
{
	switch (type)
	{
	case XUC_T_GIMBAL_IMU:    return plen == XUC_PL_GIMBAL_IMU;
	case XUC_T_CHASSIS_IMU:   return plen == XUC_PL_CHASSIS_IMU;
	case XUC_T_IMU_INFO:      return plen == XUC_PL_IMU_INFO;
	case XUC_T_AIM_COMMAND:   return plen == XUC_PL_AIM_COMMAND;
	case XUC_T_NAV_COMMAND:   return plen == XUC_PL_NAV_COMMAND;
	case XUC_T_SESSION_OPEN:
	case XUC_T_SESSION_CLOSE: return plen == XUC_PL_SESSION;
	case XUC_T_TIME_SYNC_REQ: return plen == XUC_PL_TIME_SYNC_REQ;
	case XUC_T_TIME_SYNC_RSP: return plen == XUC_PL_TIME_SYNC_RSP;
	case XUC_T_ACK:           return plen == XUC_PL_ACK;
	case XUC_T_LINK_STATUS:   return plen == XUC_PL_LINK_STATUS;
	default:                  return false;    /* 未知类型 */
	}
}

void XUC::ParseRx()
{
	uint16_t i = 0;
	const uint32_t now_ms = (uint32_t)xTaskGetTickCount();

	while ((uint16_t)(i + 1u) < rx_len)
	{
		/* 1) 找 magic（A6 6A） */
		if (rx_buf[i] != XUC_MAGIC0 || rx_buf[i + 1u] != XUC_MAGIC1)
		{
			++i;
			continue;
		}

		/* 2) 帧头不完整 */
		if ((uint16_t)(rx_len - i) < XUC_HDR_LEN)
		{
			if (pending_ms == 0u) pending_ms = now_ms;
			else if (now_ms - pending_ms > XUC_HALF_FRAME_MS)
			{
				pending_ms = 0u;
				++resync_cnt;
				++i;                        /* 候选半帧超时 ⇒ 丢首字节重同步 */
				continue;
			}
			break;                          /* 等更多数据 */
		}

		const uint8_t  ver  = rx_buf[i + 2u];
		const uint8_t  type = rx_buf[i + 3u];
		const uint16_t plen = R2(rx_buf + i + 4u);

		/* 3) 版本 / 长度上限预检查 —— 不通过就把当前字节当噪声丢掉 */
		if (ver != XUC_VERSION || plen > XUC_MAX_PAYLOAD)
		{
			++resync_cnt;
			++i;
			continue;
		}

		const uint16_t total = (uint16_t)(XUC_HDR_LEN + plen + XUC_CRC_LEN);

		/* 4) 整帧未到齐 */
		if ((uint16_t)(rx_len - i) < total)
		{
			if (pending_ms == 0u) pending_ms = now_ms;
			else if (now_ms - pending_ms > XUC_HALF_FRAME_MS)
			{
				pending_ms = 0u;
				++resync_cnt;
				++i;
				continue;
			}
			break;
		}

		/* 5) CRC —— 覆盖 [0 .. 12+N-1]，低字节在 [12+N]（复用全局 CRC.h） */
		if (!VerifyCRC16CheckSum(rx_buf + i, total))
		{
			++crc_err;
			++i;                            /* 向后移一字节重新找头（V2 §10.2） */
			continue;
		}

		pending_ms = 0u;

		/* 6) 类型/长度分发 */
		if (!PayloadLenOk(type, plen))
		{
			/* 已知类型但长度不符 ⇒ 协议不一致；未知类型 ⇒ 静默跳过（V2 §3.1）。
			 * 两者边界都由 CRC 背书，整帧跳过是安全的。 */
			if (type == XUC_T_AIM_COMMAND || type == XUC_T_NAV_COMMAND)
				last_reject = XUC_UNSUPPORTED;
			i = (uint16_t)(i + total);
			continue;
		}

		++rx_count;

		const uint16_t frame_seq = R2(rx_buf + i + 6u);
		const uint32_t f_boot    = R4u(rx_buf + i + 8u);
		const uint8_t* pl        = rx_buf + i + XUC_HDR_LEN;

		switch (type)
		{
		case XUC_T_AIM_COMMAND:
            taskENTER_CRITICAL();
            HandleAim(pl, frame_seq, f_boot);
            taskEXIT_CRITICAL();
            break;
		case XUC_T_NAV_COMMAND:
            taskENTER_CRITICAL();
            HandleNav(pl, frame_seq, f_boot);
            taskEXIT_CRITICAL();
            break;
		default: break;      /* 已知但本版不处理：0x03 / 0x20 / 0x21 / 0x30 / 0x31 / 0x80 */
		}

		i = (uint16_t)(i + total);
	}

	/* 7) 保留未消费部分（含可能的候选半帧首字节） */
	if (i > 0u)
	{
		rx_len = (uint16_t)(rx_len - i);
		if (rx_len > 0u) memmove(rx_buf, rx_buf + i, rx_len);
	}
}

/* ==========================================================================
 * 0x10 AIM_COMMAND
 * 接受顺序（V2 §7.4）：
 *   帧结构/CRC → boot_id → 新 seq → flags/有限值 → 参考缓存/epoch
 *   → 样本有效期 → 本地模式与机构保护
 * ========================================================================== */
void XUC::HandleAim(const uint8_t* pl, uint16_t frame_seq, uint32_t f_boot)
{
	aim_rx_tick = xTaskGetTickCount();

	/* --- boot_id --- */
	if (boot_id == 0u || f_boot != boot_id) { last_reject = XUC_BAD_SESSION; return; }

	const uint16_t ref_epoch = R2(pl + 4u);
	const uint16_t ref_seq   = R2(pl + 6u);
	const uint32_t ref_time  = R4u(pl + 8u);
	const uint16_t v4ms      = R2(pl + 12u);
	const uint8_t  flags     = pl[14u];
	const float    yaw_err   = RF(pl + 16u);
	const float    pit_err   = RF(pl + 20u);

	/* --- flags / 有限值 --- */
	if ((flags & (uint8_t)~XUC_AIMF_MASK) != 0u) { last_reject = XUC_INVALID_VALUE; return; }
	if (!isfinite(yaw_err) || !isfinite(pit_err)) { last_reject = XUC_INVALID_VALUE; return; }
    if (fabsf(yaw_err) > XUC_PI || fabsf(pit_err) > XUC_PI) {
        last_reject = XUC_INVALID_VALUE; return;
    }

	/* --- 新序号：0 < (uint16)(new-last) < 32768 --- */
	if (aim_seq_init)
	{
		const uint16_t d = (uint16_t)(frame_seq - aim_last_seq);
		if (d == 0u || d >= 32768u) { last_reject = XUC_DUPLICATE_OR_OLD; return; }
	}

	/* --- CONTROL_ENABLE=0：显式停止指令。
	 *     按 V2 §7.4「不要求有效参考样本，立即撤销许可」，且不能因 IMU 失效而拒绝。*/
	if ((flags & XUC_AIM_CONTROL_EN) == 0u)
	{
		if (flags != 0u || yaw_err != 0.f || pit_err != 0.f)
		{
			last_reject = XUC_INVALID_VALUE;
			return;
		}
		RevokeAim();
		aim_accepted = false;
		aim_seq_init = true;
		aim_last_seq = frame_seq;
		last_reject = XUC_OK;
		return;
	}

	/* --- valid_for_ms 范围（1..150，推荐 100） --- */
	if (v4ms < 1u || v4ms > 150u) { last_reject = XUC_INVALID_VALUE; return; }
	// Without a target, only the wheel lease is meaningful. Zero errors prevent
	// this heartbeat from becoming a latent movement/fire request on a later cycle.
	if (!(flags & XUC_AIM_VALID) &&
	    ((flags & XUC_AIM_FIRE_REQ) || yaw_err != 0.f || pit_err != 0.f)) {
		last_reject = XUC_INVALID_VALUE; return;
	}

	/* --- 参考样本四元组匹配（boot_id 已查，这里查 epoch/seq/time） --- */
	if (ref_epoch != imu_pantile.GetEpoch()) { last_reject = XUC_BAD_REFERENCE; return; }
	const XUC_RefSample* s = FindRef(ref_epoch, ref_seq, ref_time);
    if (s == nullptr) { last_reject = XUC_BAD_REFERENCE; return; }
    if (!(s->flags & XUC_F_ORIENT_VALID) || !imu_pantile.IsValid()) {
        last_reject = XUC_LOCAL_FAULT; return;
    }

	/* --- 样本有效期：从样本时刻算，不从收包时算 ---
	 * ⚠️ 用 TIM::NowUs()（真 1 µs），不是 xTaskGetTickCount()*1000（1 ms 量化）。
	 *    ref_time 是上位机原样回传我们上行帧里的 sample_time_us，
	 *    那个值是 usart.cpp 的 ISR 用 TIM::NowUs() 打的 —— 同一时基才能相减。*/
	const uint32_t now_us = TIM::NowUs();
	const uint32_t age_us = (uint32_t)(now_us - ref_time);
	if (age_us >= 0x80000000u) { last_reject = XUC_STALE; return; }          /* 未来时刻 */
	if (age_us > (uint32_t)v4ms * 1000u) { last_reject = XUC_STALE; return; }

	/* --- 前置全部通过：重建目标（V2 §7.3，只在新命令通过检查时算一次） --- */
	aim_yaw_target   = WrapPi(s->yaw + yaw_err);
	aim_pitch_target = s->pitch + pit_err;
	aim_ref_epoch = ref_epoch;
	aim_ref_time_us  = ref_time;
	aim_valid_for_ms = v4ms;
	aim_control_en   = true;
	aim_valid        = (flags & XUC_AIM_VALID) != 0u;
	aim_friction_en  = (flags & XUC_AIM_FRICTION_EN) != 0u;
	aim_fire_req     = (flags & XUC_AIM_FIRE_REQ) != 0u;
	aim_accepted     = true;
	aim_seq_init     = true;
	aim_last_seq     = frame_seq;

	/* --- 本地模式与机构保护（最后一步） --- */
    if (ctrl.mode != CONTROL::AUTO) {
        last_reject = XUC_NOT_ALLOWED; return;
    }
#if XUC_REQUIRE_VERIFIED_POSE
    if (
        (s->flags & (XUC_F_SAMPLE_VALID|XUC_F_AXES_VERIFIED)) != (XUC_F_SAMPLE_VALID|XUC_F_AXES_VERIFIED)) {
        last_reject = XUC_NOT_ALLOWED; return;
    }
#endif
#if XUC_ALLOW_CONTROL == 0
	/* 只读联调版：目标已重建但不下发，明确告知上位机「本地不允许」 */
	last_reject = XUC_NOT_ALLOWED;
#else
	last_reject = XUC_OK;
#endif
    if (last_reject==XUC_OK && aim_fire_req) {
        taskENTER_CRITICAL();
        feed_diagnostic.Request(frame_seq, frame_time_us);
        taskEXIT_CRITICAL();
    }
}

/* ==========================================================================
 * 0x11 NAV_COMMAND
 * ⚠️ 首版降级：不做 ref_mcu_time_us 的时间映射校验（0x30/0x31 未实现），
 *    改用本地租期 —— 收到后 valid_for_ms 内有效，超时即停。
 * ========================================================================== */
void XUC::HandleNav(const uint8_t* pl, uint16_t frame_seq, uint32_t f_boot)
{
	if (boot_id == 0u || f_boot != boot_id) { last_reject = XUC_BAD_SESSION; return; }

	const uint16_t v4ms  = R2(pl + 8u);
	const uint8_t  flags = pl[10u];
	const float    vx    = RF(pl + 12u);
	const float    vy    = RF(pl + 16u);
	const float    wz    = RF(pl + 20u);

	if ((flags & (uint8_t)~XUC_NAVF_MASK) != 0u) { last_reject = XUC_INVALID_VALUE; return; }
	if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz)) { last_reject = XUC_INVALID_VALUE; return; }

	if (nav_seq_init)
	{
		const uint16_t d = (uint16_t)(frame_seq - nav_last_seq);
		if (d == 0u || d >= 32768u) { last_reject = XUC_DUPLICATE_OR_OLD; return; }
	}

	/* --- NAV_ENABLE=0：速度必须全零，立即停（不要求可靠时间，V2 §8） --- */
	if ((flags & XUC_NAV_ENABLE) == 0u)
	{
		if (vx != 0.f || vy != 0.f || wz != 0.f) { last_reject = XUC_INVALID_VALUE; return; }
		nav_accepted = false;
		nav_enable = false;
		nav_vx = nav_vy = nav_wz = 0.f;
		nav_seq_init = true;
		nav_last_seq = frame_seq;
		last_reject = XUC_OK;
		return;
	}

	if (v4ms < 1u || v4ms > 150u) { last_reject = XUC_INVALID_VALUE; return; }
	if (ctrl.mode != CONTROL::AUTO) { last_reject = XUC_NOT_ALLOWED; return; }

	nav_vx = vx;
	nav_vy = vy;
	nav_wz = wz;
	nav_valid_for_ms = v4ms;
	nav_enable = true;
	nav_accepted = true;
	nav_accept_tick = xTaskGetTickCount();
	nav_seq_init = true;
	nav_last_seq = frame_seq;
	last_reject = XUC_OK;
}

/* ==========================================================================
 * 参考样本缓存
 * ========================================================================== */
void XUC::PushRef(uint16_t epoch, uint16_t seq, uint32_t t_us, float yaw, float pitch, uint8_t flags)
{
	XUC_RefSample& s = ref_cache[ref_head];
	s.boot_id = boot_id;
	s.time_us = t_us;
	s.yaw = yaw;
	s.pitch = pitch;
	s.epoch = epoch;
	s.seq = seq;
	s.valid = 1u;
	s.flags = flags;
	ref_head = (uint16_t)((ref_head + 1u) % XUC_REF_CACHE_N);
}

const XUC_RefSample* XUC::FindRef(uint16_t epoch, uint16_t seq, uint32_t t_us) const
{
	for (uint16_t i = 0; i < XUC_REF_CACHE_N; ++i)
	{
		const XUC_RefSample& s = ref_cache[i];
		if (s.valid && s.boot_id == boot_id && s.epoch == epoch
			&& s.seq == seq && s.time_us == t_us)
			return &s;
	}
	return nullptr;
}

/* ==========================================================================
 * 本侧许可
 * ========================================================================== */
void XUC::RevokeAim()
{
	aim_accepted = false;
	aim_control_en = false;
	aim_valid = false;
	aim_friction_en = false;
	aim_fire_req = false;
}

bool XUC::AimReady()
{
    return aim_valid && LeaseReady();
}

bool XUC::FrictionReady()
{
    return aim_friction_en && LeaseReady();
}

bool XUC::LeaseReady()
{
    if (!aim_accepted || !aim_control_en || ctrl.mode != CONTROL::AUTO ||
        !imu_pantile.IsValid() || aim_ref_epoch != imu_pantile.GetEpoch()) return false;
#if XUC_REQUIRE_VERIFIED_POSE
    if ((GimbalFlags() & (XUC_F_ORIENT_VALID|XUC_F_SAMPLE_VALID|XUC_F_AXES_VERIFIED)) !=
        (XUC_F_ORIENT_VALID|XUC_F_SAMPLE_VALID|XUC_F_AXES_VERIFIED)) return false;
#endif
	const uint32_t age = (uint32_t)(TIM::NowUs() - aim_ref_time_us);   /* 真 µs */
	if (age >= 0x80000000u) return false;
	if (age > (uint32_t)aim_valid_for_ms * 1000u) return false;
#if XUC_ALLOW_CONTROL == 0
	return false;                        /* 只读联调版：本地不放行 */
#else
	return true;
#endif
}

bool XUC::NavReady()
{
	if (!nav_accepted || !nav_enable || ctrl.mode != CONTROL::AUTO || boot_id == 0u) return false;
	if ((uint32_t)(xTaskGetTickCount() - nav_accept_tick) > (uint32_t)nav_valid_for_ms)
		return false;
#if XUC_ALLOW_NAV == 0
	return false;
#else
	return true;
#endif
}

uint16_t XUC::AimAgeMs()
{
	if (!aim_accepted) return 0xFFFFu;
	const uint32_t age = (uint32_t)(TIM::NowUs() - aim_ref_time_us);   /* 真 µs */
	if (age >= 0x80000000u) return 0xFFFFu;
	const uint32_t ms = age / 1000u;
	return (ms > 65534u) ? 0xFFFFu : (uint16_t)ms;
}

uint8_t XUC::LinkState()
{
	uint8_t s = 0u;
	if (ctrl.mode == CONTROL::AUTO) s |= XUC_ST_AUTO_MODE;
	if (aim_accepted && AimAgeMs() <= aim_valid_for_ms) s |= XUC_ST_AIM_FRESH;
	if (AimReady())        s |= XUC_ST_GIMBAL_ALLOW;
	if (FrictionReady()) s |= XUC_ST_FRICTION_ALLOW;
	if (aim_accepted && aim_friction_en && aim_fire_req && AimReady()) s |= XUC_ST_FIRE_ALLOW;
	if (NavReady())        s |= XUC_ST_NAV_FRESH | XUC_ST_NAV_ALLOW;
	return s;
}

/* ==========================================================================
 * 上行：组帧 + 发送
 * ========================================================================== */
uint16_t XUC::BuildFrame(uint8_t type, const uint8_t* payload, uint16_t plen,
                         uint16_t seq, uint8_t* out)
{
	out[0] = XUC_MAGIC0;
	out[1] = XUC_MAGIC1;
	out[2] = XUC_VERSION;
	out[3] = type;
	W2(out + 4u, plen);
	W2(out + 6u, seq);
	W4(out + 8u, boot_id);
	if (plen > 0u && payload != nullptr) memcpy(out + XUC_HDR_LEN, payload, plen);

	const uint16_t total = (uint16_t)(XUC_HDR_LEN + plen + XUC_CRC_LEN);
	AppendCRC16CheckSum(out, total);     /* 覆盖 [0 .. 12+N-1]，低字节落 [12+N] */
	return total;
}

uint8_t XUC::GimbalFlags()
{
	uint8_t f = 0u;
	if (imu_pantile.IsValid()) f |= XUC_F_ORIENT_VALID;

	const float gx = imu_pantile.GetGyroRoll();
	const float gy = imu_pantile.GetGyroPitch();
	const float gz = imu_pantile.GetGyroYaw();
	if (imu_pantile.IsValid() && isfinite(gx) && isfinite(gy) && isfinite(gz))
		f |= XUC_F_GYRO_VALID;

    if (imu_pantile.IsValid()) {
#if XUC_SAMPLE_TIME_VERIFIED
        f |= XUC_F_SAMPLE_VALID;
#endif
#if XUC_AXES_VERIFIED
        f |= XUC_F_AXES_VERIFIED;
#endif
    }
	return f;
}

uint8_t XUC::ChassisFlags()
{
	uint8_t f = 0u;
	if (imu_chassis.IsValid()) f |= XUC_F_ORIENT_VALID;
	/* bit1 GYRO_VALID 在底盘帧固定为 0（V2 §4.2） */
	return f;
}

void XUC::ObserveFeeder(const uint8_t* bytes, uint32_t arrival_us)
{
    // Called only for the validated, configured feeder CAN feedback in its ISR.
    const uint16_t angle=(uint16_t(bytes[0])<<8)|bytes[1];
    const int16_t rpm=int16_t((uint16_t(bytes[2])<<8)|bytes[3]);
    feed_diagnostic.Feedback(angle,rpm,arrival_us);
}

void XUC::RecordFeedIssue(bool automatic, int16_t direction)
{
    taskENTER_CRITICAL();
    feed_diagnostic.Issue(automatic ? feed_diagnostic.request_seq:0,TIM::NowUs(),automatic,direction);
    taskEXIT_CRITICAL();
}

void XUC::Encode()
{
	if (!m_uart || !m_uart->TxIdle()) return; // Do not overwrite bytes still read by DMA.

    // Timestamp belongs to the decoded IMU sample. Never use host RX/packaging time.
    // It is the MCU UART2 arrival time, not certified sensor acquisition time.
    const uint32_t t_us = imu_pantile.GetArrivalTimeUs();

	uint32_t w = 0u;

	/* ---------------- 0x01 GIMBAL_IMU（负载 32） ---------------- */
	{
		uint8_t pl[XUC_PL_GIMBAL_IMU];
		const float r = Deg2Rad(imu_pantile.GetAngleRoll());
		const float p = Deg2Rad(imu_pantile.GetAnglePitch());
		const float y = Deg2Rad(imu_pantile.GetAngleYaw());

		W1(pl + 0u,  XUC_IMU_ID_GIMBAL);
		W1(pl + 1u,  GimbalFlags());
		W2(pl + 2u,  imu_pantile.GetEpoch());
		W4(pl + 4u,  t_us);
		WF(pl + 8u,  r);
		WF(pl + 12u, p);
		WF(pl + 16u, y);
		WF(pl + 20u, Deg2Rad(imu_pantile.GetGyroRoll()));
		WF(pl + 24u, Deg2Rad(imu_pantile.GetGyroPitch()));
		WF(pl + 28u, Deg2Rad(imu_pantile.GetGyroYaw()));

		const uint16_t seq = seq_gimbal++;
		w += BuildFrame(XUC_T_GIMBAL_IMU, pl, XUC_PL_GIMBAL_IMU, seq, tx_buf + w);

	}

	/* ---------------- 0x02 CHASSIS_IMU（负载 20） ---------------- */
	{
		uint8_t pl[XUC_PL_CHASSIS_IMU];
		W1(pl + 0u,  XUC_IMU_ID_CHASSIS);
		W1(pl + 1u,  ChassisFlags());
		W2(pl + 2u,  imu_chassis.GetEpoch());
        // Independent UART5 sample, with its own arrival and reference epoch.
        W4(pl + 4u, imu_chassis.GetArrivalTimeUs());
		WF(pl + 8u,  Deg2Rad(imu_chassis.GetAngleRoll()));
		WF(pl + 12u, Deg2Rad(imu_chassis.GetAnglePitch()));
		WF(pl + 16u, Deg2Rad(imu_chassis.GetAngleYaw()));

		const uint16_t seq = seq_chassis++;
		w += BuildFrame(XUC_T_CHASSIS_IMU, pl, XUC_PL_CHASSIS_IMU, seq, tx_buf + w);
	}

	/* ---------------- 0x81 LINK_STATUS（负载 12） ---------------- */
	{
		uint8_t pl[XUC_PL_LINK_STATUS];
		W4(pl + 0u,  0u);                    /* host_session：无会话 ⇒ 0 */
		W2(pl + 4u,  aim_last_seq);
		W1(pl + 6u,  LinkState());
		W1(pl + 7u,  last_reject);
		W2(pl + 8u,  AimAgeMs());
		W2(pl + 10u, 0u);                    /* reserved */

		const uint16_t seq = seq_link++;
		w += BuildFrame(XUC_T_LINK_STATUS, pl, XUC_PL_LINK_STATUS, seq, tx_buf + w);
	}

    // Separate diagnostic message: existing IMU/status layouts are untouched.
    // Latched event timestamps survive short motor pulses and 50 Hz packaging.
    {
        uint8_t pl[XUC_PL_FEED_DIAGNOSTIC]{};
        taskENTER_CRITICAL();
        const uint32_t now_us=TIM::NowUs();
        W1(pl,1u); W1(pl+1,feed_diagnostic.Flags(now_us));
        W2(pl+2,feed_diagnostic.request_seq); W4(pl+4,feed_diagnostic.request_us);
        W2(pl+8,feed_diagnostic.issue_seq); W2(pl+10,feed_diagnostic.motion_seq);
        W4(pl+12,feed_diagnostic.issue_us); W4(pl+16,feed_diagnostic.motion_us);
        W4(pl+20,feed_diagnostic.issue_count); W4(pl+24,feed_diagnostic.motion_count);
        W4(pl+28,now_us); W4(pl+32,feed_diagnostic.feedback_us);
        W2(pl+36,uint16_t(feed_diagnostic.rpm));
#ifndef SENTINEL_HOST_TEST
        W2(pl+38,ctrl.shooter_motor[0] ? uint16_t(ctrl.shooter_motor[0]->curspeed):0);
        W2(pl+40,ctrl.shooter_motor[1] ? uint16_t(ctrl.shooter_motor[1]->curspeed):0);
        W4(pl+44,ctrl.supply_motor[0] ? uint32_t(ctrl.supply_motor[0]->sum_angle):0);
        WF(pl+48,ctrl.supply_motor[0] ? ctrl.supply_motor[0]->setangle:0.f);
#endif
        taskEXIT_CRITICAL();
        w+=BuildFrame(XUC_T_FEED_DIAGNOSTIC,pl,sizeof(pl),seq_feed++,tx_buf+w);
    }

	/* 四帧一次DMA发送。诊断不参与许可、参考新鲜度或电机控制。 */
    if (m_uart->DMATransmit(tx_buf, w)) {
        // Cache only a frame actually submitted, using exactly its serialized values.
        const uint8_t* pl = tx_buf + XUC_HDR_LEN;
        PushRef(R2(pl+2u), R2(tx_buf+6u), R4u(pl+4u), RF(pl+16u), RF(pl+12u), pl[1]);
    }
}
