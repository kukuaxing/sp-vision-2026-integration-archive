# Flash report — 2026-09-14

- Debug probe: ATK ATK-HSWL-CMSIS-DAP (`ATK 20190528`), exposed separately as COM7 for its virtual serial port.
- SWD clock: 1 MHz.
- Selected target: `stm32f405rg`.
- Read device ID: `0x413` (STM32F405/407 family).
- CPUID: `0x410fc241` (Cortex-M4).
- Flash size register: `0x0400` KiB (1 MiB).
- Option control: `0x0fffaaed`; RDP byte `0xaa` (Level 0).
- Original Flash backup: `backups/preflash-20260914/sentry_uc_original_1MiB.bin`.
- Original backup SHA-256: `7E84F1FBA6E8462CE8613166F68416F3C28085DDC5ED11ED3F45DFAF95064CED`.
- Programmed image: `build/sentry_uc.hex`.
- Programmed HEX SHA-256: `1DB85DCC8B3EDED3B1E477336DE82F790770D3F9B9C855DF05A184CBE1F69C47`.
- pyOCD result: 5 sectors erased, 68,608 bytes programmed, exit code 0.
- Independent readback file: `backups/preflash-20260914/sentry_uc_readback_pitch_safe_fix.bin`.
- Independent readback SHA-256: `B3246B981CB0E84C34E660C8F5BF6E168472DD7C0C9142A02F9CB5FF0C54121E`.
- Independent readback: all 68,450 HEX-defined bytes matched (0 mismatches).
- Post-reset core state: running.
- Post-reset CFSR: `0x00000000`.
- Post-reset HFSR: `0x00000000`.

The final image also repairs DM yaw/pitch enable recovery: each motor must produce
its own CAN feedback before ordinary control frames are sent. Missing/stale
feedback causes a rate-limited enable retry every 200 ms instead of relying on
the former one-shot enable and unreachable periodic retry condition.

## Pitch startup safety correction

The first enable-recovery image exposed a legacy absolute pitch target of
`-0.74 rad`. The replacement sentry mechanism has a different zero/direction,
so reliably enabling the motor made it chase that legacy target and drive into
the upper physical stop.

The final image removes that inherited pitch target. The first valid pitch CAN
feedback after each boot/recovery becomes the hold position and the center of a
temporary `+/-0.20 rad` software envelope. Position commands are slew-limited
to `0.50 rad/s`. Motion remains locked until feedback is stable for at least
500 ms and a fresh upper-computer aim frame explicitly reports `control=0`;
missing or timed-out communication cannot unlock the axis.

This startup-relative envelope is a temporary protection, not a substitute for
measuring this sentry's encoder direction, mechanical zero, and physical pitch
limits. Keep motor power removed while manually moving the barrel away from a
physical stop before the first powered test.

The COM7 read-only telemetry smoke test received zero bytes. This does not affect SWD programming. It means the debugger's optional VCP RX/TX is not presently connected to the board's UART4 pins (PA0/PA1), or that UART is connected exclusively to the NUC.
