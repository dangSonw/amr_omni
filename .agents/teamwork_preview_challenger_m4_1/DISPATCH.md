# Challenger M4-1 Dispatch
Task: Milestone 4 Stress Verification (Binary Serial Protocol Codec, Fuzzing & Buffer Overflows)
Directory: /home/sonev/amr_omni/.agents/teamwork_preview_challenger_m4_1

## 2026-09-20T07:44:43Z
You are teamwork_preview_challenger_m4_1, adversarially verifying Milestone 4: Serial Protocol Binary Framing, Fuzzing & Codec Robustness.
Your working directory is: /home/sonev/amr_omni/.agents/teamwork_preview_challenger_m4_1

MANDATORY FIRST STEP:
Read /home/sonev/amr_omni/ORIGINAL_REQUEST.md and /home/sonev/amr_omni/PROJECT.md before doing any work.

Challenger Objectives:
1. Adversarially stress test the binary serial protocol implementation in `firmware/stm32_f407vg_arduino_sim` and `tests/e2e/harness/serial_protocol_oracle.py`:
   - Test buffer overflows, 0-byte, 64-byte, and >64-byte payload lengths.
   - Test stream noise: random byte injection, shifted synchronization headers `[0xAA 0x55]`, corrupted CRC16, missing tail bytes `0x7D`.
   - Fuzz deserialize function with 50,000+ random byte sequences: must reject 100% without segfaults, buffer over-reads, or buffer overflows.
2. Write and execute a dedicated stress verification script or test suite.
3. Document your findings, empirical data, and verdict (APPROVE or REQUEST_CHANGES) in `/home/sonev/amr_omni/.agents/teamwork_preview_challenger_m4_1/handoff.md` and send_message back to parent.
