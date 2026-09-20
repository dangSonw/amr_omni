# BRIEFING — 2026-09-20T07:48:00Z

## Mission
Adversarially stress test YAML persistence, FastAPI endpoints concurrency, atomic write integrity, telemetry streaming, and calibration abort/reset cycles under high concurrent load.

## 🔒 My Identity
- Archetype: Empirical Challenger
- Roles: critic, specialist
- Working directory: /home/sonev/amr_omni/.agents/teamwork_preview_challenger_m4_2
- Original parent: e684f9d6-654f-439a-9e8c-99049f9780b5
- Milestone: Milestone 4: FastAPI Concurrency, Atomic YAML Write Integrity & Telemetry Streaming
- Instance: 1 of 1

## 🔒 Key Constraints
- Review-only — do NOT modify implementation code
- Empirically reproduce all bugs; do not trust claims or logs without verification
- .agents/ holds only metadata (plans, progress, handoffs) — NEVER place source code, tests, or data files here

## Current Parent
- Conversation ID: e684f9d6-654f-439a-9e8c-99049f9780b5
- Updated: 2026-09-20T07:48:00Z

## Review Scope
- **Files to review**: `web/backend/app/services/calib_service.py`, `web/backend/app/routers/calib.py`, `web/backend/app/services/telemetry_hub.py`
- **Interface contracts**: `/home/sonev/amr_omni/PROJECT.md`, `/home/sonev/amr_omni/ORIGINAL_REQUEST.md`
- **Review criteria**: Atomic write integrity under concurrent load, concurrent read/write consistency, calibration abort/reset robustness, zero race conditions

## Attack Surface
- **Hypotheses tested**:
  1. Static `.tmp` filename in `save_imu_calib_yaml` and `save_wheel_calib_yaml` causes race conditions under concurrent writes -> CONFIRMED (242/500 and 248/500 write crashes with `FileNotFoundError`).
  2. Concurrent readers reading during writes can observe 0-byte or empty YAML files -> CONFIRMED (172 zero-byte reads, `yaml.safe_load` returning `None`).
  3. POST `/api/calib/apply` crashes under multi-threaded concurrency -> CONFIRMED (60/120 requests failed with 500 error / `FileNotFoundError`).
  4. Calibration abort/reset cycles during active sampling cause task leaks or invalid state transitions -> REJECTED (25/25 cycles handled cleanly without leaks).
- **Vulnerabilities found**:
  - Deterministic `.tmp` temporary filename (`imu_calib.yaml.tmp` and `wheel_calib.yaml.tmp`) in `web/backend/app/services/calib_service.py` breaks POSIX atomic replacement guarantee under concurrent writers, truncating each other's temporary files and causing `FileNotFoundError` and zero-byte file writes.
- **Untested angles**:
  - Multi-process Uvicorn cluster writing to shared network storage (NFS/CIFS) - out of scope for local AGV Jetson architecture.

## Loaded Skills
- None provided in dispatch

## Key Decisions Made
- Authored automated stress suite in `tests/stress/test_fastapi_concurrency_stress.py`.
- Empirically reproduced 4/5 stress test failures demonstrating file corruption and race conditions.
- Proved mitigation: unique temporary file pattern with UUID (`.tmp.{pid}.{uuid}`) restores 100% write/read integrity.
- Issuing `REQUEST_CHANGES` verdict for Milestone 4.

## Artifact Index
- `/home/sonev/amr_omni/.agents/teamwork_preview_challenger_m4_2/DISPATCH.md` — Recorded dispatch instructions
- `/home/sonev/amr_omni/.agents/teamwork_preview_challenger_m4_2/BRIEFING.md` — Situational awareness
- `/home/sonev/amr_omni/.agents/teamwork_preview_challenger_m4_2/progress.md` — Liveness heartbeat
- `/home/sonev/amr_omni/.agents/teamwork_preview_challenger_m4_2/handoff.md` — Final handoff report
- `/home/sonev/amr_omni/tests/stress/test_fastapi_concurrency_stress.py` — Empirical concurrency stress test suite
