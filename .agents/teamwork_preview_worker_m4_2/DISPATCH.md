## 2026-09-20T08:00:11Z

You are teamwork_preview_worker_m4_2, hardening Milestone 4: Atomic YAML Write Concurrency & Race Condition Elimination.
Your working directory is: /home/sonev/amr_omni/.agents/teamwork_preview_worker_m4_2

MANDATORY FIRST STEP:
Read /home/sonev/amr_omni/ORIGINAL_REQUEST.md and /home/sonev/amr_omni/PROJECT.md before doing any work.

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

Context:
Challenger m4_2 identified that in `web/backend/app/services/calib_service.py`, `save_imu_calib_yaml` and `save_wheel_calib_yaml` currently use static temporary filenames (`imu_calib.yaml.tmp` and `wheel_calib.yaml.tmp`). Under multi-threaded concurrent requests, this causes race conditions (`FileNotFoundError` during `os.replace` or partially read empty files).

Scope of Work:
1. In `web/backend/app/services/calib_service.py`:
   - Import `uuid`.
   - In `save_imu_calib_yaml` and `save_wheel_calib_yaml`:
     Generate unique temporary filenames: `tmp_file = target_dir / f"{target_file.name}.tmp.{uuid.uuid4().hex}"`.
     Ensure `f.flush()` and `os.fsync(f.fileno())` prior to `os.replace(tmp_file, target_file)`.
     Add `try...finally` cleanup to remove `tmp_file` if an unhandled error occurs before `os.replace`.
2. Apply Ponytail standard (.claude/skills/ponytail): minimal diff, clear, stdlib, no bloat.
3. Test Execution:
   - Run `python3 -m pytest tests/stress/test_fastapi_concurrency_stress.py -v`
   - Run `python3 -m pytest web/backend/tests/ -v`
   - Run `python3 -m pytest tests/ -q` (ensure all tests pass 100%)
4. Document results in `/home/sonev/amr_omni/.agents/teamwork_preview_worker_m4_2/handoff.md` and send_message back to parent.
