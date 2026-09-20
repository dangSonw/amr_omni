# Dispatch Log

## 2026-09-20T07:10:12Z

You are the Project Orchestrator for the amr_omni Mecanum AGV project.

Your working directory is: /home/sonev/amr_omni/.agents/teamwork_preview_orchestrator_2
Your root workspace is: /home/sonev/amr_omni
Your reference requests are recorded in: /home/sonev/amr_omni/.agents/ORIGINAL_REQUEST.md

User Objective:
Thực thi dự án AMR Omni Mecanum AGV theo yêu cầu của user ngay trong working directory /home/sonev/amr_omni:
Dự án phát triển và chuẩn hóa hệ thống AMR Mecanum 4 bánh (amr_omni), tích hợp Firmware STM32F407 (FreeRTOS, PLL observer, M/T hybrid velocity, ST AN4508 IMU calib), ROS 2 Jazzy stack (Kinematics Kr compensation, EKF robot_localization, Nav2, Gazebo Sim vs Real Robot bridge), và Web UI (FastAPI + Next.js).

Key Guidelines & Requirements:
1. Đọc và tuân thủ các chỉ dẫn trong .agents, .claude và dùng gitnexus (chỉ mục hiện có) để phân tích code, call graph, impact. Tuân thủ nghiêm ngặt GitNexus rules trong AGENTS.md.
2. Áp dụng chuẩn Ponytail (.claude/skills/ponytail, .claude/skills/ponytail-review): code ngắn gọn, súc tích nhất, ưu tiên stdlib và ROS 2 native node, loại bỏ boilerplate/abstractions thừa (YAGNI), diff ngắn nhất có thể.
3. Đảm bảo hoạt động đồng nhất giữa môi trường thật (Real Robot với real_robot_bringup.launch.py, stm32_bridge) và môi trường ảo Gazebo (simulation_bringup.launch.py, stm32_simulator):
   - Single TF Authority: duy nhất ekf_node phát odom -> base_link TF.
   - Parity binary serial protocol frames giữa STM32 firmware C++ và Simulator.
4. Chạy và kiểm tra toàn bộ test suite (pytest tests/, 191+ tests), đảm bảo pass 100%.
5. Review the existing codebase and artifacts (including prior milestone reports in .agents/), plan the remaining work or validation, dispatch subagents as needed according to the teamwork orchestrator protocol, keep progress.md and BRIEFING.md updated regularly, and report back with your final report and handoff upon completion.

## 2026-09-20T08:00:49Z (From Parent/Sentinel)

Sentinel Liveness Nudge: Please check on your active subagents/dependents (auditor_m4_1, reviewer_m4_1, reviewer_m4_2, challenger_m4_1 have completed their handoffs with CLEAN/APPROVE verdicts) and update progress.md and BRIEFING.md with current Milestone 4 gate status.
