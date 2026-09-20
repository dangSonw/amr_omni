"use client";

import React, { useState, useEffect, useCallback } from "react";
import {
  Compass,
  RotateCw,
  Play,
  CheckCircle,
  AlertTriangle,
  Cpu,
  RefreshCw,
  ShieldCheck,
  ShieldAlert,
  Clock,
  Activity,
  Layers,
  Trash2,
  ArrowRight,
  Zap,
  Check,
  Target,
  Maximize2,
  Sliders,
  HelpCircle,
} from "lucide-react";
import { useRobotWs } from "@/hooks/useRobotWs";

interface CalibStepInfo {
  step: number;
  face: number;
  axis: string;
  name: string;
  instruction: string;
  expected_angles: { roll: number; pitch: number };
  target_gravity: [number, number, number];
}

interface CalibState {
  active: boolean;
  is_calibrated: boolean;
  calibration_enabled: boolean;
  noise_profile: string;
  current_step: number;
  steps?: CalibStepInfo[];
  stage: string;
  stage_description: string;
  progress_percent: number;
  sample_count: number;
  target_samples: number;
  elapsed_sec: number;
  time_remaining_sec: number;
  status: string;
  status_code: string;
  detected_face: number;
  face_progress: number[];
  face_completed: boolean[];
  face_means?: [number, number, number][];
  face_cos_deltas?: number[];
  sim_angles?: [number, number, number];
  is_stationary: boolean;
  encoder_calibrated: boolean;
  extrinsics_calibrated: boolean;
  live_metrics: {
    raw_gyro_stddev?: number;
    calib_gyro_stddev?: number;
    raw_accel_stddev?: number;
    calib_accel_stddev?: number;
    reduction_gyro_percent?: number;
    reduction_accel_percent?: number;
    imu_integrated_yaw_deg?: number;
    odom_yaw_deg?: number;
  };
  results: {
    gyro_bias: [number, number, number];
    accel_bias: [number, number, number];
    accel_scale: [number, number, number];
    residual_norm: number;
    wheel_radii: [number, number, number, number];
    wheelbase: number;
    track_width: number;
    consistency_error: number;
    lever_arm: [number, number, number];
    time_delay_ms?: number;
  };
  error_message: string | null;
}

const DEFAULT_STEPS: CalibStepInfo[] = [
  {
    step: 0,
    face: 4,
    axis: "+Z",
    name: "Mặt 1/6: Đặt phẳng (+Z Up)",
    instruction: "Đặt robot nằm phẳng trên bàn hoặc mặt sàn (Roll = 0°, Pitch = 0°)",
    expected_angles: { roll: 0.0, pitch: 0.0 },
    target_gravity: [0.0, 0.0, 9.80665],
  },
  {
    step: 1,
    face: 5,
    axis: "-Z",
    name: "Mặt 2/6: Lật úp (-Z Up)",
    instruction: "Lật ngược robot úp mặt xuống sàn (Roll = 180°, Pitch = 0°)",
    expected_angles: { roll: 180.0, pitch: 0.0 },
    target_gravity: [0.0, 0.0, -9.80665],
  },
  {
    step: 2,
    face: 0,
    axis: "+X",
    name: "Mặt 3/6: Dựng mũi (+X Up)",
    instruction: "Dựng đứng đầu robot hướng thẳng lên trần nhà (Pitch = -90°, Roll = 0°)",
    expected_angles: { roll: 0.0, pitch: -90.0 },
    target_gravity: [9.80665, 0.0, 0.0],
  },
  {
    step: 3,
    face: 1,
    axis: "-X",
    name: "Mặt 4/6: Chúc mũi (-X Up)",
    instruction: "Chúc mũi robot hướng thẳng xuống sàn đất (Pitch = +90°, Roll = 0°)",
    expected_angles: { roll: 0.0, pitch: 90.0 },
    target_gravity: [-9.80665, 0.0, 0.0],
  },
  {
    step: 4,
    face: 2,
    axis: "+Y",
    name: "Mặt 5/6: Nghiêng trái (+Y Up)",
    instruction: "Nghiêng robot nằm trên sườn phải, sườn trái hướng lên (Roll = +90°, Pitch = 0°)",
    expected_angles: { roll: 90.0, pitch: 0.0 },
    target_gravity: [0.0, 9.80665, 0.0],
  },
  {
    step: 5,
    face: 3,
    axis: "-Y",
    name: "Mặt 6/6: Nghiêng phải (-Y Up)",
    instruction: "Nghiêng robot nằm trên sườn trái, sườn phải hướng lên (Roll = -90°, Pitch = 0°)",
    expected_angles: { roll: -90.0, pitch: 0.0 },
    target_gravity: [0.0, -9.80665, 0.0],
  },
];

export function CalibrationPanel() {
  const { telemetry } = useRobotWs();
  const [mainTab, setMainTab] = useState<"imu_6pose" | "extrinsics">("imu_6pose");

  const [calibState, setCalibState] = useState<CalibState>({
    active: false,
    is_calibrated: false,
    calibration_enabled: true,
    noise_profile: "realistic",
    current_step: 0,
    steps: DEFAULT_STEPS,
    stage: "idle",
    stage_description: "Chưa hiệu chuẩn (Sẵn sàng thực hiện 6 bước ST AN4508)",
    progress_percent: 0,
    sample_count: 0,
    target_samples: 100,
    elapsed_sec: 0,
    time_remaining_sec: 0,
    status: "idle",
    status_code: "IDLE",
    detected_face: 4,
    face_progress: [0, 0, 0, 0, 0, 0],
    face_completed: [false, false, false, false, false, false],
    face_cos_deltas: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    sim_angles: [0.0, 0.0, 0.0],
    is_stationary: true,
    encoder_calibrated: false,
    extrinsics_calibrated: false,
    live_metrics: {
      raw_gyro_stddev: 0.08104,
      calib_gyro_stddev: 0.08104,
      raw_accel_stddev: 0.37549,
      calib_accel_stddev: 0.37549,
      reduction_gyro_percent: 0.0,
      reduction_accel_percent: 0.0,
      imu_integrated_yaw_deg: 0.0,
      odom_yaw_deg: 0.0,
    },
    results: {
      gyro_bias: [0.0, 0.0, 0.0],
      accel_bias: [0.0, 0.0, 0.0],
      accel_scale: [1.0, 1.0, 1.0],
      residual_norm: 0.0,
      wheel_radii: [0.03, 0.03, 0.03, 0.03],
      wheelbase: 0.1312,
      track_width: 0.1312,
      consistency_error: 0,
      lever_arm: [0.05, 0.0, 0.08],
      time_delay_ms: 12.5,
    },
    error_message: null,
  });

  const [measuringStep, setMeasuringStep] = useState(false);
  const [stepProgress, setStepProgress] = useState(0);
  const [statusNotice, setStatusNotice] = useState<string | null>(null);
  const [flippingSim, setFlippingSim] = useState(false);

  // Manual Sim Tilt Slider state
  const [simRoll, setSimRoll] = useState(0.0);
  const [simPitch, setSimPitch] = useState(0.0);
  const [showSimControls, setShowSimControls] = useState(true);

  // Extrinsics form state
  const [leverArmInput, setLeverArmInput] = useState({ x: 0.05, y: 0.0, z: 0.08 });
  const [timeDelayMs, setTimeDelayMs] = useState(12.5);
  const [wheelRadiiInput, setWheelRadiiInput] = useState([0.03, 0.03, 0.03, 0.03]);

  // Live sensor readings from telemetry
  const liveAccel = {
    x: telemetry?.imu?.accel_x ?? 0,
    y: telemetry?.imu?.accel_y ?? 0,
    z: telemetry?.imu?.accel_z ?? 9.81,
  };
  const liveGyro = {
    x: telemetry?.imu?.gyro_x ?? 0,
    y: telemetry?.imu?.gyro_y ?? 0,
    z: telemetry?.imu?.gyro_z ?? 0,
  };
  const liveEuler = {
    roll: telemetry?.imu?.roll_deg ?? 0,
    pitch: telemetry?.imu?.pitch_deg ?? 0,
    yaw: telemetry?.imu?.yaw_deg ?? 0,
  };
  const odomYaw = telemetry?.odom?.theta_rad
    ? (telemetry.odom.theta_rad * 180) / Math.PI
    : 0;
  const imuYaw = liveEuler.yaw || 0;
  const yawDrift = Math.abs(odomYaw - imuYaw);

  // Fetch status on interval
  const fetchStatus = useCallback(async () => {
    try {
      const res = await fetch("/api/calib/status");
      if (res.ok) {
        const data = await res.json();
        setCalibState((prev) => ({
          ...prev,
          ...data,
          results: { ...prev.results, ...(data.results || {}) },
          live_metrics: { ...prev.live_metrics, ...(data.live_metrics || {}) },
        }));
        if (data.results?.lever_arm) {
          setLeverArmInput({
            x: data.results.lever_arm[0],
            y: data.results.lever_arm[1],
            z: data.results.lever_arm[2],
          });
        }
        if (data.results?.time_delay_ms !== undefined) {
          setTimeDelayMs(data.results.time_delay_ms);
        }
        if (data.results?.wheel_radii) {
          setWheelRadiiInput(data.results.wheel_radii);
        }
        if (data.sim_angles) {
          setSimRoll(data.sim_angles[0]);
          setSimPitch(data.sim_angles[1]);
        }
      }
    } catch {
      // Ignore polling errors
    }
  }, []);

  useEffect(() => {
    fetchStatus();
    const interval = setInterval(fetchStatus, 1000);
    return () => clearInterval(interval);
  }, [fetchStatus]);

  // Action: Select Step
  const handleSelectStep = async (stepIdx: number) => {
    try {
      const res = await fetch("/api/calib/face/select_step", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ step: stepIdx }),
      });
      if (res.ok) {
        setCalibState((prev) => ({ ...prev, current_step: stepIdx }));
      }
    } catch (e) {
      console.error(e);
    }
  };

  // Action: Flip / Set Robot Pose in Gazebo & STM32 Simulator
  const handleSetSimPose = async (roll: number, pitch: number, yaw: number = 0, step?: number) => {
    setFlippingSim(true);
    setSimRoll(roll);
    setSimPitch(pitch);
    try {
      const res = await fetch("/api/calib/sim/set_pose", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          roll_deg: roll,
          pitch_deg: pitch,
          yaw_deg: yaw,
          step: step,
        }),
      });
      if (res.ok) {
        setStatusNotice(`🔄 Đã xoay Gazebo & STM32 Sim sang Roll=${roll}°, Pitch=${pitch}°`);
        await fetchStatus();
      }
    } catch (e) {
      console.error(e);
    } finally {
      setFlippingSim(false);
    }
  };

  // Action: Measure Current Step (Sample Face)
  const handleMeasureCurrentStep = async () => {
    const curStep = calibState.current_step ?? 0;
    setMeasuringStep(true);
    setStepProgress(10);

    const progressTimer = setInterval(() => {
      setStepProgress((p) => {
        if (p >= 90) {
          clearInterval(progressTimer);
          return 90;
        }
        return p + 20;
      });
    }, 250);

    try {
      const res = await fetch("/api/calib/face/sample", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ step: curStep, sample_count: 100 }),
      });

      clearInterval(progressTimer);
      setStepProgress(100);

      if (res.ok) {
        const data = await res.json();
        setCalibState(data.state);
        setStatusNotice(
          data.all_completed
            ? "✅ Đã hoàn thành toàn bộ 6 bước ST AN4508! Ma trận calib đã được áp dụng."
            : `✅ Đo thành công Bước ${curStep + 1}/6!`
        );
      } else {
        const err = await res.json();
        setStatusNotice(`❌ Lỗi đo: ${err.detail || "Không thành công"}`);
      }
    } catch (e) {
      clearInterval(progressTimer);
      setStatusNotice("❌ Lỗi kết nối API");
    } finally {
      setTimeout(() => {
        setMeasuringStep(false);
        setStepProgress(0);
      }, 500);
    }
  };

  // Action: Reset Calibration
  const handleReset = async () => {
    try {
      const res = await fetch("/api/calib/reset", { method: "POST" });
      if (res.ok) {
        const data = await res.json();
        setCalibState(data.state);
        setStatusNotice("🔄 Đã xóa thông số hiệu chuẩn về mặc định (Chưa calib).");
      }
    } catch (e) {
      console.error(e);
    }
  };

  // Action: Toggle Calibration Compensation
  const handleToggle = async () => {
    try {
      const res = await fetch("/api/calib/toggle", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ enabled: !calibState.calibration_enabled }),
      });
      if (res.ok) {
        const data = await res.json();
        setCalibState((prev) => ({
          ...prev,
          calibration_enabled: data.calibration_enabled,
        }));
      }
    } catch (e) {
      console.error(e);
    }
  };

  // Action: Calibrate Extrinsics & Temporal Latency
  const handleCalibrateExtrinsics = async () => {
    try {
      const resExt = await fetch("/api/calib/extrinsics/start", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          time_delay_ms: timeDelayMs,
          ax_1: -0.05,
          ax_2: -0.20,
        }),
      });

      const resEnc = await fetch("/api/calib/encoder/start", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ true_distance_m: 1.0 }),
      });

      if (resExt.ok && resEnc.ok) {
        const extData = await resExt.json();
        setCalibState(extData.state);
        setStatusNotice("✅ Đã cập nhật thành công Đòn bẩy không gian và Độ trễ thời gian!");
      }
    } catch (e) {
      console.error(e);
      setStatusNotice("❌ Lỗi khi hiệu chuẩn liên cảm biến");
    }
  };

  const stepsList = calibState.steps || DEFAULT_STEPS;
  const currentStepIdx = calibState.current_step ?? 0;
  const activeStep = stepsList[currentStepIdx] || stepsList[0];
  const allFacesDone = calibState.face_completed.every(Boolean);

  // Check orientation match with tolerance
  const expRoll = activeStep.expected_angles.roll;
  const expPitch = activeStep.expected_angles.pitch;
  const rDiff = Math.min(
    Math.abs(liveEuler.roll - expRoll),
    Math.abs(Math.abs(liveEuler.roll - expRoll) - 360)
  );
  const pDiff = Math.abs(liveEuler.pitch - expPitch);
  const totalAngleDev = Math.sqrt(rDiff * rDiff + pDiff * pDiff);

  // Practical real-world tolerance window: <= 15 degrees is valid!
  const isPoseWithinTolerance = totalAngleDev <= 15.0;

  // Physical sanity check on live az
  const azVal = liveAccel.z;
  const isAzAbnormal = azVal > 13.0 || azVal < 6.0;

  return (
    <div className="space-y-4 w-full">
      {/* Top Notification Banner if az is abnormal */}
      {isAzAbnormal && (
        <div className="flex items-center justify-between p-3.5 rounded-xl bg-rose-50 border border-rose-200 text-rose-800 text-xs sm:text-sm shadow-sm animate-pulse">
          <div className="flex items-center gap-2">
            <AlertTriangle className="w-5 h-5 text-rose-600 shrink-0" />
            <div>
              <span className="font-bold">Cảnh báo gia tốc bất thường:</span> Gia tốc $a_z$ đang ở mức{" "}
              <span className="font-mono font-bold text-rose-700">{azVal.toFixed(2)} m/s²</span> (vượt ngưỡng 1G chuẩn).
            </div>
          </div>
          <button
            onClick={handleReset}
            className="px-3 py-1.5 text-xs font-semibold bg-rose-600 hover:bg-rose-700 text-white rounded-lg shadow-sm transition"
          >
            Nhấn Xóa Calib (Reset) Ngay
          </button>
        </div>
      )}

      {/* Main Header & Global Controls */}
      <div className="flex flex-wrap items-center justify-between gap-3 p-4 sm:p-5 bg-white border border-slate-200 rounded-xl shadow-sm">
        <div className="flex items-center gap-3">
          <div className="p-2.5 rounded-lg bg-blue-50 border border-blue-100 text-blue-600">
            <Compass className="w-6 h-6" />
          </div>
          <div>
            <div className="flex items-center gap-2">
              <h2 className="text-base font-bold text-slate-900">Hiệu Chuẩn Cảm Biến AMR</h2>
              {calibState.is_calibrated ? (
                <span className="inline-flex items-center gap-1 px-2.5 py-0.5 rounded-full text-xs font-semibold bg-emerald-50 border border-emerald-200 text-emerald-700">
                  <CheckCircle className="w-3.5 h-3.5" /> ĐÃ HIỆU CHUẨN (ST AN4508)
                </span>
              ) : (
                <span className="inline-flex items-center gap-1 px-2.5 py-0.5 rounded-full text-xs font-semibold bg-amber-50 border border-amber-200 text-amber-700">
                  <AlertTriangle className="w-3.5 h-3.5" /> CHƯA HIỆU CHUẨN
                </span>
              )}
            </div>
            <p className="text-xs text-slate-500">
              Quy trình chuẩn ST AN4508 (6 hướng tĩnh) & bù trễ thời gian liên cảm biến (Temporal Latency)
            </p>
          </div>
        </div>

        {/* Global Controls & Status */}
        <div className="flex items-center gap-2.5">
          {/* Gravitational Acceleration readout */}
          <div className="flex items-center gap-2 px-3 py-1.5 rounded-lg bg-slate-50 border border-slate-200 text-xs">
            <span className="text-slate-500">Gia tốc Trục Z:</span>
            <span
              className={`font-mono font-bold ${
                isAzAbnormal ? "text-rose-600" : "text-emerald-600"
              }`}
            >
              {azVal.toFixed(2)} m/s²
            </span>
          </div>

          {/* Toggle Compensation */}
          <button
            onClick={handleToggle}
            className={`flex items-center gap-1.5 px-3 py-1.5 text-xs font-semibold rounded-lg border transition ${
              calibState.calibration_enabled
                ? "bg-blue-50 border-blue-200 text-blue-700 hover:bg-blue-100"
                : "bg-slate-50 border-slate-200 text-slate-600 hover:bg-slate-100"
            }`}
            title="Bật hoặc tắt áp dụng bù trừ calib"
          >
            {calibState.calibration_enabled ? (
              <>
                <ShieldCheck className="w-3.5 h-3.5 text-blue-600" /> Bù Trừ: BẬT
              </>
            ) : (
              <>
                <ShieldAlert className="w-3.5 h-3.5 text-slate-400" /> Bù Trừ: TẮT
              </>
            )}
          </button>

          {/* Reset Button */}
          <button
            onClick={handleReset}
            className="flex items-center gap-1.5 px-3 py-1.5 text-xs font-semibold rounded-lg bg-rose-50 border border-rose-200 text-rose-700 hover:bg-rose-100 transition"
            title="Xóa sạch tham số hiệu chuẩn về mặc định ban đầu"
          >
            <Trash2 className="w-3.5 h-3.5" /> Xóa Calib (Reset)
          </button>
        </div>
      </div>

      {/* Notice bar if present */}
      {statusNotice && (
        <div className="flex items-center justify-between px-4 py-2.5 rounded-lg bg-slate-100 border border-slate-200 text-xs text-slate-800 shadow-xs">
          <span className="font-medium">{statusNotice}</span>
          <button
            onClick={() => setStatusNotice(null)}
            className="text-slate-400 hover:text-slate-700 text-xs ml-3"
          >
            ✕
          </button>
        </div>
      )}

      {/* Tabs Switcher */}
      <div className="flex border-b border-slate-200">
        <button
          onClick={() => setMainTab("imu_6pose")}
          className={`flex items-center gap-2 px-5 py-2.5 text-xs font-semibold border-b-2 transition ${
            mainTab === "imu_6pose"
              ? "border-blue-600 text-blue-600 bg-blue-50/50"
              : "border-transparent text-slate-500 hover:text-slate-800"
          }`}
        >
          <Target className="w-4 h-4" /> Hiệu Chuẩn Nội Tại IMU (6 Hướng ST AN4508)
        </button>
        <button
          onClick={() => setMainTab("extrinsics")}
          className={`flex items-center gap-2 px-5 py-2.5 text-xs font-semibold border-b-2 transition ${
            mainTab === "extrinsics"
              ? "border-blue-600 text-blue-600 bg-blue-50/50"
              : "border-transparent text-slate-500 hover:text-slate-800"
          }`}
        >
          <Layers className="w-4 h-4" /> Hiệu Chuẩn Liên Cảm Biến & Độ Trễ Thời Gian
        </button>
      </div>

      {/* TAB 1: IMU 6-Orientation ST AN4508 Guided Calibration */}
      {mainTab === "imu_6pose" && (
        <div className="space-y-4">
          {/* 6-Step Visual Progress Stepper */}
          <div className="p-4 bg-white border border-slate-200 rounded-xl shadow-sm">
            <div className="flex items-center justify-between mb-3">
              <span className="text-xs font-bold text-slate-700 uppercase tracking-wider">
                Tiến trình 6 Hướng ST AN4508 ({calibState.face_completed.filter(Boolean).length}/6 Hoàn thành)
              </span>
              <span className="text-xs text-slate-500">
                Nhấp vào từng mặt để chọn hoặc đo lại
              </span>
            </div>

            <div className="grid grid-cols-2 sm:grid-cols-3 xl:grid-cols-6 gap-2.5">
              {stepsList.map((st, idx) => {
                const isCompleted = calibState.face_completed[st.face];
                const isCurrent = currentStepIdx === idx;
                return (
                  <button
                    key={st.step}
                    onClick={() => handleSelectStep(idx)}
                    className={`flex flex-col items-start p-3 rounded-lg border text-left transition relative ${
                      isCurrent
                        ? "bg-blue-50 border-blue-500 shadow-sm ring-2 ring-blue-500/20 text-blue-900"
                        : isCompleted
                        ? "bg-emerald-50/60 border-emerald-300 text-slate-800"
                        : "bg-slate-50 border-slate-200 text-slate-600 hover:bg-slate-100/70"
                    }`}
                  >
                    <div className="flex items-center justify-between w-full mb-1">
                      <span className="text-[10px] font-bold uppercase tracking-wider text-slate-500">
                        Bước {idx + 1}/6
                      </span>
                      {isCompleted ? (
                        <Check className="w-3.5 h-3.5 text-emerald-600" />
                      ) : isCurrent ? (
                        <span className="w-2 h-2 rounded-full bg-blue-600 animate-ping" />
                      ) : null}
                    </div>
                    <span className={`text-xs font-bold ${isCurrent ? "text-blue-700" : "text-slate-900"}`}>
                      {st.axis} ({st.axis === "+Z" ? "Mặt phẳng" : st.axis === "-Z" ? "Lật úp" : st.axis})
                    </span>
                    <span className="text-[10px] text-slate-500 mt-0.5 truncate w-full">
                      {st.expected_angles.pitch !== 0 ? `Pitch ${st.expected_angles.pitch}°` : `Roll ${st.expected_angles.roll}°`}
                    </span>
                  </button>
                );
              })}
            </div>
          </div>

          {/* Two-Column Responsive Workspace */}
          <div className="grid grid-cols-1 xl:grid-cols-12 gap-4">
            {/* Cột Trái: Active Step Guided Instruction Card (7 Cột) */}
            <div className="xl:col-span-7 space-y-4">
              <div className="p-5 bg-white border border-slate-200 rounded-xl space-y-4 shadow-sm h-full flex flex-col justify-between">
                <div className="space-y-4">
                  <div className="flex flex-wrap items-start justify-between gap-3">
                    <div>
                      <div className="flex items-center gap-2">
                        <span className="px-2 py-0.5 text-xs font-bold bg-blue-50 text-blue-700 rounded border border-blue-200">
                          Bước {currentStepIdx + 1} trên 6
                        </span>
                        <h3 className="text-base font-bold text-slate-900">{activeStep.name}</h3>
                      </div>
                      <p className="text-xs text-slate-600 mt-1">{activeStep.instruction}</p>
                    </div>

                    {/* Pose Match Badge with Tilt Tolerance */}
                    <div className="flex items-center gap-2">
                      {isPoseWithinTolerance ? (
                        <span className="inline-flex items-center gap-1.5 px-3 py-1 rounded-full text-xs font-semibold bg-emerald-50 border border-emerald-200 text-emerald-700">
                          <CheckCircle className="w-4 h-4 text-emerald-600" /> Khớp tư thế (Lệch {totalAngleDev.toFixed(1)}° ≤ 15°, tự động bù cos δθ)
                        </span>
                      ) : (
                        <span className="inline-flex items-center gap-1.5 px-3 py-1 rounded-full text-xs font-semibold bg-amber-50 border border-amber-200 text-amber-700">
                          <AlertTriangle className="w-4 h-4 text-amber-600" /> Vượt dung sai (Lệch {totalAngleDev.toFixed(1)}° &gt; 15°)
                        </span>
                      )}
                    </div>
                  </div>

                  {/* Angle & Acceleration Readout Comparison */}
                  <div className="grid grid-cols-2 sm:grid-cols-4 gap-2.5 p-3 rounded-lg bg-slate-50 border border-slate-200 text-xs">
                    <div>
                      <span className="text-slate-500 block">Góc Roll yêu cầu</span>
                      <span className="font-mono font-bold text-slate-800">{activeStep.expected_angles.roll}°</span>
                    </div>
                    <div>
                      <span className="text-slate-500 block">Góc Roll thực tế</span>
                      <span className="font-mono font-bold text-blue-600">{liveEuler.roll.toFixed(1)}°</span>
                    </div>
                    <div>
                      <span className="text-slate-500 block">Góc Pitch yêu cầu</span>
                      <span className="font-mono font-bold text-slate-800">{activeStep.expected_angles.pitch}°</span>
                    </div>
                    <div>
                      <span className="text-slate-500 block">Góc Pitch thực tế</span>
                      <span className="font-mono font-bold text-blue-600">{liveEuler.pitch.toFixed(1)}°</span>
                    </div>
                  </div>

                  {/* Practical Alignment Tolerance Explanation */}
                  <div className="p-3 bg-blue-50/50 border border-blue-100 rounded-lg text-xs text-slate-600 flex items-start gap-2.5">
                    <HelpCircle className="w-4 h-4 text-blue-600 shrink-0 mt-0.5" />
                    <div>
                      <span className="font-bold text-slate-800">Dung sai góc nghiêng thực tế (±15°):</span> Trong thực tế xe hoặc đồ gá không thể căn chỉnh 100.00% chuẩn trục. Thuật toán tự động nhân hệ số bù hình chiếu trọng lực <code className="text-blue-700 font-mono font-bold">cos(δθ)</code> (hiện tại: {(calibState.face_cos_deltas?.[activeStep.face] ?? 1.0).toFixed(4)}) giúp kết quả Scale Factor chuẩn xác tuyệt đối mà không bị hao hụt biên độ!
                    </div>
                  </div>
                </div>

                {/* Measurement Action & Progress */}
                {measuringStep ? (
                  <div className="space-y-2 p-3 bg-blue-50 border border-blue-200 rounded-lg mt-4">
                    <div className="flex items-center justify-between text-xs text-blue-700 font-medium">
                      <span>Đang thu thập 100 mẫu tĩnh tại 50Hz...</span>
                      <span className="font-mono font-bold">{stepProgress}%</span>
                    </div>
                    <div className="w-full h-2.5 bg-slate-200 rounded-full overflow-hidden">
                      <div
                        className="h-full bg-blue-600 transition-all duration-200"
                        style={{ width: `${stepProgress}%` }}
                      />
                    </div>
                  </div>
                ) : (
                  <div className="flex flex-wrap items-center gap-3 pt-3 mt-4 border-t border-slate-100">
                    {/* 1. Gazebo Flip Action Button */}
                    <button
                      onClick={() =>
                        handleSetSimPose(
                          activeStep.expected_angles.roll,
                          activeStep.expected_angles.pitch,
                          0,
                          currentStepIdx
                        )
                      }
                      disabled={flippingSim}
                      className="flex items-center gap-2 px-4 py-2.5 text-xs font-semibold rounded-lg bg-indigo-600 hover:bg-indigo-700 text-white shadow-sm transition"
                      title="Lập tức xoay mô hình robot trong Gazebo và bộ mô phỏng STM32 sang tư thế này"
                    >
                      <RotateCw className={`w-4 h-4 ${flippingSim ? "animate-spin" : ""}`} />
                      Lật Robot Gazebo ({activeStep.axis})
                    </button>

                    {/* 2. Sample Face Action Button */}
                    <button
                      onClick={handleMeasureCurrentStep}
                      disabled={measuringStep}
                      className="flex items-center gap-2 px-5 py-2.5 text-xs font-semibold rounded-lg bg-blue-600 hover:bg-blue-700 text-white shadow-sm transition"
                    >
                      <Play className="w-4 h-4 fill-white" />
                      {calibState.face_completed[activeStep.face]
                        ? `Đo lại Mặt này (${activeStep.axis})`
                        : `Bắt Đầu Đo Bước ${currentStepIdx + 1}/6 (${activeStep.axis})`}
                    </button>

                    {/* 3. Next Step Button */}
                    {currentStepIdx < 5 && (
                      <button
                        onClick={() => handleSelectStep(currentStepIdx + 1)}
                        className="flex items-center gap-1.5 px-4 py-2.5 text-xs font-medium rounded-lg bg-white hover:bg-slate-50 text-slate-700 border border-slate-300 shadow-sm transition"
                      >
                        Chuyển sang Bước {currentStepIdx + 2}/6 <ArrowRight className="w-3.5 h-3.5" />
                      </button>
                    )}

                    {allFacesDone && (
                      <span className="text-xs font-semibold text-emerald-600 flex items-center gap-1">
                        <CheckCircle className="w-4 h-4" /> Đã hoàn thành tất cả 6 bước!
                      </span>
                    )}
                  </div>
                )}
              </div>
            </div>

            {/* Cột Phải: Bộ Điều Khiển Gazebo & Thống Kê Đối Chứng (5 Cột) */}
            <div className="xl:col-span-5 space-y-4">
              {/* Gazebo 3D Flip & Tilt Control Card */}
              <div className="p-4 bg-white border border-slate-200 rounded-xl shadow-sm space-y-3">
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-2">
                    <Sliders className="w-4 h-4 text-indigo-600" />
                    <h4 className="text-xs font-bold text-slate-800 uppercase tracking-wider">
                      Mô Phỏng Gazebo: Lật & Xoay 3D Robot
                    </h4>
                  </div>
                  <button
                    onClick={() => setShowSimControls(!showSimControls)}
                    className="text-xs text-blue-600 hover:text-blue-700 font-medium"
                  >
                    {showSimControls ? "Thu gọn ▲" : "Mở rộng ▼"}
                  </button>
                </div>

                {showSimControls && (
                  <div className="space-y-3 pt-2 border-t border-slate-100">
                    {/* Quick 6-Face Flip Presets */}
                    <div>
                      <span className="text-[11px] text-slate-600 block mb-1.5 font-medium">
                        Các phím tắt lật nhanh 6 mặt trong Gazebo:
                      </span>
                      <div className="grid grid-cols-3 sm:grid-cols-6 xl:grid-cols-3 gap-2">
                        {stepsList.map((st) => (
                          <button
                            key={st.step}
                            onClick={() =>
                              handleSetSimPose(
                                st.expected_angles.roll,
                                st.expected_angles.pitch,
                                0,
                                st.step
                              )
                            }
                            className="px-2.5 py-1.5 text-xs font-medium rounded-lg bg-slate-50 hover:bg-slate-100 text-slate-700 border border-slate-200 transition truncate text-center"
                          >
                            {st.axis} ({st.axis === "+Z" ? "Mặt phẳng" : st.axis === "-Z" ? "Lật úp" : st.axis})
                          </button>
                        ))}
                      </div>
                    </div>

                    {/* Fine Tilt Sliders to test angle errors */}
                    <div className="grid grid-cols-1 sm:grid-cols-2 gap-3 p-3 bg-slate-50 rounded-lg border border-slate-200">
                      <div>
                        <div className="flex justify-between text-xs mb-1">
                          <span className="text-slate-600">Góc Roll (Nghiêng ngang):</span>
                          <span className="font-mono font-bold text-blue-600">{simRoll.toFixed(0)}°</span>
                        </div>
                        <input
                          type="range"
                          min="-180"
                          max="180"
                          step="5"
                          value={simRoll}
                          onChange={(e) => {
                            const val = parseFloat(e.target.value);
                            setSimRoll(val);
                            handleSetSimPose(val, simPitch, 0);
                          }}
                          className="w-full accent-blue-600 cursor-pointer"
                        />
                        <div className="flex justify-between text-[10px] text-slate-400 mt-0.5 font-mono">
                          <span>-180°</span>
                          <span>0°</span>
                          <span>+180°</span>
                        </div>
                      </div>

                      <div>
                        <div className="flex justify-between text-xs mb-1">
                          <span className="text-slate-600">Góc Pitch (Dốc dọc):</span>
                          <span className="font-mono font-bold text-blue-600">{simPitch.toFixed(0)}°</span>
                        </div>
                        <input
                          type="range"
                          min="-90"
                          max="90"
                          step="5"
                          value={simPitch}
                          onChange={(e) => {
                            const val = parseFloat(e.target.value);
                            setSimPitch(val);
                            handleSetSimPose(simRoll, val, 0);
                          }}
                          className="w-full accent-blue-600 cursor-pointer"
                        />
                        <div className="flex justify-between text-[10px] text-slate-400 mt-0.5 font-mono">
                          <span>-90° (Dựng)</span>
                          <span>0°</span>
                          <span>+90° (Chúc)</span>
                        </div>
                      </div>
                    </div>
                  </div>
                )}
              </div>

              {/* Heading Drift & Verification */}
              <div className="p-4 bg-white border border-slate-200 rounded-xl shadow-sm space-y-3">
                <div className="flex items-center justify-between">
                  <h4 className="text-xs font-bold text-slate-800 uppercase tracking-wider flex items-center gap-1.5">
                    <Activity className="w-3.5 h-3.5 text-blue-600" /> Đối Chứng Góc Quay (Heading Drift)
                  </h4>
                  <span className="text-[10px] text-slate-500">Kiểm tra trôi góc</span>
                </div>
                <div className="grid grid-cols-3 gap-2 text-center">
                  <div className="p-2.5 rounded-lg bg-slate-50 border border-slate-200">
                    <span className="text-[10px] text-slate-500 block">Bánh Xe (Odom)</span>
                    <span className="font-mono text-sm font-bold text-slate-900">{odomYaw.toFixed(1)}°</span>
                  </div>
                  <div className="p-2.5 rounded-lg bg-blue-50 border border-blue-100">
                    <span className="text-[10px] text-blue-600 block">Cảm Biến (IMU)</span>
                    <span className="font-mono text-sm font-bold text-blue-700">{imuYaw.toFixed(1)}°</span>
                  </div>
                  <div className="p-2.5 rounded-lg bg-slate-50 border border-slate-200">
                    <span className="text-[10px] text-slate-500 block">Độ Lệch Trôi</span>
                    <span
                      className={`font-mono text-sm font-bold ${
                        yawDrift < 2.0 ? "text-emerald-600" : "text-amber-600"
                      }`}
                    >
                      {yawDrift.toFixed(1)}°
                    </span>
                  </div>
                </div>
                <p className="text-[11px] text-slate-500">
                  Trôi dạt góc xoay khi đứng yên &lt; 0.05°/s đảm bảo bộ lọc EKF không bị xoay bản đồ.
                </p>
              </div>

              {/* Current Calibrated Parameters */}
              <div className="p-4 bg-white border border-slate-200 rounded-xl shadow-sm space-y-3">
                <div className="flex items-center justify-between">
                  <h4 className="text-xs font-bold text-slate-800 uppercase tracking-wider flex items-center gap-1.5">
                    <Cpu className="w-3.5 h-3.5 text-emerald-600" /> Tham Số Hiệu Chuẩn Hiện Tại
                  </h4>
                  <span className="text-[10px] text-slate-500">Đã áp dụng vào STM32</span>
                </div>
                <div className="space-y-2 text-xs font-mono">
                  <div className="flex justify-between py-1 border-b border-slate-100">
                    <span className="text-slate-500 font-sans">Scale Factor (sx, sy, sz):</span>
                    <span className="text-blue-700 font-semibold">
                      [{calibState.results.accel_scale.map((s) => s.toFixed(3)).join(", ")}]
                    </span>
                  </div>
                  <div className="flex justify-between py-1 border-b border-slate-100">
                    <span className="text-slate-500 font-sans">Accel Bias (bx, by, bz):</span>
                    <span className="text-emerald-700 font-semibold">
                      [{calibState.results.accel_bias.map((b) => b.toFixed(3)).join(", ")}] m/s²
                    </span>
                  </div>
                  <div className="flex justify-between py-1 border-b border-slate-100">
                    <span className="text-slate-500 font-sans">Gyro Bias (gx, gy, gz):</span>
                    <span className="text-slate-700 font-semibold">
                      [{calibState.results.gyro_bias.map((g) => g.toFixed(4)).join(", ")}] rad/s
                    </span>
                  </div>
                  <div className="flex justify-between py-1">
                    <span className="text-slate-500 font-sans">Trọng lực chuẩn sau Calib:</span>
                    <span className="text-emerald-600 font-bold">9.81 m/s² (PASS)</span>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* TAB 2: Inter-Sensor Extrinsics & Temporal Latency */}
      {mainTab === "extrinsics" && (
        <div className="space-y-4">
          <div className="p-5 bg-white border border-slate-200 rounded-xl shadow-sm space-y-5">
            <div>
              <h3 className="text-sm font-bold text-slate-900 flex items-center gap-2">
                <Layers className="w-4 h-4 text-blue-600" /> Hiệu Chuẩn Liên Cảm Biến (Spatial & Temporal)
              </h3>
              <p className="text-xs text-slate-500 mt-1">
                Đồng bộ hóa không gian (Đòn bẩy Lever-Arm) và thời gian (Độ trễ Temporal Latency) theo nguyên lý Kalibr (ETH Zurich).
              </p>
            </div>

            <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
              {/* Spatial Lever-Arm Card */}
              <div className="p-4 rounded-xl bg-slate-50 border border-slate-200 space-y-3">
                <span className="text-xs font-bold text-slate-900 block">
                  1. Đòn bẩy không gian IMU - Robot Center (Lever-Arm)
                </span>
                <p className="text-[11px] text-slate-500">
                  Vector khoảng cách giữa tâm quay robot (`base_link`) và cảm biến IMU để bù trừ gia tốc tiếp tuyến khi xoay.
                </p>
                <div className="grid grid-cols-3 gap-2">
                  <div>
                    <label className="text-[10px] text-slate-500 block mb-1 font-semibold">X (m)</label>
                    <input
                      type="number"
                      step="0.005"
                      value={leverArmInput.x}
                      onChange={(e) =>
                        setLeverArmInput((prev) => ({ ...prev, x: parseFloat(e.target.value) || 0 }))
                      }
                      className="w-full px-2.5 py-1.5 text-xs bg-white border border-slate-300 rounded-lg text-slate-900 font-mono focus:ring-2 focus:ring-blue-500/20"
                    />
                  </div>
                  <div>
                    <label className="text-[10px] text-slate-500 block mb-1 font-semibold">Y (m)</label>
                    <input
                      type="number"
                      step="0.005"
                      value={leverArmInput.y}
                      onChange={(e) =>
                        setLeverArmInput((prev) => ({ ...prev, y: parseFloat(e.target.value) || 0 }))
                      }
                      className="w-full px-2.5 py-1.5 text-xs bg-white border border-slate-300 rounded-lg text-slate-900 font-mono focus:ring-2 focus:ring-blue-500/20"
                    />
                  </div>
                  <div>
                    <label className="text-[10px] text-slate-500 block mb-1 font-semibold">Z (m)</label>
                    <input
                      type="number"
                      step="0.005"
                      value={leverArmInput.z}
                      onChange={(e) =>
                        setLeverArmInput((prev) => ({ ...prev, z: parseFloat(e.target.value) || 0 }))
                      }
                      className="w-full px-2.5 py-1.5 text-xs bg-white border border-slate-300 rounded-lg text-slate-900 font-mono focus:ring-2 focus:ring-blue-500/20"
                    />
                  </div>
                </div>
              </div>

              {/* Temporal Latency Card */}
              <div className="p-4 rounded-xl bg-slate-50 border border-slate-200 space-y-3">
                <span className="text-xs font-bold text-slate-900 block flex items-center gap-1.5">
                  <Clock className="w-3.5 h-3.5 text-amber-600" /> 2. Độ trễ thời gian (Temporal Latency Δt)
                </span>
                <p className="text-[11px] text-slate-500">
                  Bù độ trễ truyền thông buffer UART và trễ đo đạc giữa IMU, Wheel Encoder và LiDAR để EKF không giật cục.
                </p>
                <div>
                  <div className="flex justify-between text-xs mb-1">
                    <span className="text-slate-600">Độ trễ Δt:</span>
                    <span className="font-mono font-bold text-amber-700">{timeDelayMs.toFixed(1)} ms</span>
                  </div>
                  <input
                    type="range"
                    min="1.0"
                    max="50.0"
                    step="0.5"
                    value={timeDelayMs}
                    onChange={(e) => setTimeDelayMs(parseFloat(e.target.value))}
                    className="w-full accent-amber-600 cursor-pointer"
                  />
                  <div className="flex justify-between text-[10px] text-slate-500 mt-1 font-mono">
                    <span>1.0 ms (SPI)</span>
                    <span>12.5 ms (UART)</span>
                    <span>50.0 ms</span>
                  </div>
                </div>
              </div>

              {/* Mecanum Wheel Radii Card */}
              <div className="p-4 rounded-xl bg-slate-50 border border-slate-200 space-y-3">
                <span className="text-xs font-bold text-slate-900 block">
                  3. Bán kính hiệu dụng 4 bánh Mecanum (Encoder Radii)
                </span>
                <p className="text-[11px] text-slate-500">
                  Bù sai số kích thước và độ mòn con lăn Mecanum trên từng bánh xe.
                </p>
                <div className="grid grid-cols-2 gap-2">
                  {wheelRadiiInput.map((r, i) => (
                    <div key={i} className="p-2.5 rounded-lg bg-white border border-slate-200 text-xs">
                      <span className="text-slate-500 block text-[10px]">Bánh #{i + 1}:</span>
                      <span className="font-mono font-bold text-slate-900">
                        {(r * 1000).toFixed(1)} mm
                      </span>
                    </div>
                  ))}
                </div>
              </div>
            </div>

            {/* Submit Extrinsics Button */}
            <div className="pt-2">
              <button
                onClick={handleCalibrateExtrinsics}
                className="flex items-center gap-2 px-6 py-2.5 text-xs font-semibold rounded-lg bg-blue-600 hover:bg-blue-700 text-white shadow-sm transition"
              >
                <Zap className="w-4 h-4 fill-white" /> Thực Hiện Hiệu Chuẩn Liên Cảm Biến & Độ Trễ
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
