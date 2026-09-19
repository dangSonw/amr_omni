"use client";

import React, { useState, useEffect } from "react";
import { Compass, RotateCw, Play, Square, CheckCircle, AlertTriangle, Cpu, Save } from "lucide-react";

interface CalibState {
  active: boolean;
  target: string | null;
  routine: string | null;
  subtype: string | null;
  stage: string;
  stage_description: string;
  progress_percent: number;
  status: string;
  status_code: string;
  elapsed_sec: number;
  live_metrics: Record<string, any>;
  results: {
    gyro_bias: [number, number, number];
    accel_bias: [number, number, number];
    accel_scale: [number, number, number];
    residual_norm: number;
    wheel_radii: [number, number, number, number];
    wheelbase: number;
    track_width: number;
    consistency_error: number;
  };
  error_message: string | null;
}

export function CalibrationPanel() {
  const [calibState, setCalibState] = useState<CalibState>({
    active: false,
    target: null,
    routine: null,
    subtype: null,
    stage: "idle",
    stage_description: "Chờ kích hoạt",
    progress_percent: 0,
    status: "idle",
    status_code: "IDLE",
    elapsed_sec: 0,
    live_metrics: {},
    results: {
      gyro_bias: [0, 0, 0],
      accel_bias: [0, 0, 0],
      accel_scale: [1, 1, 1],
      residual_norm: 0,
      wheel_radii: [0.03, 0.03, 0.03, 0.03],
      wheelbase: 0.1312,
      track_width: 0.1312,
      consistency_error: 0,
    },
    error_message: null,
  });

  const [selectedRoutine, setSelectedRoutine] = useState<"imu" | "accel" | "wheel">("imu");
  const [loading, setLoading] = useState(false);
  const [notification, setNotification] = useState<{ message: string; type: "success" | "error" | "info" } | null>(null);

  const fetchStatus = async () => {
    try {
      const res = await fetch("/api/calib/status");
      if (res.ok) {
        const data = await res.json();
        setCalibState(data);
      }
    } catch (err) {
      console.error("Lỗi khi lấy trạng thái hiệu chuẩn:", err);
    }
  };

  useEffect(() => {
    fetchStatus();
    const interval = setInterval(fetchStatus, 1000);
    return () => clearInterval(interval);
  }, []);

  const handleStart = async () => {
    setLoading(true);
    setNotification(null);
    try {
      const res = await fetch("/api/calib/start", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          routine: selectedRoutine === "wheel" ? "wheel" : "imu",
          subtype: selectedRoutine === "accel" ? "6_position" : "static_bias",
          sample_count: 500,
        }),
      });
      if (res.ok) {
        const data = await res.json();
        setCalibState(data.state);
        setNotification({ message: `Đã bắt đầu hiệu chuẩn ${selectedRoutine.toUpperCase()}!`, type: "info" });
      }
    } catch (err) {
      setNotification({ message: "Lỗi kết nối khi bắt đầu hiệu chuẩn", type: "error" });
    } finally {
      setLoading(false);
    }
  };

  const handleStep = async () => {
    setLoading(true);
    try {
      const res = await fetch("/api/calib/step", { method: "POST" });
      if (res.ok) {
        const data = await res.json();
        setCalibState(data.state);
        setNotification({ message: "Đã chuyển sang bước tiếp theo", type: "info" });
      }
    } catch (err) {
      setNotification({ message: "Lỗi khi chuyển bước", type: "error" });
    } finally {
      setLoading(false);
    }
  };

  const handleAbort = async () => {
    setLoading(true);
    try {
      const res = await fetch("/api/calib/abort", { method: "POST" });
      if (res.ok) {
        const data = await res.json();
        setCalibState(data.state);
        setNotification({ message: "Đã dừng hiệu chuẩn", type: "info" });
      }
    } catch (err) {
      setNotification({ message: "Lỗi khi dừng hiệu chuẩn", type: "error" });
    } finally {
      setLoading(false);
    }
  };

  const handleApply = async () => {
    setLoading(true);
    try {
      const res = await fetch("/api/calib/apply", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ persist_flash: true, save_yaml: true }),
      });
      if (res.ok) {
        setNotification({
          message: "Tham số hiệu chuẩn đã được áp dụng và lưu vào file cấu hình ROS 2 YAML / STM32!",
          type: "success",
        });
      }
    } catch (err) {
      setNotification({ message: "Lỗi khi lưu tham số", type: "error" });
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="flex flex-col gap-5 text-slate-800">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center justify-between gap-2 p-5 rounded border border-slate-200 bg-white shadow-xs">
        <div>
          <div className="flex items-center gap-2">
            <Compass className="h-6 w-6 text-brand-600" />
            <h1 className="text-lg font-bold text-slate-900">
              Quy Trình Hiệu Chuẩn Cảm Biến & Động Học
            </h1>
          </div>
          <p className="text-xs text-slate-500 mt-1">
            Triển khai thuật toán ST AN4508 6 hướng tĩnh, khử độ lệch con quay hồi chuyển Welford và ma trận bán kính bánh xe $K_r$.
          </p>
        </div>

        <div className="flex items-center gap-2">
          <span
            className={`inline-flex items-center gap-1.5 px-3 py-1 rounded text-xs font-semibold ${
              calibState.active
                ? "bg-amber-100 text-amber-800 animate-pulse"
                : calibState.status === "success"
                ? "bg-emerald-100 text-emerald-800"
                : "bg-slate-100 text-slate-700"
            }`}
          >
            {calibState.active ? (
              <RotateCw className="h-3.5 w-3.5 animate-spin" />
            ) : calibState.status === "success" ? (
              <CheckCircle className="h-3.5 w-3.5" />
            ) : (
              <Cpu className="h-3.5 w-3.5" />
            )}
            {calibState.status.toUpperCase()}
          </span>
        </div>
      </div>

      {notification && (
        <div
          className={`p-3 rounded text-xs flex items-center gap-2 ${
            notification.type === "success"
              ? "bg-emerald-50 text-emerald-800 border border-emerald-200"
              : notification.type === "error"
              ? "bg-rose-50 text-rose-800 border border-rose-200"
              : "bg-sky-50 text-sky-800 border border-sky-200"
          }`}
        >
          <AlertTriangle className="h-4 w-4" />
          <span>{notification.message}</span>
        </div>
      )}

      {/* Routine Selector & Actions */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        {/* Card 1: IMU Gyro */}
        <div
          onClick={() => !calibState.active && setSelectedRoutine("imu")}
          className={`p-4 rounded border cursor-pointer transition-all ${
            selectedRoutine === "imu"
              ? "border-brand-600 bg-brand-50/20 shadow-sm"
              : "border-slate-200 bg-white hover:border-slate-300"
          }`}
        >
          <h3 className="font-bold text-sm text-slate-900 flex items-center gap-2">
            <Compass className="h-4 w-4 text-brand-600" />
            1. Con Quay Hồi Chuyển (Gyro Bias)
          </h3>
          <p className="text-xs text-slate-500 mt-2">
            Thuật toán Welford lọc tĩnh, đảm bảo độ trôi dạt xoay góc &lt; 0.05&deg;/s khi robot đứng yên.
          </p>
        </div>

        {/* Card 2: IMU Accel */}
        <div
          onClick={() => !calibState.active && setSelectedRoutine("accel")}
          className={`p-4 rounded border cursor-pointer transition-all ${
            selectedRoutine === "accel"
              ? "border-brand-600 bg-brand-50/20 shadow-sm"
              : "border-slate-200 bg-white hover:border-slate-300"
          }`}
        >
          <h3 className="font-bold text-sm text-slate-900 flex items-center gap-2">
            <RotateCw className="h-4 w-4 text-brand-600" />
            2. Gia Tốc Kế (ST AN4508 6 Mặt)
          </h3>
          <p className="text-xs text-slate-500 mt-2">
            Hiệu chuẩn 6 hướng tĩnh chuẩn xác tỉ lệ scale factor và bias trọng trường sai số &lt; 0.5% 1g.
          </p>
        </div>

        {/* Card 3: Wheel Kinematics */}
        <div
          onClick={() => !calibState.active && setSelectedRoutine("wheel")}
          className={`p-4 rounded border cursor-pointer transition-all ${
            selectedRoutine === "wheel"
              ? "border-brand-600 bg-brand-50/20 shadow-sm"
              : "border-slate-200 bg-white hover:border-slate-300"
          }`}
        >
          <h3 className="font-bold text-sm text-slate-900 flex items-center gap-2">
            <Cpu className="h-4 w-4 text-brand-600" />
            3. Động Học Bánh Xe (Wheel Kr)
          </h3>
          <p className="text-xs text-slate-500 mt-2">
            Ước lượng sai số bán kính bánh xe $K_r$ và kích thước cơ sở trục, triệt tiêu sai số trôi tịnh tiến.
          </p>
        </div>
      </div>

      {/* Progress & Live Controls */}
      <div className="p-5 rounded border border-slate-200 bg-white flex flex-col gap-4 shadow-xs">
        <div className="flex items-center justify-between">
          <div>
            <h2 className="text-sm font-bold text-slate-900">
              Tiến Trình: {calibState.stage_description || "Sẵn sàng"}
            </h2>
            <span className="text-xs text-slate-500">
              Giai đoạn: {calibState.stage} | Thời gian: {calibState.elapsed_sec.toFixed(1)}s
            </span>
          </div>
          <span className="text-sm font-bold text-brand-600">
            {calibState.progress_percent}%
          </span>
        </div>

        {/* Progress Bar */}
        <div className="w-full bg-slate-100 rounded-full h-3 overflow-hidden">
          <div
            className="bg-brand-600 h-full transition-all duration-300 rounded-full"
            style={{ width: `${calibState.progress_percent}%` }}
          />
        </div>

        {/* Action Buttons */}
        <div className="flex flex-wrap items-center gap-3 pt-2">
          <button
            onClick={handleStart}
            disabled={calibState.active || loading}
            className="inline-flex items-center gap-1.5 px-4 py-2 bg-brand-600 hover:bg-brand-700 disabled:opacity-50 text-white text-xs font-semibold rounded shadow-xs"
          >
            <Play className="h-3.5 w-3.5" />
            Bắt Đầu Hiệu Chuẩn
          </button>

          {calibState.active && (
            <>
              <button
                onClick={handleStep}
                disabled={loading}
                className="inline-flex items-center gap-1.5 px-4 py-2 bg-slate-800 hover:bg-slate-900 disabled:opacity-50 text-white text-xs font-semibold rounded shadow-xs"
              >
                <CheckCircle className="h-3.5 w-3.5" />
                Chuyển Bước Tiếp Theo
              </button>

              <button
                onClick={handleAbort}
                disabled={loading}
                className="inline-flex items-center gap-1.5 px-4 py-2 bg-rose-600 hover:bg-rose-700 disabled:opacity-50 text-white text-xs font-semibold rounded shadow-xs"
              >
                <Square className="h-3.5 w-3.5" />
                Hủy Bỏ
              </button>
            </>
          )}

          <button
            onClick={handleApply}
            disabled={calibState.active || loading}
            className="inline-flex items-center gap-1.5 px-4 py-2 bg-emerald-600 hover:bg-emerald-700 disabled:opacity-50 text-white text-xs font-semibold rounded shadow-xs ml-auto"
          >
            <Save className="h-3.5 w-3.5" />
            Lưu &amp; Cập Nhật Cấu Hình
          </button>
        </div>
      </div>

      {/* Results View */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
        {/* IMU Calibration Results */}
        <div className="p-5 rounded border border-slate-200 bg-white shadow-xs">
          <h3 className="font-bold text-sm text-slate-900 border-b border-slate-200 pb-2 mb-3 flex items-center justify-between">
            <span>Kết Quả Hiệu Chuẩn IMU</span>
            <span className="text-xs text-emerald-600 font-normal">REP-103 ENU</span>
          </h3>

          <div className="space-y-2.5 text-xs">
            <div>
              <span className="font-semibold text-slate-600 block mb-1">
                Gyroscope Bias [bx, by, bz] (rad/s):
              </span>
              <div className="grid grid-cols-3 gap-2 font-mono bg-slate-50 p-2 rounded border border-slate-100">
                <div>X: {calibState.results.gyro_bias[0].toFixed(6)}</div>
                <div>Y: {calibState.results.gyro_bias[1].toFixed(6)}</div>
                <div>Z: {calibState.results.gyro_bias[2].toFixed(6)}</div>
              </div>
            </div>

            <div>
              <span className="font-semibold text-slate-600 block mb-1">
                Accelerometer Scale Factor [sx, sy, sz]:
              </span>
              <div className="grid grid-cols-3 gap-2 font-mono bg-slate-50 p-2 rounded border border-slate-100">
                <div>X: {calibState.results.accel_scale[0].toFixed(4)}</div>
                <div>Y: {calibState.results.accel_scale[1].toFixed(4)}</div>
                <div>Z: {calibState.results.accel_scale[2].toFixed(4)}</div>
              </div>
            </div>

            <div>
              <span className="font-semibold text-slate-600 block mb-1">
                Accelerometer Bias [bx, by, bz] (m/s&sup2;):
              </span>
              <div className="grid grid-cols-3 gap-2 font-mono bg-slate-50 p-2 rounded border border-slate-100">
                <div>X: {calibState.results.accel_bias[0].toFixed(4)}</div>
                <div>Y: {calibState.results.accel_bias[1].toFixed(4)}</div>
                <div>Z: {calibState.results.accel_bias[2].toFixed(4)}</div>
              </div>
            </div>

            <div className="pt-1 flex items-center justify-between text-slate-500">
              <span>Sai số chuẩn bề mặt (Residual Norm):</span>
              <span className="font-mono font-semibold text-slate-800">
                {calibState.results.residual_norm.toFixed(5)} m/s&sup2;
              </span>
            </div>
          </div>
        </div>

        {/* Wheel Kinematics Results */}
        <div className="p-5 rounded border border-slate-200 bg-white shadow-xs">
          <h3 className="font-bold text-sm text-slate-900 border-b border-slate-200 pb-2 mb-3 flex items-center justify-between">
            <span>Kết Quả Hiệu Chuẩn Động Học Bánh Xe</span>
            <span className="text-xs text-brand-600 font-normal">Moore-Penrose Pseudo-Inverse</span>
          </h3>

          <div className="space-y-2.5 text-xs">
            <div>
              <span className="font-semibold text-slate-600 block mb-1">
                Bán Kính 4 Bánh Xe [FL, FR, RL, RR] (m):
              </span>
              <div className="grid grid-cols-2 sm:grid-cols-4 gap-2 font-mono bg-slate-50 p-2 rounded border border-slate-100">
                <div>FL: {calibState.results.wheel_radii[0].toFixed(4)}</div>
                <div>FR: {calibState.results.wheel_radii[1].toFixed(4)}</div>
                <div>RL: {calibState.results.wheel_radii[2].toFixed(4)}</div>
                <div>RR: {calibState.results.wheel_radii[3].toFixed(4)}</div>
              </div>
            </div>

            <div className="grid grid-cols-2 gap-3 pt-1">
              <div>
                <span className="font-semibold text-slate-600 block mb-1">Wheelbase (m):</span>
                <div className="font-mono bg-slate-50 p-2 rounded border border-slate-100">
                  {calibState.results.wheelbase.toFixed(4)}
                </div>
              </div>
              <div>
                <span className="font-semibold text-slate-600 block mb-1">Track Width (m):</span>
                <div className="font-mono bg-slate-50 p-2 rounded border border-slate-100">
                  {calibState.results.track_width.toFixed(4)}
                </div>
              </div>
            </div>

            <div className="pt-2 flex items-center justify-between text-slate-500">
              <span>Sai số nhất quán động học (Consistency Error):</span>
              <span className="font-mono font-semibold text-emerald-700">
                {calibState.results.consistency_error.toExponential(2)}
              </span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
