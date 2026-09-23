"use client";

import { useEffect, useState } from "react";
import { RobotConfig } from "@/types/robot";
import { Check, Copy, RefreshCw, Save, Settings2, ShieldCheck, Zap } from "lucide-react";

const MOTOR_LABELS = [
  { id: 0, code: "M1", name: "Trước Phải (FR)", loc: "Front-Right" },
  { id: 1, code: "M2", name: "Trước Trái (FL)", loc: "Front-Left" },
  { id: 2, code: "M3", name: "Sau Trái (RL)", loc: "Rear-Left" },
  { id: 3, code: "M4", name: "Sau Phải (RR)", loc: "Rear-Right" },
];

export function ConfigPanel() {
  const getArray4 = (val: number | number[] | undefined, defaultVal: number): [number, number, number, number] => {
    if (Array.isArray(val)) {
      return [
        typeof val[0] === "number" ? val[0] : defaultVal,
        typeof val[1] === "number" ? val[1] : defaultVal,
        typeof val[2] === "number" ? val[2] : defaultVal,
        typeof val[3] === "number" ? val[3] : defaultVal,
      ];
    }
    const s = typeof val === "number" ? val : defaultVal;
    return [s, s, s, s];
  };

  const [config, setConfig] = useState<RobotConfig>({
    wheel_radius_m: 0.03,
    wheelbase_m: 0.1312,
    track_width_m: 0.1312,
    max_wheel_speed_rad_s: 18.0,
    max_linear_speed_mps: 0.54,
    max_angular_speed_rad_s: 3.0,
    command_timeout_sec: 0.25,
    control_frequency_hz: 100.0,
    telemetry_frequency_hz: 50.0,
    motor_kp: [1.0, 1.0, 1.0, 1.0],
    motor_ki: [0.0, 0.0, 0.0, 0.0],
    motor_kd: [0.0, 0.0, 0.0, 0.0],
    kalman_q: [0.5, 0.5, 0.5, 0.5],
    kalman_r: [0.04, 0.04, 0.04, 0.04],
    debug_telemetry: true,
  });

  const [saving, setSaving] = useState(false);
  const [savedSuccess, setSavedSuccess] = useState(false);

  // Tải cấu hình hiện tại từ backend API
  useEffect(() => {
    const fetchConfig = async () => {
      try {
        const res = await fetch("/api/config");
        if (res.ok) {
          const data = await res.json();
          setConfig({
            ...data,
            motor_kp: getArray4(data.motor_kp, 1.0),
            motor_ki: getArray4(data.motor_ki, 0.0),
            motor_kd: getArray4(data.motor_kd, 0.0),
            kalman_q: getArray4(data.kalman_q, 0.5),
            kalman_r: getArray4(data.kalman_r, 0.04),
          });
        }
      } catch (err) {
        console.error("Lỗi khi tải cấu hình:", err);
      }
    };
    fetchConfig();
  }, []);

  const handleSave = async (e: React.FormEvent) => {
    e.preventDefault();
    setSaving(true);
    setSavedSuccess(false);
    try {
      const res = await fetch("/api/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(config),
      });
      if (res.ok) {
        setSavedSuccess(true);
        setTimeout(() => setSavedSuccess(false), 2500);
      }
    } catch (err) {
      console.error("Lỗi khi lưu cấu hình:", err);
    } finally {
      setSaving(false);
    }
  };

  const handleChange = (key: keyof RobotConfig, val: string | boolean | number) => {
    setConfig((prev) => ({
      ...prev,
      [key]: typeof val === "boolean" ? val : parseFloat(String(val)) || 0,
    }));
  };

  // Xử lý thay đổi PID từng động cơ
  const handleMotorPidChange = (
    motorIdx: number,
    param: "kp" | "ki" | "kd",
    val: string
  ) => {
    const num = parseFloat(val) || 0;
    const key = `motor_${param}` as "motor_kp" | "motor_ki" | "motor_kd";
    const current = getArray4(config[key], param === "kp" ? 1.0 : 0.0);
    const updated = [...current] as [number, number, number, number];
    updated[motorIdx] = num;
    setConfig((prev) => ({ ...prev, [key]: updated }));
  };

  // Xử lý thay đổi Kalman từng bánh xe
  const handleKalmanChange = (
    wheelIdx: number,
    param: "q" | "r",
    val: string
  ) => {
    const num = parseFloat(val) || 0.0001;
    const key = `kalman_${param}` as "kalman_q" | "kalman_r";
    const current = getArray4(config[key], param === "q" ? 0.5 : 0.04);
    const updated = [...current] as [number, number, number, number];
    updated[wheelIdx] = num;
    setConfig((prev) => ({ ...prev, [key]: updated }));
  };

  // Thao tác nhanh: Đặt tất cả PID về Passthrough (Kp=1.0, Ki=0.0, Kd=0.0)
  const setAllPidPassthrough = () => {
    setConfig((prev) => ({
      ...prev,
      motor_kp: [1.0, 1.0, 1.0, 1.0],
      motor_ki: [0.0, 0.0, 0.0, 0.0],
      motor_kd: [0.0, 0.0, 0.0, 0.0],
    }));
  };

  // Thao tác nhanh: Sao chép PID Động cơ 1 sang 2, 3, 4
  const copyMotor1PidToAll = () => {
    const kp0 = getArray4(config.motor_kp, 1.0)[0];
    const ki0 = getArray4(config.motor_ki, 0.0)[0];
    const kd0 = getArray4(config.motor_kd, 0.0)[0];
    setConfig((prev) => ({
      ...prev,
      motor_kp: [kp0, kp0, kp0, kp0],
      motor_ki: [ki0, ki0, ki0, ki0],
      motor_kd: [kd0, kd0, kd0, kd0],
    }));
  };

  // Thao tác nhanh: Đặt tất cả Kalman về Mặc định (Q=0.5, R=0.04)
  const setAllKalmanDefault = () => {
    setConfig((prev) => ({
      ...prev,
      kalman_q: [0.5, 0.5, 0.5, 0.5],
      kalman_r: [0.04, 0.04, 0.04, 0.04],
    }));
  };

  // Thao tác nhanh: Sao chép Kalman Bánh xe 1 sang 2, 3, 4
  const copyWheel1KalmanToAll = () => {
    const q0 = getArray4(config.kalman_q, 0.5)[0];
    const r0 = getArray4(config.kalman_r, 0.04)[0];
    setConfig((prev) => ({
      ...prev,
      kalman_q: [q0, q0, q0, q0],
      kalman_r: [r0, r0, r0, r0],
    }));
  };

  const kpArray = getArray4(config.motor_kp, 1.0);
  const kiArray = getArray4(config.motor_ki, 0.0);
  const kdArray = getArray4(config.motor_kd, 0.0);
  const qArray = getArray4(config.kalman_q, 0.5);
  const rArray = getArray4(config.kalman_r, 0.04);

  return (
    <div className="flex flex-col rounded-xl border border-slate-200 bg-white p-6 text-slate-800 shadow-sm w-full space-y-6">
      {/* Header */}
      <div className="flex flex-wrap items-center justify-between pb-4 border-b border-slate-100 gap-2">
        <div className="flex items-center gap-2.5">
          <div className="p-2 rounded-lg bg-blue-50 text-blue-600 border border-blue-100">
            <Settings2 className="h-5 w-5" />
          </div>
          <div>
            <h2 className="text-base font-bold text-slate-900">
              Cấu Hình Tham Số Robot, 4 Bộ PID Động Cơ & 4 Bộ Lọc Kalman Bánh Xe
            </h2>
            <p className="text-xs text-slate-500">
              Đồng bộ hai chiều qua topic ROS 2 <code className="text-blue-600 bg-blue-50 px-1 py-0.5 rounded">config/cmd</code> tới Firmware STM32 và mô phỏng Gazebo
            </p>
          </div>
        </div>
      </div>

      <form onSubmit={handleSave} className="space-y-6 text-xs">
        {/* Section 1: Chassis Geometry & Kinematics */}
        <div className="p-4 rounded-xl bg-slate-50/70 border border-slate-200/80 space-y-3">
          <div className="flex items-center justify-between">
            <h3 className="font-bold text-slate-900 uppercase tracking-wider text-xs flex items-center gap-1.5">
              1. Kích Thước Khung Gầm & Giới Hạn Vận Tốc
            </h3>
            <span className="text-[11px] text-slate-500 font-normal">Kinematics & Safety Limits</span>
          </div>

          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-3.5">
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Bán kính bánh xe Wheel Radius (m)
              </label>
              <input
                type="number"
                step="0.001"
                value={config.wheel_radius_m}
                onChange={(e) => handleChange("wheel_radius_m", e.target.value)}
                className="w-full rounded-lg border border-slate-300 bg-white px-3 py-2 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-blue-500/20 focus:border-blue-600"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Chiều dài cơ sở Wheelbase (m)
              </label>
              <input
                type="number"
                step="0.001"
                value={config.wheelbase_m}
                onChange={(e) => handleChange("wheelbase_m", e.target.value)}
                className="w-full rounded-lg border border-slate-300 bg-white px-3 py-2 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-blue-500/20 focus:border-blue-600"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Độ rộng vệt bánh Track Width (m)
              </label>
              <input
                type="number"
                step="0.001"
                value={config.track_width_m}
                onChange={(e) => handleChange("track_width_m", e.target.value)}
                className="w-full rounded-lg border border-slate-300 bg-white px-3 py-2 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-blue-500/20 focus:border-blue-600"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Vận tốc dài tối đa Max Linear Speed (m/s)
              </label>
              <input
                type="number"
                step="0.02"
                value={config.max_linear_speed_mps}
                onChange={(e) => handleChange("max_linear_speed_mps", e.target.value)}
                className="w-full rounded-lg border border-slate-300 bg-white px-3 py-2 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-blue-500/20 focus:border-blue-600"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Vận tốc góc tối đa Max Angular Speed (rad/s)
              </label>
              <input
                type="number"
                step="0.1"
                value={config.max_angular_speed_rad_s}
                onChange={(e) => handleChange("max_angular_speed_rad_s", e.target.value)}
                className="w-full rounded-lg border border-slate-300 bg-white px-3 py-2 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-blue-500/20 focus:border-blue-600"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Thời gian Watchdog ngắt an toàn (s)
              </label>
              <input
                type="number"
                step="0.05"
                value={config.command_timeout_sec}
                onChange={(e) => handleChange("command_timeout_sec", e.target.value)}
                className="w-full rounded-lg border border-slate-300 bg-white px-3 py-2 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-blue-500/20 focus:border-blue-600"
              />
            </div>
          </div>
        </div>

        {/* Section 2: 4 Individual Motor Velocity PID Controllers */}
        <div className="p-4 rounded-xl bg-slate-50/70 border border-slate-200/80 space-y-4">
          <div className="flex flex-wrap items-center justify-between gap-2">
            <div>
              <h3 className="font-bold text-slate-900 uppercase tracking-wider text-xs flex items-center gap-1.5">
                <Zap className="h-4 w-4 text-amber-500" />
                2. Cài Đặt 4 Bộ Điều Khiển PID Vận Tốc Cho 4 Động Cơ Riêng Biệt
              </h3>
              <p className="text-[11px] text-slate-500 mt-0.5">
                Mặc định: <code className="bg-slate-200/70 px-1 py-0.5 rounded font-mono text-slate-800">Kp=1.0, Ki=0.0, Kd=0.0</code> (Chế độ Passthrough tạm thời tắt PID, chuyển tiếp lệnh điều khiển trực tiếp tới actuator)
              </p>
            </div>
            {/* Quick Actions */}
            <div className="flex items-center gap-2">
              <button
                type="button"
                onClick={setAllPidPassthrough}
                className="flex items-center gap-1 px-2.5 py-1.5 rounded-lg border border-slate-200 bg-white hover:bg-slate-100 text-slate-700 text-[11px] font-medium transition shadow-xs"
              >
                <RefreshCw className="h-3 w-3 text-slate-500" />
                Đặt 4 Động Cơ về Passthrough
              </button>
              <button
                type="button"
                onClick={copyMotor1PidToAll}
                className="flex items-center gap-1 px-2.5 py-1.5 rounded-lg border border-blue-200 bg-blue-50 hover:bg-blue-100 text-blue-700 text-[11px] font-medium transition shadow-xs"
              >
                <Copy className="h-3 w-3 text-blue-500" />
                Sao Chép Động Cơ 1 Sang 2, 3, 4
              </button>
            </div>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-3.5">
            {MOTOR_LABELS.map((m) => {
              const isPassthrough =
                Math.abs(kpArray[m.id] - 1.0) < 1e-4 &&
                Math.abs(kiArray[m.id]) < 1e-4 &&
                Math.abs(kdArray[m.id]) < 1e-4;

              return (
                <div
                  key={m.id}
                  className="p-3.5 rounded-xl bg-white border border-slate-200 shadow-xs space-y-2.5"
                >
                  <div className="flex items-center justify-between pb-1.5 border-b border-slate-100">
                    <div>
                      <span className="font-bold text-slate-800 text-xs">{m.code} - {m.name}</span>
                      <span className="block text-[10px] text-slate-400 font-mono">{m.loc}</span>
                    </div>
                    {isPassthrough ? (
                      <span className="px-2 py-0.5 rounded-full text-[10px] font-semibold bg-emerald-100 text-emerald-700">
                        Passthrough
                      </span>
                    ) : (
                      <span className="px-2 py-0.5 rounded-full text-[10px] font-semibold bg-purple-100 text-purple-700">
                        PID Vòng Kín
                      </span>
                    )}
                  </div>

                  <div className="space-y-2 font-mono">
                    <div>
                      <label className="block text-slate-500 font-sans text-[11px] mb-0.5">
                        Hệ số tỉ lệ Kp:
                      </label>
                      <input
                        type="number"
                        step="0.01"
                        value={kpArray[m.id]}
                        onChange={(e) => handleMotorPidChange(m.id, "kp", e.target.value)}
                        className="w-full rounded border border-slate-200 bg-slate-50 px-2.5 py-1.5 text-xs text-slate-900 focus:bg-white focus:outline-none focus:border-blue-500"
                      />
                    </div>
                    <div>
                      <label className="block text-slate-500 font-sans text-[11px] mb-0.5">
                        Hệ số tích phân Ki:
                      </label>
                      <input
                        type="number"
                        step="0.01"
                        value={kiArray[m.id]}
                        onChange={(e) => handleMotorPidChange(m.id, "ki", e.target.value)}
                        className="w-full rounded border border-slate-200 bg-slate-50 px-2.5 py-1.5 text-xs text-slate-900 focus:bg-white focus:outline-none focus:border-blue-500"
                      />
                    </div>
                    <div>
                      <label className="block text-slate-500 font-sans text-[11px] mb-0.5">
                        Hệ số vi phân Kd:
                      </label>
                      <input
                        type="number"
                        step="0.0001"
                        value={kdArray[m.id]}
                        onChange={(e) => handleMotorPidChange(m.id, "kd", e.target.value)}
                        className="w-full rounded border border-slate-200 bg-slate-50 px-2.5 py-1.5 text-xs text-slate-900 focus:bg-white focus:outline-none focus:border-blue-500"
                      />
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        </div>

        {/* Section 3: 4 Individual Wheel Velocity Scalar Kalman Filters */}
        <div className="p-4 rounded-xl bg-slate-50/70 border border-slate-200/80 space-y-4">
          <div className="flex flex-wrap items-center justify-between gap-2">
            <div>
              <h3 className="font-bold text-slate-900 uppercase tracking-wider text-xs flex items-center gap-1.5">
                <ShieldCheck className="h-4 w-4 text-blue-500" />
                3. Cài Đặt 4 Bộ Lọc Nhiễu Vận Tốc Kalman Cho 4 Bánh Xe Riêng Biệt
              </h3>
              <p className="text-[11px] text-slate-500 mt-0.5">
                Mặc định: <code className="bg-slate-200/70 px-1 py-0.5 rounded font-mono text-slate-800">Q = 0.5, R = 0.04</code> (Lọc nhiễu đo lường encoder bánh xe trước khi đưa vào Odometry và Odometry EKF)
              </p>
            </div>
            {/* Quick Actions */}
            <div className="flex items-center gap-2">
              <button
                type="button"
                onClick={setAllKalmanDefault}
                className="flex items-center gap-1 px-2.5 py-1.5 rounded-lg border border-slate-200 bg-white hover:bg-slate-100 text-slate-700 text-[11px] font-medium transition shadow-xs"
              >
                <RefreshCw className="h-3 w-3 text-slate-500" />
                Đặt 4 Bánh về Mặc Định (0.5, 0.04)
              </button>
              <button
                type="button"
                onClick={copyWheel1KalmanToAll}
                className="flex items-center gap-1 px-2.5 py-1.5 rounded-lg border border-blue-200 bg-blue-50 hover:bg-blue-100 text-blue-700 text-[11px] font-medium transition shadow-xs"
              >
                <Copy className="h-3 w-3 text-blue-500" />
                Sao Chép Bánh 1 Sang 2, 3, 4
              </button>
            </div>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-3.5">
            {MOTOR_LABELS.map((m) => (
              <div
                key={m.id}
                className="p-3.5 rounded-xl bg-white border border-slate-200 shadow-xs space-y-2.5"
              >
                <div className="flex items-center justify-between pb-1.5 border-b border-slate-100">
                  <div>
                    <span className="font-bold text-slate-800 text-xs">Bánh {m.id + 1} - {m.name}</span>
                    <span className="block text-[10px] text-slate-400 font-mono">Wheel Filter {m.id + 1}</span>
                  </div>
                  <span className="px-2 py-0.5 rounded-full text-[10px] font-semibold bg-sky-100 text-sky-700">
                    Scalar Kalman
                  </span>
                </div>

                <div className="space-y-2 font-mono">
                  <div>
                    <label className="block text-slate-500 font-sans text-[11px] mb-0.5">
                      Nhiễu quá trình Q (Process Noise):
                    </label>
                    <input
                      type="number"
                      step="0.01"
                      value={qArray[m.id]}
                      onChange={(e) => handleKalmanChange(m.id, "q", e.target.value)}
                      className="w-full rounded border border-slate-200 bg-slate-50 px-2.5 py-1.5 text-xs text-slate-900 focus:bg-white focus:outline-none focus:border-blue-500"
                    />
                  </div>
                  <div>
                    <label className="block text-slate-500 font-sans text-[11px] mb-0.5">
                      Nhiễu đo lường R (Measurement Noise):
                    </label>
                    <input
                      type="number"
                      step="0.001"
                      value={rArray[m.id]}
                      onChange={(e) => handleKalmanChange(m.id, "r", e.target.value)}
                      className="w-full rounded border border-slate-200 bg-slate-50 px-2.5 py-1.5 text-xs text-slate-900 focus:bg-white focus:outline-none focus:border-blue-500"
                    />
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Buttons */}
        <div className="flex items-center justify-end gap-3 pt-2">
          {savedSuccess && (
            <span className="flex items-center gap-1.5 font-semibold text-emerald-600 text-xs">
              <Check className="h-4 w-4" /> Đã lưu và truyền tham số tới STM32 (config/cmd) thành công!
            </span>
          )}
          <button
            type="submit"
            disabled={saving}
            className="flex items-center gap-2 rounded-lg bg-blue-600 px-6 py-2.5 font-semibold text-white hover:bg-blue-700 transition active:scale-95 disabled:opacity-50 shadow-sm"
          >
            <Save className="h-4 w-4" />
            <span>{saving ? "Đang lưu..." : "Áp Dụng Cấu Hình & Gửi STM32"}</span>
          </button>
        </div>
      </form>
    </div>
  );
}
