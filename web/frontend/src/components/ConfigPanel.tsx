"use client";

import { useEffect, useState } from "react";
import { RobotConfig } from "@/types/robot";
import { Check, Save, Settings2 } from "lucide-react";

export function ConfigPanel() {
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
    motor_kp: 0.08,
    motor_ki: 0.25,
    motor_kd: 0.0005,
    debug_telemetry: true,
  });

  const [saving, setSaving] = useState(false);
  const [savedSuccess, setSavedSuccess] = useState(false);

  // Fetch initial config from backend
  useEffect(() => {
    const fetchConfig = async () => {
      try {
        const res = await fetch("/api/config");
        if (res.ok) {
          const data = await res.json();
          setConfig(data);
        }
      } catch (err) {
        console.error("Lỗi khi tải config:", err);
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
      console.error("Lỗi khi lưu config:", err);
    } finally {
      setSaving(false);
    }
  };

  const handleChange = (key: keyof RobotConfig, val: string | boolean | number) => {
    setConfig((prev) => ({
      ...prev,
      [key]: key === "noise_profile" ? String(val) : (typeof val === "boolean" ? val : parseFloat(String(val)) || 0),
    }));
  };

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
              Cấu Hình Tham Số Robot, Bộ Điều Khiển & Nhiễu Mô Phỏng
            </h2>
            <p className="text-xs text-slate-500">
              Đồng bộ hai chiều với Firmware vi điều khiển STM32 và môi trường mô phỏng Gazebo
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

        {/* Section 2: Motor Velocity PID Gains */}
        <div className="p-4 rounded-xl bg-slate-50/70 border border-slate-200/80 space-y-3">
          <div className="flex items-center justify-between">
            <h3 className="font-bold text-slate-900 uppercase tracking-wider text-xs">
              2. Hệ Số Bộ Điều Khiển Vận Tốc Động Cơ (PID Loop)
            </h3>
            <span className="text-[11px] text-slate-500 font-normal">Feedback Closed-Loop</span>
          </div>

          <div className="grid grid-cols-1 sm:grid-cols-3 gap-3.5">
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Hệ số tỉ lệ Kp (Proportional)
              </label>
              <input
                type="number"
                step="0.01"
                value={config.motor_kp}
                onChange={(e) => handleChange("motor_kp", e.target.value)}
                className="w-full rounded-lg border border-slate-300 bg-white px-3 py-2 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-blue-500/20 focus:border-blue-600"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Hệ số tích phân Ki (Integral)
              </label>
              <input
                type="number"
                step="0.01"
                value={config.motor_ki}
                onChange={(e) => handleChange("motor_ki", e.target.value)}
                className="w-full rounded-lg border border-slate-300 bg-white px-3 py-2 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-blue-500/20 focus:border-blue-600"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Hệ số vi phân Kd (Derivative)
              </label>
              <input
                type="number"
                step="0.0001"
                value={config.motor_kd}
                onChange={(e) => handleChange("motor_kd", e.target.value)}
                className="w-full rounded-lg border border-slate-300 bg-white px-3 py-2 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-blue-500/20 focus:border-blue-600"
              />
            </div>
          </div>
        </div>

        {/* Buttons */}
        <div className="flex items-center justify-end gap-3 pt-2">
          {savedSuccess && (
            <span className="flex items-center gap-1.5 font-semibold text-emerald-600 text-xs">
              <Check className="h-4 w-4" /> Đã lưu và đồng bộ cấu hình thành công!
            </span>
          )}
          <button
            type="submit"
            disabled={saving}
            className="flex items-center gap-2 rounded-lg bg-blue-600 px-6 py-2.5 font-semibold text-white hover:bg-blue-700 transition active:scale-95 disabled:opacity-50 shadow-sm"
          >
            <Save className="h-4 w-4" />
            <span>{saving ? "Đang lưu..." : "Áp Dụng Cấu Hình"}</span>
          </button>
        </div>
      </form>
    </div>
  );
}
