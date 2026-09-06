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

  const handleChange = (key: keyof RobotConfig, val: string | boolean) => {
    setConfig((prev) => ({
      ...prev,
      [key]: typeof val === "boolean" ? val : parseFloat(val) || 0,
    }));
  };

  return (
    <div className="flex flex-col rounded border border-slate-200 bg-white p-5 text-slate-800">
      {/* Header */}
      <div className="flex items-center justify-between pb-3 border-b border-slate-200">
        <div className="flex items-center gap-2">
          <Settings2 className="h-5 w-5 text-brand-600" />
          <h2 className="text-base font-bold text-slate-900">
            Robot Parameters & Safety Watchdog
          </h2>
        </div>
        <span className="text-xs text-slate-500">
          Applied to firmware / simulator
        </span>
      </div>

      <form onSubmit={handleSave} className="mt-4 space-y-4 text-xs">
        {/* Section 1: Chassis Geometry & Kinematics */}
        <div>
          <h3 className="font-bold text-slate-900 uppercase tracking-wider text-[11px] mb-2 text-brand-600">
            1. Chassis Geometry & Velocity Limits
          </h3>
          <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Wheel Radius (m)
              </label>
              <input
                type="number"
                step="0.001"
                value={config.wheel_radius_m}
                onChange={(e) => handleChange("wheel_radius_m", e.target.value)}
                className="w-full rounded border border-slate-200 bg-slate-50 px-3 py-1.5 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-brand-500/30"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Wheelbase (m)
              </label>
              <input
                type="number"
                step="0.001"
                value={config.wheelbase_m}
                onChange={(e) => handleChange("wheelbase_m", e.target.value)}
                className="w-full rounded border border-slate-200 bg-slate-50 px-3 py-1.5 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-brand-500/30"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Track Width (m)
              </label>
              <input
                type="number"
                step="0.001"
                value={config.track_width_m}
                onChange={(e) => handleChange("track_width_m", e.target.value)}
                className="w-full rounded border border-slate-200 bg-slate-50 px-3 py-1.5 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-brand-500/30"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Max Linear Speed (m/s)
              </label>
              <input
                type="number"
                step="0.02"
                value={config.max_linear_speed_mps}
                onChange={(e) => handleChange("max_linear_speed_mps", e.target.value)}
                className="w-full rounded border border-slate-200 bg-slate-50 px-3 py-1.5 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-brand-500/30"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Max Angular Speed (rad/s)
              </label>
              <input
                type="number"
                step="0.1"
                value={config.max_angular_speed_rad_s}
                onChange={(e) => handleChange("max_angular_speed_rad_s", e.target.value)}
                className="w-full rounded border border-slate-200 bg-slate-50 px-3 py-1.5 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-brand-500/30"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Safety Watchdog Timeout (s)
              </label>
              <input
                type="number"
                step="0.05"
                value={config.command_timeout_sec}
                onChange={(e) => handleChange("command_timeout_sec", e.target.value)}
                className="w-full rounded border border-slate-200 bg-slate-50 px-3 py-1.5 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-brand-500/30"
              />
            </div>
          </div>
        </div>

        {/* Section 2: Motor Velocity PID Gains */}
        <div className="pt-2 border-t border-slate-100">
          <h3 className="font-bold text-slate-900 uppercase tracking-wider text-[11px] mb-2 text-brand-600">
            2. Wheel Velocity Controller Gains (PID)
          </h3>
          <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Proportional Gain (Kp)
              </label>
              <input
                type="number"
                step="0.01"
                value={config.motor_kp}
                onChange={(e) => handleChange("motor_kp", e.target.value)}
                className="w-full rounded border border-slate-200 bg-slate-50 px-3 py-1.5 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-brand-500/30"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Integral Gain (Ki)
              </label>
              <input
                type="number"
                step="0.01"
                value={config.motor_ki}
                onChange={(e) => handleChange("motor_ki", e.target.value)}
                className="w-full rounded border border-slate-200 bg-slate-50 px-3 py-1.5 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-brand-500/30"
              />
            </div>
            <div>
              <label className="block text-slate-600 font-medium mb-1">
                Derivative Gain (Kd)
              </label>
              <input
                type="number"
                step="0.0001"
                value={config.motor_kd}
                onChange={(e) => handleChange("motor_kd", e.target.value)}
                className="w-full rounded border border-slate-200 bg-slate-50 px-3 py-1.5 font-mono text-slate-900 focus:outline-none focus:ring-2 focus:ring-brand-500/30"
              />
            </div>
          </div>
        </div>

        {/* Buttons */}
        <div className="flex items-center justify-end gap-3 pt-3">
          {savedSuccess && (
            <span className="flex items-center gap-1 font-semibold text-emerald-600">
              <Check className="h-4 w-4" /> Parameters saved successfully!
            </span>
          )}
          <button
            type="submit"
            disabled={saving}
            className="flex items-center gap-2 rounded bg-brand-600 px-4 py-2 font-bold text-white hover:bg-brand-700 transition active:scale-95 disabled:opacity-50"
          >
            <Save className="h-4 w-4" />
            <span>{saving ? "Saving..." : "Apply Configuration"}</span>
          </button>
        </div>
      </form>
    </div>
  );
}
