"use client";

import { useEffect, useState } from "react";
import { RobotConfig } from "@/types/robot";
import { Check, Copy, RefreshCw, Save, Settings2, ShieldCheck, Zap } from "lucide-react";

const MOTOR_LABELS = [
  { id: 0, code: "M1", loc: "FR", name: "Front-Right" },
  { id: 1, code: "M2", loc: "FL", name: "Front-Left" },
  { id: 2, code: "M3", loc: "RL", name: "Rear-Left" },
  { id: 3, code: "M4", loc: "RR", name: "Rear-Right" },
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
        console.error("Config fetch error:", err);
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
      console.error("Config save error:", err);
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

  const setAllPidPassthrough = () => {
    setConfig((prev) => ({
      ...prev,
      motor_kp: [1.0, 1.0, 1.0, 1.0],
      motor_ki: [0.0, 0.0, 0.0, 0.0],
      motor_kd: [0.0, 0.0, 0.0, 0.0],
    }));
  };

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

  const setAllKalmanDefault = () => {
    setConfig((prev) => ({
      ...prev,
      kalman_q: [0.5, 0.5, 0.5, 0.5],
      kalman_r: [0.04, 0.04, 0.04, 0.04],
    }));
  };

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
    <div className="border-2 border-charcoal bg-white rounded-[2px] shadow-[-4px_4px_0px_#383838] p-5 text-charcoal w-full space-y-6">
      {/* Header */}
      <div className="flex flex-wrap items-center justify-between pb-3 border-b-2 border-charcoal gap-2">
        <div className="flex items-center gap-2">
          <Settings2 className="h-5 w-5 text-charcoal" />
          <div>
            <h2 className="text-sm sm:text-base font-bold tracking-wider">
              CONFIG // PARAMETERS & PID
            </h2>
            <p className="text-[11px] text-graphite font-mono">
              SYNC // TOPIC: /config/cmd (STM32 & GAZEBO)
            </p>
          </div>
        </div>
      </div>

      <form onSubmit={handleSave} className="space-y-6 text-xs font-mono">
        {/* Section 1: Chassis Geometry & Kinematics */}
        <div className="border border-charcoal bg-chalk p-4 rounded-[2px] space-y-3 shadow-[-2px_2px_0px_#383838]">
          <div className="flex items-center justify-between border-b border-charcoal pb-2">
            <h3 className="font-bold uppercase tracking-wider text-xs flex items-center gap-1.5 text-charcoal">
              01 // KINEMATICS & LIMITS
            </h3>
            <span className="text-[10px] text-graphite">BASE GEOMETRY</span>
          </div>

          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-3.5">
            <div>
              <label className="block text-graphite font-bold mb-1">
                WHEEL RADIUS (m)
              </label>
              <input
                type="number"
                step="0.001"
                value={config.wheel_radius_m}
                onChange={(e) => handleChange("wheel_radius_m", e.target.value)}
                className="w-full border border-charcoal bg-white px-2.5 py-1.5 text-charcoal font-mono rounded-[2px] focus:bg-notebook focus:outline-none"
              />
            </div>
            <div>
              <label className="block text-graphite font-bold mb-1">
                WHEELBASE (m)
              </label>
              <input
                type="number"
                step="0.001"
                value={config.wheelbase_m}
                onChange={(e) => handleChange("wheelbase_m", e.target.value)}
                className="w-full border border-charcoal bg-white px-2.5 py-1.5 text-charcoal font-mono rounded-[2px] focus:bg-notebook focus:outline-none"
              />
            </div>
            <div>
              <label className="block text-graphite font-bold mb-1">
                TRACK WIDTH (m)
              </label>
              <input
                type="number"
                step="0.001"
                value={config.track_width_m}
                onChange={(e) => handleChange("track_width_m", e.target.value)}
                className="w-full border border-charcoal bg-white px-2.5 py-1.5 text-charcoal font-mono rounded-[2px] focus:bg-notebook focus:outline-none"
              />
            </div>
            <div>
              <label className="block text-graphite font-bold mb-1">
                MAX LINEAR (m/s)
              </label>
              <input
                type="number"
                step="0.02"
                value={config.max_linear_speed_mps}
                onChange={(e) => handleChange("max_linear_speed_mps", e.target.value)}
                className="w-full border border-charcoal bg-white px-2.5 py-1.5 text-charcoal font-mono rounded-[2px] focus:bg-notebook focus:outline-none"
              />
            </div>
            <div>
              <label className="block text-graphite font-bold mb-1">
                MAX ANGULAR (rad/s)
              </label>
              <input
                type="number"
                step="0.1"
                value={config.max_angular_speed_rad_s}
                onChange={(e) => handleChange("max_angular_speed_rad_s", e.target.value)}
                className="w-full border border-charcoal bg-white px-2.5 py-1.5 text-charcoal font-mono rounded-[2px] focus:bg-notebook focus:outline-none"
              />
            </div>
            <div>
              <label className="block text-graphite font-bold mb-1">
                WATCHDOG TIMEOUT (s)
              </label>
              <input
                type="number"
                step="0.05"
                value={config.command_timeout_sec}
                onChange={(e) => handleChange("command_timeout_sec", e.target.value)}
                className="w-full border border-charcoal bg-white px-2.5 py-1.5 text-charcoal font-mono rounded-[2px] focus:bg-notebook focus:outline-none"
              />
            </div>
          </div>
        </div>

        {/* Section 2: 4 Motor PID Controllers */}
        <div className="border border-charcoal bg-chalk p-4 rounded-[2px] space-y-4 shadow-[-2px_2px_0px_#383838]">
          <div className="flex flex-wrap items-center justify-between border-b border-charcoal pb-2 gap-2">
            <div>
              <h3 className="font-bold uppercase tracking-wider text-xs flex items-center gap-1.5 text-charcoal">
                <Zap className="h-4 w-4 text-duck-orange" />
                02 // MOTOR PID (M1–M4)
              </h3>
              <p className="text-[10px] text-graphite mt-0.5">
                DEFAULT: PASSTHROUGH (Kp=1.0, Ki=0.0, Kd=0.0)
              </p>
            </div>
            {/* Action Buttons */}
            <div className="flex items-center gap-2">
              <button
                type="button"
                onClick={setAllPidPassthrough}
                className="flex items-center gap-1 px-2.5 py-1 border border-charcoal bg-white hover:bg-ice text-charcoal font-bold text-[11px] shadow-[-2px_2px_0px_#383838] transition"
              >
                <RefreshCw className="h-3 w-3" />
                RESET PASSTHROUGH
              </button>
              <button
                type="button"
                onClick={copyMotor1PidToAll}
                className="flex items-center gap-1 px-2.5 py-1 border border-charcoal bg-sky hover:bg-sky-hover text-charcoal font-bold text-[11px] shadow-[-2px_2px_0px_#383838] transition"
              >
                <Copy className="h-3 w-3" />
                COPY M1 TO ALL
              </button>
            </div>
          </div>

          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-3.5">
            {MOTOR_LABELS.map((m) => {
              const isPassthrough =
                Math.abs(kpArray[m.id] - 1.0) < 1e-4 &&
                Math.abs(kiArray[m.id]) < 1e-4 &&
                Math.abs(kdArray[m.id]) < 1e-4;

              return (
                <div
                  key={m.id}
                  className="p-3 bg-white border border-charcoal shadow-[-2px_2px_0px_#383838] space-y-2"
                >
                  <div className="flex items-center justify-between pb-1 border-b border-charcoal">
                    <span className="font-bold text-xs">{m.code} ({m.loc})</span>
                    <span
                      className={`px-1.5 py-0.5 text-[9px] font-bold border border-charcoal ${
                        isPassthrough
                          ? "bg-canary text-charcoal"
                          : "bg-sketch-mint text-charcoal"
                      }`}
                    >
                      {isPassthrough ? "PASSTHROUGH" : "CLOSED-LOOP"}
                    </span>
                  </div>

                  <div className="space-y-1.5 font-mono text-[11px]">
                    <div>
                      <label className="block text-graphite text-[10px] mb-0.5 font-bold">
                        Kp:
                      </label>
                      <input
                        type="number"
                        step="0.01"
                        value={kpArray[m.id]}
                        onChange={(e) => handleMotorPidChange(m.id, "kp", e.target.value)}
                        className="w-full border border-charcoal bg-chalk px-2 py-1 text-charcoal focus:bg-notebook focus:outline-none"
                      />
                    </div>
                    <div>
                      <label className="block text-graphite text-[10px] mb-0.5 font-bold">
                        Ki:
                      </label>
                      <input
                        type="number"
                        step="0.01"
                        value={kiArray[m.id]}
                        onChange={(e) => handleMotorPidChange(m.id, "ki", e.target.value)}
                        className="w-full border border-charcoal bg-chalk px-2 py-1 text-charcoal focus:bg-notebook focus:outline-none"
                      />
                    </div>
                    <div>
                      <label className="block text-graphite text-[10px] mb-0.5 font-bold">
                        Kd:
                      </label>
                      <input
                        type="number"
                        step="0.0001"
                        value={kdArray[m.id]}
                        onChange={(e) => handleMotorPidChange(m.id, "kd", e.target.value)}
                        className="w-full border border-charcoal bg-chalk px-2 py-1 text-charcoal focus:bg-notebook focus:outline-none"
                      />
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        </div>

        {/* Section 3: 4 Kalman Filters */}
        <div className="border border-charcoal bg-chalk p-4 rounded-[2px] space-y-4 shadow-[-2px_2px_0px_#383838]">
          <div className="flex flex-wrap items-center justify-between border-b border-charcoal pb-2 gap-2">
            <div>
              <h3 className="font-bold uppercase tracking-wider text-xs flex items-center gap-1.5 text-charcoal">
                <ShieldCheck className="h-4 w-4 text-sky" />
                03 // WHEEL KALMAN (W1–W4)
              </h3>
              <p className="text-[10px] text-graphite mt-0.5">
                DEFAULT: Q = 0.50, R = 0.04
              </p>
            </div>
            {/* Quick Actions */}
            <div className="flex items-center gap-2">
              <button
                type="button"
                onClick={setAllKalmanDefault}
                className="flex items-center gap-1 px-2.5 py-1 border border-charcoal bg-white hover:bg-ice text-charcoal font-bold text-[11px] shadow-[-2px_2px_0px_#383838] transition"
              >
                <RefreshCw className="h-3 w-3" />
                RESET DEFAULTS
              </button>
              <button
                type="button"
                onClick={copyWheel1KalmanToAll}
                className="flex items-center gap-1 px-2.5 py-1 border border-charcoal bg-sky hover:bg-sky-hover text-charcoal font-bold text-[11px] shadow-[-2px_2px_0px_#383838] transition"
              >
                <Copy className="h-3 w-3" />
                COPY W1 TO ALL
              </button>
            </div>
          </div>

          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-3.5">
            {MOTOR_LABELS.map((m) => (
              <div
                key={m.id}
                className="p-3 bg-white border border-charcoal shadow-[-2px_2px_0px_#383838] space-y-2"
              >
                <div className="flex items-center justify-between pb-1 border-b border-charcoal">
                  <span className="font-bold text-xs">W{m.id + 1} ({m.loc})</span>
                  <span className="px-1.5 py-0.5 text-[9px] font-bold border border-charcoal bg-chalk text-charcoal">
                    SCALAR
                  </span>
                </div>

                <div className="space-y-1.5 font-mono text-[11px]">
                  <div>
                    <label className="block text-graphite text-[10px] mb-0.5 font-bold">
                      Q (PROCESS):
                    </label>
                    <input
                      type="number"
                      step="0.01"
                      value={qArray[m.id]}
                      onChange={(e) => handleKalmanChange(m.id, "q", e.target.value)}
                      className="w-full border border-charcoal bg-chalk px-2 py-1 text-charcoal focus:bg-notebook focus:outline-none"
                    />
                  </div>
                  <div>
                    <label className="block text-graphite text-[10px] mb-0.5 font-bold">
                      R (MEASUREMENT):
                    </label>
                    <input
                      type="number"
                      step="0.001"
                      value={rArray[m.id]}
                      onChange={(e) => handleKalmanChange(m.id, "r", e.target.value)}
                      className="w-full border border-charcoal bg-chalk px-2 py-1 text-charcoal focus:bg-notebook focus:outline-none"
                    />
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Footer Actions */}
        <div className="flex items-center justify-end gap-3 pt-2">
          {savedSuccess && (
            <span className="flex items-center gap-1 font-bold text-charcoal bg-canary border border-charcoal px-2.5 py-1 text-xs shadow-[-2px_2px_0px_#383838]">
              <Check className="h-3.5 w-3.5" /> SAVED & SYNCED (/config/cmd)
            </span>
          )}
          <button
            type="submit"
            disabled={saving}
            className="flex items-center gap-2 border-2 border-charcoal bg-sky hover:bg-sky-hover px-6 py-2 font-bold text-charcoal shadow-[-4px_4px_0px_#383838] transition active:translate-x-[2px] active:translate-y-[-2px] active:shadow-none disabled:opacity-50 text-xs tracking-wider"
          >
            <Save className="h-4 w-4" />
            <span>{saving ? "SAVING..." : "SAVE CONFIG"}</span>
          </button>
        </div>
      </form>
    </div>
  );
}
