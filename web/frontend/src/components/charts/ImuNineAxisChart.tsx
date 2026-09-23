import React, { useState } from "react";
import {
  CartesianGrid,
  Legend,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from "recharts";
import { DebugTelemetry, ImuTelemetry } from "@/types/robot";
import { Compass, Gauge, Move3d } from "lucide-react";

interface ImuNineAxisChartProps {
  imu?: ImuTelemetry;
  debug?: DebugTelemetry;
  data: Array<{ time: number; [key: string]: any }>;
}

export function ImuNineAxisChart({ imu, debug, data }: ImuNineAxisChartProps) {
  const [activeSubTab, setActiveSubTab] = useState<"all" | "accel" | "gyro" | "mag">("all");

  const latest = data.length > 0 ? data[data.length - 1] : null;

  // Accel (m/s^2)
  const ax = debug?.imu_accel_xyz?.[0] ?? imu?.accel_x ?? latest?.accel_x ?? 0;
  const ay = debug?.imu_accel_xyz?.[1] ?? imu?.accel_y ?? latest?.accel_y ?? 0;
  const az = debug?.imu_accel_xyz?.[2] ?? imu?.accel_z ?? latest?.accel_z ?? 9.81;

  // Gyro (rad/s)
  const gx = debug?.imu_gyro_xyz?.[0] ?? imu?.gyro_x ?? latest?.gyro_x ?? 0;
  const gy = debug?.imu_gyro_xyz?.[1] ?? imu?.gyro_y ?? latest?.gyro_y ?? 0;
  const gz = debug?.imu_gyro_xyz?.[2] ?? imu?.gyro_z ?? latest?.gyro_z ?? 0;

  // Mag (uT)
  const mx = debug?.imu_mag_xyz?.[0] ?? latest?.mag_x ?? 0;
  const my = debug?.imu_mag_xyz?.[1] ?? latest?.mag_y ?? 20.0;
  const mz = debug?.imu_mag_xyz?.[2] ?? latest?.mag_z ?? -45.0;

  return (
    <div className="bg-white p-5 rounded-xl border border-slate-200 shadow-sm col-span-full space-y-4">
      {/* Header */}
      <div className="flex flex-wrap items-center justify-between pb-3 border-b border-slate-100 gap-2">
        <div className="flex items-center gap-2">
          <div className="p-1.5 rounded-lg bg-indigo-50 text-indigo-600 border border-indigo-100">
            <Move3d className="h-4 w-4" />
          </div>
          <div>
            <h2 className="text-sm font-bold text-slate-900">
              Dữ Liệu Cảm Biến IMU 9 Trục Độc Lập (9-Axis Sensor Telemetry)
            </h2>
            <p className="text-[11px] text-slate-500">
              Thu nhận trực tiếp từ cảm biến BNO080 qua STM32 (<code className="text-blue-600 bg-blue-50 px-1 py-0.5 rounded">debug/data</code>): Gia tốc tuyến tính 3 trục, Gia tốc quay 3 trục và Từ trường 3 trục
            </p>
          </div>
        </div>

        {/* Sub-tabs */}
        <div className="flex items-center gap-1.5 bg-slate-100 p-1 rounded-lg text-xs">
          <button
            type="button"
            onClick={() => setActiveSubTab("all")}
            className={`px-2.5 py-1 rounded-md font-medium transition ${
              activeSubTab === "all" ? "bg-white text-slate-900 shadow-xs" : "text-slate-600 hover:text-slate-900"
            }`}
          >
            Tất Cả 9 Trục
          </button>
          <button
            type="button"
            onClick={() => setActiveSubTab("accel")}
            className={`px-2.5 py-1 rounded-md font-medium transition ${
              activeSubTab === "accel" ? "bg-white text-slate-900 shadow-xs" : "text-slate-600 hover:text-slate-900"
            }`}
          >
            Gia Tốc Tuyến Tính
          </button>
          <button
            type="button"
            onClick={() => setActiveSubTab("gyro")}
            className={`px-2.5 py-1 rounded-md font-medium transition ${
              activeSubTab === "gyro" ? "bg-white text-slate-900 shadow-xs" : "text-slate-600 hover:text-slate-900"
            }`}
          >
            Vận Tốc Góc Gyro
          </button>
          <button
            type="button"
            onClick={() => setActiveSubTab("mag")}
            className={`px-2.5 py-1 rounded-md font-medium transition ${
              activeSubTab === "mag" ? "bg-white text-slate-900 shadow-xs" : "text-slate-600 hover:text-slate-900"
            }`}
          >
            Từ Trường Mag
          </button>
        </div>
      </div>

      {/* 3 Sub-Panels */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
        {/* Panel 1: Linear Acceleration (ax, ay, az) */}
        {(activeSubTab === "all" || activeSubTab === "accel") && (
          <div className="bg-slate-50 p-3.5 rounded-xl border border-slate-200 flex flex-col space-y-2.5">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-1.5">
                <Gauge className="h-3.5 w-3.5 text-blue-600" />
                <span className="font-bold text-xs text-slate-800">Gia Tốc Tuyến Tính (Linear Accel)</span>
              </div>
              <span className="text-[10px] text-slate-500 font-mono">m/s²</span>
            </div>

            {/* Badges */}
            <div className="grid grid-cols-3 gap-1.5 text-center font-mono">
              <div className="bg-white p-1.5 rounded border border-slate-200">
                <span className="text-[10px] text-rose-500 font-bold block">AX</span>
                <span className="text-xs font-semibold text-slate-900">{ax.toFixed(2)}</span>
              </div>
              <div className="bg-white p-1.5 rounded border border-slate-200">
                <span className="text-[10px] text-emerald-500 font-bold block">AY</span>
                <span className="text-xs font-semibold text-slate-900">{ay.toFixed(2)}</span>
              </div>
              <div className="bg-white p-1.5 rounded border border-slate-200">
                <span className="text-[10px] text-blue-500 font-bold block">AZ</span>
                <span className="text-xs font-semibold text-slate-900">{az.toFixed(2)}</span>
              </div>
            </div>

            {/* Chart */}
            <div className="h-44 w-full">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={data}>
                  <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
                  <XAxis dataKey="time" type="number" domain={["dataMin", "dataMax"]} hide />
                  <YAxis domain={["auto", "auto"]} stroke="#94a3b8" fontSize={10} width={30} />
                  <Tooltip
                    contentStyle={{ backgroundColor: "#1e293b", borderColor: "#334155", color: "#f8fafc", fontSize: 11 }}
                    formatter={(val: any, name: any) => [`${Number(val).toFixed(2)} m/s²`, name]}
                  />
                  <Legend wrapperStyle={{ fontSize: 10, paddingTop: 2 }} />
                  <Line type="monotone" dataKey="accel_x" stroke="#ef4444" dot={false} isAnimationActive={false} name="AX" />
                  <Line type="monotone" dataKey="accel_y" stroke="#10b981" dot={false} isAnimationActive={false} name="AY" />
                  <Line type="monotone" dataKey="accel_z" stroke="#3b82f6" dot={false} isAnimationActive={false} name="AZ" />
                </LineChart>
              </ResponsiveContainer>
            </div>
          </div>
        )}

        {/* Panel 2: Gyroscope (gx, gy, gz) */}
        {(activeSubTab === "all" || activeSubTab === "gyro") && (
          <div className="bg-slate-50 p-3.5 rounded-xl border border-slate-200 flex flex-col space-y-2.5">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-1.5">
                <Move3d className="h-3.5 w-3.5 text-purple-600" />
                <span className="font-bold text-xs text-slate-800">Gia Tốc Góc Quay (Gyroscope)</span>
              </div>
              <span className="text-[10px] text-slate-500 font-mono">rad/s</span>
            </div>

            {/* Badges */}
            <div className="grid grid-cols-3 gap-1.5 text-center font-mono">
              <div className="bg-white p-1.5 rounded border border-slate-200">
                <span className="text-[10px] text-rose-500 font-bold block">GX</span>
                <span className="text-xs font-semibold text-slate-900">{gx.toFixed(3)}</span>
              </div>
              <div className="bg-white p-1.5 rounded border border-slate-200">
                <span className="text-[10px] text-emerald-500 font-bold block">GY</span>
                <span className="text-xs font-semibold text-slate-900">{gy.toFixed(3)}</span>
              </div>
              <div className="bg-white p-1.5 rounded border border-slate-200">
                <span className="text-[10px] text-purple-500 font-bold block">GZ</span>
                <span className="text-xs font-semibold text-slate-900">{gz.toFixed(3)}</span>
              </div>
            </div>

            {/* Chart */}
            <div className="h-44 w-full">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={data}>
                  <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
                  <XAxis dataKey="time" type="number" domain={["dataMin", "dataMax"]} hide />
                  <YAxis domain={["auto", "auto"]} stroke="#94a3b8" fontSize={10} width={30} />
                  <Tooltip
                    contentStyle={{ backgroundColor: "#1e293b", borderColor: "#334155", color: "#f8fafc", fontSize: 11 }}
                    formatter={(val: any, name: any) => [`${Number(val).toFixed(3)} rad/s`, name]}
                  />
                  <Legend wrapperStyle={{ fontSize: 10, paddingTop: 2 }} />
                  <Line type="monotone" dataKey="gyro_x" stroke="#f43f5e" dot={false} isAnimationActive={false} name="GX" />
                  <Line type="monotone" dataKey="gyro_y" stroke="#10b981" dot={false} isAnimationActive={false} name="GY" />
                  <Line type="monotone" dataKey="gyro_z" stroke="#8b5cf6" dot={false} isAnimationActive={false} name="GZ" />
                </LineChart>
              </ResponsiveContainer>
            </div>
          </div>
        )}

        {/* Panel 3: Magnetometer (mx, my, mz) */}
        {(activeSubTab === "all" || activeSubTab === "mag") && (
          <div className="bg-slate-50 p-3.5 rounded-xl border border-slate-200 flex flex-col space-y-2.5">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-1.5">
                <Compass className="h-3.5 w-3.5 text-amber-600" />
                <span className="font-bold text-xs text-slate-800">Từ Trường 3 Trục (Magnetometer)</span>
              </div>
              <span className="text-[10px] text-slate-500 font-mono">µT</span>
            </div>

            {/* Badges */}
            <div className="grid grid-cols-3 gap-1.5 text-center font-mono">
              <div className="bg-white p-1.5 rounded border border-slate-200">
                <span className="text-[10px] text-rose-500 font-bold block">MX</span>
                <span className="text-xs font-semibold text-slate-900">{mx.toFixed(1)}</span>
              </div>
              <div className="bg-white p-1.5 rounded border border-slate-200">
                <span className="text-[10px] text-emerald-500 font-bold block">MY</span>
                <span className="text-xs font-semibold text-slate-900">{my.toFixed(1)}</span>
              </div>
              <div className="bg-white p-1.5 rounded border border-slate-200">
                <span className="text-[10px] text-amber-500 font-bold block">MZ</span>
                <span className="text-xs font-semibold text-slate-900">{mz.toFixed(1)}</span>
              </div>
            </div>

            {/* Chart */}
            <div className="h-44 w-full">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={data}>
                  <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
                  <XAxis dataKey="time" type="number" domain={["dataMin", "dataMax"]} hide />
                  <YAxis domain={["auto", "auto"]} stroke="#94a3b8" fontSize={10} width={30} />
                  <Tooltip
                    contentStyle={{ backgroundColor: "#1e293b", borderColor: "#334155", color: "#f8fafc", fontSize: 11 }}
                    formatter={(val: any, name: any) => [`${Number(val).toFixed(1)} µT`, name]}
                  />
                  <Legend wrapperStyle={{ fontSize: 10, paddingTop: 2 }} />
                  <Line type="monotone" dataKey="mag_x" stroke="#ef4444" dot={false} isAnimationActive={false} name="MX" />
                  <Line type="monotone" dataKey="mag_y" stroke="#10b981" dot={false} isAnimationActive={false} name="MY" />
                  <Line type="monotone" dataKey="mag_z" stroke="#f59e0b" dot={false} isAnimationActive={false} name="MZ" />
                </LineChart>
              </ResponsiveContainer>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

