"use client";

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

  const tooltipStyle = {
    backgroundColor: "#ffffff",
    borderColor: "#383838",
    borderWidth: 1.5,
    borderRadius: 2,
    boxShadow: "-2px 2px 0px #383838",
    color: "#383838",
    fontSize: 10,
    fontFamily: "monospace",
  };

  return (
    <div className="border-2 border-charcoal bg-white rounded-[2px] shadow-[-4px_4px_0px_#383838] p-5 col-span-full space-y-4 text-charcoal">
      {/* Header & Sub-tabs */}
      <div className="flex flex-wrap items-center justify-between pb-3 border-b-2 border-charcoal gap-2 font-mono">
        <div className="flex items-center gap-2">
          <Move3d className="h-4 w-4 text-charcoal" />
          <div>
            <h2 className="text-sm sm:text-base font-bold tracking-wider">
              IMU // 9-AXIS (BNO080)
            </h2>
            <p className="text-[11px] text-graphite">
              SOURCE: /debug/data @ 50HZ (ACCEL, GYRO, MAG)
            </p>
          </div>
        </div>

        {/* Sub-tabs */}
        <div className="flex items-center gap-1 bg-chalk p-1 border border-charcoal text-xs">
          <button
            type="button"
            onClick={() => setActiveSubTab("all")}
            className={`px-2 py-0.5 font-bold transition ${
              activeSubTab === "all"
                ? "bg-sky text-charcoal border border-charcoal shadow-[-1px_1px_0px_#383838]"
                : "text-graphite hover:text-charcoal"
            }`}
          >
            ALL
          </button>
          <button
            type="button"
            onClick={() => setActiveSubTab("accel")}
            className={`px-2 py-0.5 font-bold transition ${
              activeSubTab === "accel"
                ? "bg-canary text-charcoal border border-charcoal shadow-[-1px_1px_0px_#383838]"
                : "text-graphite hover:text-charcoal"
            }`}
          >
            ACCEL
          </button>
          <button
            type="button"
            onClick={() => setActiveSubTab("gyro")}
            className={`px-2 py-0.5 font-bold transition ${
              activeSubTab === "gyro"
                ? "bg-sketch-mint text-charcoal border border-charcoal shadow-[-1px_1px_0px_#383838]"
                : "text-graphite hover:text-charcoal"
            }`}
          >
            GYRO
          </button>
          <button
            type="button"
            onClick={() => setActiveSubTab("mag")}
            className={`px-2 py-0.5 font-bold transition ${
              activeSubTab === "mag"
                ? "bg-sketch-coral text-charcoal border border-charcoal shadow-[-1px_1px_0px_#383838]"
                : "text-graphite hover:text-charcoal"
            }`}
          >
            MAG
          </button>
        </div>
      </div>

      {/* 3 Panels */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-3.5 font-mono">
        {/* Panel 1: Linear Acceleration */}
        {(activeSubTab === "all" || activeSubTab === "accel") && (
          <div className="border border-charcoal bg-chalk p-3 rounded-[2px] shadow-[-2px_2px_0px_#383838] flex flex-col space-y-2">
            <div className="flex items-center justify-between border-b border-charcoal pb-1">
              <div className="flex items-center gap-1 text-xs font-bold">
                <Gauge className="h-3.5 w-3.5" />
                <span>ACCEL (m/s²)</span>
              </div>
            </div>

            {/* Value boxes */}
            <div className="grid grid-cols-3 gap-1.5 text-center">
              <div className="bg-white border border-charcoal p-1">
                <span className="text-[9px] font-bold text-graphite block">AX</span>
                <span className="text-xs font-bold">{ax.toFixed(2)}</span>
              </div>
              <div className="bg-white border border-charcoal p-1">
                <span className="text-[9px] font-bold text-graphite block">AY</span>
                <span className="text-xs font-bold">{ay.toFixed(2)}</span>
              </div>
              <div className="bg-white border border-charcoal p-1">
                <span className="text-[9px] font-bold text-graphite block">AZ</span>
                <span className="text-xs font-bold">{az.toFixed(2)}</span>
              </div>
            </div>

            {/* Chart */}
            <div className="h-36 w-full">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={data}>
                  <CartesianGrid strokeDasharray="2 2" stroke="#d5cfc7" />
                  <XAxis dataKey="time" type="number" domain={["dataMin", "dataMax"]} hide />
                  <YAxis domain={["auto", "auto"]} stroke="#383838" fontSize={9} width={28} />
                  <Tooltip
                    contentStyle={tooltipStyle}
                    formatter={(val: any, name: any) => [`${Number(val).toFixed(2)} m/s²`, name]}
                  />
                  <Legend wrapperStyle={{ fontSize: 9, paddingTop: 2 }} />
                  <Line type="monotone" dataKey="accel_x" stroke="#f38e84" strokeWidth={1.5} dot={false} isAnimationActive={false} name="AX" />
                  <Line type="monotone" dataKey="accel_y" stroke="#38c1b0" strokeWidth={1.5} dot={false} isAnimationActive={false} name="AY" />
                  <Line type="monotone" dataKey="accel_z" stroke="#6fc2ff" strokeWidth={1.5} dot={false} isAnimationActive={false} name="AZ" />
                </LineChart>
              </ResponsiveContainer>
            </div>
          </div>
        )}

        {/* Panel 2: Gyroscope */}
        {(activeSubTab === "all" || activeSubTab === "gyro") && (
          <div className="border border-charcoal bg-chalk p-3 rounded-[2px] shadow-[-2px_2px_0px_#383838] flex flex-col space-y-2">
            <div className="flex items-center justify-between border-b border-charcoal pb-1">
              <div className="flex items-center gap-1 text-xs font-bold">
                <Move3d className="h-3.5 w-3.5" />
                <span>GYRO (rad/s)</span>
              </div>
            </div>

            {/* Value boxes */}
            <div className="grid grid-cols-3 gap-1.5 text-center">
              <div className="bg-white border border-charcoal p-1">
                <span className="text-[9px] font-bold text-graphite block">GX</span>
                <span className="text-xs font-bold">{gx.toFixed(3)}</span>
              </div>
              <div className="bg-white border border-charcoal p-1">
                <span className="text-[9px] font-bold text-graphite block">GY</span>
                <span className="text-xs font-bold">{gy.toFixed(3)}</span>
              </div>
              <div className="bg-white border border-charcoal p-1">
                <span className="text-[9px] font-bold text-graphite block">GZ</span>
                <span className="text-xs font-bold">{gz.toFixed(3)}</span>
              </div>
            </div>

            {/* Chart */}
            <div className="h-36 w-full">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={data}>
                  <CartesianGrid strokeDasharray="2 2" stroke="#d5cfc7" />
                  <XAxis dataKey="time" type="number" domain={["dataMin", "dataMax"]} hide />
                  <YAxis domain={["auto", "auto"]} stroke="#383838" fontSize={9} width={28} />
                  <Tooltip
                    contentStyle={tooltipStyle}
                    formatter={(val: any, name: any) => [`${Number(val).toFixed(3)} rad/s`, name]}
                  />
                  <Legend wrapperStyle={{ fontSize: 9, paddingTop: 2 }} />
                  <Line type="monotone" dataKey="gyro_x" stroke="#f38e84" strokeWidth={1.5} dot={false} isAnimationActive={false} name="GX" />
                  <Line type="monotone" dataKey="gyro_y" stroke="#38c1b0" strokeWidth={1.5} dot={false} isAnimationActive={false} name="GY" />
                  <Line type="monotone" dataKey="gyro_z" stroke="#b291de" strokeWidth={1.5} dot={false} isAnimationActive={false} name="GZ" />
                </LineChart>
              </ResponsiveContainer>
            </div>
          </div>
        )}

        {/* Panel 3: Magnetometer */}
        {(activeSubTab === "all" || activeSubTab === "mag") && (
          <div className="border border-charcoal bg-chalk p-3 rounded-[2px] shadow-[-2px_2px_0px_#383838] flex flex-col space-y-2">
            <div className="flex items-center justify-between border-b border-charcoal pb-1">
              <div className="flex items-center gap-1 text-xs font-bold">
                <Compass className="h-3.5 w-3.5" />
                <span>MAG (µT)</span>
              </div>
            </div>

            {/* Value boxes */}
            <div className="grid grid-cols-3 gap-1.5 text-center">
              <div className="bg-white border border-charcoal p-1">
                <span className="text-[9px] font-bold text-graphite block">MX</span>
                <span className="text-xs font-bold">{mx.toFixed(1)}</span>
              </div>
              <div className="bg-white border border-charcoal p-1">
                <span className="text-[9px] font-bold text-graphite block">MY</span>
                <span className="text-xs font-bold">{my.toFixed(1)}</span>
              </div>
              <div className="bg-white border border-charcoal p-1">
                <span className="text-[9px] font-bold text-graphite block">MZ</span>
                <span className="text-xs font-bold">{mz.toFixed(1)}</span>
              </div>
            </div>

            {/* Chart */}
            <div className="h-36 w-full">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={data}>
                  <CartesianGrid strokeDasharray="2 2" stroke="#d5cfc7" />
                  <XAxis dataKey="time" type="number" domain={["dataMin", "dataMax"]} hide />
                  <YAxis domain={["auto", "auto"]} stroke="#383838" fontSize={9} width={28} />
                  <Tooltip
                    contentStyle={tooltipStyle}
                    formatter={(val: any, name: any) => [`${Number(val).toFixed(1)} µT`, name]}
                  />
                  <Legend wrapperStyle={{ fontSize: 9, paddingTop: 2 }} />
                  <Line type="monotone" dataKey="mag_x" stroke="#f38e84" strokeWidth={1.5} dot={false} isAnimationActive={false} name="MX" />
                  <Line type="monotone" dataKey="mag_y" stroke="#38c1b0" strokeWidth={1.5} dot={false} isAnimationActive={false} name="MY" />
                  <Line type="monotone" dataKey="mag_z" stroke="#ff9538" strokeWidth={1.5} dot={false} isAnimationActive={false} name="MZ" />
                </LineChart>
              </ResponsiveContainer>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
