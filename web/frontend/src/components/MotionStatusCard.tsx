"use client";

import React from "react";
import { OdometryTelemetry } from "@/types/robot";
import { Compass, Navigation2, ArrowRight, ArrowUp, RotateCw, MapPin } from "lucide-react";

interface MotionStatusCardProps {
  odom: OdometryTelemetry | undefined;
}

export function MotionStatusCard({ odom }: MotionStatusCardProps) {
  const vx = odom?.vx ?? 0;
  const vy = odom?.vy ?? 0;
  const wz = odom?.wz ?? 0;
  const x = odom?.x ?? 0;
  const y = odom?.y ?? 0;
  const yawRad = odom?.theta_rad ?? 0;
  const yawDeg = ((yawRad * 180) / Math.PI) % 360;
  const normalizedYaw = (yawDeg + 360) % 360;
  const speed = Math.sqrt(vx * vx + vy * vy);

  return (
    <div className="rounded border border-slate-200 bg-white p-5">
      {/* Header */}
      <div className="flex items-center justify-between pb-3 border-b border-slate-100">
        <div className="flex items-center gap-2">
          <div className="flex h-8 w-8 items-center justify-center rounded-lg bg-brand-50 text-brand-600">
            <Navigation2 className="h-4 w-4" />
          </div>
          <div>
            <h3 className="text-sm font-bold text-slate-900">
              Kinematics & Orientation
            </h3>
            <p className="text-[11px] text-slate-500">
              Normalized /odom (4x Encoders + BNO08x IMU)
            </p>
          </div>
        </div>

        {/* Speed Badge */}
        <div className="text-right">
          <span className="text-xs text-slate-400 block">Current Speed</span>
          <span className="font-mono text-base font-bold text-brand-600">
            {speed.toFixed(2)} <span className="text-xs font-normal text-slate-500">m/s</span>
          </span>
        </div>
      </div>

      {/* Grid Content */}
      <div className="mt-4 grid grid-cols-2 sm:grid-cols-4 gap-3">
        {/* Vx - Forward/Backward */}
        <div className="rounded-sm border border-slate-100 bg-slate-50 p-3">
          <div className="flex items-center justify-between text-xs text-slate-500 mb-1">
            <span className="flex items-center gap-1 font-medium">
              <ArrowUp className="h-3.5 w-3.5 text-blue-500" /> Vx (Forward/Back)
            </span>
          </div>
          <div className="font-mono text-lg font-bold text-slate-900">
            {vx >= 0 ? `+${vx.toFixed(2)}` : vx.toFixed(2)}
            <span className="text-xs font-normal text-slate-400 ml-1">m/s</span>
          </div>
          <div className="text-[10px] text-slate-400 mt-0.5">Body X-axis linear</div>
        </div>

        {/* Vy - Strafe Left/Right */}
        <div className="rounded-sm border border-slate-100 bg-slate-50 p-3">
          <div className="flex items-center justify-between text-xs text-slate-500 mb-1">
            <span className="flex items-center gap-1 font-medium">
              <ArrowRight className="h-3.5 w-3.5 text-emerald-500" /> Vy (Strafe)
            </span>
          </div>
          <div className="font-mono text-lg font-bold text-slate-900">
            {vy >= 0 ? `+${vy.toFixed(2)}` : vy.toFixed(2)}
            <span className="text-xs font-normal text-slate-400 ml-1">m/s</span>
          </div>
          <div className="text-[10px] text-slate-400 mt-0.5">Body Y-axis linear</div>
        </div>

        {/* Wz - Angular Velocity */}
        <div className="rounded-sm border border-slate-100 bg-slate-50 p-3">
          <div className="flex items-center justify-between text-xs text-slate-500 mb-1">
            <span className="flex items-center gap-1 font-medium">
              <RotateCw className="h-3.5 w-3.5 text-purple-500" /> Wz (Angular)
            </span>
          </div>
          <div className="font-mono text-lg font-bold text-slate-900">
            {wz >= 0 ? `+${wz.toFixed(2)}` : wz.toFixed(2)}
            <span className="text-xs font-normal text-slate-400 ml-1">rad/s</span>
          </div>
          <div className="text-[10px] text-slate-400 mt-0.5">Z-axis rotational</div>
        </div>

        {/* Yaw Heading */}
        <div className="rounded-sm border border-slate-100 bg-slate-50 p-3 flex items-center justify-between">
          <div>
            <div className="flex items-center gap-1 text-xs text-slate-500 mb-1 font-medium">
              <Compass className="h-3.5 w-3.5 text-cyan-500" /> Heading (Yaw)
            </div>
            <div className="font-mono text-lg font-bold text-slate-900">
              {normalizedYaw.toFixed(1)}°
            </div>
            <div className="text-[10px] text-slate-400 mt-0.5">IMU Compass (0-360°)</div>
          </div>
          {/* Visual Compass Needle */}
          <div className="relative flex h-10 w-10 items-center justify-center rounded-full border border-slate-200 bg-white shadow-sm">
            <Navigation2
              className="h-5 w-5 text-cyan-500 transition-transform duration-200"
              style={{ transform: `rotate(${normalizedYaw}deg)` }}
            />
          </div>
        </div>
      </div>

      {/* Odometry Position Footer */}
      <div className="mt-3.5 pt-3 border-t border-slate-100 flex flex-wrap items-center justify-between text-xs text-slate-500">
        <div className="flex items-center gap-1.5">
          <MapPin className="h-3.5 w-3.5 text-slate-400" />
          <span>Odometry Pose (Filtered):</span>
        </div>
        <div className="flex items-center gap-4 font-mono font-medium text-slate-700">
          <span>X: <strong>{x.toFixed(3)}</strong> m</span>
          <span>Y: <strong>{y.toFixed(3)}</strong> m</span>
          <span>Theta: <strong>{yawRad.toFixed(3)}</strong> rad</span>
        </div>
      </div>
    </div>
  );
}
