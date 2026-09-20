"use client";

import { ImuTelemetry, OdometryTelemetry } from "@/types/robot";
import { Compass, Navigation2, RotateCcw } from "lucide-react";

interface SensorCardsProps {
  imu: ImuTelemetry | undefined;
  odom: OdometryTelemetry | undefined;
  onResetOdom: () => void;
}

export function SensorCards({ imu, odom, onResetOdom }: SensorCardsProps) {
  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
      {/* IMU Card */}
      <div className="flex flex-col justify-between rounded-xl border border-slate-200 bg-white p-4 shadow-sm">
        <div className="flex items-center justify-between pb-2 border-b border-slate-100">
          <div className="flex items-center gap-2">
            <Compass className="h-5 w-5 text-blue-600" />
            <h3 className="text-sm font-bold text-slate-900">
              Cảm Biến Quán Tính (IMU 6-DoF)
            </h3>
          </div>
          <span className="text-[11px] font-mono text-slate-400">50 Hz</span>
        </div>

        {/* IMU Orientation & Angular Rates */}
        <div className="mt-3 grid grid-cols-3 gap-2 text-center text-xs">
          <div className="rounded-lg bg-slate-50 border border-slate-100 p-2.5">
            <span className="text-slate-500 font-mono text-[10px]">ROLL</span>
            <div className="mt-0.5 text-base font-bold text-slate-900 font-mono">
              {imu?.roll_deg?.toFixed(1) ?? 0}°
            </div>
          </div>
          <div className="rounded-lg bg-slate-50 border border-slate-100 p-2.5">
            <span className="text-slate-500 font-mono text-[10px]">PITCH</span>
            <div className="mt-0.5 text-base font-bold text-slate-900 font-mono">
              {imu?.pitch_deg?.toFixed(1) ?? 0}°
            </div>
          </div>
          <div className="rounded-lg bg-slate-50 border border-slate-100 p-2.5">
            <span className="text-slate-500 font-mono text-[10px]">YAW (HƯỚNG)</span>
            <div className="mt-0.5 text-base font-bold text-blue-600 font-mono">
              {imu?.yaw_deg?.toFixed(1) ?? 0}°
            </div>
          </div>
        </div>

        {/* IMU Accelerometer & Gyro detail */}
        <div className="mt-3 grid grid-cols-2 gap-2 text-xs font-mono pt-2 border-t border-slate-100">
          <div className="space-y-0.5 text-slate-700">
            <div className="text-[10px] text-slate-500 uppercase font-semibold">Gia Tốc Tuyến Tính (m/s²)</div>
            <div>X: <span className="font-semibold text-slate-900">{imu?.accel_x?.toFixed(2) ?? 0}</span></div>
            <div>Y: <span className="font-semibold text-slate-900">{imu?.accel_y?.toFixed(2) ?? 0}</span></div>
            <div>Z: <span className="font-semibold text-slate-900">{imu?.accel_z?.toFixed(2) ?? 9.81}</span></div>
          </div>
          <div className="space-y-0.5 text-slate-700">
            <div className="text-[10px] text-slate-500 uppercase font-semibold">Vận Tốc Góc (rad/s)</div>
            <div>Gyro X: <span className="font-semibold text-slate-900">{imu?.gyro_x?.toFixed(2) ?? 0}</span></div>
            <div>Gyro Y: <span className="font-semibold text-slate-900">{imu?.gyro_y?.toFixed(2) ?? 0}</span></div>
            <div>Gyro Z: <span className="font-semibold text-slate-900">{imu?.gyro_z?.toFixed(2) ?? 0}</span></div>
          </div>
        </div>
      </div>

      {/* Odometry Card */}
      <div className="flex flex-col justify-between rounded-xl border border-slate-200 bg-white p-4 shadow-sm">
        <div className="flex items-center justify-between pb-2 border-b border-slate-100">
          <div className="flex items-center gap-2">
            <Navigation2 className="h-5 w-5 text-blue-600" />
            <h3 className="text-sm font-bold text-slate-900">
              Định Vị Tọa Độ Odometry (odom frame)
            </h3>
          </div>
          <button
            onClick={onResetOdom}
            title="Reset tọa độ về (0, 0, 0)"
            className="flex items-center gap-1 rounded-lg border border-slate-200 bg-slate-50 px-2.5 py-1 text-[11px] font-semibold text-slate-700 hover:bg-slate-100 hover:text-rose-600 shadow-xs transition"
          >
            <RotateCcw className="h-3 w-3" />
            Reset Odom
          </button>
        </div>

        {/* Odometry Coordinates */}
        <div className="mt-3 grid grid-cols-3 gap-2 text-center text-xs">
          <div className="rounded-lg bg-slate-50 border border-slate-100 p-2.5">
            <span className="text-slate-500 font-mono text-[10px]">TỌA ĐỘ X</span>
            <div className="mt-0.5 text-base font-bold text-slate-900 font-mono">
              {odom?.x?.toFixed(2) ?? 0} m
            </div>
          </div>
          <div className="rounded-lg bg-slate-50 border border-slate-100 p-2.5">
            <span className="text-slate-500 font-mono text-[10px]">TỌA ĐỘ Y</span>
            <div className="mt-0.5 text-base font-bold text-slate-900 font-mono">
              {odom?.y?.toFixed(2) ?? 0} m
            </div>
          </div>
          <div className="rounded-lg bg-slate-50 border border-slate-100 p-2.5">
            <span className="text-slate-500 font-mono text-[10px]">GÓC THETA</span>
            <div className="mt-0.5 text-base font-bold text-blue-600 font-mono">
              {(((odom?.theta_rad ?? 0) * 180) / Math.PI).toFixed(1)}°
            </div>
          </div>
        </div>

        {/* Odometry Linear & Angular Velocities */}
        <div className="mt-3 grid grid-cols-3 gap-2 text-center text-xs font-mono pt-2 border-t border-slate-100">
          <div>
            <div className="text-[10px] text-slate-500 font-semibold">Vx (Tiến)</div>
            <div className="font-bold text-slate-900">{odom?.vx?.toFixed(2) ?? 0} m/s</div>
          </div>
          <div>
            <div className="text-[10px] text-slate-500 font-semibold">Vy (Ngang)</div>
            <div className="font-bold text-slate-900">{odom?.vy?.toFixed(2) ?? 0} m/s</div>
          </div>
          <div>
            <div className="text-[10px] text-slate-500 font-semibold">Wz (Góc)</div>
            <div className="font-bold text-slate-900">{odom?.wz?.toFixed(2) ?? 0} rad/s</div>
          </div>
        </div>
      </div>
    </div>
  );
}

