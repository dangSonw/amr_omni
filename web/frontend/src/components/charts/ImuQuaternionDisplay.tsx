import React from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import { ImuTelemetry, DebugTelemetry } from '@/types/robot';

interface ImuQuaternionDisplayProps {
  imu?: ImuTelemetry;
  debug?: DebugTelemetry;
  data: Array<{ time: number; [key: string]: any }>;
}

export function ImuQuaternionDisplay({ imu, debug, data }: ImuQuaternionDisplayProps) {
  const qx = imu?.qx ?? debug?.imu_quaternion_xyzw?.[0] ?? 0;
  const qy = imu?.qy ?? debug?.imu_quaternion_xyzw?.[1] ?? 0;
  const qz = imu?.qz ?? debug?.imu_quaternion_xyzw?.[2] ?? 0;
  const qw = imu?.qw ?? debug?.imu_quaternion_xyzw?.[3] ?? 1;

  return (
    <div className="bg-white p-5 rounded-xl border border-slate-200 shadow-sm">
      <h2 className="text-sm font-bold text-slate-900 mb-3">IMU Quaternion (Orientation)</h2>
      <div className="grid grid-cols-4 gap-2 mb-4">
        <div className="bg-slate-50 p-2.5 rounded-lg border border-slate-100 text-center">
          <div className="text-[11px] text-slate-500 font-bold">QX</div>
          <div className="font-mono text-xs font-semibold text-slate-900 mt-0.5">{qx.toFixed(4)}</div>
        </div>
        <div className="bg-slate-50 p-2.5 rounded-lg border border-slate-100 text-center">
          <div className="text-[11px] text-slate-500 font-bold">QY</div>
          <div className="font-mono text-xs font-semibold text-slate-900 mt-0.5">{qy.toFixed(4)}</div>
        </div>
        <div className="bg-slate-50 p-2.5 rounded-lg border border-slate-100 text-center">
          <div className="text-[11px] text-slate-500 font-bold">QZ</div>
          <div className="font-mono text-xs font-semibold text-slate-900 mt-0.5">{qz.toFixed(4)}</div>
        </div>
        <div className="bg-slate-50 p-2.5 rounded-lg border border-slate-100 text-center">
          <div className="text-[11px] text-slate-500 font-bold">QW</div>
          <div className="font-mono text-xs font-semibold text-slate-900 mt-0.5">{qw.toFixed(4)}</div>
        </div>
      </div>
      <div className="h-48">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data}>
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis dataKey="time" type="number" domain={['dataMin', 'dataMax']} hide />
            <YAxis domain={[-1.1, 1.1]} />
            <Tooltip />
            <Legend />
            <Line type="monotone" dataKey="qx" stroke="#ef4444" dot={false} isAnimationActive={false} />
            <Line type="monotone" dataKey="qy" stroke="#22c55e" dot={false} isAnimationActive={false} />
            <Line type="monotone" dataKey="qz" stroke="#3b82f6" dot={false} isAnimationActive={false} />
            <Line type="monotone" dataKey="qw" stroke="#eab308" dot={false} isAnimationActive={false} />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
