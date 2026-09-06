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
    <div className="bg-white p-4 rounded border border-slate-200">
      <h2 className="text-lg font-bold mb-4">IMU Quaternion</h2>
      <div className="grid grid-cols-4 gap-2 mb-4">
        <div className="bg-slate-50 p-2 rounded border border-slate-200 text-center">
          <div className="text-xs text-slate-500 font-bold">X</div>
          <div className="font-mono text-sm">{qx.toFixed(4)}</div>
        </div>
        <div className="bg-slate-50 p-2 rounded border border-slate-200 text-center">
          <div className="text-xs text-slate-500 font-bold">Y</div>
          <div className="font-mono text-sm">{qy.toFixed(4)}</div>
        </div>
        <div className="bg-slate-50 p-2 rounded border border-slate-200 text-center">
          <div className="text-xs text-slate-500 font-bold">Z</div>
          <div className="font-mono text-sm">{qz.toFixed(4)}</div>
        </div>
        <div className="bg-slate-50 p-2 rounded border border-slate-200 text-center">
          <div className="text-xs text-slate-500 font-bold">W</div>
          <div className="font-mono text-sm">{qw.toFixed(4)}</div>
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
