import React from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import { DebugTelemetry } from '@/types/robot';

interface MotorSpeedChartProps {
  data: Array<{ time: number; [key: string]: any }>;
  debug?: DebugTelemetry;
}

export function MotorSpeedChart({ data }: MotorSpeedChartProps) {
  const renderMotorChart = (motorIdx: number) => (
    <div key={motorIdx} className="bg-slate-50 p-2 rounded border border-slate-200">
      <h3 className="text-sm font-semibold mb-2">Motor {motorIdx + 1}</h3>
      <div className="h-40">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data}>
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis dataKey="time" type="number" domain={['dataMin', 'dataMax']} hide />
            <YAxis domain={['auto', 'auto']} />
            <Tooltip />
            <Legend />
            <Line type="monotone" dataKey={`m${motorIdx}_raw`} stroke="#94a3b8" dot={false} isAnimationActive={false} name="Raw" />
            <Line type="monotone" dataKey={`m${motorIdx}_filtered`} stroke="#3b82f6" dot={false} isAnimationActive={false} name="Filtered" />
            <Line type="monotone" dataKey={`m${motorIdx}_target`} stroke="#22c55e" dot={false} isAnimationActive={false} name="Target" />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );

  return (
    <div className="bg-white p-4 rounded border border-slate-200 col-span-full">
      <h2 className="text-lg font-bold mb-4">Motor Speeds (rad/s)</h2>
      <div className="grid grid-cols-2 gap-4">
        {[0, 1, 2, 3].map(renderMotorChart)}
      </div>
    </div>
  );
}
