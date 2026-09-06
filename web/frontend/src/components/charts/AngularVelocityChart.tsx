import React from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';

interface AngularVelocityChartProps {
  data: Array<{ time: number; [key: string]: any }>;
}

export function AngularVelocityChart({ data }: AngularVelocityChartProps) {
  return (
    <div className="bg-white p-4 rounded border border-slate-200">
      <h2 className="text-lg font-bold mb-4">Angular Velocity (rad/s)</h2>
      <div className="h-64">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data}>
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis dataKey="time" type="number" domain={['dataMin', 'dataMax']} hide />
            <YAxis />
            <Tooltip />
            <Legend />
            <Line type="monotone" dataKey="cmd_wz" stroke="#94a3b8" dot={false} isAnimationActive={false} name="Command Wz" />
            <Line type="monotone" dataKey="actual_wz" stroke="#3b82f6" dot={false} isAnimationActive={false} name="Actual Wz" />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
