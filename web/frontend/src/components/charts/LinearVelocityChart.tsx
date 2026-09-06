import React from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';

interface LinearVelocityChartProps {
  data: Array<{ time: number; [key: string]: any }>;
}

export function LinearVelocityChart({ data }: LinearVelocityChartProps) {
  return (
    <div className="bg-white p-4 rounded border border-slate-200">
      <h2 className="text-lg font-bold mb-4">Linear Velocity (m/s)</h2>
      <div className="h-64">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data}>
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis dataKey="time" type="number" domain={['dataMin', 'dataMax']} hide />
            <YAxis />
            <Tooltip />
            <Legend />
            <Line type="monotone" dataKey="cmd_vx" stroke="#94a3b8" dot={false} isAnimationActive={false} name="Cmd Vx" />
            <Line type="monotone" dataKey="actual_vx" stroke="#3b82f6" dot={false} isAnimationActive={false} name="Act Vx" />
            <Line type="monotone" dataKey="cmd_vy" stroke="#cbd5e1" dot={false} isAnimationActive={false} name="Cmd Vy" />
            <Line type="monotone" dataKey="actual_vy" stroke="#22c55e" dot={false} isAnimationActive={false} name="Act Vy" />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
