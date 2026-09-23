import React from "react";
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
import { DebugTelemetry } from "@/types/robot";
import { Gauge, Zap } from "lucide-react";

interface MotorSpeedChartProps {
  data: Array<{ time: number; [key: string]: any }>;
  debug?: DebugTelemetry;
}

const WHEEL_INFO = [
  { id: 0, code: "W1", name: "Trước Phải (FR)", loc: "Front-Right" },
  { id: 1, code: "W2", name: "Trước Trái (FL)", loc: "Front-Left" },
  { id: 2, code: "W3", name: "Sau Trái (RL)", loc: "Rear-Left" },
  { id: 3, code: "W4", name: "Sau Phải (RR)", loc: "Rear-Right" },
];

export function MotorSpeedChart({ data, debug }: MotorSpeedChartProps) {
  const latestData = data.length > 0 ? data[data.length - 1] : null;

  const renderWheelChart = (idx: number) => {
    const rawVal = debug?.raw_wheel_speed_rad_s?.[idx] ?? latestData?.[`m${idx}_raw`] ?? 0;
    const filtVal = debug?.filtered_wheel_speed_rad_s?.[idx] ?? latestData?.[`m${idx}_filtered`] ?? 0;
    const targetVal = debug?.target_wheel_speed_rad_s?.[idx] ?? latestData?.[`m${idx}_target`] ?? 0;
    const diff = Math.abs(rawVal - filtVal);

    const info = WHEEL_INFO[idx];

    return (
      <div key={idx} className="bg-slate-50 p-3.5 rounded-xl border border-slate-200 flex flex-col space-y-2.5">
        {/* Wheel Header & Real-time Badges */}
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-1.5">
            <span className="font-bold text-slate-800 text-xs">{info.code}: {info.name}</span>
            <span className="text-[10px] text-slate-400 font-mono">({info.loc})</span>
          </div>
          <div className="flex items-center gap-1.5 text-[10px] font-mono">
            <span className="bg-slate-200/80 px-1.5 py-0.5 rounded text-slate-700 font-medium" title="Vận tốc thô trước Kalman">
              Thô: {rawVal.toFixed(2)}
            </span>
            <span className="bg-blue-100 text-blue-800 px-1.5 py-0.5 rounded font-bold" title="Vận tốc sau lọc Kalman">
              Lọc: {filtVal.toFixed(2)}
            </span>
            <span className="bg-emerald-100 text-emerald-800 px-1.5 py-0.5 rounded font-bold" title="Vận tốc mục tiêu">
              Mục tiêu: {targetVal.toFixed(2)}
            </span>
            <span className="text-slate-400 text-[9px]">rad/s</span>
          </div>
        </div>

        {/* Chart */}
        <div className="h-40 w-full">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
              <XAxis dataKey="time" type="number" domain={["dataMin", "dataMax"]} hide />
              <YAxis domain={["auto", "auto"]} stroke="#94a3b8" fontSize={10} width={35} />
              <Tooltip
                contentStyle={{ backgroundColor: "#1e293b", borderColor: "#334155", color: "#f8fafc", fontSize: 11 }}
                formatter={(val: any, name: any) => [`${Number(val).toFixed(2)} rad/s`, name]}
              />
              <Legend wrapperStyle={{ fontSize: 11, paddingTop: 4 }} />
              <Line
                type="monotone"
                dataKey={`m${idx}_raw`}
                stroke="#94a3b8"
                strokeWidth={1.5}
                strokeDasharray="3 3"
                dot={false}
                isAnimationActive={false}
                name="Trước Kalman (Thô)"
              />
              <Line
                type="monotone"
                dataKey={`m${idx}_filtered`}
                stroke="#0284c7"
                strokeWidth={2}
                dot={false}
                isAnimationActive={false}
                name="Sau Kalman (Lọc)"
              />
              <Line
                type="monotone"
                dataKey={`m${idx}_target`}
                stroke="#10b981"
                strokeWidth={1.5}
                dot={false}
                isAnimationActive={false}
                name="Mục Tiêu (Target)"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>
    );
  };

  return (
    <div className="bg-white p-5 rounded-xl border border-slate-200 shadow-sm col-span-full space-y-3">
      <div className="flex flex-wrap items-center justify-between gap-2 pb-2 border-b border-slate-100">
        <div className="flex items-center gap-2">
          <div className="p-1.5 rounded-lg bg-sky-50 text-sky-600 border border-sky-100">
            <Gauge className="h-4 w-4" />
          </div>
          <div>
            <h2 className="text-sm font-bold text-slate-900">
              Vận Tốc 4 Bánh Xe Trước & Sau Lọc Kalman (Pre vs Post Kalman Wheel Speeds)
            </h2>
            <p className="text-[11px] text-slate-500">
              Được truyền trực tiếp từ STM32 qua topic <code className="text-blue-600 bg-blue-50 px-1 py-0.5 rounded">debug/data</code> với độ chính xác cao 50Hz
            </p>
          </div>
        </div>
        <div className="text-[11px] text-slate-500 flex items-center gap-3">
          <span className="flex items-center gap-1"><span className="w-3 h-0.5 bg-slate-400 inline-block border-b border-dashed"></span> Thô trước Kalman</span>
          <span className="flex items-center gap-1"><span className="w-3 h-0.5 bg-sky-600 inline-block"></span> Đã lọc Kalman</span>
          <span className="flex items-center gap-1"><span className="w-3 h-0.5 bg-emerald-500 inline-block"></span> Vận tốc mục tiêu</span>
        </div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {[0, 1, 2, 3].map(renderWheelChart)}
      </div>
    </div>
  );
}
