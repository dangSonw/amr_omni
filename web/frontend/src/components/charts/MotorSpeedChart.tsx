"use client";

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
import { Gauge } from "lucide-react";

interface MotorSpeedChartProps {
  data: Array<{ time: number; [key: string]: any }>;
  debug?: DebugTelemetry;
}

const WHEEL_INFO = [
  { id: 0, code: "W1", loc: "FR", name: "Front-Right" },
  { id: 1, code: "W2", loc: "FL", name: "Front-Left" },
  { id: 2, code: "W3", loc: "RL", name: "Rear-Left" },
  { id: 3, code: "W4", loc: "RR", name: "Rear-Right" },
];

export function MotorSpeedChart({ data, debug }: MotorSpeedChartProps) {
  const latestData = data.length > 0 ? data[data.length - 1] : null;

  const renderWheelChart = (idx: number) => {
    const rawVal = debug?.raw_wheel_speed_rad_s?.[idx] ?? latestData?.[`m${idx}_raw`] ?? 0;
    const filtVal = debug?.filtered_wheel_speed_rad_s?.[idx] ?? latestData?.[`m${idx}_filtered`] ?? 0;
    const targetVal = debug?.target_wheel_speed_rad_s?.[idx] ?? latestData?.[`m${idx}_target`] ?? 0;

    const info = WHEEL_INFO[idx];

    return (
      <div key={idx} className="border border-charcoal bg-chalk p-3 rounded-[2px] shadow-[-2px_2px_0px_#383838] space-y-2">
        {/* Wheel Header & Readouts */}
        <div className="flex items-center justify-between border-b border-charcoal pb-1.5 font-mono text-[11px]">
          <div className="flex items-center gap-1.5">
            <span className="font-bold text-charcoal">{info.code} ({info.loc})</span>
          </div>
          <div className="flex items-center gap-1.5 text-[10px]">
            <span className="border border-charcoal bg-white px-1 py-0.5 text-graphite" title="Raw speed">
              RAW: {rawVal.toFixed(2)}
            </span>
            <span className="border border-charcoal bg-sky px-1 py-0.5 font-bold text-charcoal" title="Filtered speed">
              FILT: {filtVal.toFixed(2)}
            </span>
            <span className="border border-charcoal bg-sketch-mint px-1 py-0.5 font-bold text-charcoal" title="Target speed">
              TGT: {targetVal.toFixed(2)}
            </span>
            <span className="text-graphite">rad/s</span>
          </div>
        </div>

        {/* Chart */}
        <div className="h-36 w-full">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="2 2" stroke="#d5cfc7" />
              <XAxis dataKey="time" type="number" domain={["dataMin", "dataMax"]} hide />
              <YAxis domain={["auto", "auto"]} stroke="#383838" fontSize={9} width={30} />
              <Tooltip
                contentStyle={{
                  backgroundColor: "#ffffff",
                  borderColor: "#383838",
                  borderWidth: 1.5,
                  borderRadius: 2,
                  boxShadow: "-2px 2px 0px #383838",
                  color: "#383838",
                  fontSize: 10,
                  fontFamily: "monospace",
                }}
                formatter={(val: any, name: any) => [`${Number(val).toFixed(2)} rad/s`, name]}
              />
              <Legend wrapperStyle={{ fontSize: 10, paddingTop: 2, fontFamily: "monospace" }} />
              <Line
                type="monotone"
                dataKey={`m${idx}_raw`}
                stroke="#a1a1a1"
                strokeWidth={1.5}
                strokeDasharray="3 3"
                dot={false}
                isAnimationActive={false}
                name="RAW"
              />
              <Line
                type="monotone"
                dataKey={`m${idx}_filtered`}
                stroke="#6fc2ff"
                strokeWidth={2}
                dot={false}
                isAnimationActive={false}
                name="FILT"
              />
              <Line
                type="monotone"
                dataKey={`m${idx}_target`}
                stroke="#38c1b0"
                strokeWidth={1.5}
                dot={false}
                isAnimationActive={false}
                name="TGT"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>
    );
  };

  return (
    <div className="border-2 border-charcoal bg-white rounded-[2px] shadow-[-4px_4px_0px_#383838] p-5 col-span-full space-y-3 text-charcoal">
      <div className="flex flex-wrap items-center justify-between gap-2 pb-2 border-b-2 border-charcoal">
        <div className="flex items-center gap-2">
          <Gauge className="h-4 w-4 text-charcoal" />
          <div>
            <h2 className="text-sm sm:text-base font-bold tracking-wider font-mono">
              WHEEL VELOCITY // PRE VS POST KALMAN
            </h2>
            <p className="text-[11px] text-graphite font-mono">
              SOURCE: /debug/data @ 50HZ (STM32)
            </p>
          </div>
        </div>
        <div className="text-[10px] font-mono text-graphite flex items-center gap-3">
          <span className="flex items-center gap-1"><span className="w-2.5 h-0.5 bg-pencil inline-block border-b border-dashed"></span> RAW</span>
          <span className="flex items-center gap-1"><span className="w-2.5 h-0.5 bg-sky inline-block"></span> FILTERED</span>
          <span className="flex items-center gap-1"><span className="w-2.5 h-0.5 bg-sketch-mint inline-block"></span> TARGET</span>
        </div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-3.5">
        {[0, 1, 2, 3].map(renderWheelChart)}
      </div>
    </div>
  );
}
