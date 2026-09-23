"use client";

import React from "react";
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from "recharts";
import { ImuTelemetry, DebugTelemetry } from "@/types/robot";
import { Compass } from "lucide-react";

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
    <div className="border-2 border-charcoal bg-white rounded-[2px] shadow-[-4px_4px_0px_#383838] p-5 font-mono text-charcoal space-y-3">
      <div className="flex items-center gap-2 pb-2 border-b-2 border-charcoal">
        <Compass className="h-4 w-4" />
        <h2 className="text-sm font-bold tracking-wider">
          ORIENTATION // QUATERNION
        </h2>
      </div>

      <div className="grid grid-cols-4 gap-2">
        <div className="border border-charcoal bg-chalk p-1.5 text-center shadow-[-1px_1px_0px_#383838]">
          <div className="text-[10px] text-graphite font-bold">QX</div>
          <div className="text-xs font-bold mt-0.5">{qx.toFixed(3)}</div>
        </div>
        <div className="border border-charcoal bg-chalk p-1.5 text-center shadow-[-1px_1px_0px_#383838]">
          <div className="text-[10px] text-graphite font-bold">QY</div>
          <div className="text-xs font-bold mt-0.5">{qy.toFixed(3)}</div>
        </div>
        <div className="border border-charcoal bg-chalk p-1.5 text-center shadow-[-1px_1px_0px_#383838]">
          <div className="text-[10px] text-graphite font-bold">QZ</div>
          <div className="text-xs font-bold mt-0.5">{qz.toFixed(3)}</div>
        </div>
        <div className="border border-charcoal bg-chalk p-1.5 text-center shadow-[-1px_1px_0px_#383838]">
          <div className="text-[10px] text-graphite font-bold">QW</div>
          <div className="text-xs font-bold mt-0.5">{qw.toFixed(3)}</div>
        </div>
      </div>

      <div className="h-44">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data}>
            <CartesianGrid strokeDasharray="2 2" stroke="#d5cfc7" />
            <XAxis dataKey="time" type="number" domain={["dataMin", "dataMax"]} hide />
            <YAxis domain={[-1.1, 1.1]} stroke="#383838" fontSize={9} width={28} />
            <Tooltip contentStyle={tooltipStyle} />
            <Legend wrapperStyle={{ fontSize: 9, paddingTop: 2 }} />
            <Line type="monotone" dataKey="qx" stroke="#f38e84" strokeWidth={1.5} dot={false} isAnimationActive={false} />
            <Line type="monotone" dataKey="qy" stroke="#38c1b0" strokeWidth={1.5} dot={false} isAnimationActive={false} />
            <Line type="monotone" dataKey="qz" stroke="#6fc2ff" strokeWidth={1.5} dot={false} isAnimationActive={false} />
            <Line type="monotone" dataKey="qw" stroke="#ffde00" strokeWidth={1.5} dot={false} isAnimationActive={false} />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
