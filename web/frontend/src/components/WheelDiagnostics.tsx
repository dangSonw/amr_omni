"use client";

import { WheelTelemetry } from "@/types/robot";
import { Disc, Layers, Zap } from "lucide-react";

interface WheelDiagnosticsProps {
  wheels: WheelTelemetry | undefined;
}

export function WheelDiagnostics({ wheels }: WheelDiagnosticsProps) {
  const wheelLabels = [
    { id: 1, pos: "FR", name: "Bánh 1 (Trước - Phải)", angle: "-45°", mount: "(+L/2, -W/2)" },
    { id: 2, pos: "FL", name: "Bánh 2 (Trước - Trái)", angle: "+45°", mount: "(+L/2, +W/2)" },
    { id: 3, pos: "RL", name: "Bánh 3 (Sau - Trái)", angle: "+135°", mount: "(-L/2, +W/2)" },
    { id: 4, pos: "RR", name: "Bánh 4 (Sau - Phải)", angle: "-135°", mount: "(-L/2, -W/2)" },
  ];

  return (
    <div className="flex flex-col rounded-2xl border border-slate-200 bg-white p-4 shadow-sm dark:border-slate-800 dark:bg-slate-900">
      {/* Header */}
      <div className="flex items-center justify-between pb-3 border-b border-slate-200 dark:border-slate-800">
        <div className="flex items-center gap-2">
          <Disc className="h-5 w-5 text-brand-600 dark:text-brand-500" />
          <h2 className="text-base font-bold text-slate-900 dark:text-white">
            Trạng Thái Động Cơ 4 Bánh Xe Omni (Holonomic Drive)
          </h2>
        </div>
        <span className="text-xs text-slate-500 dark:text-slate-400">
          Tần số cập nhật: 50 Hz
        </span>
      </div>

      {/* 4-Wheel Cards Grid */}
      <div className="mt-4 grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-3">
        {wheelLabels.map((item, index) => {
          const target = wheels?.target_rad_s?.[index] ?? 0;
          const measured = wheels?.measured_rad_s?.[index] ?? 0;
          const ticks = wheels?.encoder_ticks?.[index] ?? 0;
          const pwm = wheels?.pwm_commands?.[index] ?? 0;
          const error = Math.abs(target - measured);

          return (
            <div
              key={item.id}
              className="flex flex-col justify-between rounded-xl border border-slate-200/80 bg-slate-50/60 p-3 dark:border-slate-800/80 dark:bg-slate-800/40"
            >
              {/* Card top */}
              <div>
                <div className="flex items-center justify-between">
                  <span className="rounded-md bg-brand-100 px-2 py-0.5 text-xs font-bold text-brand-800 dark:bg-brand-950 dark:text-brand-300">
                    {item.pos}
                  </span>
                  <span className="text-[11px] font-mono text-slate-400">
                    {item.angle}
                  </span>
                </div>
                <h3 className="mt-1 text-xs font-semibold text-slate-900 dark:text-white">
                  {item.name}
                </h3>
              </div>

              {/* Metrics */}
              <div className="mt-3 space-y-1.5 text-xs">
                {/* Measured speed */}
                <div className="flex items-center justify-between font-mono">
                  <span className="text-slate-500 dark:text-slate-400">Đo được:</span>
                  <span className="font-bold text-slate-900 dark:text-white">
                    {measured.toFixed(2)} rad/s
                  </span>
                </div>

                {/* Target speed */}
                <div className="flex items-center justify-between font-mono">
                  <span className="text-slate-500 dark:text-slate-400">Mục tiêu:</span>
                  <span className="text-slate-700 dark:text-slate-300">
                    {target.toFixed(2)} rad/s
                  </span>
                </div>

                {/* Speed error indicator */}
                <div className="h-1.5 w-full overflow-hidden rounded-full bg-slate-200 dark:bg-slate-700">
                  <div
                    className={`h-full rounded-full transition-all ${
                      error < 0.2 ? "bg-emerald-500" : error < 0.8 ? "bg-amber-500" : "bg-rose-500"
                    }`}
                    style={{
                      width: `${Math.min(100, (Math.abs(measured) / 18.0) * 100)}%`,
                    }}
                  />
                </div>

                {/* Encoders & PWM */}
                <div className="pt-1 border-t border-slate-200/60 dark:border-slate-700/60 flex items-center justify-between text-[11px] font-mono text-slate-500">
                  <span className="flex items-center gap-1">
                    <Layers className="h-3 w-3" /> {ticks.toLocaleString()} ticks
                  </span>
                  <span className="flex items-center gap-1">
                    <Zap className="h-3 w-3 text-amber-500" /> {pwm.toFixed(0)} PWM
                  </span>
                </div>
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}

