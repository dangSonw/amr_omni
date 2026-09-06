"use client";

import { StreamInfo } from "@/types/robot";
import { ArrowRight, CheckCircle2, Clock, Cpu, Gauge, Network, Radio, AlertTriangle, XCircle } from "lucide-react";

interface StreamMatrixProps {
  streams: StreamInfo[] | undefined;
}

export function StreamMatrix({ streams }: StreamMatrixProps) {
  const activeCount = streams?.filter((s) => s.status === "active").length ?? 0;
  const totalCount = streams?.length ?? 0;

  return (
    <div className="flex flex-col rounded border border-slate-200 bg-white p-4 dark:border-slate-800 dark:bg-slate-900">
      {/* Section Header */}
      <div className="flex flex-wrap items-center justify-between gap-2 pb-3 border-b border-slate-200 dark:border-slate-800">
        <div className="flex items-center gap-2">
          <Network className="h-5 w-5 text-brand-600 dark:text-brand-500" />
          <h2 className="text-base font-bold text-slate-900 dark:text-white">
            Giám Sát Luồng Dữ Liệu (Jetson ↔ STM32 ↔ Cảm Biến)
          </h2>
        </div>
        <div className="flex items-center gap-2 text-xs">
          <span className="text-slate-500 dark:text-slate-400">Trạng thái luồng:</span>
          <span className="rounded-full bg-emerald-100 px-2.5 py-0.5 font-semibold text-emerald-800 dark:bg-emerald-950/60 dark:text-emerald-300">
            {activeCount}/{totalCount} Đang hoạt động
          </span>
        </div>
      </div>

      {/* Stream Architecture Diagram / Topology */}
      <div className="my-3 rounded-sm border border-slate-200/80 bg-slate-50/70 p-3 dark:border-slate-800/80 dark:bg-slate-800/40">
        <div className="grid grid-cols-1 md:grid-cols-3 gap-3 text-xs">
          {/* Node 1: Jetson */}
          <div className="flex flex-col items-center justify-center rounded-lg border border-blue-200 bg-blue-50/50 p-2.5 dark:border-blue-900/60 dark:bg-blue-950/30">
            <div className="flex items-center gap-1.5 font-bold text-blue-800 dark:text-blue-300">
              <Cpu className="h-4 w-4" />
              <span>JETSON NANO</span>
            </div>
            <p className="text-[11px] text-blue-600 dark:text-blue-400 mt-1 text-center">
              ROS Node, Watchdog, EKF, SLAM
            </p>
          </div>

          {/* Links description */}
          <div className="flex flex-col items-center justify-center text-center px-2">
            <div className="flex items-center gap-2 font-mono text-[11px] font-semibold text-slate-700 dark:text-slate-300">
              <span>safe_cmd_vel (50Hz)</span>
              <ArrowRight className="h-3.5 w-3.5 text-slate-400" />
            </div>
            <div className="my-1 h-[1px] w-full bg-slate-200 dark:bg-slate-700" />
            <div className="flex items-center gap-2 font-mono text-[11px] font-semibold text-slate-700 dark:text-slate-300">
              <ArrowRight className="h-3.5 w-3.5 rotate-180 text-slate-400" />
              <span>wheel_state, encoder, imu (50Hz)</span>
            </div>
          </div>

          {/* Node 2: STM32 */}
          <div className="flex flex-col items-center justify-center rounded-lg border border-purple-200 bg-purple-50/50 p-2.5 dark:border-purple-900/60 dark:bg-purple-950/30">
            <div className="flex items-center gap-1.5 font-bold text-purple-800 dark:text-purple-300">
              <Radio className="h-4 w-4" />
              <span>STM32 MCU (F4/F1)</span>
            </div>
            <p className="text-[11px] text-purple-600 dark:text-purple-400 mt-1 text-center">
              4x PWM Motor, 4x Encoder, IMU UART
            </p>
          </div>
        </div>
      </div>

      {/* Stream Matrix Table */}
      <div className="overflow-x-auto">
        <table className="w-full text-left text-xs">
          <thead>
            <tr className="border-b border-slate-200 text-slate-500 dark:border-slate-800 dark:text-slate-400">
              <th className="pb-2 font-semibold">Tên Luồng & Topic</th>
              <th className="pb-2 font-semibold">Hướng Truyền</th>
              <th className="pb-2 font-semibold">Tần Số (Hz)</th>
              <th className="pb-2 font-semibold">Gói Tin</th>
              <th className="pb-2 font-semibold">Độ Trễ</th>
              <th className="pb-2 font-semibold">Trạng Thái</th>
              <th className="pb-2 font-semibold">Payload Gần Nhất</th>
            </tr>
          </thead>
          <tbody className="divide-y divide-slate-100 dark:divide-slate-800/60">
            {streams && streams.length > 0 ? (
              streams.map((stream) => {
                const isHealthy = stream.status === "active";
                const isDegraded = stream.status === "degraded";
                const isStale = stream.status === "stale";
                const isOffline = stream.status === "offline";

                return (
                  <tr
                    key={stream.id}
                    className="hover:bg-slate-50/60 dark:hover:bg-slate-800/40 transition-colors"
                  >
                    {/* Tên & Topic */}
                    <td className="py-2.5 pr-3">
                      <div className="font-bold text-slate-900 dark:text-white">
                        {stream.name}
                      </div>
                      <div className="font-mono text-[11px] text-slate-500 dark:text-slate-400">
                        /{stream.topic} ({stream.message_type.split("/").pop()})
                      </div>
                    </td>

                    {/* Hướng truyền */}
                    <td className="py-2.5 pr-3 whitespace-nowrap">
                      <div className="flex items-center gap-1.5 text-slate-700 dark:text-slate-300 font-medium">
                        <span>{stream.source}</span>
                        <ArrowRight className="h-3 w-3 text-slate-400" />
                        <span>{stream.destination}</span>
                      </div>
                    </td>

                    {/* Tần số Hz */}
                    <td className="py-2.5 pr-3 whitespace-nowrap">
                      <div className="flex items-center gap-1.5">
                        <Gauge className="h-3.5 w-3.5 text-slate-400" />
                        <span className="font-mono font-bold text-slate-900 dark:text-white">
                          {stream.actual_frequency_hz}
                        </span>
                        <span className="text-slate-400">/ {stream.target_frequency_hz} Hz</span>
                      </div>
                      {/* Mini frequency bar */}
                      <div className="mt-1 h-1.5 w-20 overflow-hidden rounded-full bg-slate-200 dark:bg-slate-700">
                        <div
                          className={`h-full rounded-full transition-all ${
                            isHealthy
                              ? "bg-emerald-500"
                              : isDegraded
                              ? "bg-amber-500"
                              : "bg-rose-500"
                          }`}
                          style={{
                            width: `${Math.min(
                              100,
                              (stream.actual_frequency_hz / stream.target_frequency_hz) * 100
                            )}%`,
                          }}
                        />
                      </div>
                    </td>

                    {/* Packet Count */}
                    <td className="py-2.5 pr-3 font-mono text-slate-600 dark:text-slate-300 whitespace-nowrap">
                      {stream.packet_count.toLocaleString()}
                    </td>

                    {/* Latency */}
                    <td className="py-2.5 pr-3 whitespace-nowrap">
                      <div className="flex items-center gap-1 font-mono text-slate-600 dark:text-slate-400">
                        <Clock className="h-3 w-3" />
                        <span>{stream.latency_ms > 5000 ? "∞" : `${stream.latency_ms}ms`}</span>
                      </div>
                    </td>

                    {/* Trạng thái Badge */}
                    <td className="py-2.5 pr-3 whitespace-nowrap">
                      {isHealthy ? (
                        <span className="inline-flex items-center gap-1 rounded-full bg-emerald-100 px-2 py-0.5 text-[11px] font-semibold text-emerald-800 dark:bg-emerald-950/60 dark:text-emerald-300">
                          <CheckCircle2 className="h-3 w-3 text-emerald-600 dark:text-emerald-400" />
                          Hoạt động
                        </span>
                      ) : isDegraded ? (
                        <span className="inline-flex items-center gap-1 rounded-full bg-amber-100 px-2 py-0.5 text-[11px] font-semibold text-amber-800 dark:bg-amber-950/60 dark:text-amber-300">
                          <AlertTriangle className="h-3 w-3 text-amber-600 dark:text-amber-400" />
                          Giảm tần số
                        </span>
                      ) : isStale ? (
                        <span className="inline-flex items-center gap-1 rounded-full bg-amber-100 px-2 py-0.5 text-[11px] font-semibold text-amber-800 dark:bg-amber-950/60 dark:text-amber-300">
                          <AlertTriangle className="h-3 w-3 text-amber-600 dark:text-amber-400" />
                          Bị chậm
                        </span>
                      ) : (
                        <span className="inline-flex items-center gap-1 rounded-full bg-rose-100 px-2 py-0.5 text-[11px] font-semibold text-rose-800 dark:bg-rose-950/60 dark:text-rose-300">
                          <XCircle className="h-3 w-3 text-rose-600 dark:text-rose-400" />
                          Mất kết nối
                        </span>
                      )}
                    </td>

                    {/* Payload Preview */}
                    <td className="py-2.5 max-w-[200px] truncate font-mono text-[11px] text-slate-500 dark:text-slate-400">
                      {stream.payload_preview || "—"}
                    </td>
                  </tr>
                );
              })
            ) : (
              <tr>
                <td colSpan={7} className="py-6 text-center text-slate-400">
                  Đang tải danh sách luồng dữ liệu...
                </td>
              </tr>
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
}

