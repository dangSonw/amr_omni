"use client";

import { StreamInfo } from "@/types/robot";
import { CheckCircle2, AlertTriangle, XCircle, Activity } from "lucide-react";

interface StreamMatrixProps {
  streams: StreamInfo[] | undefined;
}

export function StreamMatrix({ streams }: StreamMatrixProps) {
  const activeCount = streams?.filter((s) => s.status === "active").length ?? 0;
  const totalCount = streams?.length ?? 0;

  return (
    <div className="flex flex-col rounded-xl border border-slate-200 bg-white p-5 shadow-sm w-full">
      {/* Header */}
      <div className="flex items-center justify-between pb-3 border-b border-slate-100">
        <div className="flex items-center gap-2">
          <Activity className="h-4 w-4 text-brand-600" />
          <h2 className="text-sm font-bold text-slate-900">
            Giám Sát Luồng Giao Tiếp (Jetson ↔ STM32)
          </h2>
        </div>
        <div className="flex items-center gap-2 text-xs">
          <span className="text-slate-500">Trạng thái:</span>
          <span className="rounded-full bg-emerald-100 px-2.5 py-0.5 font-semibold text-emerald-800">
            {activeCount}/{totalCount} Hoạt động
          </span>
        </div>
      </div>

      {/* Stream List Table */}
      <div className="mt-3 overflow-x-auto">
        <table className="w-full text-left text-xs text-slate-600">
          <thead className="bg-slate-50 text-[11px] font-semibold text-slate-500 uppercase tracking-wider">
            <tr>
              <th className="py-2 px-3">Tên Luồng</th>
              <th className="py-2 px-3">Topic ROS</th>
              <th className="py-2 px-3">Tần Số (Hz)</th>
              <th className="py-2 px-3">Trạng Thái</th>
            </tr>
          </thead>
          <tbody className="divide-y divide-slate-100 font-mono">
            {streams && streams.length > 0 ? (
              streams.map((stream) => (
                <tr key={stream.id} className="hover:bg-slate-50/50">
                  <td className="py-2 px-3 font-sans font-medium text-slate-900">
                    {stream.name}
                  </td>
                  <td className="py-2 px-3 text-slate-500">{stream.topic}</td>
                  <td className="py-2 px-3 font-bold text-slate-800">
                    {stream.actual_frequency_hz.toFixed(1)} / {stream.target_frequency_hz}
                  </td>
                  <td className="py-2 px-3 font-sans">
                    {stream.status === "active" ? (
                      <span className="inline-flex items-center gap-1 text-emerald-600 font-semibold text-[11px]">
                        <CheckCircle2 className="h-3.5 w-3.5" /> OK
                      </span>
                    ) : stream.status === "degraded" ? (
                      <span className="inline-flex items-center gap-1 text-amber-600 font-semibold text-[11px]">
                        <AlertTriangle className="h-3.5 w-3.5" /> Chậm
                      </span>
                    ) : (
                      <span className="inline-flex items-center gap-1 text-rose-600 font-semibold text-[11px]">
                        <XCircle className="h-3.5 w-3.5" /> Mất kết nối
                      </span>
                    )}
                  </td>
                </tr>
              ))
            ) : (
              <tr>
                <td colSpan={4} className="py-4 text-center text-slate-400 font-sans">
                  Đang đồng bộ luồng dữ liệu...
                </td>
              </tr>
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
}
