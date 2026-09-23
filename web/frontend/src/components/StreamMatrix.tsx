"use client";

import { useState } from "react";
import { StreamInfo } from "@/types/robot";
import {
  Activity,
  AlertTriangle,
  ArrowRight,
  CheckCircle2,
  Clock,
  Filter,
  Layers,
  Radio,
  XCircle,
} from "lucide-react";

interface StreamMatrixProps {
  streams: StreamInfo[] | undefined;
}

export function StreamMatrix({ streams }: StreamMatrixProps) {
  const [filterCategory, setFilterCategory] = useState<"all" | "stm32" | "control" | "sensor">("all");

  const totalCount = streams?.length ?? 0;
  const activeCount = streams?.filter((s) => s.status === "active").length ?? 0;
  const degradedCount = streams?.filter((s) => s.status === "degraded").length ?? 0;
  const offlineCount = streams?.filter((s) => s.status === "offline" || s.status === "stale").length ?? 0;

  // Phân loại luồng theo nghiệp vụ
  const filteredStreams = (streams ?? []).filter((s) => {
    if (filterCategory === "all") return true;
    const isStm32Related =
      s.id.startsWith("stm32_") ||
      s.id.includes("calib") ||
      s.id.includes("config") ||
      s.id === "estop" ||
      s.id === "hardware_status" ||
      s.source.includes("STM32") ||
      s.destination.includes("STM32");

    if (filterCategory === "stm32") return isStm32Related;

    if (filterCategory === "control") {
      return (
        s.id.includes("cmd_vel") ||
        s.id.includes("safety") ||
        s.id === "estop"
      );
    }

    if (filterCategory === "sensor") {
      return (
        s.id.includes("lidar") ||
        s.id.includes("imu") ||
        s.id.includes("odom") ||
        s.id.includes("map") ||
        s.id.includes("plan") ||
        s.id.includes("camera")
      );
    }

    return true;
  });

  const getNodeBadgeClass = (node: string) => {
    if (node.includes("STM32")) {
      return "bg-amber-100 text-amber-800 border-amber-200";
    }
    if (node.includes("Jetson")) {
      return "bg-blue-100 text-blue-800 border-blue-200";
    }
    if (node.includes("Web")) {
      return "bg-purple-100 text-purple-800 border-purple-200";
    }
    if (node.includes("Sensor") || node.includes("LiDAR") || node.includes("Camera") || node.includes("BNO")) {
      return "bg-emerald-100 text-emerald-800 border-emerald-200";
    }
    return "bg-slate-100 text-slate-700 border-slate-200";
  };

  return (
    <div className="flex flex-col rounded-xl border border-slate-200 bg-white p-5 shadow-sm w-full space-y-4">
      {/* Header & Badges */}
      <div className="flex flex-wrap items-center justify-between pb-3 border-b border-slate-100 gap-3">
        <div className="flex items-center gap-2.5">
          <div className="p-2 rounded-lg bg-blue-50 text-blue-600 border border-blue-100">
            <Activity className="h-5 w-5" />
          </div>
          <div>
            <h2 className="text-sm font-bold text-slate-900">
              Giám Sát Ma Trận Luồng Giao Tiếp Hệ Thống (Jetson ↔ STM32 ↔ Web)
            </h2>
            <p className="text-xs text-slate-500">
              Kiểm toán thời gian thực tất cả 22 topic truyền thông, tần số Hz, độ trễ và luồng nguồn → đích
            </p>
          </div>
        </div>

        {/* Status Pills */}
        <div className="flex items-center gap-2 text-xs flex-wrap">
          <span className="inline-flex items-center gap-1 rounded-full bg-emerald-50 px-2.5 py-1 font-semibold text-emerald-700 border border-emerald-200">
            <CheckCircle2 className="h-3.5 w-3.5" /> {activeCount} Hoạt động
          </span>
          {degradedCount > 0 && (
            <span className="inline-flex items-center gap-1 rounded-full bg-amber-50 px-2.5 py-1 font-semibold text-amber-700 border border-amber-200">
              <AlertTriangle className="h-3.5 w-3.5" /> {degradedCount} Giảm tần số
            </span>
          )}
          <span className="inline-flex items-center gap-1 rounded-full bg-slate-50 px-2.5 py-1 font-semibold text-slate-600 border border-slate-200">
            <Radio className="h-3.5 w-3.5 text-slate-400" /> {offlineCount} Chờ / Chế độ nghỉ
          </span>
        </div>
      </div>

      {/* Filter Tabs */}
      <div className="flex items-center gap-2 overflow-x-auto pb-1 text-xs">
        <span className="flex items-center gap-1 text-slate-500 font-medium mr-1 text-[11px]">
          <Filter className="h-3.5 w-3.5" /> Lọc Luồng:
        </span>
        <button
          type="button"
          onClick={() => setFilterCategory("all")}
          className={`px-3 py-1.5 rounded-lg font-medium transition ${
            filterCategory === "all"
              ? "bg-blue-600 text-white shadow-xs"
              : "bg-slate-100 text-slate-600 hover:bg-slate-200"
          }`}
        >
          Tất Cả ({totalCount})
        </button>
        <button
          type="button"
          onClick={() => setFilterCategory("stm32")}
          className={`px-3 py-1.5 rounded-lg font-medium transition ${
            filterCategory === "stm32"
              ? "bg-amber-600 text-white shadow-xs"
              : "bg-slate-100 text-slate-600 hover:bg-slate-200"
          }`}
        >
          STM32 ↔ Jetson & Web
        </button>
        <button
          type="button"
          onClick={() => setFilterCategory("control")}
          className={`px-3 py-1.5 rounded-lg font-medium transition ${
            filterCategory === "control"
              ? "bg-purple-600 text-white shadow-xs"
              : "bg-slate-100 text-slate-600 hover:bg-slate-200"
          }`}
        >
          Điều Khiển & An Toàn
        </button>
        <button
          type="button"
          onClick={() => setFilterCategory("sensor")}
          className={`px-3 py-1.5 rounded-lg font-medium transition ${
            filterCategory === "sensor"
              ? "bg-emerald-600 text-white shadow-xs"
              : "bg-slate-100 text-slate-600 hover:bg-slate-200"
          }`}
        >
          Cảm Biến & Định Vị
        </button>
      </div>

      {/* Stream List Table with Dedicated Source -> Destination Column */}
      <div className="overflow-x-auto rounded-lg border border-slate-200">
        <table className="w-full text-left text-xs text-slate-700 min-w-[850px]">
          <thead className="bg-slate-50 text-[11px] font-semibold text-slate-500 uppercase tracking-wider border-b border-slate-200">
            <tr>
              <th className="py-2.5 px-3">Tên Luồng / Chức Năng</th>
              <th className="py-2.5 px-3">Topic ROS 2</th>
              <th className="py-2.5 px-3 font-bold text-blue-700 bg-blue-50/50">
                Nguồn → Đích (Source → Destination)
              </th>
              <th className="py-2.5 px-3 text-center">Tần Số Thực / Mục Tiêu</th>
              <th className="py-2.5 px-3 text-center">Độ Trễ</th>
              <th className="py-2.5 px-3 text-center">Số Gói</th>
              <th className="py-2.5 px-3">Trạng Thái</th>
            </tr>
          </thead>
          <tbody className="divide-y divide-slate-100 font-mono">
            {filteredStreams.length > 0 ? (
              filteredStreams.map((stream) => {
                const isHealthy = stream.status === "active";
                const isDegraded = stream.status === "degraded";

                return (
                  <tr key={stream.id} className="hover:bg-slate-50/70 transition">
                    {/* Stream Name */}
                    <td className="py-2 px-3 font-sans">
                      <div className="font-semibold text-slate-900">{stream.name}</div>
                      <div className="text-[10px] text-slate-400 font-mono flex items-center gap-1 mt-0.5">
                        <Layers className="h-3 w-3" />
                        {stream.message_type}
                      </div>
                    </td>

                    {/* Topic ROS 2 */}
                    <td className="py-2 px-3">
                      <code className="text-slate-700 bg-slate-100 px-1.5 py-0.5 rounded text-[11px] font-bold">
                        /{stream.topic}
                      </code>
                      {stream.payload_preview && (
                        <div
                          className="text-[10px] text-slate-400 truncate max-w-[200px] mt-0.5"
                          title={stream.payload_preview}
                        >
                          {stream.payload_preview}
                        </div>
                      )}
                    </td>

                    {/* Dedicated Column: Nguồn -> Đích */}
                    <td className="py-2 px-3 font-sans bg-blue-50/20">
                      <div className="flex items-center gap-1.5 flex-wrap">
                        <span
                          className={`px-2 py-0.5 rounded-md text-[10px] font-medium border ${getNodeBadgeClass(
                            stream.source
                          )}`}
                        >
                          {stream.source}
                        </span>
                        <ArrowRight className="h-3 w-3 text-slate-400 shrink-0" />
                        <span
                          className={`px-2 py-0.5 rounded-md text-[10px] font-medium border ${getNodeBadgeClass(
                            stream.destination
                          )}`}
                        >
                          {stream.destination}
                        </span>
                      </div>
                    </td>

                    {/* Frequencies */}
                    <td className="py-2 px-3 text-center">
                      <span
                        className={`font-bold ${
                          isHealthy
                            ? "text-emerald-700"
                            : isDegraded
                            ? "text-amber-700"
                            : "text-slate-400"
                        }`}
                      >
                        {stream.actual_frequency_hz.toFixed(1)}
                      </span>
                      <span className="text-slate-400 text-[11px]"> / {stream.target_frequency_hz} Hz</span>
                    </td>

                    {/* Latency */}
                    <td className="py-2 px-3 text-center font-sans">
                      {stream.last_timestamp_sec > 0 ? (
                        <span className="text-slate-600 text-[11px] flex items-center justify-center gap-0.5">
                          <Clock className="h-3 w-3 text-slate-400" />
                          {stream.latency_ms < 1000
                            ? `${stream.latency_ms.toFixed(0)}ms`
                            : `${(stream.latency_ms / 1000).toFixed(1)}s`}
                        </span>
                      ) : (
                        <span className="text-slate-400 text-[11px]">--</span>
                      )}
                    </td>

                    {/* Packet Count */}
                    <td className="py-2 px-3 text-center text-slate-600 font-mono text-[11px]">
                      {stream.packet_count.toLocaleString()}
                    </td>

                    {/* Status */}
                    <td className="py-2 px-3 font-sans">
                      {isHealthy ? (
                        <span className="inline-flex items-center gap-1 text-emerald-700 font-semibold text-[11px] bg-emerald-50 px-2 py-0.5 rounded border border-emerald-200">
                          <CheckCircle2 className="h-3 w-3" /> Hoạt Động
                        </span>
                      ) : isDegraded ? (
                        <span className="inline-flex items-center gap-1 text-amber-700 font-semibold text-[11px] bg-amber-50 px-2 py-0.5 rounded border border-amber-200">
                          <AlertTriangle className="h-3 w-3" /> Chậm
                        </span>
                      ) : (
                        <span className="inline-flex items-center gap-1 text-slate-500 font-medium text-[11px] bg-slate-100 px-2 py-0.5 rounded border border-slate-200">
                          <XCircle className="h-3 w-3 text-slate-400" /> Chờ Tín Hiệu
                        </span>
                      )}
                    </td>
                  </tr>
                );
              })
            ) : (
              <tr>
                <td colSpan={7} className="py-6 text-center text-slate-400 font-sans">
                  Không tìm thấy luồng dữ liệu nào trong danh mục đã chọn.
                </td>
              </tr>
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
}
