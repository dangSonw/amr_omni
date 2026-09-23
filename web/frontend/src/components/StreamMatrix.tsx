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
      return "bg-canary text-charcoal";
    }
    if (node.includes("Jetson")) {
      return "bg-sky text-charcoal";
    }
    if (node.includes("Web")) {
      return "bg-sketch-lilac text-charcoal";
    }
    if (node.includes("Sensor") || node.includes("LiDAR") || node.includes("Camera") || node.includes("BNO")) {
      return "bg-sketch-mint text-charcoal";
    }
    return "bg-chalk text-charcoal";
  };

  return (
    <div className="border-2 border-charcoal bg-white rounded-[2px] shadow-[-4px_4px_0px_#383838] p-5 text-charcoal w-full space-y-4">
      {/* Header & Badges */}
      <div className="flex flex-wrap items-center justify-between pb-3 border-b-2 border-charcoal gap-3">
        <div className="flex items-center gap-2">
          <Activity className="h-5 w-5 text-charcoal" />
          <div>
            <h2 className="text-sm sm:text-base font-bold tracking-wider">
              STREAM AUDIT // 22 TOPICS
            </h2>
            <p className="text-[11px] text-graphite font-mono">
              REAL-TIME COMM MATRIX (JETSON ↔ STM32 ↔ WEB)
            </p>
          </div>
        </div>

        {/* Status Pills */}
        <div className="flex items-center gap-2 text-xs font-mono font-bold flex-wrap">
          <span className="inline-flex items-center gap-1 border border-charcoal bg-sketch-mint px-2 py-0.5 shadow-[-2px_2px_0px_#383838]">
            <CheckCircle2 className="h-3.5 w-3.5 text-charcoal" /> {activeCount} ACTIVE
          </span>
          {degradedCount > 0 && (
            <span className="inline-flex items-center gap-1 border border-charcoal bg-canary px-2 py-0.5 shadow-[-2px_2px_0px_#383838]">
              <AlertTriangle className="h-3.5 w-3.5 text-charcoal" /> {degradedCount} DEGRADED
            </span>
          )}
          <span className="inline-flex items-center gap-1 border border-charcoal bg-chalk px-2 py-0.5 text-graphite shadow-[-2px_2px_0px_#383838]">
            <Radio className="h-3.5 w-3.5 text-graphite" /> {offlineCount} IDLE
          </span>
        </div>
      </div>

      {/* Filter Tabs */}
      <div className="flex items-center gap-2 overflow-x-auto pb-1 text-xs font-mono">
        <span className="flex items-center gap-1 text-graphite font-bold mr-1 text-[11px]">
          <Filter className="h-3.5 w-3.5" /> FILTER:
        </span>
        <button
          type="button"
          onClick={() => setFilterCategory("all")}
          className={`px-2.5 py-1 border border-charcoal font-bold transition ${
            filterCategory === "all"
              ? "bg-sky text-charcoal shadow-[-2px_2px_0px_#383838]"
              : "bg-white hover:bg-ice text-charcoal"
          }`}
        >
          ALL ({totalCount})
        </button>
        <button
          type="button"
          onClick={() => setFilterCategory("stm32")}
          className={`px-2.5 py-1 border border-charcoal font-bold transition ${
            filterCategory === "stm32"
              ? "bg-canary text-charcoal shadow-[-2px_2px_0px_#383838]"
              : "bg-white hover:bg-ice text-charcoal"
          }`}
        >
          STM32
        </button>
        <button
          type="button"
          onClick={() => setFilterCategory("control")}
          className={`px-2.5 py-1 border border-charcoal font-bold transition ${
            filterCategory === "control"
              ? "bg-sketch-lilac text-charcoal shadow-[-2px_2px_0px_#383838]"
              : "bg-white hover:bg-ice text-charcoal"
          }`}
        >
          CONTROL
        </button>
        <button
          type="button"
          onClick={() => setFilterCategory("sensor")}
          className={`px-2.5 py-1 border border-charcoal font-bold transition ${
            filterCategory === "sensor"
              ? "bg-sketch-mint text-charcoal shadow-[-2px_2px_0px_#383838]"
              : "bg-white hover:bg-ice text-charcoal"
          }`}
        >
          SENSORS
        </button>
      </div>

      {/* Stream Table */}
      <div className="overflow-x-auto border-2 border-charcoal rounded-[2px]">
        <table className="w-full text-left text-xs text-charcoal min-w-[800px]">
          <thead className="bg-chalk text-[10px] font-bold text-graphite uppercase tracking-wider border-b-2 border-charcoal font-mono">
            <tr>
              <th className="py-2 px-3 border-r border-charcoal">STREAM</th>
              <th className="py-2 px-3 border-r border-charcoal">TOPIC</th>
              <th className="py-2 px-3 border-r border-charcoal">SRC → DEST</th>
              <th className="py-2 px-3 text-center border-r border-charcoal">HZ (ACT/TGT)</th>
              <th className="py-2 px-3 text-center border-r border-charcoal">LATENCY</th>
              <th className="py-2 px-3 text-center border-r border-charcoal">PACKETS</th>
              <th className="py-2 px-3">STATUS</th>
            </tr>
          </thead>
          <tbody className="divide-y border-charcoal font-mono text-xs">
            {filteredStreams.length > 0 ? (
              filteredStreams.map((stream) => {
                const isHealthy = stream.status === "active";
                const isDegraded = stream.status === "degraded";

                return (
                  <tr key={stream.id} className="hover:bg-ice/50 transition">
                    {/* Stream Name */}
                    <td className="py-2 px-3 border-r border-charcoal">
                      <div className="font-bold text-charcoal">{stream.name}</div>
                      <div className="text-[10px] text-graphite flex items-center gap-1 mt-0.5">
                        <Layers className="h-3 w-3" />
                        {stream.message_type}
                      </div>
                    </td>

                    {/* Topic */}
                    <td className="py-2 px-3 border-r border-charcoal">
                      <span className="font-bold text-charcoal">/{stream.topic}</span>
                      {stream.payload_preview && (
                        <div
                          className="text-[9px] text-pencil truncate max-w-[180px] mt-0.5"
                          title={stream.payload_preview}
                        >
                          {stream.payload_preview}
                        </div>
                      )}
                    </td>

                    {/* Source -> Destination */}
                    <td className="py-2 px-3 border-r border-charcoal">
                      <div className="flex items-center gap-1 flex-wrap">
                        <span
                          className={`px-1.5 py-0.5 text-[9px] font-bold border border-charcoal ${getNodeBadgeClass(
                            stream.source
                          )}`}
                        >
                          {stream.source}
                        </span>
                        <ArrowRight className="h-3 w-3 text-graphite shrink-0" />
                        <span
                          className={`px-1.5 py-0.5 text-[9px] font-bold border border-charcoal ${getNodeBadgeClass(
                            stream.destination
                          )}`}
                        >
                          {stream.destination}
                        </span>
                      </div>
                    </td>

                    {/* Frequencies */}
                    <td className="py-2 px-3 text-center border-r border-charcoal font-bold">
                      <span
                        className={
                          isHealthy
                            ? "text-charcoal"
                            : isDegraded
                            ? "text-amber-700 bg-canary px-1"
                            : "text-pencil"
                        }
                      >
                        {stream.actual_frequency_hz.toFixed(1)}
                      </span>
                      <span className="text-pencil text-[10px]"> / {stream.target_frequency_hz}</span>
                    </td>

                    {/* Latency */}
                    <td className="py-2 px-3 text-center border-r border-charcoal">
                      {stream.last_timestamp_sec > 0 ? (
                        <span className="text-[11px] flex items-center justify-center gap-0.5">
                          <Clock className="h-3 w-3 text-graphite" />
                          {stream.latency_ms < 1000
                            ? `${stream.latency_ms.toFixed(0)}ms`
                            : `${(stream.latency_ms / 1000).toFixed(1)}s`}
                        </span>
                      ) : (
                        <span className="text-pencil">--</span>
                      )}
                    </td>

                    {/* Packet Count */}
                    <td className="py-2 px-3 text-center border-r border-charcoal text-[11px]">
                      {stream.packet_count.toLocaleString()}
                    </td>

                    {/* Status */}
                    <td className="py-2 px-3">
                      {isHealthy ? (
                        <span className="inline-flex items-center gap-1 font-bold text-[10px] bg-sketch-mint text-charcoal px-1.5 py-0.5 border border-charcoal">
                          <CheckCircle2 className="h-3 w-3" /> ACTIVE
                        </span>
                      ) : isDegraded ? (
                        <span className="inline-flex items-center gap-1 font-bold text-[10px] bg-canary text-charcoal px-1.5 py-0.5 border border-charcoal">
                          <AlertTriangle className="h-3 w-3" /> SLOW
                        </span>
                      ) : (
                        <span className="inline-flex items-center gap-1 font-bold text-[10px] bg-chalk text-graphite px-1.5 py-0.5 border border-charcoal">
                          <XCircle className="h-3 w-3" /> IDLE
                        </span>
                      )}
                    </td>
                  </tr>
                );
              })
            ) : (
              <tr>
                <td colSpan={7} className="py-6 text-center text-graphite">
                  NO STREAMS IN CATEGORY
                </td>
              </tr>
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
}
