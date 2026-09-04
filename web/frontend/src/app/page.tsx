"use client";

import { useState } from "react";
import { useRobotWs } from "@/hooks/useRobotWs";
import { Header } from "@/components/Header";
import { LidarViewer } from "@/components/LidarViewer";
import { StreamMatrix } from "@/components/StreamMatrix";
import { TeleopControl } from "@/components/TeleopControl";
import { WheelDiagnostics } from "@/components/WheelDiagnostics";
import { SensorCards } from "@/components/SensorCards";
import { ConfigPanel } from "@/components/ConfigPanel";
import { LayoutDashboard, Sliders, Network } from "lucide-react";

export default function DashboardPage() {
  const { connected, pingMs, telemetry, sendCmdVel, sendEStop, resetOdom } = useRobotWs();
  const [activeTab, setActiveTab] = useState<"dashboard" | "streams" | "config">("dashboard");

  const status = telemetry?.status;
  const isEstopActive = status?.estop_active ?? false;

  return (
    <div className="flex min-h-screen flex-col bg-slate-100 text-slate-900 transition-colors duration-200 dark:bg-slate-950 dark:text-slate-100">
      {/* Top Header */}
      <Header
        status={status}
        connected={connected}
        pingMs={pingMs}
        onToggleEstop={sendEStop}
      />

      {/* Main Container */}
      <main className="flex-1 p-3 sm:p-5 max-w-7xl w-full mx-auto space-y-4">
        {/* Navigation Tabs */}
        <div className="flex items-center gap-2 border-b border-slate-200 pb-2 dark:border-slate-800">
          <button
            onClick={() => setActiveTab("dashboard")}
            className={`flex items-center gap-2 rounded-xl px-4 py-2 text-xs font-bold transition ${
              activeTab === "dashboard"
                ? "bg-brand-600 text-white shadow-md shadow-brand-500/20"
                : "text-slate-600 hover:bg-slate-200/60 dark:text-slate-300 dark:hover:bg-slate-800"
            }`}
          >
            <LayoutDashboard className="h-4 w-4" />
            <span>Bảng Điều Khiển & Bản Đồ LiDAR</span>
          </button>

          <button
            onClick={() => setActiveTab("streams")}
            className={`flex items-center gap-2 rounded-xl px-4 py-2 text-xs font-bold transition ${
              activeTab === "streams"
                ? "bg-brand-600 text-white shadow-md shadow-brand-500/20"
                : "text-slate-600 hover:bg-slate-200/60 dark:text-slate-300 dark:hover:bg-slate-800"
            }`}
          >
            <Network className="h-4 w-4" />
            <span>Giám Sát Luồng Jetson-STM32</span>
            {telemetry?.streams && (
              <span className="rounded-full bg-emerald-500/20 px-1.5 py-0.2 text-[10px] text-emerald-600 dark:text-emerald-400">
                {telemetry.streams.filter((s) => s.status === "active").length} active
              </span>
            )}
          </button>

          <button
            onClick={() => setActiveTab("config")}
            className={`flex items-center gap-2 rounded-xl px-4 py-2 text-xs font-bold transition ${
              activeTab === "config"
                ? "bg-brand-600 text-white shadow-md shadow-brand-500/20"
                : "text-slate-600 hover:bg-slate-200/60 dark:text-slate-300 dark:hover:bg-slate-800"
            }`}
          >
            <Sliders className="h-4 w-4" />
            <span>Cấu Hình Thông Số Robot</span>
          </button>
        </div>

        {/* Tab 1: Dashboard */}
        {activeTab === "dashboard" && (
          <div className="space-y-4">
            {/* Top Row: LiDAR 2D Map (60%) + Teleop Control (40%) */}
            <div className="grid grid-cols-1 lg:grid-cols-12 gap-4">
              {/* LiDAR Canvas */}
              <div className="lg:col-span-7 h-[460px]">
                <LidarViewer lidar={telemetry?.lidar} odom={telemetry?.odom} />
              </div>

              {/* Teleop Control */}
              <div className="lg:col-span-5 h-[460px]">
                <TeleopControl onSendCmdVel={sendCmdVel} disabled={isEstopActive} />
              </div>
            </div>

            {/* Middle Row: Wheel Diagnostics */}
            <WheelDiagnostics wheels={telemetry?.wheels} />

            {/* Bottom Row: IMU & Odometry Cards */}
            <SensorCards
              imu={telemetry?.imu}
              odom={telemetry?.odom}
              onResetOdom={resetOdom}
            />

            {/* Embedded Live Stream Table Preview */}
            <StreamMatrix streams={telemetry?.streams} />
          </div>
        )}

        {/* Tab 2: Dedicated Streams View */}
        {activeTab === "streams" && (
          <div className="space-y-4">
            <StreamMatrix streams={telemetry?.streams} />
            <WheelDiagnostics wheels={telemetry?.wheels} />
          </div>
        )}

        {/* Tab 3: Config View */}
        {activeTab === "config" && (
          <div className="max-w-3xl mx-auto">
            <ConfigPanel />
          </div>
        )}
      </main>
    </div>
  );
}

