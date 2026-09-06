"use client";

import { useState, useEffect, useRef } from "react";
import { useRobotWs } from "@/hooks/useRobotWs";
import { Header } from "@/components/Header";
import { Sidebar, NavTab } from "@/components/Sidebar";
import { MotionStatusCard } from "@/components/MotionStatusCard";
import { ConfigPanel } from "@/components/ConfigPanel";
import { CameraFeed } from "@/components/CameraFeed";
import { MapNavigation } from "@/components/MapNavigation";
import { SensorCards } from "@/components/SensorCards";
import { WheelDiagnostics } from "@/components/WheelDiagnostics";
import { StreamMatrix } from "@/components/StreamMatrix";
import { MotorSpeedChart } from "@/components/charts/MotorSpeedChart";
import { ImuQuaternionDisplay } from "@/components/charts/ImuQuaternionDisplay";
import { AngularVelocityChart } from "@/components/charts/AngularVelocityChart";
import { LinearVelocityChart } from "@/components/charts/LinearVelocityChart";
import { DebugTelemetry } from "@/types/robot";

const LINEAR_SPEED = 0.35; // m/s
const ANGULAR_SPEED = 1.5; // rad/s

export default function DashboardPage() {
  const { connected, telemetry, sendCmdVel, resetOdom } = useRobotWs();
  const [activeTab, setActiveTab] = useState<NavTab>("cockpit");

  const status = telemetry?.status;

  // Ref quản lý phím bấm và stream vận tốc toàn cục
  const activeKeysRef = useRef<Set<string>>(new Set());
  const streamIntervalRef = useRef<NodeJS.Timeout | null>(null);
  const [debugHistory, setDebugHistory] = useState<Array<{time: number; [key: string]: any}>>([]);
  const debugHistoryRef = useRef<Array<{time: number; [key: string]: any}>>([]);
  const currentTwistRef = useRef<{ vx: number; vy: number; wz: number }>({
    vx: 0,
    vy: 0,
    wz: 0,
  });

  useEffect(() => {
    if (!telemetry) return;
    const dbg = telemetry.debug;
    const now = Date.now();
    
    const newPoint: any = { time: now };
    
    for (let i = 0; i < 4; i++) {
      newPoint[`m${i}_raw`] = dbg?.raw_wheel_speed_rad_s?.[i] ?? telemetry.wheels?.measured_rad_s?.[i] ?? 0;
      newPoint[`m${i}_filtered`] = dbg?.filtered_wheel_speed_rad_s?.[i] ?? telemetry.wheels?.measured_rad_s?.[i] ?? 0;
      newPoint[`m${i}_target`] = dbg?.target_wheel_speed_rad_s?.[i] ?? telemetry.wheels?.target_rad_s?.[i] ?? 0;
    }
    
    newPoint.qx = telemetry.imu?.qx ?? dbg?.imu_quaternion_xyzw?.[0] ?? 0;
    newPoint.qy = telemetry.imu?.qy ?? dbg?.imu_quaternion_xyzw?.[1] ?? 0;
    newPoint.qz = telemetry.imu?.qz ?? dbg?.imu_quaternion_xyzw?.[2] ?? 0;
    newPoint.qw = telemetry.imu?.qw ?? dbg?.imu_quaternion_xyzw?.[3] ?? 1;

    newPoint.cmd_wz = dbg?.cmd_wz_rad_s ?? 0;
    newPoint.actual_wz = dbg?.body_wz_rad_s ?? telemetry.odom?.wz ?? 0;
    newPoint.cmd_vx = dbg?.cmd_vx_mps ?? 0;
    newPoint.actual_vx = dbg?.body_vx_mps ?? telemetry.odom?.vx ?? 0;
    newPoint.cmd_vy = dbg?.cmd_vy_mps ?? 0;
    newPoint.actual_vy = dbg?.body_vy_mps ?? telemetry.odom?.vy ?? 0;

    debugHistoryRef.current.push(newPoint);
    if (debugHistoryRef.current.length > 200) {
      debugHistoryRef.current.shift();
    }
    setDebugHistory([...debugHistoryRef.current]);
  }, [telemetry]);

  useEffect(() => {
    const computeTwist = (keys: Set<string>) => {
      if (keys.has(" ")) return { vx: 0, vy: 0, wz: 0 };
      let vx = 0;
      let vy = 0;
      let wz = 0;
      if (keys.has("w")) vx += LINEAR_SPEED;
      if (keys.has("s")) vx -= LINEAR_SPEED;
      if (keys.has("a")) vy += LINEAR_SPEED; // Strafe Left
      if (keys.has("d")) vy -= LINEAR_SPEED; // Strafe Right
      if (keys.has("q")) wz += ANGULAR_SPEED; // Quay ngược chiều kim đồng hồ (CCW)
      if (keys.has("e")) wz -= ANGULAR_SPEED; // Quay cùng chiều kim đồng hồ (CW)
      return { vx, vy, wz };
    };

    const stopMoving = () => {
      if (streamIntervalRef.current) {
        clearInterval(streamIntervalRef.current);
        streamIntervalRef.current = null;
      }
      activeKeysRef.current.clear();
      currentTwistRef.current = { vx: 0, vy: 0, wz: 0 };
      sendCmdVel(0, 0, 0);
    };

    const handleKeyDown = (e: KeyboardEvent) => {
      const targetTag = (e.target as HTMLElement)?.tagName?.toLowerCase();
      if (["input", "textarea", "select"].includes(targetTag)) return;

      const key = e.key.toLowerCase();
      if (!["w", "a", "s", "d", "q", "e", " "].includes(key)) return;

      e.preventDefault();

      if (key === " ") {
        stopMoving();
        return;
      }

      if (!activeKeysRef.current.has(key)) {
        activeKeysRef.current.add(key);
        const twist = computeTwist(activeKeysRef.current);
        currentTwistRef.current = twist;
        sendCmdVel(twist.vx, twist.vy, twist.wz);

        if (
          !streamIntervalRef.current &&
          (twist.vx !== 0 || twist.vy !== 0 || twist.wz !== 0)
        ) {
          // Stream liên tục ở chu kỳ 80ms (12.5Hz) để không bị watchdog timeout 0.25s
          streamIntervalRef.current = setInterval(() => {
            const cur = currentTwistRef.current;
            sendCmdVel(cur.vx, cur.vy, cur.wz);
          }, 80);
        }
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const targetTag = (e.target as HTMLElement)?.tagName?.toLowerCase();
      if (["input", "textarea", "select"].includes(targetTag)) return;

      const key = e.key.toLowerCase();
      if (!["w", "a", "s", "d", "q", "e", " "].includes(key)) return;

      e.preventDefault();

      activeKeysRef.current.delete(key);
      const twist = computeTwist(activeKeysRef.current);
      currentTwistRef.current = twist;

      if (twist.vx === 0 && twist.vy === 0 && twist.wz === 0) {
        if (streamIntervalRef.current) {
          clearInterval(streamIntervalRef.current);
          streamIntervalRef.current = null;
        }
        sendCmdVel(0, 0, 0);
      } else {
        sendCmdVel(twist.vx, twist.vy, twist.wz);
      }
    };

    const handleBlur = () => {
      stopMoving();
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    window.addEventListener("blur", handleBlur);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      window.removeEventListener("blur", handleBlur);
      if (streamIntervalRef.current) {
        clearInterval(streamIntervalRef.current);
        streamIntervalRef.current = null;
      }
    };
  }, [sendCmdVel]);

  return (
    <div className="flex min-h-screen flex-col bg-slate-50 text-slate-900">
      {/* Top Header */}
      <Header status={status} />

      {/* Main Body: Sidebar + Dynamic Workspace */}
      <div className="flex-1 flex flex-col md:flex-row w-full overflow-hidden">
        {/* Sidebar Navigation */}
        <Sidebar activeTab={activeTab} onSelectTab={setActiveTab} />

        {/* Center Content Workspace */}
        <main className="flex-1 p-3 sm:p-5 overflow-y-auto">
          {/* Tab 1: Trạm Điều Khiển Trung Tâm (Cockpit) */}
          {activeTab === "cockpit" && (
            <div className="grid grid-cols-1 lg:grid-cols-12 gap-4 max-w-[1600px] mx-auto">
              {/* Cột Trái (Bản Đồ 2D) */}
              <div className="lg:col-span-7 flex flex-col gap-4">
                <div className="min-h-[520px] flex-1">
                  <MapNavigation
                    odom={telemetry?.odom}
                    lidar={telemetry?.lidar}
                    paths={telemetry?.paths}
                  />
                </div>
              </div>

              {/* Cột Phải (Camera Live Feed & Thẻ Động Học) */}
              <div className="lg:col-span-5 flex flex-col gap-4">
                {/* Camera Live Feed */}
                <div className="h-[300px]">
                  <CameraFeed />
                </div>

                {/* Thẻ Động Học & Hướng Di Chuyển */}
                <MotionStatusCard odom={telemetry?.odom} />
              </div>
            </div>
          )}

          {/* Tab 2: Cấu Hình Tham Số (Settings) */}
          {activeTab === "config" && (
            <div className="max-w-4xl mx-auto py-2">
              <ConfigPanel />
            </div>
          )}

          {activeTab === "debug" && (
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-3 max-w-[1600px] mx-auto">
              <MotorSpeedChart data={debugHistory} debug={telemetry?.debug} />
              <ImuQuaternionDisplay imu={telemetry?.imu} debug={telemetry?.debug} data={debugHistory} />
              <AngularVelocityChart data={debugHistory} />
              <LinearVelocityChart data={debugHistory} />
              <SensorCards imu={telemetry?.imu} odom={telemetry?.odom} onResetOdom={resetOdom} />
              <WheelDiagnostics wheels={telemetry?.wheels} />
              <div className="col-span-1 lg:col-span-2">
                <StreamMatrix streams={telemetry?.streams} />
              </div>
            </div>
          )}
        </main>
      </div>
    </div>
  );
}
