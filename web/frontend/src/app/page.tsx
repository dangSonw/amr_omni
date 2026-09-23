"use client";

import { useState, useEffect, useRef } from "react";
import { useRobotWs } from "@/hooks/useRobotWs";
import { Header } from "@/components/Header";
import { Sidebar, NavTab } from "@/components/Sidebar";
import { ConfigPanel } from "@/components/ConfigPanel";
import { CameraFeed } from "@/components/CameraFeed";
import { MapNavigation } from "@/components/MapNavigation";
import { StreamMatrix } from "@/components/StreamMatrix";
import { ImuQuaternionDisplay } from "@/components/charts/ImuQuaternionDisplay";
import { ImuNineAxisChart } from "@/components/charts/ImuNineAxisChart";
import { MotorSpeedChart } from "@/components/charts/MotorSpeedChart";
import { Activity, Cpu, RotateCcw, ShieldAlert, Zap } from "lucide-react";

const LINEAR_SPEED = 0.35; // m/s
const ANGULAR_SPEED = 1.5; // rad/s

export default function DashboardPage() {
  const { connected, telemetry, sendCmdVel, resetOdom } = useRobotWs();
  const [activeTab, setActiveTab] = useState<NavTab>("cockpit");

  const status = telemetry?.status;
  const odom = telemetry?.odom;

  const activeKeysRef = useRef<Set<string>>(new Set());
  const streamIntervalRef = useRef<NodeJS.Timeout | null>(null);
  const [debugHistory, setDebugHistory] = useState<Array<{ time: number; [key: string]: any }>>([]);
  const debugHistoryRef = useRef<Array<{ time: number; [key: string]: any }>>([]);
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

    newPoint.accel_x = dbg?.imu_accel_xyz?.[0] ?? telemetry.imu?.accel_x ?? 0;
    newPoint.accel_y = dbg?.imu_accel_xyz?.[1] ?? telemetry.imu?.accel_y ?? 0;
    newPoint.accel_z = dbg?.imu_accel_xyz?.[2] ?? telemetry.imu?.accel_z ?? 9.81;

    newPoint.gyro_x = dbg?.imu_gyro_xyz?.[0] ?? telemetry.imu?.gyro_x ?? 0;
    newPoint.gyro_y = dbg?.imu_gyro_xyz?.[1] ?? telemetry.imu?.gyro_y ?? 0;
    newPoint.gyro_z = dbg?.imu_gyro_xyz?.[2] ?? telemetry.imu?.gyro_z ?? 0;

    newPoint.mag_x = dbg?.imu_mag_xyz?.[0] ?? 0;
    newPoint.mag_y = dbg?.imu_mag_xyz?.[1] ?? 20.0;
    newPoint.mag_z = dbg?.imu_mag_xyz?.[2] ?? -45.0;

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
      if (keys.has("a")) vy += LINEAR_SPEED;
      if (keys.has("d")) vy -= LINEAR_SPEED;
      if (keys.has("q")) wz += ANGULAR_SPEED;
      if (keys.has("e")) wz -= ANGULAR_SPEED;
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
    <div className="flex min-h-screen flex-col bg-cream text-charcoal font-mono">
      {/* Top Header */}
      <Header status={status} />

      {/* Main Body */}
      <div className="flex-1 flex flex-col md:flex-row w-full overflow-hidden">
        {/* Sidebar */}
        <Sidebar activeTab={activeTab} onSelectTab={setActiveTab} />

        {/* Workspace */}
        <main className="flex-1 p-3 sm:p-5 overflow-y-auto w-full">
          {/* TAB 1: COCKPIT */}
          {activeTab === "cockpit" && (
            <div className="grid grid-cols-1 xl:grid-cols-12 gap-4 w-full">
              {/* Left Column: 2D Autonomous Map */}
              <div className="xl:col-span-7 2xl:col-span-8 flex flex-col gap-4">
                <div className="min-h-[580px] flex-1">
                  <MapNavigation
                    odom={telemetry?.odom}
                    lidar={telemetry?.lidar}
                    paths={telemetry?.paths}
                  />
                </div>
              </div>

              {/* Right Column: Camera + Quick Telemetry */}
              <div className="xl:col-span-5 2xl:col-span-4 flex flex-col gap-4">
                <div className="h-[360px]">
                  <CameraFeed />
                </div>

                {/* Quick Telemetry Box */}
                <div className="border-2 border-charcoal bg-white rounded-[2px] shadow-[-4px_4px_0px_#383838] p-4 space-y-3">
                  <div className="flex items-center justify-between border-b-2 border-charcoal pb-2 font-mono">
                    <div className="flex items-center gap-1.5 font-bold text-xs tracking-wider">
                      <Activity className="h-4 w-4" />
                      <span>TELEMETRY // LIVE</span>
                    </div>
                    <button
                      onClick={resetOdom}
                      className="flex items-center gap-1 px-2 py-0.5 border border-charcoal bg-sky hover:bg-sky-hover text-charcoal font-bold text-[10px] shadow-[-1px_1px_0px_#383838] transition"
                      title="Reset Odometry"
                    >
                      <RotateCcw className="h-3 w-3" />
                      RESET ODOM
                    </button>
                  </div>

                  {/* Velocity metrics */}
                  <div className="grid grid-cols-3 gap-2 text-center text-xs">
                    <div className="border border-charcoal bg-chalk p-1.5">
                      <div className="text-[10px] text-graphite font-bold">VX</div>
                      <div className="font-bold text-charcoal mt-0.5">
                        {odom ? `${odom.vx.toFixed(2)} m/s` : "--"}
                      </div>
                    </div>
                    <div className="border border-charcoal bg-chalk p-1.5">
                      <div className="text-[10px] text-graphite font-bold">VY</div>
                      <div className="font-bold text-charcoal mt-0.5">
                        {odom ? `${odom.vy.toFixed(2)} m/s` : "--"}
                      </div>
                    </div>
                    <div className="border border-charcoal bg-chalk p-1.5">
                      <div className="text-[10px] text-graphite font-bold">WZ</div>
                      <div className="font-bold text-charcoal mt-0.5">
                        {odom ? `${odom.wz.toFixed(2)} rad/s` : "--"}
                      </div>
                    </div>
                  </div>

                  {/* System Health */}
                  <div className="grid grid-cols-3 gap-2 text-center text-xs pt-1">
                    <div className="border border-charcoal bg-chalk p-1.5">
                      <div className="text-[10px] text-graphite font-bold flex items-center justify-center gap-0.5">
                        <Cpu className="h-3 w-3" /> CPU
                      </div>
                      <div className="font-bold text-charcoal mt-0.5">
                        {status ? `${status.cpu_percent.toFixed(0)}%` : "--"}
                      </div>
                    </div>
                    <div className="border border-charcoal bg-chalk p-1.5">
                      <div className="text-[10px] text-graphite font-bold">RAM</div>
                      <div className="font-bold text-charcoal mt-0.5">
                        {status ? `${status.ram_percent.toFixed(0)}%` : "--"}
                      </div>
                    </div>
                    <div className="border border-charcoal bg-chalk p-1.5">
                      <div className="text-[10px] text-graphite font-bold flex items-center justify-center gap-0.5">
                        <ShieldAlert className="h-3 w-3" /> E-STOP
                      </div>
                      <div
                        className={`font-bold mt-0.5 text-[11px] ${
                          status?.estop_active ? "text-rose-600" : "text-emerald-700"
                        }`}
                      >
                        {status?.estop_active ? "ACTIVE" : "READY"}
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          )}

          {/* TAB 2: CONFIG */}
          {activeTab === "config" && (
            <div className="w-full">
              <ConfigPanel />
            </div>
          )}

          {/* TAB 3: MONITOR */}
          {activeTab === "debug" && (
            <div className="flex flex-col gap-4 w-full">
              {/* IMU Row */}
              <div className="grid grid-cols-1 xl:grid-cols-3 gap-4 w-full">
                <div className="xl:col-span-1">
                  <ImuQuaternionDisplay
                    imu={telemetry?.imu}
                    debug={telemetry?.debug}
                    data={debugHistory}
                  />
                </div>
                <div className="xl:col-span-2">
                  <ImuNineAxisChart
                    imu={telemetry?.imu}
                    debug={telemetry?.debug}
                    data={debugHistory}
                  />
                </div>
              </div>

              {/* Wheel Velocity Chart */}
              <MotorSpeedChart data={debugHistory} debug={telemetry?.debug} />

              {/* Stream Audit Matrix */}
              <StreamMatrix streams={telemetry?.streams} />
            </div>
          )}
        </main>
      </div>
    </div>
  );
}
