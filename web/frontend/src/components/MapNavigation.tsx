"use client";

import React, { useRef, useEffect, useState, useCallback } from "react";
import {
  Navigation,
  Target,
  Save,
  CheckCircle2,
  XCircle,
  Crosshair,
  ZoomIn,
  ZoomOut,
  Compass,
  RotateCw,
  Trash2,
  Bot,
} from "lucide-react";
import { OdometryTelemetry, PathTelemetry, LidarTelemetry } from "../types/robot";

interface MapNavigationProps {
  odom?: OdometryTelemetry;
  lidar?: LidarTelemetry;
  paths?: PathTelemetry;
}

export const MapNavigation: React.FC<MapNavigationProps> = ({
  odom,
  lidar,
  paths,
}) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  // Bộ nhớ vật cản tích lũy (Persistent Obstacle Map Memory)
  // Lưu tọa độ thế giới của tất cả điểm tường/vật thể đã quét qua
  const persistentObstaclesRef = useRef<Map<string, [number, number]>>(new Map());

  const [goal, setGoal] = useState<{ x: number; y: number } | null>(null);
  const [navState, setNavState] = useState<string>("idle");
  const [distanceRemaining, setDistanceRemaining] = useState<number | null>(null);
  const [saveStatus, setSaveStatus] = useState<string | null>(null);
  const [clearStatus, setClearStatus] = useState<string | null>(null);
  const [mapScale, setMapScale] = useState<number>(45); // pixels per meter
  const [offset, setOffset] = useState<{ x: number; y: number }>({ x: 0, y: 0 });

  // Chế độ bản đồ: "north-up" (Cố định hệ tọa độ thế giới - mặc định, không chóng mặt)
  // "heading-up" (Tự động xoay theo chiều của xe) hoặc "custom" (Xoay góc thủ công)
  const [mapMode, setMapMode] = useState<"north-up" | "heading-up" | "custom">("north-up");
  const [customAngleDeg, setCustomAngleDeg] = useState<number>(0);
  const [followRobot, setFollowRobot] = useState<boolean>(false);

  const robotX = odom?.x ?? 0.0;
  const robotY = odom?.y ?? 0.0;
  const robotTheta = odom?.theta_rad ?? 0.0;

  // Đồng bộ bản đồ vật cản chuẩn xác từ backend (Probabilistic Grid Planner có Ray Clearing)
  useEffect(() => {
    if (paths?.obstacles) {
      const newMap = new Map<string, [number, number]>();
      for (const [ox, oy] of paths.obstacles) {
        const key = `${ox.toFixed(2)},${oy.toFixed(2)}`;
        newMap.set(key, [ox, oy]);
      }
      persistentObstaclesRef.current = newMap;
    }
  }, [paths?.obstacles]);

  // 3. Tự động bám theo xe nếu followRobot = true
  useEffect(() => {
    if (followRobot) {
      setOffset({
        x: -robotX * mapScale,
        y: robotY * mapScale,
      });
    }
  }, [followRobot, robotX, robotY, mapScale]);

  // Poll trạng thái tự hành từ backend mỗi 500ms
  useEffect(() => {
    const interval = setInterval(async () => {
      try {
        const res = await fetch("/api/nav/status");
        if (res.ok) {
          const data = await res.json();
          if (data.navigation) {
            setNavState(data.navigation.state || "idle");
            if (data.navigation.goal) {
              setGoal({ x: data.navigation.goal.x, y: data.navigation.goal.y });
            } else if (data.navigation.state === "idle" || data.navigation.state === "cancelled") {
              setGoal(null);
            }
            setDistanceRemaining(data.navigation.distance_remaining_m ?? null);
          }
        }
      } catch {
        // Ignored
      }
    }, 500);
    return () => clearInterval(interval);
  }, []);

  const handleCancelNav = async () => {
    try {
      await fetch("/api/nav/cancel", { method: "POST" });
      setGoal(null);
      setNavState("idle");
      setDistanceRemaining(null);
    } catch {
      // Ignored
    }
  };

  const handleResetCenter = useCallback(() => {
    setFollowRobot(false);
    setOffset({ x: 0, y: 0 });
  }, []);

  const handleCenterOnRobot = useCallback(() => {
    setOffset({
      x: -robotX * mapScale,
      y: robotY * mapScale,
    });
  }, [robotX, robotY, mapScale]);

  const handleClearMap = async () => {
    persistentObstaclesRef.current.clear();
    setClearStatus("Cleared");
    try {
      await fetch("/api/map/clear", { method: "POST" });
    } catch {
      // Ignored
    }
    setTimeout(() => setClearStatus(null), 2000);
  };

  // Tính toán góc xoay của góc nhìn hiển thị (View Rotation Angle)
  const viewAngleRad =
    mapMode === "heading-up"
      ? robotTheta - Math.PI / 2
      : mapMode === "custom"
      ? (customAngleDeg * Math.PI) / 180
      : 0.0;

  // Render Canvas chính
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const width = canvas.width;
    const height = canvas.height;
    const cx = width / 2 + offset.x;
    const cy = height / 2 + offset.y;

    // Clear background
    ctx.fillStyle = "#e2e8f0";
    ctx.fillRect(0, 0, width, height);

    ctx.save();

    // Áp dụng góc xoay góc nhìn nếu không phải North-Up
    if (viewAngleRad !== 0) {
      ctx.translate(cx, cy);
      ctx.rotate(-viewAngleRad);
      ctx.translate(-cx, -cy);
    }

    // 1. Draw coordinate grid (1 meter intervals)
    ctx.strokeStyle = "#cbd5e1";
    ctx.lineWidth = 1;
    const step = mapScale;
    for (let x = cx % step; x < width * 2; x += step) {
      ctx.beginPath();
      ctx.moveTo(x - width, -height);
      ctx.lineTo(x - width, height * 2);
      ctx.stroke();
    }
    for (let y = cy % step; y < height * 2; y += step) {
      ctx.beginPath();
      ctx.moveTo(-width, y - height);
      ctx.lineTo(width * 2, y - height);
      ctx.stroke();
    }

    // Origin cross
    ctx.strokeStyle = "#94a3b8";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(cx, -height);
    ctx.lineTo(cx, height * 2);
    ctx.moveTo(-width, cy);
    ctx.lineTo(width * 2, cy);
    ctx.stroke();

    // 2. Draw Persistent Obstacles (Recorded obstacle memory)
    if (persistentObstaclesRef.current.size > 0) {
      ctx.fillStyle = "rgba(16, 185, 129, 0.75)";
      persistentObstaclesRef.current.forEach(([ox, oy]) => {
        const px = cx + ox * mapScale;
        const py = cy - oy * mapScale;
        ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
      });
    }

    // 3. Draw Live LiDAR Points
    if (lidar?.points && lidar.points.length > 0) {
      ctx.fillStyle = "#059669";
      const cosR = Math.cos(robotTheta);
      const sinR = Math.sin(robotTheta);

      for (const [lx, ly] of lidar.points) {
        const wx = robotX + (cosR * lx - sinR * ly);
        const wy = robotY + (sinR * lx + cosR * ly);
        const px = cx + wx * mapScale;
        const py = cy - wy * mapScale;
        ctx.fillRect(px - 2, py - 2, 4, 4);
      }
    }

    // 4. Draw Global Path (Lộ trình né tường toàn cục)
    const hasGlobalPath = paths?.global_path && paths.global_path.length > 1;
    if (hasGlobalPath) {
      ctx.save();
      ctx.strokeStyle = "#38bdf8"; // Sky blue
      ctx.lineWidth = 2.5;
      ctx.setLineDash([6, 6]);
      ctx.shadowColor = "rgba(56, 189, 248, 0.5)";
      ctx.shadowBlur = 8;
      ctx.beginPath();
      paths.global_path.forEach(([gx, gy], index) => {
        const px = cx + gx * mapScale;
        const py = cy - gy * mapScale;
        if (index === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.restore();
    } else if (goal) {
      // Fallback: Tự động vẽ đường nối trực tiếp từ xe tới đích khi chưa nhận được global_path
      ctx.save();
      ctx.strokeStyle = "#38bdf8";
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 6]);
      ctx.beginPath();
      ctx.moveTo(cx + robotX * mapScale, cy - robotY * mapScale);
      ctx.lineTo(cx + goal.x * mapScale, cy - goal.y * mapScale);
      ctx.stroke();
      ctx.restore();
    }

    // 5. Draw Local Path (Quỹ đạo bẻ cua né vật cản thời gian thực)
    if (paths?.local_path && paths.local_path.length > 1) {
      ctx.save();
      ctx.strokeStyle = "#34d399"; // Emerald 400
      ctx.lineWidth = 3.5;
      ctx.shadowColor = "rgba(52, 211, 153, 0.6)";
      ctx.shadowBlur = 6;
      ctx.beginPath();
      paths.local_path.forEach(([lx, ly], index) => {
        const px = cx + lx * mapScale;
        const py = cy - ly * mapScale;
        if (index === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.restore();

      // Vẽ các nút điểm uốn né vật cản
      paths.local_path.forEach(([lx, ly]) => {
        const px = cx + lx * mapScale;
        const py = cy - ly * mapScale;
        ctx.fillStyle = "#10b981";
        ctx.beginPath();
        ctx.arc(px, py, 3, 0, Math.PI * 2);
        ctx.fill();
      });
    }

    // 6. Draw Goal Pose & Target Ring
    if (goal) {
      const gx = cx + goal.x * mapScale;
      const gy = cy - goal.y * mapScale;

      ctx.save();
      ctx.strokeStyle = "#f43f5e"; // Rose 500
      ctx.fillStyle = "rgba(244, 63, 94, 0.25)";
      ctx.lineWidth = 2;
      ctx.shadowColor = "rgba(244, 63, 94, 0.7)";
      ctx.shadowBlur = 10;
      ctx.beginPath();
      ctx.arc(gx, gy, 10, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Tâm cờ đích
      ctx.fillStyle = "#f43f5e";
      ctx.beginPath();
      ctx.arc(gx, gy, 4, 0, Math.PI * 2);
      ctx.fill();

      // Nhãn tọa độ đích
      ctx.fillStyle = "#881337";
      ctx.font = "bold 10px monospace";
      ctx.fillText(`(${goal.x}m, ${goal.y}m)`, gx + 12, gy + 4);
      ctx.restore();
    }

    // 7. Draw Robot Chassis & Forward Field-of-View Cone
    const rx = cx + robotX * mapScale;
    const ry = cy - robotY * mapScale;
    const radius = 0.16 * mapScale; // ~16cm

    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-robotTheta);

    // Nón tầm nhìn Camera & LiDAR hướng về phía trước đầu xe
    const fovLength = 0.7 * mapScale;
    const fovAngle = (45 * Math.PI) / 180;
    const fovGrad = ctx.createRadialGradient(0, 0, radius, 0, 0, fovLength);
    fovGrad.addColorStop(0, "rgba(99, 102, 241, 0.35)");
    fovGrad.addColorStop(1, "rgba(99, 102, 241, 0.0)");
    ctx.fillStyle = fovGrad;
    ctx.beginPath();
    ctx.moveTo(radius * 0.8, 0);
    ctx.lineTo(fovLength, -fovLength * Math.tan(fovAngle * 0.5));
    ctx.lineTo(fovLength, fovLength * Math.tan(fovAngle * 0.5));
    ctx.closePath();
    ctx.fill();

    // Thân xe Omni (Chassis vuông)
    ctx.fillStyle = "rgba(99, 102, 241, 0.85)"; // Indigo
    ctx.strokeStyle = "#e0e7ff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.rect(-radius, -radius, radius * 2, radius * 2);
    ctx.fill();
    ctx.stroke();

    // 4 Bánh xe Omni ở 4 góc
    const wheelW = radius * 0.45;
    const wheelH = radius * 0.22;
    ctx.fillStyle = "#1e293b";
    ctx.strokeStyle = "#94a3b8";
    ctx.lineWidth = 1;
    [
      [-radius, -radius],
      [radius - wheelW, -radius],
      [-radius, radius - wheelH],
      [radius - wheelW, radius - wheelH],
    ].forEach(([wx, wy]) => {
      ctx.fillRect(wx, wy, wheelW, wheelH);
      ctx.strokeRect(wx, wy, wheelW, wheelH);
    });

    // Mũi tên chỉ hướng đầu xe (Heading Arrow)
    ctx.fillStyle = "#fbbf24"; // Amber 400
    ctx.beginPath();
    ctx.moveTo(radius + 10, 0);
    ctx.lineTo(radius - 2, -6);
    ctx.lineTo(radius - 2, 6);
    ctx.closePath();
    ctx.fill();

    ctx.restore(); // Kết thúc vẽ robot
    ctx.restore(); // Kết thúc view transform

    // 8. HUD La Bàn Định Hướng (Compass Rose) cố định ở góc trái trên
    ctx.save();
    const compassX = 35;
    const compassY = 35;
    const compassR = 18;

    // Nền la bàn
    ctx.fillStyle = "rgba(15, 23, 42, 0.85)";
    ctx.strokeStyle = "#334155";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(compassX, compassY, compassR, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Kim la bàn xoay theo viewAngleRad
    ctx.translate(compassX, compassY);
    ctx.rotate(-viewAngleRad);

    // Mũi kim Bắc (Đỏ)
    ctx.fillStyle = "#f43f5e";
    ctx.beginPath();
    ctx.moveTo(0, -compassR + 4);
    ctx.lineTo(4, 0);
    ctx.lineTo(-4, 0);
    ctx.closePath();
    ctx.fill();

    // Mũi kim Nam (Trắng)
    ctx.fillStyle = "#94a3b8";
    ctx.beginPath();
    ctx.moveTo(0, compassR - 4);
    ctx.lineTo(4, 0);
    ctx.lineTo(-4, 0);
    ctx.closePath();
    ctx.fill();

    // Chữ N
    ctx.fillStyle = "#f43f5e";
    ctx.font = "bold 9px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("N", 0, -compassR - 3);

    ctx.restore();
  }, [
    robotX,
    robotY,
    robotTheta,
    lidar,
    paths,
    goal,
    mapScale,
    offset,
    viewAngleRad,
  ]);

  const handleCanvasClick = async (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / (rect.width || 1);
    const scaleY = canvas.height / (rect.height || 1);
    const clickX = (e.clientX - rect.left) * scaleX;
    const clickY = (e.clientY - rect.top) * scaleY;

    const cx = canvas.width / 2 + offset.x;
    const cy = canvas.height / 2 + offset.y;

    const dx = clickX - cx;
    const dy = clickY - cy;

    // Nghịch đảo phép xoay góc nhìn để lấy tọa độ thế giới chính xác tuyệt đối
    const cosA = Math.cos(viewAngleRad);
    const sinA = Math.sin(viewAngleRad);
    const unrotX = dx * cosA - dy * sinA;
    const unrotY = dx * sinA + dy * cosA;

    const targetX = Number((unrotX / mapScale).toFixed(2));
    const targetY = Number((-unrotY / mapScale).toFixed(2));

    setGoal({ x: targetX, y: targetY });
    setNavState("navigating");

    try {
      await fetch("/api/nav/goal", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ x: targetX, y: targetY, theta_rad: 0.0 }),
      });
    } catch {
      // Ignored
    }
  };

  const handleSaveMap = async () => {
    setSaveStatus("Saving...");
    try {
      await fetch("/api/map/save", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ map_name: `amr_map_${Date.now()}` }),
      });
      setSaveStatus("Saved!");
      setTimeout(() => setSaveStatus(null), 3000);
    } catch {
      setSaveStatus("Save failed");
      setTimeout(() => setSaveStatus(null), 3000);
    }
  };

  return (
    <div className="bg-white border border-slate-200 rounded p-3 sm:p-4 flex flex-col h-full text-slate-800">
      {/* Header Map */}
      <div className="flex flex-wrap items-center justify-between gap-2 pb-2.5 border-b border-slate-200 mb-2.5">
        <div className="flex items-center gap-2">
          <Navigation className="w-4 h-4 text-brand-600" />
          <h3 className="font-bold text-slate-900 text-xs sm:text-sm tracking-wide">
            2D Autonomous Map
          </h3>
        </div>

        {/* Navigation Status & Actions */}
        <div className="flex flex-wrap items-center gap-2">
          {navState === "navigating" && goal ? (
            <div className="flex items-center gap-1.5 bg-amber-50 border border-amber-300 px-2.5 py-1 rounded-lg text-amber-700 text-xs animate-pulse">
              <Target className="w-3.5 h-3.5" />
              <span>
                Navigating to ({goal.x}m, {goal.y}m)
                {distanceRemaining !== null && ` • ${distanceRemaining}m left`}
              </span>
              <button
                onClick={handleCancelNav}
                className="ml-1 text-slate-400 hover:text-rose-600 transition"
                title="Cancel navigation"
              >
                <XCircle className="w-3.5 h-3.5" />
              </button>
            </div>
          ) : navState === "reached" ? (
            <div className="flex items-center gap-1 bg-emerald-50 border border-emerald-300 px-2.5 py-1 rounded-lg text-emerald-700 text-xs">
              <CheckCircle2 className="w-3.5 h-3.5" />
              <span>Goal Reached!</span>
            </div>
          ) : (
            <span className="text-[11px] text-slate-500 hidden sm:inline">
              {persistentObstaclesRef.current.size > 0
                ? `${persistentObstaclesRef.current.size} map points recorded`
                : "Localization ready"}
            </span>
          )}

          {/* Map Orientation Switcher */}
          <button
            onClick={() =>
              setMapMode((m) => (m === "north-up" ? "heading-up" : "north-up"))
            }
            className={`px-2.5 py-1 rounded-lg text-xs font-medium flex items-center gap-1.5 transition border ${
              mapMode === "north-up"
                ? "bg-slate-100 text-slate-700 border-slate-200 hover:bg-slate-200"
                : "bg-amber-50 text-amber-700 border-amber-300 hover:bg-amber-100"
            }`}
            title="Toggle between Fixed Map (North-Up) and Robot Frame (Heading-Up)"
          >
            <Compass className="w-3.5 h-3.5" />
            <span>
              {mapMode === "north-up" ? "North-Up" : "Heading-Up"}
            </span>
          </button>

          {/* Clear Map Memory */}
          <button
            onClick={handleClearMap}
            className="px-2.5 py-1 bg-slate-100 hover:bg-rose-50 hover:text-rose-600 text-slate-700 rounded-lg text-xs font-medium flex items-center gap-1.5 transition border border-slate-200"
            title="Clear all recorded obstacle points from map memory"
          >
            <Trash2 className="w-3.5 h-3.5" />
            <span>{clearStatus || "Clear Map"}</span>
          </button>

          {/* Save Map */}
          <button
            onClick={handleSaveMap}
            className="px-2.5 py-1 bg-slate-100 hover:bg-slate-200 text-slate-700 rounded-lg text-xs font-medium flex items-center gap-1.5 transition border border-slate-200"
          >
            {saveStatus ? (
              <CheckCircle2 className="w-3.5 h-3.5 text-emerald-600" />
            ) : (
              <Save className="w-3.5 h-3.5" />
            )}
            {saveStatus || "Save Map"}
          </button>
        </div>
      </div>

      {/* Map Canvas */}
      <div className="relative flex-1 bg-slate-200 rounded-lg overflow-hidden border border-slate-300 flex items-center justify-center min-h-[420px]">
        <canvas
          ref={canvasRef}
          width={720}
          height={460}
          onClick={handleCanvasClick}
          className="w-full h-full cursor-crosshair object-cover"
          title="Click anywhere on map to set navigation goal"
        />

        {/* Bottom HUD bar */}
        <div className="absolute bottom-2 left-2 bg-white/90 backdrop-blur-md px-2.5 py-1 rounded-lg text-[11px] font-mono text-slate-700 border border-slate-200 shadow-sm flex items-center gap-3">
          <span className="text-indigo-600 font-semibold">
            Pose: ({robotX.toFixed(2)}m, {robotY.toFixed(2)}m)
          </span>
          <span className="text-slate-600">
            Yaw: {((robotTheta * 180) / Math.PI).toFixed(0)}°
          </span>
          <span className="text-slate-500">
            Mode: {mapMode === "north-up" ? "Fixed (0°)" : mapMode === "heading-up" ? "Heading" : `${customAngleDeg}°`}
          </span>
          {goal && (
            <span className="text-rose-600 font-semibold flex items-center gap-1">
              <Target className="w-3 h-3" /> Goal: ({goal.x}m, {goal.y}m)
            </span>
          )}
        </div>

        {/* Zoom, Rotate, and Center Controls */}
        <div className="absolute top-2 right-2 flex flex-col gap-1">
          <button
            onClick={() => setMapScale((s) => Math.min(100, s + 10))}
            className="p-1.5 bg-white/90 text-slate-700 rounded-lg hover:bg-slate-100 border border-slate-200 shadow-sm transition"
            title="Zoom in (+)"
          >
            <ZoomIn className="w-3.5 h-3.5" />
          </button>
          <button
            onClick={() => setMapScale((s) => Math.max(20, s - 10))}
            className="p-1.5 bg-white/90 text-slate-700 rounded-lg hover:bg-slate-100 border border-slate-200 shadow-sm transition"
            title="Zoom out (-)"
          >
            <ZoomOut className="w-3.5 h-3.5" />
          </button>
          <button
            onClick={() => {
              setMapMode("custom");
              setCustomAngleDeg((a) => (a + 90) % 360);
            }}
            className="p-1.5 bg-white/90 text-slate-700 rounded-lg hover:bg-slate-100 border border-slate-200 shadow-sm transition"
            title="Rotate view 90°"
          >
            <RotateCw className="w-3.5 h-3.5" />
          </button>
          <button
            onClick={() => setFollowRobot((f) => !f)}
            className={`p-1.5 rounded-lg border transition ${
              followRobot
                ? "bg-indigo-600 text-white border-indigo-500 shadow-sm"
                : "bg-white/90 text-slate-700 hover:bg-slate-100 border border-slate-200 shadow-sm"
            }`}
            title={followRobot ? "Disable auto-follow" : "Enable auto-follow"}
          >
            <Bot className="w-3.5 h-3.5" />
          </button>
          <button
            onClick={handleCenterOnRobot}
            className="p-1.5 bg-white/90 text-slate-700 rounded-lg hover:bg-slate-100 border border-slate-200 shadow-sm transition"
            title="Center on robot"
          >
            <Crosshair className="w-3.5 h-3.5 text-indigo-600" />
          </button>
          <button
            onClick={handleResetCenter}
            className="p-1.5 bg-white/90 text-slate-700 rounded-lg hover:bg-slate-100 border border-slate-200 shadow-sm transition"
            title="Reset center (0,0)"
          >
            <Crosshair className="w-3.5 h-3.5" />
          </button>
        </div>
      </div>
    </div>
  );
};
