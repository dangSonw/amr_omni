"use client";

import React, { useRef, useEffect, useState, useCallback } from "react";
import {
  Navigation,
  Target,
  Save,
  CheckCircle2,
  XCircle,
  ZoomIn,
  ZoomOut,
  Compass,
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
  const persistentObstaclesRef = useRef<Map<string, [number, number]>>(new Map());

  const [goal, setGoal] = useState<{ x: number; y: number } | null>(null);
  const [navState, setNavState] = useState<string>("idle");
  const [distanceRemaining, setDistanceRemaining] = useState<number | null>(null);
  const [saveStatus, setSaveStatus] = useState<string | null>(null);
  const [clearStatus, setClearStatus] = useState<string | null>(null);
  const [mapScale, setMapScale] = useState<number>(45); // px/m
  const [offset, setOffset] = useState<{ x: number; y: number }>({ x: 0, y: 0 });

  const [mapMode, setMapMode] = useState<"north-up" | "heading-up">("north-up");
  const [followRobot, setFollowRobot] = useState<boolean>(false);

  const robotX = odom?.x ?? 0.0;
  const robotY = odom?.y ?? 0.0;
  const robotTheta = odom?.theta_rad ?? 0.0;

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

  useEffect(() => {
    if (followRobot) {
      setOffset({
        x: -robotX * mapScale,
        y: robotY * mapScale,
      });
    }
  }, [followRobot, robotX, robotY, mapScale]);

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

  const handleClearMap = async () => {
    persistentObstaclesRef.current.clear();
    setClearStatus("CLEARED");
    try {
      await fetch("/api/map/clear", { method: "POST" });
    } catch {
      // Ignored
    }
    setTimeout(() => setClearStatus(null), 1500);
  };

  const handleSaveMap = async () => {
    setSaveStatus("SAVING");
    try {
      await fetch("/api/map/save", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ map_name: `amr_map_${Date.now()}` }),
      });
      setSaveStatus("SAVED");
      setTimeout(() => setSaveStatus(null), 2000);
    } catch {
      setSaveStatus("FAILED");
      setTimeout(() => setSaveStatus(null), 2000);
    }
  };

  const viewAngleRad = mapMode === "heading-up" ? robotTheta - Math.PI / 2 : 0.0;

  // Traversed trajectory history to form explored free space corridor
  const trajectoryRef = useRef<[number, number][]>([]);

  useEffect(() => {
    const last = trajectoryRef.current[trajectoryRef.current.length - 1];
    if (!last || Math.hypot(robotX - last[0], robotY - last[1]) > 0.08) {
      trajectoryRef.current.push([robotX, robotY]);
      if (trajectoryRef.current.length > 500) {
        trajectoryRef.current.shift();
      }
    }
  }, [robotX, robotY]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const width = canvas.width;
    const height = canvas.height;
    const cx = width / 2 + offset.x;
    const cy = height / 2 + offset.y;

    // 1. Unexplored / Unknown Space: Warm light gray background per web theme
    ctx.fillStyle = "#e2ddd5";
    ctx.fillRect(0, 0, width, height);

    ctx.save();

    if (viewAngleRad !== 0) {
      ctx.translate(cx, cy);
      ctx.rotate(-viewAngleRad);
      ctx.translate(-cx, -cy);
    }

    // Technical grid (1m lines) in subtle warm tone
    ctx.strokeStyle = "#d6d0c6";
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

    // Coordinate Axes (subtle)
    ctx.strokeStyle = "#b5afa6";
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(cx, -height);
    ctx.lineTo(cx, height * 2);
    ctx.moveTo(-width, cy);
    ctx.lineTo(width * 2, cy);
    ctx.stroke();

    // 2. Explored Free Space: Pure white corridors along traversed trajectory
    if (trajectoryRef.current.length > 0) {
      ctx.save();
      ctx.strokeStyle = "#ffffff";
      ctx.fillStyle = "#ffffff";
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.lineWidth = Math.max(16, 0.75 * mapScale);
      ctx.beginPath();
      trajectoryRef.current.forEach(([tx, ty], idx) => {
        const px = cx + tx * mapScale;
        const py = cy - ty * mapScale;
        if (idx === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.restore();
    }

    // Explored circular area around current robot position
    const currPx = cx + robotX * mapScale;
    const currPy = cy - robotY * mapScale;
    ctx.fillStyle = "#ffffff";
    ctx.beginPath();
    ctx.arc(currPx, currPy, 0.55 * mapScale, 0, Math.PI * 2);
    ctx.fill();

    // 3. Active LiDAR Scan Polygon: Free space clearing + soft tinted beam
    const cosR = Math.cos(robotTheta);
    const sinR = Math.sin(robotTheta);

    if (lidar?.points && lidar.points.length > 2) {
      // Calculate world coordinates and angles for all lidar points
      const scanPoints = lidar.points.map(([lx, ly]) => {
        const wx = robotX + (cosR * lx - sinR * ly);
        const wy = robotY + (sinR * lx + cosR * ly);
        const px = cx + wx * mapScale;
        const py = cy - wy * mapScale;
        const angle = Math.atan2(wy - robotY, wx - robotX);
        return { px, py, angle };
      });

      // Sort points angularly to create a continuous polygon
      scanPoints.sort((a, b) => a.angle - b.angle);

      // (a) Clear interior to pure white (free space explored by LiDAR)
      ctx.save();
      ctx.fillStyle = "#ffffff";
      ctx.beginPath();
      ctx.moveTo(currPx, currPy);
      for (const pt of scanPoints) {
        ctx.lineTo(pt.px, pt.py);
      }
      ctx.closePath();
      ctx.fill();

      // (b) Subtle soft tint for active LiDAR beam field-of-view (per user request)
      ctx.fillStyle = "rgba(111, 194, 255, 0.14)";
      ctx.strokeStyle = "rgba(111, 194, 255, 0.4)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(currPx, currPy);
      for (const pt of scanPoints) {
        ctx.lineTo(pt.px, pt.py);
      }
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    }

    // 4. Walls & Obstacles: Deep Black solid points & segments (matching reference photo)
    ctx.fillStyle = "#18181b";
    if (persistentObstaclesRef.current.size > 0) {
      persistentObstaclesRef.current.forEach(([ox, oy]) => {
        const px = cx + ox * mapScale;
        const py = cy - oy * mapScale;
        ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
      });
    }

    // Live LiDAR hits in solid black
    if (lidar?.points && lidar.points.length > 0) {
      for (const [lx, ly] of lidar.points) {
        const wx = robotX + (cosR * lx - sinR * ly);
        const wy = robotY + (sinR * lx + cosR * ly);
        const px = cx + wx * mapScale;
        const py = cy - wy * mapScale;
        ctx.fillRect(px - 2, py - 2, 4, 4);
      }
    }

    // 5. Sleek Global Path: Precision micro-dash with nodes
    const hasGlobalPath = paths?.global_path && paths.global_path.length > 1;
    if (hasGlobalPath) {
      ctx.save();
      ctx.strokeStyle = "#6fc2ff";
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      paths.global_path.forEach(([gx, gy], index) => {
        const px = cx + gx * mapScale;
        const py = cy - gy * mapScale;
        if (index === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();

      // Micro waypoint nodes
      ctx.fillStyle = "#6fc2ff";
      ctx.setLineDash([]);
      for (let i = 0; i < paths.global_path.length; i += 2) {
        const [gx, gy] = paths.global_path[i];
        const px = cx + gx * mapScale;
        const py = cy - gy * mapScale;
        ctx.beginPath();
        ctx.arc(px, py, 1.2, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.restore();
    } else if (goal) {
      ctx.save();
      ctx.strokeStyle = "#6fc2ff";
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(currPx, currPy);
      ctx.lineTo(cx + goal.x * mapScale, cy - goal.y * mapScale);
      ctx.stroke();
      ctx.restore();
    }

    // 6. Sleek Local Path: Emerald accent curve
    if (paths?.local_path && paths.local_path.length > 1) {
      ctx.save();
      ctx.strokeStyle = "#10b981";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      paths.local_path.forEach(([lx, ly], index) => {
        const px = cx + lx * mapScale;
        const py = cy - ly * mapScale;
        if (index === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.restore();
    }

    // 7. Sleek Goal Marker: Precision Reticle + Small Coordinate Tag
    if (goal) {
      const gx = cx + goal.x * mapScale;
      const gy = cy - goal.y * mapScale;

      ctx.save();
      // Outer radar pulse circle
      ctx.strokeStyle = "rgba(111, 194, 255, 0.45)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(gx, gy, 10, 0, Math.PI * 2);
      ctx.stroke();

      // Precision Reticle Ring
      ctx.strokeStyle = "#383838";
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.arc(gx, gy, 4.5, 0, Math.PI * 2);
      ctx.stroke();

      // 4 Crosshair Ticks (+)
      ctx.beginPath();
      ctx.moveTo(gx - 7, gy);
      ctx.lineTo(gx - 4.5, gy);
      ctx.moveTo(gx + 4.5, gy);
      ctx.lineTo(gx + 7, gy);
      ctx.moveTo(gx, gy - 7);
      ctx.lineTo(gx, gy - 4.5);
      ctx.moveTo(gx, gy + 4.5);
      ctx.lineTo(gx, gy + 7);
      ctx.stroke();

      // Center Focal Dot
      ctx.fillStyle = "#e11d48";
      ctx.beginPath();
      ctx.arc(gx, gy, 1.5, 0, Math.PI * 2);
      ctx.fill();

      // Refined Minimalist Coordinate Badge
      const text = `[${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}]`;
      ctx.font = "bold 9px monospace";
      const textWidth = ctx.measureText(text).width;
      const tagX = gx + 9;
      const tagY = gy - 9;

      // Shadow
      ctx.fillStyle = "#383838";
      ctx.fillRect(tagX - 1, tagY + 1, textWidth + 6, 13);
      // Box
      ctx.fillStyle = "#ffffff";
      ctx.strokeStyle = "#383838";
      ctx.lineWidth = 1;
      ctx.fillRect(tagX - 2, tagY, textWidth + 6, 13);
      ctx.strokeRect(tagX - 2, tagY, textWidth + 6, 13);
      // Text
      ctx.fillStyle = "#18181b";
      ctx.fillText(text, tagX + 1, tagY + 9.5);
      ctx.restore();
    }

    // 8. Robot Chassis: Omni / Mecanum 4-Wheel Top-down view
    const rx = currPx;
    const ry = currPy;
    const radius = 0.16 * mapScale;

    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-robotTheta);

    // Chassis Box
    ctx.fillStyle = "#ffffff";
    ctx.strokeStyle = "#383838";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.roundRect(-radius, -radius, radius * 2, radius * 2, 2);
    ctx.fill();
    ctx.stroke();

    // 4 Corner Mecanum Wheels (black)
    const wheelW = radius * 0.45;
    const wheelH = radius * 0.24;
    ctx.fillStyle = "#18181b";
    [
      [-radius, -radius],
      [radius - wheelW, -radius],
      [-radius, radius - wheelH],
      [radius - wheelW, radius - wheelH],
    ].forEach(([wx, wy]) => {
      ctx.fillRect(wx, wy, wheelW, wheelH);
    });

    // Center Badge
    ctx.fillStyle = "#6fc2ff";
    ctx.strokeStyle = "#383838";
    ctx.lineWidth = 1;
    ctx.fillRect(-radius * 0.35, -radius * 0.35, radius * 0.7, radius * 0.7);
    ctx.strokeRect(-radius * 0.35, -radius * 0.35, radius * 0.7, radius * 0.7);

    // Forward Directional Heading Arrow
    ctx.fillStyle = "#ff9538";
    ctx.strokeStyle = "#383838";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(radius + 7, 0);
    ctx.lineTo(radius - 1, -4);
    ctx.lineTo(radius - 1, 4);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();
    ctx.restore();

    // 9. Compass (top-left)
    ctx.save();
    const compassX = 28;
    const compassY = 28;
    const compassR = 14;

    ctx.fillStyle = "#ffffff";
    ctx.strokeStyle = "#383838";
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.arc(compassX, compassY, compassR, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.translate(compassX, compassY);
    ctx.rotate(-viewAngleRad);

    ctx.fillStyle = "#e11d48";
    ctx.beginPath();
    ctx.moveTo(0, -compassR + 2.5);
    ctx.lineTo(3, 0);
    ctx.lineTo(-3, 0);
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = "#383838";
    ctx.beginPath();
    ctx.moveTo(0, compassR - 2.5);
    ctx.lineTo(3, 0);
    ctx.lineTo(-3, 0);
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = "#383838";
    ctx.font = "bold 8px monospace";
    ctx.textAlign = "center";
    ctx.fillText("N", 0, -compassR - 2);

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

  return (
    <div className="border-2 border-charcoal bg-white rounded-[2px] shadow-[-4px_4px_0px_#383838] p-3 sm:p-4 flex flex-col h-full text-charcoal">
      {/* Header */}
      <div className="flex flex-wrap items-center justify-between gap-2 pb-2.5 border-b-2 border-charcoal mb-2.5">
        <div className="flex items-center gap-2">
          <Navigation className="w-4 h-4 text-charcoal" />
          <h3 className="font-bold text-xs sm:text-sm tracking-wider">
            MAP // 2D NAV
          </h3>
        </div>

        {/* Navigation State & Controls */}
        <div className="flex flex-wrap items-center gap-1.5 text-xs">
          {navState === "navigating" && goal ? (
            <div className="flex items-center gap-1.5 bg-white border border-charcoal px-2 py-0.5 font-bold text-xs shadow-[-2px_2px_0px_#383838]">
              <Target className="w-3.5 h-3.5 text-sky" />
              <span>
                [{goal.x.toFixed(2)}, {goal.y.toFixed(2)}]
                {distanceRemaining !== null && ` • ${distanceRemaining.toFixed(2)}m`}
              </span>
              <button
                onClick={handleCancelNav}
                className="ml-1 text-charcoal hover:text-rose-600 transition"
                title="Cancel Goal"
              >
                <XCircle className="w-3.5 h-3.5" />
              </button>
            </div>
          ) : navState === "reached" ? (
            <div className="flex items-center gap-1 bg-sketch-mint border border-charcoal px-2 py-0.5 font-bold text-xs">
              <CheckCircle2 className="w-3.5 h-3.5" />
              <span>REACHED</span>
            </div>
          ) : (
            <span className="border border-charcoal bg-chalk px-2 py-0.5 text-[10px] font-bold text-graphite hidden sm:inline">
              READY • {persistentObstaclesRef.current.size} PTS
            </span>
          )}

          {/* Orientation Toggle */}
          <button
            onClick={() =>
              setMapMode((m) => (m === "north-up" ? "heading-up" : "north-up"))
            }
            className={`border border-charcoal px-2 py-1 font-bold text-xs flex items-center gap-1 transition ${
              mapMode === "north-up"
                ? "bg-white hover:bg-ice shadow-[-2px_2px_0px_#383838]"
                : "bg-canary shadow-[-2px_2px_0px_#383838]"
            }`}
          >
            <Compass className="w-3.5 h-3.5" />
            <span>{mapMode === "north-up" ? "NORTH-UP" : "HEADING"}</span>
          </button>

          {/* Clear Map */}
          <button
            onClick={handleClearMap}
            className="border border-charcoal px-2 py-1 bg-white hover:bg-sketch-coral text-charcoal font-bold text-xs flex items-center gap-1 shadow-[-2px_2px_0px_#383838] transition"
          >
            <Trash2 className="w-3.5 h-3.5" />
            <span>{clearStatus || "CLEAR"}</span>
          </button>

          {/* Save Map */}
          <button
            onClick={handleSaveMap}
            className="border border-charcoal px-2 py-1 bg-sky hover:bg-sky-hover text-charcoal font-bold text-xs flex items-center gap-1 shadow-[-2px_2px_0px_#383838] transition"
          >
            <Save className="w-3.5 h-3.5" />
            <span>{saveStatus || "SAVE"}</span>
          </button>
        </div>
      </div>

      {/* Canvas Box */}
      <div className="relative flex-1 bg-[#e2ddd5] border-2 border-charcoal rounded-[2px] overflow-hidden flex items-center justify-center min-h-[420px]">
        <canvas
          ref={canvasRef}
          width={720}
          height={460}
          onClick={handleCanvasClick}
          className="w-full h-full cursor-crosshair object-cover"
        />

        {/* HUD bottom readout */}
        <div className="absolute bottom-2 left-2 border border-charcoal bg-white/95 px-2.5 py-1 text-[11px] font-mono text-charcoal shadow-[-2px_2px_0px_#383838] flex items-center gap-3">
          <span className="font-bold">
            POS: [{robotX.toFixed(2)}, {robotY.toFixed(2)}]m
          </span>
          <span>YAW: {((robotTheta * 180) / Math.PI).toFixed(0)}°</span>
          <span className="text-graphite">
            MODE: {mapMode === "north-up" ? "FIXED" : "HEADING"}
          </span>
          {goal && (
            <span className="font-bold text-rose-600 flex items-center gap-1">
              <Target className="w-3 h-3" /> [{goal.x}, {goal.y}]
            </span>
          )}
        </div>

        {/* Canvas Zoom & Follow Buttons */}
        <div className="absolute top-2 right-2 flex flex-col gap-1.5">
          <button
            onClick={() => setMapScale((s) => Math.min(100, s + 10))}
            className="p-1.5 border border-charcoal bg-white hover:bg-ice shadow-[-2px_2px_0px_#383838] transition"
            title="Zoom In"
          >
            <ZoomIn className="w-3.5 h-3.5" />
          </button>
          <button
            onClick={() => setMapScale((s) => Math.max(20, s - 10))}
            className="p-1.5 border border-charcoal bg-white hover:bg-ice shadow-[-2px_2px_0px_#383838] transition"
            title="Zoom Out"
          >
            <ZoomOut className="w-3.5 h-3.5" />
          </button>
          <button
            onClick={() => setFollowRobot((f) => !f)}
            className={`p-1.5 border border-charcoal shadow-[-2px_2px_0px_#383838] transition ${
              followRobot ? "bg-canary font-bold" : "bg-white hover:bg-ice"
            }`}
            title="Follow Robot"
          >
            <Bot className="w-3.5 h-3.5" />
          </button>
        </div>
      </div>
    </div>
  );
};
