"use client";

import { useEffect, useRef, useState, useCallback } from "react";
import { LidarTelemetry, OdometryTelemetry } from "@/types/robot";
import { Compass, Maximize2, RotateCcw, ZoomIn, ZoomOut, Layers, Eye } from "lucide-react";

interface LidarViewerProps {
  lidar: LidarTelemetry | undefined;
  odom: OdometryTelemetry | undefined;
}

export function LidarViewer({ lidar, odom }: LidarViewerProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  // Viewport transforms (pan & zoom)
  const [zoom, setZoom] = useState<number>(45); // pixels per meter
  const [pan, setPan] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const dragStartRef = useRef<{ x: number; y: number }>({ x: 0, y: 0 });

  // Display toggles
  const [showRays, setShowRays] = useState(false);
  const [showGrid, setShowGrid] = useState(true);
  const [accumulateMap, setAccumulateMap] = useState(true);

  // Accumulated obstacle point cloud (Local Map)
  const accumulatedPointsRef = useRef<Array<[number, number]>>([]);

  // Reset view
  const handleResetView = () => {
    setZoom(45);
    setPan({ x: 0, y: 0 });
  };

  const handleClearMap = () => {
    accumulatedPointsRef.current = [];
  };

  // Drag pan handlers
  const handleMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    setIsDragging(true);
    dragStartRef.current = { x: e.clientX - pan.x, y: e.clientY - pan.y };
  };

  const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!isDragging) return;
    setPan({
      x: e.clientX - dragStartRef.current.x,
      y: e.clientY - dragStartRef.current.y,
    });
  };

  const handleMouseUp = () => setIsDragging(false);

  const handleWheel = (e: React.WheelEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    const factor = e.deltaY < 0 ? 1.15 : 0.87;
    setZoom((prev) => Math.max(15, Math.min(180, prev * factor)));
  };

  // Tích lũy điểm vào bản đồ khi xe di chuyển
  useEffect(() => {
    if (!accumulateMap || !lidar || !odom || !lidar.points) return;
    const cos_th = Math.cos(odom.theta_rad);
    const sin_th = Math.sin(odom.theta_rad);

    // Chuyển đổi các điểm scan từ hệ robot sang hệ bản đồ thế giới (world frame)
    const newWorldPoints: Array<[number, number]> = [];
    for (const [px, py] of lidar.points) {
      const wx = odom.x + (cos_th * px - sin_th * py);
      const wy = odom.y + (sin_th * px + cos_th * py);
      newWorldPoints.push([wx, wy]);
    }

    // Giới hạn buffer điểm bản đồ tích lũy (~3000 điểm)
    const buffer = accumulatedPointsRef.current;
    accumulatedPointsRef.current = [...buffer, ...newWorldPoints].slice(-3200);
  }, [lidar, odom, accumulateMap]);

  // Main Canvas Render Loop
  const render = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const width = canvas.width;
    const height = canvas.height;
    const isDark = document.documentElement.classList.contains("dark");

    // Background color
    ctx.fillStyle = isDark ? "#090d16" : "#ffffff";
    ctx.fillRect(0, 0, width, height);

    // Gốc tọa độ hiển thị (trung tâm canvas + pan)
    const centerX = width / 2 + pan.x;
    const centerY = height / 2 + pan.y;

    // Vị trí robot trong hệ vẽ
    const robotWorldX = odom?.x ?? 0;
    const robotWorldY = odom?.y ?? 0;
    const robotTheta = odom?.theta_rad ?? 0;

    // Chuyển đổi tọa độ thế giới sang tọa độ Canvas (y hướng lên trong thế giới, hướng xuống trên màn hình)
    const worldToCanvas = (wx: number, wy: number) => {
      return {
        cx: centerX + wx * zoom,
        cy: centerY - wy * zoom,
      };
    };

    const robotCanvas = worldToCanvas(robotWorldX, robotWorldY);

    // 1. Vẽ lưới tọa độ Grid
    if (showGrid) {
      ctx.lineWidth = 1;
      ctx.strokeStyle = isDark ? "rgba(51, 65, 85, 0.35)" : "rgba(226, 232, 240, 0.9)";
      const gridSize = zoom; // 1 mét 1 ô

      const startX = centerX % gridSize;
      for (let x = startX; x < width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
      }

      const startY = centerY % gridSize;
      for (let y = startY; y < height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
      }

      // Trục tọa độ chính (X, Y world)
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = isDark ? "rgba(100, 116, 139, 0.5)" : "rgba(148, 163, 184, 0.7)";
      ctx.beginPath();
      ctx.moveTo(0, centerY);
      ctx.lineTo(width, centerY);
      ctx.moveTo(centerX, 0);
      ctx.lineTo(centerX, height);
      ctx.stroke();
    }

    // 2. Vẽ vòng tròn khoảng cách Radar xung quanh Robot (0.5m, 1m, 2m, 3m, 5m)
    const rings = [0.5, 1.0, 2.0, 3.0, 5.0];
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.font = "10px monospace";

    for (const r of rings) {
      const radiusPx = r * zoom;
      ctx.beginPath();
      ctx.arc(robotCanvas.cx, robotCanvas.cy, radiusPx, 0, Math.PI * 2);
      ctx.strokeStyle = isDark ? "rgba(56, 189, 248, 0.15)" : "rgba(14, 165, 233, 0.2)";
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.stroke();
      ctx.setLineDash([]);

      // Nhãn khoảng cách
      ctx.fillStyle = isDark ? "rgba(148, 163, 184, 0.7)" : "rgba(100, 116, 139, 0.8)";
      ctx.fillText(`${r}m`, robotCanvas.cx + radiusPx, robotCanvas.cy - 6);
    }

    // Vùng cảnh báo nguy hiểm gần xe (Safety Warning Ring < 0.4m)
    ctx.beginPath();
    ctx.arc(robotCanvas.cx, robotCanvas.cy, 0.4 * zoom, 0, Math.PI * 2);
    ctx.fillStyle = isDark ? "rgba(244, 63, 94, 0.08)" : "rgba(244, 63, 94, 0.1)";
    ctx.fill();
    ctx.strokeStyle = "rgba(244, 63, 94, 0.4)";
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // 3. Vẽ bản đồ chướng ngại vật tích lũy (Accumulated 2D Map Points)
    if (accumulateMap && accumulatedPointsRef.current.length > 0) {
      ctx.fillStyle = isDark ? "rgba(148, 163, 184, 0.45)" : "rgba(71, 85, 105, 0.45)";
      for (const [wx, wy] of accumulatedPointsRef.current) {
        const pt = worldToCanvas(wx, wy);
        ctx.fillRect(pt.cx - 1, pt.cy - 1, 2.5, 2.5);
      }
    }

    // 4. Vẽ chùm quét LiDAR hiện thời (Live Scan Rays / Points)
    if (lidar && lidar.points && lidar.points.length > 0) {
      const cos_th = Math.cos(robotTheta);
      const sin_th = Math.sin(robotTheta);

      for (let i = 0; i < lidar.points.length; i++) {
        const [px, py] = lidar.points[i];
        const range = lidar.ranges[i] || Math.sqrt(px * px + py * py);

        // Chuyển sang tọa độ thế giới
        const wx = robotWorldX + (cos_th * px - sin_th * py);
        const wy = robotWorldY + (sin_th * px + cos_th * py);
        const pt = worldToCanvas(wx, wy);

        // Vẽ tia quét laser nếu bật
        if (showRays && i % 4 === 0) {
          ctx.beginPath();
          ctx.moveTo(robotCanvas.cx, robotCanvas.cy);
          ctx.lineTo(pt.cx, pt.cy);
          ctx.strokeStyle = isDark ? "rgba(6, 182, 212, 0.08)" : "rgba(14, 165, 233, 0.12)";
          ctx.lineWidth = 0.8;
          ctx.stroke();
        }

        // Đổi màu điểm phản xạ theo cự ly an toàn
        if (range < 0.6) {
          ctx.fillStyle = "#ef4444"; // Đỏ nguy hiểm
        } else if (range < 1.5) {
          ctx.fillStyle = "#f59e0b"; // Vàng cảnh báo
        } else {
          ctx.fillStyle = isDark ? "#38bdf8" : "#0284c7"; // Xanh an toàn
        }

        ctx.beginPath();
        ctx.arc(pt.cx, pt.cy, 2.5, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    // 5. Vẽ Robot Footprint và Heading Arrow
    // Kích thước robot: chiều dài ~0.35m, chiều rộng ~0.30m
    const robotLengthPx = 0.35 * zoom;
    const robotWidthPx = 0.30 * zoom;

    ctx.save();
    ctx.translate(robotCanvas.cx, robotCanvas.cy);
    ctx.rotate(-robotTheta); // Canvas xoay ngược chiều kim đồng hồ

    // Thân robot Mecanum 4 bánh
    ctx.fillStyle = isDark ? "#1e293b" : "#e2e8f0";
    ctx.strokeStyle = isDark ? "#38bdf8" : "#0284c7";
    ctx.lineWidth = 2;

    // Khung xe bo góc
    ctx.beginPath();
    ctx.roundRect(-robotLengthPx / 2, -robotWidthPx / 2, robotLengthPx, robotWidthPx, 6);
    ctx.fill();
    ctx.stroke();

    // 4 Bánh xe Omni đặt chéo 45 độ (FR, FL, RL, RR) theo đúng mô hình 3D wheels.xacro
    // Toạ độ thực: x = +-0.0656m, y = +-0.0656m, bán kính r = 0.03m
    const halfL = 0.0656 * zoom;
    const halfW = 0.0656 * zoom;
    const omniRadius = 0.035 * zoom;
    const omniThickness = 0.02 * zoom;

    const omniWheels = [
      { name: "FR", x: halfL, y: halfW, angle: -Math.PI / 4 },     // Front-Right (y âm trong ROS -> canvas y dương)
      { name: "FL", x: halfL, y: -halfW, angle: Math.PI / 4 },    // Front-Left  (y dương trong ROS -> canvas y âm)
      { name: "RL", x: -halfL, y: -halfW, angle: 3 * Math.PI / 4 }, // Rear-Left
      { name: "RR", x: -halfL, y: halfW, angle: -3 * Math.PI / 4 }, // Rear-Right
    ];

    ctx.fillStyle = isDark ? "#334155" : "#64748b";
    ctx.strokeStyle = isDark ? "#38bdf8" : "#0284c7";
    ctx.lineWidth = 1.5;

    for (const w of omniWheels) {
      ctx.save();
      ctx.translate(w.x, w.y);
      ctx.rotate(w.angle);
      // Vành bánh xe
      ctx.beginPath();
      ctx.roundRect(-omniRadius, -omniThickness / 2, omniRadius * 2, omniThickness, 3);
      ctx.fill();
      ctx.stroke();
      // Con lăn phụ (sub-rollers) của bánh Omni
      ctx.fillStyle = isDark ? "#94a3b8" : "#cbd5e1";
      ctx.fillRect(-omniRadius * 0.5, -omniThickness / 2, omniRadius, omniThickness);
      ctx.restore();
    }

    // Mũi tên chỉ hướng Heading Arrow (Trục X phía trước)
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(robotLengthPx / 2 + 10, 0);
    ctx.lineTo(robotLengthPx / 2 + 4, -5);
    ctx.moveTo(robotLengthPx / 2 + 10, 0);
    ctx.lineTo(robotLengthPx / 2 + 4, 5);
    ctx.strokeStyle = "#ef4444";
    ctx.lineWidth = 2.5;
    ctx.stroke();

    // Tâm robot
    ctx.beginPath();
    ctx.arc(0, 0, 3, 0, Math.PI * 2);
    ctx.fillStyle = "#ef4444";
    ctx.fill();

    ctx.restore();
  }, [lidar, odom, zoom, pan, showRays, showGrid, accumulateMap]);

  // Handle Canvas Resize
  useEffect(() => {
    const handleResize = () => {
      const canvas = canvasRef.current;
      if (canvas && canvas.parentElement) {
        canvas.width = canvas.parentElement.clientWidth;
        canvas.height = canvas.parentElement.clientHeight;
        render();
      }
    };
    handleResize();
    window.addEventListener("resize", handleResize);
    return () => window.removeEventListener("resize", handleResize);
  }, [render]);

  useEffect(() => {
    render();
  }, [render]);

  const minRange = lidar?.ranges && lidar.ranges.length > 0 ? Math.min(...lidar.ranges) : 0;

  return (
    <div className="relative flex flex-col h-full w-full overflow-hidden rounded-2xl border border-slate-200 bg-white shadow-sm dark:border-slate-800 dark:bg-slate-900">
      {/* Canvas Header */}
      <div className="flex items-center justify-between border-b border-slate-200 px-4 py-2.5 dark:border-slate-800">
        <div className="flex items-center gap-2">
          <Compass className="h-4 w-4 text-brand-600 dark:text-brand-500" />
          <h2 className="text-sm font-bold text-slate-900 dark:text-white">
            Bản Đồ 2D & Chùm Quét LiDAR
          </h2>
          <span className="rounded-md bg-cyan-50 px-2 py-0.5 text-[11px] font-semibold text-cyan-700 dark:bg-cyan-950/60 dark:text-cyan-300">
            {lidar?.points?.length ?? 0} tia quét
          </span>
        </div>

        {/* Quick controls */}
        <div className="flex items-center gap-1">
          <button
            onClick={() => setShowRays(!showRays)}
            title={showRays ? "Ẩn tia quét" : "Hiện tia quét"}
            className={`rounded-lg p-1.5 transition ${
              showRays
                ? "bg-brand-50 text-brand-600 dark:bg-brand-950/60 dark:text-brand-400"
                : "text-slate-500 hover:bg-slate-100 dark:hover:bg-slate-800"
            }`}
          >
            <Eye className="h-4 w-4" />
          </button>
          <button
            onClick={() => setShowGrid(!showGrid)}
            title="Bật/tắt lưới tọa độ"
            className={`rounded-lg p-1.5 transition ${
              showGrid
                ? "bg-brand-50 text-brand-600 dark:bg-brand-950/60 dark:text-brand-400"
                : "text-slate-500 hover:bg-slate-100 dark:hover:bg-slate-800"
            }`}
          >
            <Maximize2 className="h-4 w-4" />
          </button>
          <button
            onClick={() => setAccumulateMap(!accumulateMap)}
            title={accumulateMap ? "Dừng tích lũy bản đồ" : "Tích lũy vệt quét làm bản đồ"}
            className={`rounded-lg p-1.5 transition ${
              accumulateMap
                ? "bg-emerald-50 text-emerald-600 dark:bg-emerald-950/60 dark:text-emerald-400"
                : "text-slate-500 hover:bg-slate-100 dark:hover:bg-slate-800"
            }`}
          >
            <Layers className="h-4 w-4" />
          </button>
          <button
            onClick={() => setZoom((z) => Math.min(180, z * 1.2))}
            title="Phóng to"
            className="rounded-lg p-1.5 text-slate-500 hover:bg-slate-100 dark:hover:bg-slate-800"
          >
            <ZoomIn className="h-4 w-4" />
          </button>
          <button
            onClick={() => setZoom((z) => Math.max(15, z / 1.2))}
            title="Thu nhỏ"
            className="rounded-lg p-1.5 text-slate-500 hover:bg-slate-100 dark:hover:bg-slate-800"
          >
            <ZoomOut className="h-4 w-4" />
          </button>
          <button
            onClick={handleResetView}
            title="Đặt lại góc nhìn"
            className="rounded-lg p-1.5 text-slate-500 hover:bg-slate-100 dark:hover:bg-slate-800"
          >
            <RotateCcw className="h-4 w-4" />
          </button>
        </div>
      </div>

      {/* Main Interactive Canvas */}
      <div className="relative flex-1 w-full min-h-[360px] cursor-grab active:cursor-grabbing">
        <canvas
          ref={canvasRef}
          onMouseDown={handleMouseDown}
          onMouseMove={handleMouseMove}
          onMouseUp={handleMouseUp}
          onMouseLeave={handleMouseUp}
          onWheel={handleWheel}
          className="absolute inset-0 h-full w-full touch-none"
        />

        {/* Floating Telemetry HUD */}
        <div className="absolute bottom-3 left-3 flex flex-wrap items-center gap-2 rounded-xl border border-slate-200/80 bg-white/90 p-2 text-xs font-mono backdrop-blur-md shadow-sm dark:border-slate-800/80 dark:bg-slate-900/90 text-slate-700 dark:text-slate-300">
          <div>
            Pose: <span className="font-bold text-brand-600 dark:text-brand-400">X={odom?.x?.toFixed(2) ?? 0}m, Y={odom?.y?.toFixed(2) ?? 0}m</span>
          </div>
          <div className="hidden sm:inline">|</div>
          <div>
            Yaw: <span className="font-bold">{((odom?.theta_rad ?? 0) * (180 / Math.PI)).toFixed(1)}°</span>
          </div>
          <div className="hidden sm:inline">|</div>
          <div>
            Vật cản gần nhất:{" "}
            <span
              className={`font-bold ${
                minRange < 0.6
                  ? "text-rose-500"
                  : minRange < 1.2
                  ? "text-amber-500"
                  : "text-emerald-500"
              }`}
            >
              {minRange.toFixed(2)}m
            </span>
          </div>
          {accumulateMap && (
            <>
              <div className="hidden sm:inline">|</div>
              <button
                onClick={handleClearMap}
                className="text-[10px] text-slate-500 underline hover:text-rose-500"
              >
                Xóa vết bản đồ
              </button>
            </>
          )}
        </div>
      </div>
    </div>
  );
}

