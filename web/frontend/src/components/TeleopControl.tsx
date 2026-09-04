"use client";

import React, { useState, useEffect, useRef, useCallback } from "react";
import {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  ArrowUpLeft,
  ArrowUpRight,
  ArrowDownLeft,
  ArrowDownRight,
  RotateCcw,
  RotateCw,
  Octagon,
  Sliders,
  Keyboard,
  Compass,
} from "lucide-react";

interface TeleopControlProps {
  onSendCmdVel: (vx: number, vy: number, wz: number) => void;
  disabled?: boolean;
}

export function TeleopControl({ onSendCmdVel, disabled = false }: TeleopControlProps) {
  const [maxLinearSpeed, setMaxLinearSpeed] = useState<number>(0.35); // m/s
  const [maxAngularSpeed, setMaxAngularSpeed] = useState<number>(1.5); // rad/s

  // Joystick state
  const joystickAreaRef = useRef<HTMLDivElement | null>(null);
  const [joystickPos, setJoystickPos] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const activeKeysRef = useRef<Set<string>>(new Set());

  // Tham chiếu lệnh vận tốc hiện tại để stream liên tục (giữ sống watchdog 0.25s)
  const currentTwistRef = useRef<{ vx: number; vy: number; wz: number }>({ vx: 0, vy: 0, wz: 0 });
  const streamIntervalRef = useRef<NodeJS.Timeout | null>(null);

  // Bắt đầu phát luồng liên tục 12.5 Hz (mỗi 80ms)
  const startStreaming = useCallback(() => {
    if (streamIntervalRef.current) return;
    streamIntervalRef.current = setInterval(() => {
      const { vx, vy, wz } = currentTwistRef.current;
      onSendCmdVel(vx, vy, wz);
    }, 80);
  }, [onSendCmdVel]);

  // Dừng phát luồng và gửi lệnh dừng
  const handleStop = useCallback(() => {
    if (streamIntervalRef.current) {
      clearInterval(streamIntervalRef.current);
      streamIntervalRef.current = null;
    }
    currentTwistRef.current = { vx: 0, vy: 0, wz: 0 };
    setJoystickPos({ x: 0, y: 0 });
    setIsDragging(false);
    activeKeysRef.current.clear();
    onSendCmdVel(0, 0, 0);
  }, [onSendCmdVel]);

  // Thiết lập vận tốc và bắt đầu stream
  const setTwistAndStream = useCallback(
    (vx: number, vy: number, wz: number) => {
      if (disabled) return;
      currentTwistRef.current = { vx, vy, wz };
      onSendCmdVel(vx, vy, wz);
      startStreaming();
    },
    [disabled, onSendCmdVel, startStreaming]
  );

  // Dọn dẹp interval khi unmount
  useEffect(() => {
    return () => {
      if (streamIntervalRef.current) {
        clearInterval(streamIntervalRef.current);
      }
    };
  }, []);

  // Điều khiển bằng nút bấm hướng
  const handleDirectionDown = (vxFactor: number, vyFactor: number, wzFactor: number) => {
    const vx = vxFactor * maxLinearSpeed;
    const vy = vyFactor * maxLinearSpeed;
    const wz = wzFactor * maxAngularSpeed;
    setTwistAndStream(vx, vy, wz);
  };

  // Keyboard controls listener (W, A, S, D, Q, E, Space)
  useEffect(() => {
    if (disabled) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (["input", "textarea"].includes((e.target as HTMLElement)?.tagName?.toLowerCase())) {
        return;
      }

      const key = e.key.toLowerCase();
      if (["w", "a", "s", "d", "q", "e", " "].includes(key)) {
        e.preventDefault();
        activeKeysRef.current.add(key);

        if (activeKeysRef.current.has(" ")) {
          handleStop();
          return;
        }

        let vx = 0;
        let vy = 0;
        let wz = 0;

        if (activeKeysRef.current.has("w")) vx += maxLinearSpeed;
        if (activeKeysRef.current.has("s")) vx -= maxLinearSpeed;
        if (activeKeysRef.current.has("a")) vy += maxLinearSpeed; // Strafe trái
        if (activeKeysRef.current.has("d")) vy -= maxLinearSpeed; // Strafe phải
        if (activeKeysRef.current.has("q")) wz += maxAngularSpeed; // Quay trái
        if (activeKeysRef.current.has("e")) wz -= maxAngularSpeed; // Quay phải

        setTwistAndStream(vx, vy, wz);
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (activeKeysRef.current.has(key)) {
        activeKeysRef.current.delete(key);
        if (activeKeysRef.current.size === 0) {
          handleStop();
        } else {
          let vx = 0;
          let vy = 0;
          let wz = 0;
          if (activeKeysRef.current.has("w")) vx += maxLinearSpeed;
          if (activeKeysRef.current.has("s")) vx -= maxLinearSpeed;
          if (activeKeysRef.current.has("a")) vy += maxLinearSpeed;
          if (activeKeysRef.current.has("d")) vy -= maxLinearSpeed;
          if (activeKeysRef.current.has("q")) wz += maxAngularSpeed;
          if (activeKeysRef.current.has("e")) wz -= maxAngularSpeed;
          setTwistAndStream(vx, vy, wz);
        }
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, [disabled, maxLinearSpeed, maxAngularSpeed, setTwistAndStream, handleStop]);

  // Virtual Joystick handlers
  const updateJoystick = useCallback(
    (clientX: number, clientY: number) => {
      const area = joystickAreaRef.current;
      if (!area || disabled) return;
      const rect = area.getBoundingClientRect();
      const centerX = rect.width / 2;
      const centerY = rect.height / 2;
      const radius = rect.width / 2 - 15;

      let dx = clientX - rect.left - centerX;
      let dy = clientY - rect.top - centerY;
      const distance = Math.sqrt(dx * dx + dy * dy);

      if (distance > radius) {
        dx = (dx / distance) * radius;
        dy = (dy / distance) * radius;
      }

      setJoystickPos({ x: dx, y: dy });

      // Tọa độ chuẩn hóa [-1, 1]
      // dy âm là đẩy lên -> vx dương (tiến)
      const normVx = -(dy / radius) * maxLinearSpeed;
      // dx âm là đẩy sang trái -> vy dương (strafe trái)
      const normVy = -(dx / radius) * maxLinearSpeed;

      currentTwistRef.current = { vx: normVx, vy: normVy, wz: 0 };
      onSendCmdVel(normVx, normVy, 0);
    },
    [disabled, maxLinearSpeed, onSendCmdVel]
  );

  const handlePointerDown = (e: React.PointerEvent) => {
    setIsDragging(true);
    updateJoystick(e.clientX, e.clientY);
    startStreaming();
  };

  const handlePointerMove = (e: React.PointerEvent) => {
    if (!isDragging) return;
    updateJoystick(e.clientX, e.clientY);
  };

  const handlePointerUp = () => {
    handleStop();
  };

  return (
    <div className="flex flex-col gap-4 rounded-2xl border border-slate-200/80 bg-white p-5 shadow-sm dark:border-slate-800 dark:bg-slate-900/60 backdrop-blur-md">
      {/* Panel Header */}
      <div className="flex items-center justify-between border-b border-slate-100 pb-3 dark:border-slate-800">
        <div className="flex items-center gap-2">
          <Compass className="h-5 w-5 text-brand-600 dark:text-brand-400" />
          <h2 className="font-semibold text-slate-900 dark:text-white">Điều Khiển Teleop (Omni 3-DoF)</h2>
        </div>
        <div className="flex items-center gap-2 text-xs text-slate-500 dark:text-slate-400">
          <Keyboard className="h-3.5 w-3.5" />
          <span>W/A/S/D/Q/E hoặc Chuột/Cảm ứng</span>
        </div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-6 items-center">
        {/* Left: Direction Buttons Matrix (Omnidirectional) */}
        <div className="flex flex-col items-center justify-center">
          <div className="grid grid-cols-3 gap-2 w-56">
            {/* Row 1: Diagonals & Forward */}
            <button
              onPointerDown={() => handleDirectionDown(1, 1, 0)}
              onPointerUp={handleStop}
              onPointerLeave={handleStop}
              className="flex h-12 w-full items-center justify-center rounded-xl border border-slate-200 bg-slate-50 text-slate-700 hover:bg-brand-50 hover:text-brand-600 active:scale-95 transition dark:border-slate-700 dark:bg-slate-800 dark:text-slate-200"
              title="Tiến Chéo Trái"
            >
              <ArrowUpLeft className="h-5 w-5" />
            </button>
            <button
              onPointerDown={() => handleDirectionDown(1, 0, 0)}
              onPointerUp={handleStop}
              onPointerLeave={handleStop}
              className="flex h-12 w-full items-center justify-center rounded-xl border border-slate-200 bg-slate-50 font-bold text-slate-900 hover:bg-brand-50 hover:text-brand-600 active:scale-95 transition dark:border-slate-700 dark:bg-slate-800 dark:text-white"
              title="Tiến về trước (W)"
            >
              <ArrowUp className="h-5 w-5" />
            </button>
            <button
              onPointerDown={() => handleDirectionDown(1, -1, 0)}
              onPointerUp={handleStop}
              onPointerLeave={handleStop}
              className="flex h-12 w-full items-center justify-center rounded-xl border border-slate-200 bg-slate-50 text-slate-700 hover:bg-brand-50 hover:text-brand-600 active:scale-95 transition dark:border-slate-700 dark:bg-slate-800 dark:text-slate-200"
              title="Tiến Chéo Phải"
            >
              <ArrowUpRight className="h-5 w-5" />
            </button>

            {/* Row 2: Strafe Left, Stop, Strafe Right */}
            <button
              onPointerDown={() => handleDirectionDown(0, 1, 0)}
              onPointerUp={handleStop}
              onPointerLeave={handleStop}
              className="flex h-12 w-full items-center justify-center rounded-xl border border-slate-200 bg-slate-50 font-bold text-slate-900 hover:bg-brand-50 hover:text-brand-600 active:scale-95 transition dark:border-slate-700 dark:bg-slate-800 dark:text-white"
              title="Đi Ngang Trái (A)"
            >
              <ArrowLeft className="h-5 w-5" />
            </button>
            <button
              onClick={handleStop}
              className="flex h-12 w-full items-center justify-center rounded-xl bg-rose-500 font-bold text-white hover:bg-rose-600 active:scale-90 shadow-md shadow-rose-500/30 transition"
              title="Dừng Xe (Space)"
            >
              <Octagon className="h-5 w-5" />
            </button>
            <button
              onPointerDown={() => handleDirectionDown(0, -1, 0)}
              onPointerUp={handleStop}
              onPointerLeave={handleStop}
              className="flex h-12 w-full items-center justify-center rounded-xl border border-slate-200 bg-slate-50 font-bold text-slate-900 hover:bg-brand-50 hover:text-brand-600 active:scale-95 transition dark:border-slate-700 dark:bg-slate-800 dark:text-white"
              title="Đi Ngang Phải (D)"
            >
              <ArrowRight className="h-5 w-5" />
            </button>

            {/* Row 3: Diagonals & Backward */}
            <button
              onPointerDown={() => handleDirectionDown(-1, 1, 0)}
              onPointerUp={handleStop}
              onPointerLeave={handleStop}
              className="flex h-12 w-full items-center justify-center rounded-xl border border-slate-200 bg-slate-50 text-slate-700 hover:bg-brand-50 hover:text-brand-600 active:scale-95 transition dark:border-slate-700 dark:bg-slate-800 dark:text-slate-200"
              title="Lùi Chéo Trái"
            >
              <ArrowDownLeft className="h-5 w-5" />
            </button>
            <button
              onPointerDown={() => handleDirectionDown(-1, 0, 0)}
              onPointerUp={handleStop}
              onPointerLeave={handleStop}
              className="flex h-12 w-full items-center justify-center rounded-xl border border-slate-200 bg-slate-50 font-bold text-slate-900 hover:bg-brand-50 hover:text-brand-600 active:scale-95 transition dark:border-slate-700 dark:bg-slate-800 dark:text-white"
              title="Lùi lại (S)"
            >
              <ArrowDown className="h-5 w-5" />
            </button>
            <button
              onPointerDown={() => handleDirectionDown(-1, -1, 0)}
              onPointerUp={handleStop}
              onPointerLeave={handleStop}
              className="flex h-12 w-full items-center justify-center rounded-xl border border-slate-200 bg-slate-50 text-slate-700 hover:bg-brand-50 hover:text-brand-600 active:scale-95 transition dark:border-slate-700 dark:bg-slate-800 dark:text-slate-200"
              title="Lùi Chéo Phải"
            >
              <ArrowDownRight className="h-5 w-5" />
            </button>
          </div>

          {/* Turn Buttons (Yaw rotation) */}
          <div className="mt-3 flex gap-2 w-56">
            <button
              onPointerDown={() => handleDirectionDown(0, 0, 1)}
              onPointerUp={handleStop}
              onPointerLeave={handleStop}
              className="flex h-10 flex-1 items-center justify-center gap-1.5 rounded-xl border border-slate-200 bg-slate-100 font-semibold text-slate-800 hover:bg-cyan-50 hover:text-cyan-700 active:scale-95 transition dark:border-slate-700 dark:bg-slate-800 dark:text-slate-200"
              title="Quay Trái (Q)"
            >
              <RotateCcw className="h-4 w-4" />
              <span className="text-xs">Quay Trái (Q)</span>
            </button>
            <button
              onPointerDown={() => handleDirectionDown(0, 0, -1)}
              onPointerUp={handleStop}
              onPointerLeave={handleStop}
              className="flex h-10 flex-1 items-center justify-center gap-1.5 rounded-xl border border-slate-200 bg-slate-100 font-semibold text-slate-800 hover:bg-cyan-50 hover:text-cyan-700 active:scale-95 transition dark:border-slate-700 dark:bg-slate-800 dark:text-slate-200"
              title="Quay Phải (E)"
            >
              <RotateCw className="h-4 w-4" />
              <span className="text-xs">Quay Phải (E)</span>
            </button>
          </div>
        </div>

        {/* Right: Analog Virtual Joystick & Speed Sliders */}
        <div className="flex flex-col items-center justify-center">
          {/* Virtual Joystick Circle */}
          <div
            ref={joystickAreaRef}
            onPointerDown={handlePointerDown}
            onPointerMove={handlePointerMove}
            onPointerUp={handlePointerUp}
            onPointerCancel={handlePointerUp}
            className="relative flex h-40 w-40 touch-none items-center justify-center rounded-full border-2 border-dashed border-slate-300 bg-slate-50/80 shadow-inner dark:border-slate-700 dark:bg-slate-800/60 cursor-crosshair"
          >
            {/* Center crosshair */}
            <div className="absolute h-px w-full bg-slate-200 dark:bg-slate-700" />
            <div className="absolute h-full w-px bg-slate-200 dark:bg-slate-700" />

            {/* Draggable Knob */}
            <div
              style={{
                transform: `translate(${joystickPos.x}px, ${joystickPos.y}px)`,
                transition: isDragging ? "none" : "transform 0.15s ease-out",
              }}
              className="relative z-10 flex h-14 w-14 items-center justify-center rounded-full bg-brand-500 shadow-lg shadow-brand-500/40 cursor-grab active:cursor-grabbing border-2 border-white dark:border-slate-900"
            >
              <div className="h-4 w-4 rounded-full bg-white/80" />
            </div>
          </div>

          <span className="mt-2 text-xs text-slate-400">Virtual 2D Joystick (Kéo thả di chuyển)</span>

          {/* Speed Limit Sliders */}
          <div className="mt-4 w-full max-w-xs space-y-3">
            <div>
              <div className="flex justify-between text-xs text-slate-600 dark:text-slate-400 mb-1 font-medium">
                <span className="flex items-center gap-1">
                  <Sliders className="h-3 w-3" /> Tốc độ tịnh tiến tối đa:
                </span>
                <span className="font-mono text-brand-600 dark:text-brand-400">{maxLinearSpeed.toFixed(2)} m/s</span>
              </div>
              <input
                type="range"
                min="0.05"
                max="0.54"
                step="0.01"
                value={maxLinearSpeed}
                onChange={(e) => setMaxLinearSpeed(parseFloat(e.target.value))}
                className="w-full accent-brand-500 cursor-pointer"
              />
            </div>

            <div>
              <div className="flex justify-between text-xs text-slate-600 dark:text-slate-400 mb-1 font-medium">
                <span className="flex items-center gap-1">
                  <Sliders className="h-3 w-3" /> Tốc độ quay góc tối đa:
                </span>
                <span className="font-mono text-brand-600 dark:text-brand-400">{maxAngularSpeed.toFixed(2)} rad/s</span>
              </div>
              <input
                type="range"
                min="0.2"
                max="3.0"
                step="0.1"
                value={maxAngularSpeed}
                onChange={(e) => setMaxAngularSpeed(parseFloat(e.target.value))}
                className="w-full accent-brand-500 cursor-pointer"
              />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
