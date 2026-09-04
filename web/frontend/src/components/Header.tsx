"use client";

import { useEffect, useState } from "react";
import { RobotStatus } from "@/types/robot";
import { Activity, AlertOctagon, Cpu, HardDrive, Moon, Radio, Sun, ShieldAlert, ShieldCheck } from "lucide-react";

interface HeaderProps {
  status: RobotStatus | undefined;
  connected: boolean;
  pingMs: number;
  onToggleEstop: (active: boolean) => void;
}

export function Header({ status, connected, pingMs, onToggleEstop }: HeaderProps) {
  const [isDark, setIsDark] = useState(false);

  useEffect(() => {
    // Đọc theme từ localStorage hoặc system preference
    const saved = localStorage.getItem("amr_theme");
    const prefersDark = window.matchMedia("(prefers-color-scheme: dark)").matches;
    const shouldDark = saved === "dark" || (!saved && prefersDark);
    setIsDark(shouldDark);
    if (shouldDark) {
      document.documentElement.classList.add("dark");
    } else {
      document.documentElement.classList.remove("dark");
    }
  }, []);

  const toggleTheme = () => {
    const nextDark = !isDark;
    setIsDark(nextDark);
    if (nextDark) {
      document.documentElement.classList.add("dark");
      localStorage.setItem("amr_theme", "dark");
    } else {
      document.documentElement.classList.remove("dark");
      localStorage.setItem("amr_theme", "light");
    }
  };

  const isEstopActive = status?.estop_active ?? false;
  const isSafetyStop = status?.safety_stop ?? false;

  return (
    <header className="sticky top-0 z-30 flex flex-wrap items-center justify-between gap-4 border-b border-slate-200 bg-white/90 px-4 py-3 backdrop-blur-md dark:border-slate-800 dark:bg-slate-900/90 sm:px-6">
      {/* Logo & Brand */}
      <div className="flex items-center gap-3">
        <div className="flex h-10 w-10 items-center justify-center rounded-xl bg-gradient-to-tr from-brand-600 to-cyan-500 shadow-md shadow-brand-500/20 text-white font-bold tracking-wider text-sm">
          AMR
        </div>
        <div>
          <div className="flex items-center gap-2">
            <h1 className="text-lg font-bold text-slate-900 dark:text-white tracking-tight">
              AMR OMNI
            </h1>
            <span className="rounded-full bg-slate-100 px-2 py-0.5 text-xs font-semibold text-slate-600 dark:bg-slate-800 dark:text-slate-300">
              v1.0
            </span>
            <span
              className={`rounded-full px-2.5 py-0.5 text-xs font-bold uppercase tracking-wider ${
                status?.mode === "hardware"
                  ? "bg-purple-100 text-purple-700 dark:bg-purple-950/60 dark:text-purple-300"
                  : status?.mode === "simulation"
                  ? "bg-blue-100 text-blue-700 dark:bg-blue-950/60 dark:text-blue-300"
                  : "bg-emerald-100 text-emerald-700 dark:bg-emerald-950/60 dark:text-emerald-300"
              }`}
            >
              {status?.mode || "MOCK"}
            </span>
          </div>
          <p className="text-xs text-slate-500 dark:text-slate-400">
            Giám sát luồng dữ liệu & Điều khiển xe 4 bánh Omni (Holonomic)
          </p>
        </div>
      </div>

      {/* Metrics & Connection status */}
      <div className="flex flex-wrap items-center gap-2 sm:gap-4 text-xs font-medium">
        {/* Link Status */}
        <div className="flex items-center gap-2 rounded-lg border border-slate-200 bg-slate-50 px-3 py-1.5 dark:border-slate-800 dark:bg-slate-800/60">
          <Radio
            className={`h-4 w-4 ${
              connected ? "animate-pulse text-emerald-500" : "text-rose-500"
            }`}
          />
          <span className="text-slate-700 dark:text-slate-200">
            {connected ? "KẾT NỐI (LIVE)" : "MẤT KẾT NỐI"}
          </span>
          {connected && (
            <span className="rounded bg-slate-200/60 px-1.5 py-0.2 font-mono text-[10px] text-slate-600 dark:bg-slate-700 dark:text-slate-300">
              {pingMs}ms
            </span>
          )}
        </div>

        {/* Safety State */}
        <div
          className={`flex items-center gap-1.5 rounded-lg border px-3 py-1.5 ${
            isSafetyStop || isEstopActive
              ? "border-amber-300 bg-amber-50 text-amber-800 dark:border-amber-900/60 dark:bg-amber-950/40 dark:text-amber-300"
              : "border-emerald-200 bg-emerald-50 text-emerald-800 dark:border-emerald-900/60 dark:bg-emerald-950/40 dark:text-emerald-300"
          }`}
        >
          {isSafetyStop || isEstopActive ? (
            <ShieldAlert className="h-4 w-4 text-amber-500" />
          ) : (
            <ShieldCheck className="h-4 w-4 text-emerald-500" />
          )}
          <span>
            {isEstopActive
              ? "E-STOP KÍCH HOẠT"
              : isSafetyStop
              ? "WATCHDOG STOP"
              : "WATCHDOG AN TOÀN"}
          </span>
        </div>

        {/* ROS 2 Node info & streams */}
        <div className="hidden lg:flex items-center gap-2 font-mono text-xs">
          <div className="flex items-center gap-1.5 rounded-lg border border-slate-200 bg-slate-50 px-2.5 py-1.5 dark:border-slate-800 dark:bg-slate-800/60 text-slate-700 dark:text-slate-300">
            <span className="text-[10px] text-slate-400 font-sans uppercase">Node</span>
            <span className="font-bold">amr_web_bridge</span>
          </div>
          <div className="flex items-center gap-1.5 rounded-lg border border-slate-200 bg-slate-50 px-2.5 py-1.5 dark:border-slate-800 dark:bg-slate-800/60 text-slate-700 dark:text-slate-300">
            <Activity className="h-3.5 w-3.5 text-cyan-500" />
            <span>{status?.active_streams ?? 0} luồng DDS</span>
          </div>
        </div>

        {/* Theme Toggle Button */}
        <button
          onClick={toggleTheme}
          aria-label="Chuyển đổi giao diện sáng/tối"
          title={isDark ? "Chuyển sang giao diện Sáng" : "Chuyển sang giao diện Tối"}
          className="flex h-9 w-9 items-center justify-center rounded-lg border border-slate-200 bg-white text-slate-700 shadow-sm transition hover:bg-slate-100 hover:text-slate-900 dark:border-slate-800 dark:bg-slate-800 dark:text-slate-300 dark:hover:bg-slate-700 dark:hover:text-white"
        >
          {isDark ? <Sun className="h-4 w-4 text-amber-400" /> : <Moon className="h-4 w-4 text-slate-600" />}
        </button>

        {/* EMERGENCY STOP BUTTON */}
        <button
          onClick={() => onToggleEstop(!isEstopActive)}
          className={`flex items-center gap-2 rounded-xl px-4 py-2 font-bold shadow-md transition-all active:scale-95 ${
            isEstopActive
              ? "bg-rose-600 text-white hover:bg-rose-700 shadow-rose-600/30 animate-pulse ring-4 ring-rose-300 dark:ring-rose-900"
              : "bg-slate-900 text-white hover:bg-rose-600 dark:bg-rose-700 dark:hover:bg-rose-600"
          }`}
        >
          <AlertOctagon className="h-4 w-4" />
          <span>{isEstopActive ? "MỞ KHÓA E-STOP" : "DỪNG KHẨN CẤP (E-STOP)"}</span>
        </button>
      </div>
    </header>
  );
}

