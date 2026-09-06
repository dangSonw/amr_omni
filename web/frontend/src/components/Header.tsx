"use client";

import React from "react";
import { RobotStatus } from "@/types/robot";

interface HeaderProps {
  status: RobotStatus | undefined;
}

export function Header({ status }: HeaderProps) {
  return (
    <header className="sticky top-0 z-30 flex items-center justify-between border-b border-slate-200 bg-white px-4 py-3 sm:px-6 shadow-sm">
      {/* Logo & Brand */}
      <div className="flex items-center gap-3">
        <div className="flex h-10 w-10 items-center justify-center rounded bg-gradient-to-tr from-brand-600 to-cyan-500 text-white font-bold tracking-wider text-sm">
          AMR
        </div>
        <div>
          <div className="flex items-center gap-2">
            <h1 className="text-lg font-bold text-slate-900 tracking-tight">
              AMR OMNI
            </h1>
            <span className="rounded-full bg-slate-100 px-2 py-0.5 text-xs font-semibold text-slate-600">
              v1.0
            </span>
            <span
              className={`rounded-full px-2.5 py-0.5 text-xs font-bold uppercase tracking-wider ${
                status?.mode === "hardware"
                  ? "bg-purple-100 text-purple-700"
                  : "bg-blue-100 text-blue-700"
              }`}
            >
              {status?.mode || "OFFLINE"}
            </span>
          </div>
        </div>
      </div>
    </header>
  );
}
