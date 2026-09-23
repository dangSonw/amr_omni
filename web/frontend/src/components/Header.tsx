"use client";

import React from "react";
import { RobotStatus } from "@/types/robot";
import { Bot, Wifi } from "lucide-react";

interface HeaderProps {
  status: RobotStatus | undefined;
}

export function Header({ status }: HeaderProps) {
  const isHardware = status?.mode === "hardware";
  const isOnline = status?.connected ?? true;

  return (
    <header className="sticky top-0 z-30 flex items-center justify-between border-b-2 border-charcoal bg-white px-4 py-2.5 sm:px-6">
      {/* Brand & Mode */}
      <div className="flex items-center gap-3">
        <div className="flex h-8 w-8 items-center justify-center border-2 border-charcoal bg-duck-orange text-charcoal font-bold text-xs shadow-[-2px_2px_0px_#383838]">
          <Bot className="h-4 w-4" />
        </div>
        <div className="flex items-center gap-2">
          <span className="font-bold tracking-wider text-sm sm:text-base text-charcoal">
            AMR OMNI
          </span>
          <span className="border border-charcoal bg-chalk px-1.5 py-0.5 text-[10px] font-semibold text-charcoal">
            v1.0
          </span>
          <span
            className={`border border-charcoal px-2 py-0.5 text-[10px] font-bold uppercase tracking-wider ${
              isHardware
                ? "bg-sketch-mint text-charcoal"
                : "bg-canary text-charcoal"
            }`}
          >
            {status?.mode || "SIMULATION"}
          </span>
        </div>
      </div>

      {/* Connection & Telemetry Status */}
      <div className="flex items-center gap-2">
        <div className="flex items-center gap-1.5 border border-charcoal bg-white px-2 py-1 text-xs shadow-[-2px_2px_0px_#383838]">
          <span
            className={`h-2 w-2 rounded-[1px] border border-charcoal ${
              isOnline ? "bg-emerald-500 animate-pulse" : "bg-rose-500"
            }`}
          />
          <Wifi className="h-3.5 w-3.5 text-charcoal" />
          <span className="text-[11px] font-bold uppercase tracking-wide">
            {isOnline ? "ONLINE" : "DISCONNECTED"}
          </span>
        </div>
      </div>
    </header>
  );
}
