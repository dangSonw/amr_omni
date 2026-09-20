"use client";

import React from "react";
import { LayoutDashboard, Sliders, Activity, Compass } from "lucide-react";

export type NavTab = "cockpit" | "calib" | "config" | "debug";

interface SidebarProps {
  activeTab: NavTab;
  onSelectTab: (tab: NavTab) => void;
}

export function Sidebar({ activeTab, onSelectTab }: SidebarProps) {
  const navItems = [
    {
      id: "cockpit" as NavTab,
      label: "Control Cockpit",
      sublabel: "Map & Telemetry",
      icon: LayoutDashboard,
    },
    {
      id: "calib" as NavTab,
      label: "Calibration",
      sublabel: "IMU & Kinematics",
      icon: Compass,
    },
    {
      id: "config" as NavTab,
      label: "System Config",
      sublabel: "Parameters & Limits",
      icon: Sliders,
    },
    {
      id: "debug" as NavTab,
      label: "Debug Monitor",
      sublabel: "Charts & Diagnostics",
      icon: Activity,
    },
  ];

  return (
    <aside className="w-full md:w-60 shrink-0 border-r border-slate-200 bg-white p-3 sm:p-4 flex flex-col justify-between">
      <nav className="space-y-1.5">
        {navItems.map((item) => {
          const Icon = item.icon;
          const isActive = activeTab === item.id;
          return (
            <button
              key={item.id}
              onClick={() => onSelectTab(item.id)}
              className={`w-full flex items-center gap-3 px-3.5 py-3 rounded-xl text-left transition-all duration-150 ${
                isActive
                  ? "bg-blue-600 text-white font-semibold shadow-sm"
                  : "text-slate-600 hover:text-slate-900 hover:bg-slate-100 font-medium"
              }`}
            >
              <div
                className={`flex h-9 w-9 items-center justify-center rounded-lg ${
                  isActive ? "bg-white/20 text-white" : "bg-slate-100 text-slate-500"
                }`}
              >
                <Icon className="h-5 w-5" />
              </div>
              <div>
                <div className="text-sm leading-snug">{item.label}</div>
                <div
                  className={`text-[11px] ${
                    isActive ? "text-white/80" : "text-slate-400"
                  }`}
                >
                  {item.sublabel}
                </div>
              </div>
            </button>
          );
        })}
      </nav>
    </aside>
  );
}
