"use client";

import React from "react";
import { LayoutDashboard, Sliders, Activity } from "lucide-react";

export type NavTab = "cockpit" | "config" | "debug";

interface SidebarProps {
  activeTab: NavTab;
  onSelectTab: (tab: NavTab) => void;
}

export function Sidebar({ activeTab, onSelectTab }: SidebarProps) {
  const navItems = [
    {
      id: "cockpit" as NavTab,
      label: "COCKPIT",
      code: "01",
      icon: LayoutDashboard,
    },
    {
      id: "config" as NavTab,
      label: "CONFIG",
      code: "02",
      icon: Sliders,
    },
    {
      id: "debug" as NavTab,
      label: "MONITOR",
      code: "03",
      icon: Activity,
    },
  ];

  return (
    <aside className="w-full md:w-56 shrink-0 border-b-2 md:border-b-0 md:border-r-2 border-charcoal bg-white p-3 flex flex-col justify-between">
      <nav className="flex flex-row md:flex-col gap-2">
        {navItems.map((item) => {
          const Icon = item.icon;
          const isActive = activeTab === item.id;
          return (
            <button
              key={item.id}
              onClick={() => onSelectTab(item.id)}
              className={`flex-1 md:flex-none flex items-center justify-between px-3 py-2.5 rounded-[2px] border-2 border-charcoal text-left transition-all ${
                isActive
                  ? "bg-sky text-charcoal font-bold shadow-[-3px_3px_0px_#383838] translate-x-[-1px] translate-y-[-1px]"
                  : "bg-white text-charcoal hover:bg-ice font-medium"
              }`}
            >
              <div className="flex items-center gap-2.5">
                <Icon className="h-4 w-4 shrink-0 text-charcoal" />
                <span className="text-xs tracking-wider">{item.label}</span>
              </div>
              <span className="text-[10px] text-pencil font-bold hidden sm:inline">
                [{item.code}]
              </span>
            </button>
          );
        })}
      </nav>

      <div className="hidden md:block border-t border-charcoal pt-3 mt-4">
        <div className="border border-charcoal bg-chalk p-2 text-[10px] text-graphite space-y-1">
          <div className="font-bold text-charcoal flex justify-between">
            <span>NAV KEYS</span>
            <span className="text-duck-orange">[W/A/S/D]</span>
          </div>
          <div>ROT: Q / E • STOP: SPACE</div>
        </div>
      </div>
    </aside>
  );
}
