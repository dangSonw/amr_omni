"use client";

import React, { useState } from "react";
import { Camera, RefreshCw, Eye, EyeOff } from "lucide-react";

interface CameraFeedProps {
  streamUrl?: string;
}

export const CameraFeed: React.FC<CameraFeedProps> = () => {
  const [cameraType, setCameraType] = useState<"rgb" | "depth">("rgb");
  const [isPlaying, setIsPlaying] = useState(true);
  const [reloadKey, setReloadKey] = useState(0);

  const handleRefresh = () => {
    setReloadKey((prev) => prev + 1);
  };

  const streamSrc = `/api/camera/stream?type=${cameraType}&t=${reloadKey}`;

  return (
    <div className="bg-white border border-slate-200 rounded p-3 sm:p-4 flex flex-col h-full text-slate-800 overflow-hidden">
      <div className="flex items-center justify-between pb-2.5 border-b border-slate-200 mb-2.5">
        <div className="flex items-center gap-2">
          <Camera className={`w-4 h-4 ${cameraType === "rgb" ? "text-emerald-600" : "text-cyan-600"}`} />
          <h3 className="font-bold text-slate-900 text-xs sm:text-sm tracking-wide">
            {cameraType === "rgb" ? "RGB Camera" : "Depth Camera (Heatmap)"}
          </h3>
        </div>

        {/* Camera type switcher */}
        <div className="flex items-center gap-1.5">
          <div className="bg-slate-100 p-0.5 rounded-sm flex border border-slate-200">
            <button
              onClick={() => setCameraType("rgb")}
              className={`px-2 py-1 rounded text-[11px] font-medium transition ${
                cameraType === "rgb"
                  ? "bg-emerald-600 text-white"
                  : "text-slate-600 hover:text-slate-900"
              }`}
            >
              RGB
            </button>
            <button
              onClick={() => setCameraType("depth")}
              className={`px-2 py-1 rounded text-[11px] font-medium transition ${
                cameraType === "depth"
                  ? "bg-cyan-600 text-white"
                  : "text-slate-600 hover:text-slate-900"
              }`}
            >
              Depth
            </button>
          </div>

          <button
            onClick={() => setIsPlaying(!isPlaying)}
            className={`p-1.5 rounded-sm text-xs font-medium flex items-center transition ${
              isPlaying
                ? "bg-slate-100 hover:bg-slate-200 text-slate-700 border border-slate-200"
                : "bg-emerald-600 hover:bg-emerald-700 text-white"
            }`}
            title={isPlaying ? "Pause stream" : "Play stream"}
          >
            {isPlaying ? <EyeOff className="w-3.5 h-3.5" /> : <Eye className="w-3.5 h-3.5" />}
          </button>
          <button
            onClick={handleRefresh}
            className="p-1.5 bg-slate-100 hover:bg-slate-200 text-slate-700 border border-slate-200 rounded-sm transition"
            title="Refresh stream"
          >
            <RefreshCw className="w-3.5 h-3.5" />
          </button>
        </div>
      </div>

      <div className="relative flex-1 bg-slate-200 rounded-sm overflow-hidden flex items-center justify-center min-h-0 w-full border border-slate-300">
        {isPlaying ? (
          /* eslint-disable-next-line @next/next/no-img-element */
          <img
            key={`${cameraType}-${reloadKey}`}
            src={streamSrc}
            alt="AMR Camera Live Feed"
            className="w-full h-full object-contain"
            onError={() => {}}
          />
        ) : (
          <div className="text-center text-slate-500 text-xs">
            <Camera className="w-8 h-8 mx-auto mb-2 opacity-40" />
            <p>Camera stream paused</p>
          </div>
        )}

        <div className="absolute top-2 left-2 bg-slate-900/70 backdrop-blur-sm px-2 py-0.5 rounded text-[10px] text-emerald-400 font-mono flex items-center gap-1.5">
          <span className="w-2 h-2 rounded-full bg-emerald-500 animate-pulse"></span>
          {cameraType === "rgb" ? "RGB 640x480 @ 15 FPS" : "Depth Turbo 320x240 @ 10 FPS"}
        </div>
      </div>
    </div>
  );
};
