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
    <div className="border-2 border-charcoal bg-white rounded-[2px] shadow-[-4px_4px_0px_#383838] p-3 sm:p-4 flex flex-col h-full text-charcoal overflow-hidden">
      {/* Header */}
      <div className="flex items-center justify-between pb-2.5 border-b-2 border-charcoal mb-2.5">
        <div className="flex items-center gap-2">
          <Camera className="w-4 h-4 text-charcoal" />
          <h3 className="font-bold text-xs sm:text-sm tracking-wider">
            CAMERA // LIVE
          </h3>
        </div>

        {/* Camera Selector & Action Buttons */}
        <div className="flex items-center gap-1.5">
          <div className="flex border border-charcoal bg-chalk p-0.5">
            <button
              onClick={() => setCameraType("rgb")}
              className={`px-2 py-0.5 text-[11px] font-bold transition ${
                cameraType === "rgb"
                  ? "bg-sky text-charcoal border border-charcoal shadow-[-1px_1px_0px_#383838]"
                  : "text-graphite hover:text-charcoal"
              }`}
            >
              RGB
            </button>
            <button
              onClick={() => setCameraType("depth")}
              className={`px-2 py-0.5 text-[11px] font-bold transition ${
                cameraType === "depth"
                  ? "bg-canary text-charcoal border border-charcoal shadow-[-1px_1px_0px_#383838]"
                  : "text-graphite hover:text-charcoal"
              }`}
            >
              DEPTH
            </button>
          </div>

          <button
            onClick={() => setIsPlaying(!isPlaying)}
            className="p-1 border border-charcoal bg-white hover:bg-ice text-charcoal shadow-[-2px_2px_0px_#383838] transition"
            title={isPlaying ? "Pause" : "Play"}
          >
            {isPlaying ? <EyeOff className="w-3.5 h-3.5" /> : <Eye className="w-3.5 h-3.5" />}
          </button>
          <button
            onClick={handleRefresh}
            className="p-1 border border-charcoal bg-white hover:bg-ice text-charcoal shadow-[-2px_2px_0px_#383838] transition"
            title="Reload"
          >
            <RefreshCw className="w-3.5 h-3.5" />
          </button>
        </div>
      </div>

      {/* Video / Frame Screen */}
      <div className="relative flex-1 bg-[#222222] border-2 border-charcoal rounded-[2px] overflow-hidden flex items-center justify-center min-h-0 w-full">
        {isPlaying ? (
          /* eslint-disable-next-line @next/next/no-img-element */
          <img
            key={`${cameraType}-${reloadKey}`}
            src={streamSrc}
            alt="AMR Camera Stream"
            className="w-full h-full object-contain"
            onError={() => {}}
          />
        ) : (
          <div className="text-center text-silver text-xs">
            <Camera className="w-8 h-8 mx-auto mb-2 opacity-30" />
            <p className="font-mono">STREAM PAUSED</p>
          </div>
        )}

        {/* Telemetry Tag */}
        <div className="absolute top-2 left-2 border border-charcoal bg-white/95 px-2 py-0.5 text-[10px] font-mono font-bold text-charcoal shadow-[-2px_2px_0px_#383838] flex items-center gap-1.5">
          <span className="w-2 h-2 rounded-[1px] bg-emerald-500 animate-pulse border border-charcoal"></span>
          {cameraType === "rgb" ? "RGB 640x480 @ 15 FPS" : "DEPTH 320x240 @ 10 FPS"}
        </div>
      </div>
    </div>
  );
};
