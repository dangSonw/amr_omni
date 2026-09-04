"use client";

import { useEffect, useRef, useState, useCallback } from "react";
import { FullTelemetryMessage, LidarTelemetry, OdometryTelemetry, RobotStatus, StreamInfo, WheelTelemetry, ImuTelemetry } from "@/types/robot";

export function useRobotWs() {
  const [connected, setConnected] = useState(false);
  const [pingMs, setPingMs] = useState<number>(0);
  const [telemetry, setTelemetry] = useState<FullTelemetryMessage | null>(null);

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const lastPacketTimeRef = useRef<number>(Date.now());

  const connect = useCallback(() => {
    if (typeof window === "undefined") return;

    // Xác định URL WebSocket dựa trên host hiện tại
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const host = window.location.hostname || "localhost";
    // Nếu chạy ở port 3000 (Next.js dev), kết nối tới port 8000 (FastAPI). Nếu phục vụ cùng port thì dùng port đó.
    const port = window.location.port === "3000" ? "8000" : (window.location.port || "8000");
    const wsUrl = `${protocol}//${host}:${port}/ws/telemetry`;

    try {
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        setConnected(true);
        lastPacketTimeRef.current = Date.now();
      };

      ws.onmessage = (event) => {
        const now = Date.now();
        setPingMs(now - lastPacketTimeRef.current);
        lastPacketTimeRef.current = now;

        try {
          const data = JSON.parse(event.data);
          if (data.type === "telemetry") {
            setTelemetry(data);
          }
        } catch (err) {
          console.error("Lỗi parse telemetry:", err);
        }
      };

      ws.onclose = () => {
        setConnected(false);
        wsRef.current = null;
        // Tự động kết nối lại sau 1.5 giây
        reconnectTimeoutRef.current = setTimeout(connect, 1500);
      };

      ws.onerror = () => {
        ws.close();
      };
    } catch (err) {
      console.error("WebSocket init error:", err);
      reconnectTimeoutRef.current = setTimeout(connect, 2000);
    }
  }, []);

  useEffect(() => {
    connect();
    return () => {
      if (reconnectTimeoutRef.current) clearTimeout(reconnectTimeoutRef.current);
      if (wsRef.current) wsRef.current.close();
    };
  }, [connect]);

  const sendCmdVel = useCallback((vx: number, vy: number, wz: number) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(
        JSON.stringify({
          type: "cmd_vel",
          vx: Math.round(vx * 1000) / 1000,
          vy: Math.round(vy * 1000) / 1000,
          wz: Math.round(wz * 1000) / 1000,
        })
      );
    }
  }, []);

  const sendEStop = useCallback((active: boolean) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(
        JSON.stringify({
          type: "estop",
          active,
        })
      );
    }
  }, []);

  const resetOdom = useCallback(() => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(
        JSON.stringify({
          type: "reset_odom",
        })
      );
    }
  }, []);

  return {
    connected,
    pingMs,
    telemetry,
    sendCmdVel,
    sendEStop,
    resetOdom,
  };
}

