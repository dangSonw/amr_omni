"use client";

import { useEffect, useRef, useState, useCallback } from "react";
import { FullTelemetryMessage, LidarTelemetry, OdometryTelemetry, RobotStatus, StreamInfo, WheelTelemetry, ImuTelemetry } from "@/types/robot";

export function useRobotWs() {
  const [connected, setConnected] = useState(false);
  const [pingMs, setPingMs] = useState<number>(0);
  const [telemetry, setTelemetry] = useState<FullTelemetryMessage | null>(null);

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const reconnectDelayRef = useRef<number>(1500);
  const lastPacketTimeRef = useRef<number>(Date.now());

  const connect = useCallback(() => {
    if (typeof window === "undefined") return;

    // Xác định URL WebSocket: ưu tiên biến môi trường NEXT_PUBLIC_WS_URL
    const envWsUrl = process.env.NEXT_PUBLIC_WS_URL;
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const host = window.location.hostname || "localhost";
    const port = window.location.port === "3000" ? "8000" : (window.location.port || "8000");
    const wsUrl = envWsUrl || `${protocol}//${host}:${port}/ws/telemetry`;

    try {
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        setConnected(true);
        lastPacketTimeRef.current = Date.now();
        reconnectDelayRef.current = 1500;
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.type === "pong") {
            const rtt = Date.now() - (data.ts || Date.now());
            setPingMs(Math.max(1, rtt));
          } else if (data.type === "telemetry") {
            setTelemetry(data);
          }
        } catch (err) {
          console.error("Lỗi parse telemetry:", err);
        }
      };

      ws.onclose = () => {
        setConnected(false);
        wsRef.current = null;
        reconnectTimeoutRef.current = setTimeout(connect, reconnectDelayRef.current);
        reconnectDelayRef.current = Math.min(reconnectDelayRef.current * 2, 30000);
      };

      ws.onerror = () => {
        ws.close();
      };
    } catch (err) {
      console.error("WebSocket init error:", err);
      reconnectTimeoutRef.current = setTimeout(connect, reconnectDelayRef.current);
      reconnectDelayRef.current = Math.min(reconnectDelayRef.current * 2, 30000);
    }
  }, []);

  useEffect(() => {
    connect();
    const pingInterval = setInterval(() => {
      if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
        wsRef.current.send(JSON.stringify({ type: "ping", ts: Date.now() }));
      }
    }, 1000);

    return () => {
      clearInterval(pingInterval);
      if (reconnectTimeoutRef.current) clearTimeout(reconnectTimeoutRef.current);
      if (wsRef.current) wsRef.current.close();
    };
  }, [connect]);

  const sendCmdVel = useCallback((vx: number, vy: number, wz: number) => {
    const rVx = Math.round(vx * 1000) / 1000;
    const rVy = Math.round(vy * 1000) / 1000;
    const rWz = Math.round(wz * 1000) / 1000;
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(
        JSON.stringify({
          type: "cmd_vel",
          vx: rVx,
          vy: rVy,
          wz: rWz,
        })
      );
    } else {
      fetch("/api/cmd-vel", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ linear_x: rVx, linear_y: rVy, angular_z: rWz }),
      }).catch(() => {});
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

