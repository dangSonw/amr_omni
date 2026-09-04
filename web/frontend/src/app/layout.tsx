import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "AMR Omni — Mission Control & Telemetry",
  description: "Bảng điều khiển, giám sát luồng dữ liệu Jetson-STM32 và bản đồ LiDAR xe tự hành AMR Omni",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="vi" suppressHydrationWarning>
      <body className="min-h-screen antialiased transition-colors duration-200">
        {children}
      </body>
    </html>
  );
}

