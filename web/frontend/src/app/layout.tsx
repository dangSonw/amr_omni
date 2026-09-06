import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "AMR Omni — Mission Control & Telemetry",
  description: "Control cockpit, Jetson-STM32 telemetry stream and 2D LiDAR navigation map for AMR Omni robot",
  other: {
    "Cache-Control": "no-cache, no-store, must-revalidate",
    "Pragma": "no-cache",
    "Expires": "0",
  },
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body className="min-h-screen antialiased transition-colors duration-200">
        {children}
      </body>
    </html>
  );
}

