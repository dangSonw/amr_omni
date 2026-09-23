import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "AMR Omni // Mission Control",
  description: "AMR Omni robot mission cockpit and telemetry system",
  other: {
    "Cache-Control": "no-cache, no-store, must-revalidate",
    Pragma: "no-cache",
    Expires: "0",
  },
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body className="min-h-screen bg-cream text-charcoal font-mono antialiased">
        {children}
      </body>
    </html>
  );
}
