import type { Config } from "tailwindcss";

const config: Config = {
  darkMode: "class",
  content: [
    "./src/pages/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/components/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/app/**/*.{js,ts,jsx,tsx,mdx}",
  ],
  theme: {
    extend: {
      colors: {
        cream: {
          DEFAULT: "#f4efea",
          dark: "#eae3dc",
        },
        frost: "#ffffff",
        chalk: "#f8f8f7",
        ice: "#ebf9ff",
        notebook: "#f9fbe7",
        charcoal: {
          DEFAULT: "#383838",
          dark: "#222222",
        },
        pencil: "#a1a1a1",
        graphite: "#818181",
        silver: "#c0c0c0",
        sky: {
          DEFAULT: "#6fc2ff",
          hover: "#5eb3f2",
        },
        canary: "#ffde00",
        duck: {
          orange: "#ff9538",
        },
        sketch: {
          coral: "#f38e84",
          peach: "#f5b161",
          mint: "#38c1b0",
          lilac: "#b291de",
          lime: "#b3c419",
          periwinkle: "#7597ee",
          slate: "#84a6bc",
          marigold: "#e1c427",
        },
      },
      fontFamily: {
        mono: [
          "Aeonik Mono",
          "JetBrains Mono",
          "ui-monospace",
          "SFMono-Regular",
          "Menlo",
          "Monaco",
          "Consolas",
          "monospace",
        ],
      },
      borderRadius: {
        DEFAULT: "2px",
        sm: "2px",
      },
      boxShadow: {
        hard: "-4px 4px 0px #383838",
        "hard-sm": "-2px 2px 0px #383838",
        "hard-lg": "-6px 6px 0px #383838",
      },
    },
  },
  plugins: [],
};

export default config;
