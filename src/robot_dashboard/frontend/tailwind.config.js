/** @type {import('tailwindcss').Config} */
export default {
  content: ["./index.html", "./src/**/*.{ts,tsx}"],
  theme: {
    extend: {
      colors: {
        terracotta: {
          DEFAULT: "#A96D4B",
          hover: "#8F5A3C",
          soft: "#A17258",
          tint: "rgba(169,109,75,0.08)",
        },
        ink: {
          DEFAULT: "#000000",
          soft: "#1E1E1E",
          2: "#3a3a3a",
        },
        paper: "#FFFFFF",
        cream: {
          DEFAULT: "#F6F4EC",
          deep: "#FBF9F5",
        },
        stone: "#F0E9E2",
        hair: {
          DEFAULT: "#D7D7D7",
          soft: "#EEEEEE",
        },
        muted: {
          DEFAULT: "#9a8f83",
          2: "#AAAAAA",
        },
        ok: {
          DEFAULT: "#2f7d5f",
          soft: "#E8F2EC",
        },
        err: {
          DEFAULT: "#9B2C2C",
          soft: "#F7E7E7",
        },
        running: {
          DEFAULT: "#51748C",
          soft: "#E4ECF2",
        },
      },
      fontFamily: {
        sans: ['"Source Sans 3"', '"Helvetica Neue"', "system-ui", "sans-serif"],
        mono: ['"Source Code Pro"', '"SF Mono"', "ui-monospace", "monospace"],
      },
      fontSize: {
        display: ["32px", { lineHeight: "1.12", letterSpacing: "-0.01em", fontWeight: "300" }],
        h1: ["26px", { lineHeight: "1.15", fontWeight: "300" }],
        h2: ["22px", { lineHeight: "1.2", fontWeight: "300" }],
        lede: ["16px", { lineHeight: "1.55" }],
        body: ["14.5px", { lineHeight: "1.55" }],
        label: ["14px", { letterSpacing: "0.03em", fontWeight: "500" }],
        "mono-label": ["11px", { letterSpacing: "0.1em", fontWeight: "600" }],
        "mono-data": ["12px", { letterSpacing: "0.06em" }],
      },
      borderRadius: {
        none: "0",
        sm: "6px",
        DEFAULT: "8px",
        pill: "999px",
      },
    },
  },
  plugins: [],
};
