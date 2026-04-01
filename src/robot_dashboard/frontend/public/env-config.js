// Runtime configuration — overwritten by the launch file at startup.
// Vite copies this from public/ into dist/ during build.
// The launch file then overwrites the copy in the www/dist directory
// with actual port values before serving.
window.ENV_CONFIG = {
  ROSBRIDGE_PORT: "9090",
};
