import { createRoot } from "react-dom/client";
import App from "./App.tsx";
import "./index.css";

// Suppress ResizeObserver loop warnings (this is a known browser issue)
const originalError = console.error;
console.error = (...args) => {
  if (
    typeof args[0] === "string" &&
    args[0].includes(
      "ResizeObserver loop completed with undelivered notifications",
    )
  ) {
    return;
  }
  originalError.call(console, ...args);
};

// Global error boundary for ResizeObserver
window.addEventListener("error", (event) => {
  if (
    event.message &&
    event.message.includes("ResizeObserver loop limit exceeded")
  ) {
    event.preventDefault();
    return false;
  }
});

createRoot(document.getElementById("root")!).render(<App />);
