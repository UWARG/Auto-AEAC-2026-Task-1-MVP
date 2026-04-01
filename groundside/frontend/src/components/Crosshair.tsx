import type { Point } from "../types";

export function Crosshair({ point, color }: { point: Point; color: string }) {
  return (
    <div
      style={{
        position: "absolute",
        left: `${point.x}%`,
        top: `${point.y}%`,
        transform: "translate(-50%, -50%)",
        pointerEvents: "none",
      }}
    >
      <svg width="28" height="28" viewBox="-14 -14 28 28">
        <line x1="-11" y1="0" x2="11" y2="0" stroke={color} strokeWidth="2.5" />
        <line x1="0" y1="-11" x2="0" y2="11" stroke={color} strokeWidth="2.5" />
        <circle r="0" fill="none" stroke={color} strokeWidth="2" />
      </svg>
    </div>
  );
}
