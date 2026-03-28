import type { Point } from "../types";
import { Crosshair } from "./Crosshair";

export function AnnotationOverlay({ green, red }: { green: Point | null; red: Point | null }) {
  return (
    <div style={{ position: "absolute", inset: 0, pointerEvents: "none" }}>
      {green && red && (
        <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%" }}>
          <line
            x1={`${green.x}%`} y1={`${green.y}%`}
            x2={`${red.x}%`} y2={`${red.y}%`}
            stroke="white" strokeWidth="2" strokeDasharray="6 3" opacity="0.75"
          />
        </svg>
      )}
      {green && <Crosshair point={green} color="#22c55e" />}
      {red && <Crosshair point={red} color="#ef4444" />}
    </div>
  );
}
