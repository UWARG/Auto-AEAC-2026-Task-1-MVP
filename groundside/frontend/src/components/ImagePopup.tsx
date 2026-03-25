import type { RefObject } from "react";
import type { PopupState } from "../types";
import { AnnotationOverlay } from "./AnnotationOverlay";

type Props = {
  popup: PopupState;
  zoom: number;
  pan: { x: number; y: number };
  popupContainerRef: RefObject<HTMLDivElement | null>;
  onClose: () => void;
  onMouseDown: (e: React.MouseEvent) => void;
  onMouseMove: (e: React.MouseEvent) => void;
  onMouseUp: () => void;
  onClick: (e: React.MouseEvent<HTMLDivElement>) => void;
};

export function ImagePopup({
  popup, zoom, pan, popupContainerRef,
  onClose, onMouseDown, onMouseMove, onMouseUp, onClick,
}: Props) {
  if (!popup) return null;

  return (
    <div
      className="fixed inset-0 z-50 flex flex-col items-center justify-center"
      style={{ background: "rgba(0,0,0,0.85)" }}
    >
      <div className="w-full flex items-center justify-between px-6 py-3 text-white text-sm">
        <div className="flex items-center gap-4">
          <span className="font-semibold capitalize">{popup.imageType} camera</span>
          <span className="text-zinc-400 text-xs">
            {!popup.green
              ? "Click to place green crosshair"
              : !popup.red
              ? "Click to place red crosshair"
              : "Both crosshairs placed"}
          </span>
        </div>
        <div className="flex items-center gap-4">
          <span className="text-zinc-500 text-xs">
            Scroll to zoom · Drag to pan · Esc to close
          </span>
          <button
            onClick={onClose}
            className="px-4 py-1.5 bg-zinc-700 hover:bg-zinc-600 rounded text-sm"
          >
            Done
          </button>
        </div>
      </div>

      {(popup.green || popup.red) && (
        <div className="flex gap-6 px-6 pb-2 text-xs font-mono">
          {popup.green && (
            <span className="text-green-400">
              Green: ({popup.green.x.toFixed(1)}%, {popup.green.y.toFixed(1)}%)
            </span>
          )}
          {popup.red && (
            <span className="text-red-400">
              Red: ({popup.red.x.toFixed(1)}%, {popup.red.y.toFixed(1)}%)
            </span>
          )}
        </div>
      )}

      <div
        ref={popupContainerRef}
        className="relative overflow-hidden"
        style={{ width: "95vw", height: "88vh", cursor: zoom > 1 ? "grab" : "crosshair" }}
        onMouseDown={onMouseDown}
        onMouseMove={onMouseMove}
        onMouseUp={onMouseUp}
        onMouseLeave={onMouseUp}
      >
        <div
          style={{
            position: "absolute",
            inset: 0,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            transform: `translate(${pan.x}px, ${pan.y}px) scale(${zoom})`,
            transformOrigin: "center center",
          }}
        >
          <div style={{ position: "relative", display: "inline-block" }}>
            <img
              src={popup.url}
              alt="Capture"
              style={{ display: "block", maxWidth: "93vw", maxHeight: "84vh", userSelect: "none" }}
              draggable={false}
            />
            <div
              style={{ position: "absolute", inset: 0, cursor: "crosshair" }}
              onClick={onClick}
            >
              <AnnotationOverlay green={popup.green} red={popup.red} />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
