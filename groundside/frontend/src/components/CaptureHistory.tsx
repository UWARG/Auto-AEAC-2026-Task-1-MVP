import type { Capture } from "../types";
import { AnnotationOverlay } from "./AnnotationOverlay";

type Props = {
  captures: Capture[];
  selectedId: string | null;
  onSelect: (id: string) => void;
  isLoading: boolean;
  selectedCapture: Capture | null;
  onDelete: (id: string) => void;
};

export function CaptureHistory({ captures, selectedId, onSelect, isLoading, selectedCapture, onDelete }: Props) {
  return (
    <section className="w-full">
      <h2 className="text-base font-semibold text-zinc-900 mb-4">Capture History</h2>
      <div className="flex gap-4" style={{ height: "400px" }}>

        {/* Descriptions column */}
        <div className="flex flex-col w-[45%] border border-zinc-200 rounded">
          <div className="px-4 py-2 border-b border-zinc-200 text-sm font-medium text-zinc-700">
            Descriptions
          </div>
          <div className="overflow-y-auto flex-1 p-2 flex flex-col gap-2">
            {isLoading ? (
              <p className="text-sm text-zinc-400 px-2 py-2">Loading...</p>
            ) : captures.length === 0 ? (
              <p className="text-sm text-zinc-400 px-2 py-2">No captures yet.</p>
            ) : (
              captures.map((capture) => (
                <div
                  key={capture.id}
                  onClick={() => onSelect(capture.id)}
                  className={`rounded-lg border p-3 cursor-pointer flex justify-between gap-2 ${
                    selectedId === capture.id
                      ? "border-blue-200 bg-blue-50"
                      : "border-zinc-100 bg-white hover:bg-zinc-50"
                  }`}
                >
                  <div className="flex flex-col gap-0.5 text-sm min-w-0">
                    <span className="text-zinc-400 text-xs">
                      {new Date(capture.time).toLocaleString()}
                    </span>
                    <span>
                      <span className="font-semibold">Colour:</span>{" "}
                      {capture.colour ?? "N/A"}
                    </span>
                    <span>
                      <span className="font-semibold">Direction:</span>{" "}
                      {capture.direction ?? "N/A"}
                    </span>
                    <span>
                      <span className="font-semibold">Reference:</span>{" "}
                      {capture.reference ?? "N/A"}
                    </span>
                    {(capture.green || capture.red) && (
                      <span className="text-xs font-mono mt-0.5">
                        {capture.green && (
                          <span className="text-green-600">
                            G({capture.green.x.toFixed(1)}, {capture.green.y.toFixed(1)})
                          </span>
                        )}
                        {capture.green && capture.red && " → "}
                        {capture.red && (
                          <span className="text-red-500">
                            R({capture.red.x.toFixed(1)}, {capture.red.y.toFixed(1)})
                          </span>
                        )}
                      </span>
                    )}
                    <span className="text-zinc-400 italic text-xs mt-1">
                      {capture.desc ?? "No description provided"}
                    </span>
                  </div>

                  {/* Action buttons */}
                  <div className="flex flex-col gap-1 shrink-0">
                    <div className="w-6 h-6 rounded bg-zinc-800 flex items-center justify-center">
                      <svg
                        xmlns="http://www.w3.org/2000/svg"
                        className="h-3.5 w-3.5 text-white"
                        viewBox="0 0 20 20"
                        fill="currentColor"
                      >
                        <path
                          fillRule="evenodd"
                          d="M16.707 5.293a1 1 0 0 1 0 1.414l-8 8a1 1 0 0 1-1.414 0l-4-4a1 1 0 0 1 1.414-1.414L8 12.586l7.293-7.293a1 1 0 0 1 1.414 0Z"
                          clipRule="evenodd"
                        />
                      </svg>
                    </div>
                    <button
                      onClick={(e) => {
                        e.stopPropagation();
                        onDelete(capture.id);
                      }}
                      className="w-6 h-6 rounded bg-red-50 hover:bg-red-100 flex items-center justify-center transition-colors"
                      title="Delete"
                    >
                      <svg
                        xmlns="http://www.w3.org/2000/svg"
                        className="h-3.5 w-3.5 text-red-400"
                        viewBox="0 0 20 20"
                        fill="currentColor"
                      >
                        <path
                          fillRule="evenodd"
                          d="M4.293 4.293a1 1 0 0 1 1.414 0L10 8.586l4.293-4.293a1 1 0 1 1 1.414 1.414L11.414 10l4.293 4.293a1 1 0 0 1-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 0 1-1.414-1.414L8.586 10 4.293 5.707a1 1 0 0 1 0-1.414Z"
                          clipRule="evenodd"
                        />
                      </svg>
                    </button>
                  </div>
                </div>
              ))
            )}
          </div>
        </div>

        {/* Image column */}
        <div className="flex flex-col w-[55%] border border-zinc-200 rounded">
          <div className="px-4 py-2 border-b border-zinc-200 text-sm font-medium text-zinc-700 shrink-0">
            Image
          </div>
          <div
            style={{
              flex: 1,
              display: "flex",
              flexDirection: "column",
              padding: "12px",
              minHeight: 0,
              overflow: "hidden",
            }}
          >
            {selectedCapture ? (
              <>
                <p className="text-xs text-zinc-400 mb-2 shrink-0">
                  {new Date(selectedCapture.time).toLocaleString()}
                </p>
                <div
                  style={{
                    flex: 1,
                    minHeight: 0,
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center",
                    overflow: "hidden",
                  }}
                >
                  <div style={{ position: "relative", display: "inline-block", maxWidth: "100%", maxHeight: "100%" }}>
                    <img
                      src={selectedCapture.imageUrl ?? "/warg.jpg"}
                      alt="Capture"
                      style={{
                        display: "block",
                        maxWidth: "100%",
                        maxHeight: "300px",
                        borderRadius: "6px",
                      }}
                    />
                    <AnnotationOverlay
                      green={selectedCapture.green}
                      red={selectedCapture.red}
                    />
                  </div>
                </div>
                {(selectedCapture.green || selectedCapture.red) && (
                  <p className="mt-1 text-xs font-mono shrink-0">
                    {selectedCapture.green && (
                      <span className="text-green-600">
                        G({selectedCapture.green.x.toFixed(1)},{" "}
                        {selectedCapture.green.y.toFixed(1)})
                      </span>
                    )}
                    {selectedCapture.green && selectedCapture.red && " → "}
                    {selectedCapture.red && (
                      <span className="text-red-500">
                        R({selectedCapture.red.x.toFixed(1)},{" "}
                        {selectedCapture.red.y.toFixed(1)})
                      </span>
                    )}
                  </p>
                )}
              </>
            ) : (
              <p className="text-sm text-zinc-400">Select a capture to view its image.</p>
            )}
          </div>
        </div>

      </div>
    </section>
  );
}
