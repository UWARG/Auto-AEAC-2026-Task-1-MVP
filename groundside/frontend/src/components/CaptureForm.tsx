import type { RefObject } from "react";
import type { ImagePair, SavedAnnotation } from "../types";

type Props = {
  imagePair: ImagePair;
  isCapturing: boolean;
  onCapture: () => void;
  form: { colour: string; reference: string };
  setForm: React.Dispatch<React.SetStateAction<{ colour: string; reference: string }>>;
  savedAnnotation: SavedAnnotation;
  outputPending: boolean;
  setOutputPending: (v: boolean) => void;
  onSubmit: () => void;
  output: string;
  colourRef: RefObject<HTMLInputElement | null>;
  referenceRef: RefObject<HTMLInputElement | null>;
  outputBoxRef: RefObject<HTMLDivElement | null>;
  error: string;
  onOpenPopup: (url: string, imageType: "forward" | "downward") => void;
};

export function CaptureForm({
  imagePair, isCapturing, onCapture, form, setForm,
  outputPending, setOutputPending, onSubmit, output,
  colourRef, referenceRef, outputBoxRef, error, onOpenPopup,
}: Props) {
  return (
    <div className="w-full flex flex-col gap-6" style={{ maxWidth: "48rem", marginLeft: "auto", marginRight: "auto" }}>

      {/* Capture button */}
      <div className="flex flex-col items-center gap-2">
        <button
          onClick={onCapture}
          disabled={isCapturing}
          className="flex items-center gap-2 bg-zinc-900 text-white px-8 py-3 rounded-md text-sm font-semibold tracking-widest disabled:opacity-60 disabled:cursor-wait"
        >
          <svg xmlns="http://www.w3.org/2000/svg" className="h-4 w-4" viewBox="0 0 24 24" fill="currentColor">
            <path d="M12 15.2a3.2 3.2 0 1 0 0-6.4 3.2 3.2 0 0 0 0 6.4Z" />
            <path
              fillRule="evenodd"
              d="M9 2L7.17 4H4a2 2 0 0 0-2 2v12a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V6a2 2 0 0 0-2-2h-3.17L15 2H9Zm3 15a5 5 0 1 0 0-10 5 5 0 0 0 0 10Z"
              clipRule="evenodd"
            />
          </svg>
          {isCapturing ? "CAPTURING..." : "CAPTURE"}
        </button>
        <p className="text-xs text-zinc-400">
          or press{" "}
          <kbd className="px-1.5 py-0.5 bg-zinc-100 rounded border border-zinc-300 text-zinc-600 font-mono text-xs">
            Space
          </kbd>
        </p>
      </div>

      {/* Images */}
      {imagePair ? (
        <div className="grid grid-cols-2 gap-3">
          <div className="flex flex-col gap-1">
            <span className="text-xs text-zinc-500 text-center font-medium">
              Forward — click or press{" "}
              <kbd className="px-1 bg-zinc-100 rounded border border-zinc-300 text-zinc-600 font-mono text-xs">F</kbd>
            </span>
            <img
              src={imagePair.forwardUrl}
              alt="Forward capture"
              className="w-full rounded-md cursor-pointer object-contain max-h-44 hover:ring-2 hover:ring-blue-300 transition-all"
              onClick={() => onOpenPopup(imagePair.forwardUrl, "forward")}
            />
          </div>
          <div className="flex flex-col gap-1">
            <span className="text-xs text-zinc-500 text-center font-medium">
              Downward — click or press{" "}
              <kbd className="px-1 bg-zinc-100 rounded border border-zinc-300 text-zinc-600 font-mono text-xs">D</kbd>
            </span>
            <img
              src={imagePair.downwardUrl}
              alt="Downward capture"
              className="w-full rounded-md cursor-pointer object-contain max-h-44 hover:ring-2 hover:ring-blue-300 transition-all"
              onClick={() => onOpenPopup(imagePair.downwardUrl, "downward")}
            />
          </div>
        </div>
      ) : (
        <div className="flex justify-center">
          <img
            src="/api/stream"
            alt="Live feed"
            className="max-h-52 w-auto rounded-md object-contain"
            onError={(e) => {
              (e.target as HTMLImageElement).src = "/warg.jpg";
            }}
          />
        </div>
      )}

      {/* Input fields */}
      <div className="grid grid-cols-2 gap-4">
        <label className="flex flex-col gap-1 text-sm text-zinc-500">
          Colour:
          <input
            ref={colourRef}
            className="border border-zinc-200 rounded px-3 py-2 text-zinc-900 outline-none focus:border-zinc-400"
            placeholder="Enter colour"
            value={form.colour}
            onChange={(e) => setForm((f) => ({ ...f, colour: e.target.value }))}
            onKeyDown={(e) => {
              if (e.key === "Enter") {
                e.preventDefault();
                referenceRef.current?.focus();
              }
            }}
          />
        </label>
        <label className="flex flex-col gap-1 text-sm text-zinc-500">
          Reference Point:
          <input
            ref={referenceRef}
            className="border border-zinc-200 rounded px-3 py-2 text-zinc-900 outline-none focus:border-zinc-400"
            placeholder="Enter reference point"
            value={form.reference}
            onChange={(e) => setForm((f) => ({ ...f, reference: e.target.value }))}
            onKeyDown={(e) => {
              if (e.key === "Enter") {
                e.preventDefault();
                if (outputPending) {
                  onSubmit();
                } else {
                  setOutputPending(true);
                  setTimeout(() => outputBoxRef.current?.focus(), 50);
                }
              }
            }}
          />
        </label>
      </div>

      {/* Output box */}
      <div
        ref={outputBoxRef}
        tabIndex={0}
        onKeyDown={(e) => {
          if (e.key === "Enter" && outputPending) {
            e.preventDefault();
            onSubmit();
          }
        }}
        className={`rounded p-4 text-sm transition-all duration-150 outline-none flex items-center justify-between gap-4 ${
          outputPending
            ? "bg-blue-50 border-2 border-blue-400"
            : "bg-zinc-50 border border-zinc-100"
        }`}
      >
        <div>
          <span className="font-medium text-zinc-700">Output: </span>
          {output ? (
            <span className="text-zinc-900">{output}</span>
          ) : (
            <span className="text-amber-500">Fill in the fields above to generate output</span>
          )}
        </div>
        {outputPending && (
          <span className="text-xs text-blue-500 font-medium whitespace-nowrap shrink-0">
            Press Enter to save →
          </span>
        )}
      </div>

      {error && <p className="text-sm text-red-500">{error}</p>}
    </div>
  );
}
