import { useCallback, useEffect, useRef, useState } from "react";

type Point = { x: number; y: number };

type Capture = {
  id: string;
  time: string;
  colour: string | null;
  direction: string | null;
  reference: string | null;
  desc: string | null;
  imageUrl: string | null;
  green: Point | null;
  red: Point | null;
};

type ImagePair = { forwardUrl: string; downwardUrl: string } | null;

type PopupState = {
  url: string;
  imageType: "forward" | "downward";
  green: Point | null;
  red: Point | null;
} | null;

type SavedAnnotation = {
  green: Point | null;
  red: Point | null;
  imageType: "forward" | "downward" | null;
} | null;

function Crosshair({ point, color }: { point: Point; color: string }) {
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
        <circle r="4" fill="none" stroke={color} strokeWidth="2" />
      </svg>
    </div>
  );
}

function AnnotationOverlay({ green, red }: { green: Point | null; red: Point | null }) {
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

function App() {
  const [captures, setCaptures] = useState<Capture[]>([]);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [imagePair, setImagePair] = useState<ImagePair>(null);
  const [popup, setPopup] = useState<PopupState>(null);
  const [zoom, setZoom] = useState(1.5);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [form, setForm] = useState({ colour: "", reference: "" });
  const [savedAnnotation, setSavedAnnotation] = useState<SavedAnnotation>(null);
  const [outputPending, setOutputPending] = useState(false);
  const [isCapturing, setIsCapturing] = useState(false);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState("");

  const colourRef = useRef<HTMLInputElement>(null);
  const referenceRef = useRef<HTMLInputElement>(null);
  const outputBoxRef = useRef<HTMLDivElement>(null);
  const isCapturingRef = useRef(false);
  const isPanningRef = useRef(false);
  const hasPannedRef = useRef(false);
  const lastMouseRef = useRef({ x: 0, y: 0 });
  const popupRef = useRef<PopupState>(null);
  const imagePairRef = useRef<ImagePair>(null);
  const popupContainerRef = useRef<HTMLDivElement>(null);

  useEffect(() => { popupRef.current = popup; }, [popup]);
  useEffect(() => { imagePairRef.current = imagePair; }, [imagePair]);

  // Native wheel listener — required so preventDefault works
  useEffect(() => {
    const el = popupContainerRef.current;
    if (!el) return;
    function onWheel(e: WheelEvent) {
      e.preventDefault();
      setZoom((z) => Math.max(1, Math.min(8, z + (e.deltaY > 0 ? -0.2 : 0.2))));
    }
    el.addEventListener("wheel", onWheel, { passive: false });
    return () => el.removeEventListener("wheel", onWheel);
  }, [popup]);

  const selectedCapture =
    captures.find((c) => c.id === selectedId) ?? captures[0] ?? null;

  const direction =
    savedAnnotation?.imageType === "forward" ? "F"
    : savedAnnotation?.imageType === "downward" ? "D"
    : null;

  const output = [
    form.colour && `Colour: ${form.colour}`,
    direction && `Direction: ${direction}`,
    form.reference && `Reference Point: ${form.reference}`,
    savedAnnotation?.green &&
      `G(${savedAnnotation.green.x.toFixed(1)}, ${savedAnnotation.green.y.toFixed(1)})`,
    savedAnnotation?.red &&
      `R(${savedAnnotation.red.x.toFixed(1)}, ${savedAnnotation.red.y.toFixed(1)})`,
  ]
    .filter(Boolean)
    .join(" | ");

  async function loadCaptures() {
    try {
      setIsLoading(true);
      setError("");
      const res = await fetch("/api/captures");
      if (!res.ok) throw new Error("Failed to load captures");
      const data: Capture[] = await res.json();
      setCaptures(data);
      if (data.length > 0) setSelectedId(data[0].id);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to load captures");
    } finally {
      setIsLoading(false);
    }
  }

  useEffect(() => { void loadCaptures(); }, []);

  const handleFetchImages = useCallback(async () => {
    if (isCapturingRef.current) return;
    isCapturingRef.current = true;
    setIsCapturing(true);
    setError("");
    setForm({ colour: "", reference: "" });
    setSavedAnnotation(null);
    setOutputPending(false);
    setImagePair(null);
    try {
      const res = await fetch("/api/images/capture", { method: "POST" });
      if (!res.ok) throw new Error("Failed to capture images");
      const data = await res.json();
      setImagePair({ forwardUrl: data.forward_url, downwardUrl: data.downward_url });
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to capture images");
      setImagePair({ forwardUrl: "/warg.jpg", downwardUrl: "/warg.jpg" });
    } finally {
      isCapturingRef.current = false;
      setIsCapturing(false);
    }
  }, []);

  const handleClosePopup = useCallback(() => {
    const p = popupRef.current;
    if (!p) return;
    setSavedAnnotation({ green: p.green, red: p.red, imageType: p.imageType });
    setPopup(null);
    setTimeout(() => colourRef.current?.focus(), 50);
  }, []);

  useEffect(() => {
    function onKeyDown(e: KeyboardEvent) {
      const inInput =
        e.target instanceof HTMLInputElement ||
        e.target instanceof HTMLTextAreaElement ||
        (e.target as HTMLElement)?.hasAttribute?.("tabindex");

      if (e.code === "Space" && !inInput) {
        e.preventDefault();
        void handleFetchImages();
      }
      if (e.code === "Escape" && popupRef.current) {
        handleClosePopup();
      }
      // F key: open forward popup
      if (e.code === "KeyF" && !(e.target instanceof HTMLInputElement) && imagePairRef.current && !popupRef.current) {
        setPopup({ url: imagePairRef.current.forwardUrl, imageType: "forward", green: null, red: null });
        setZoom(1.5);
        setPan({ x: 0, y: 0 });
      }
      // D key: open downward popup
      if (e.code === "KeyD" && !(e.target instanceof HTMLInputElement) && imagePairRef.current && !popupRef.current) {
        setPopup({ url: imagePairRef.current.downwardUrl, imageType: "downward", green: null, red: null });
        setZoom(1.5);
        setPan({ x: 0, y: 0 });
      }
    }
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [handleFetchImages, handleClosePopup]);

  function handleOpenPopup(url: string, imageType: "forward" | "downward") {
    setPopup({ url, imageType, green: null, red: null });
    setZoom(1.5);
    setPan({ x: 0, y: 0 });
  }

  function handlePopupClick(e: React.MouseEvent<HTMLDivElement>) {
    if (hasPannedRef.current) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const x = ((e.clientX - rect.left) / rect.width) * 100;
    const y = ((e.clientY - rect.top) / rect.height) * 100;
    setPopup((p) => {
      if (!p) return p;
      if (!p.green) return { ...p, green: { x, y } };
      if (!p.red) return { ...p, red: { x, y } };
      return p;
    });
  }

  function handleMouseDown(e: React.MouseEvent) {
    isPanningRef.current = true;
    hasPannedRef.current = false;
    lastMouseRef.current = { x: e.clientX, y: e.clientY };
  }

  function handleMouseMove(e: React.MouseEvent) {
    if (!isPanningRef.current) return;
    const dx = e.clientX - lastMouseRef.current.x;
    const dy = e.clientY - lastMouseRef.current.y;
    if (zoom > 1) {
      if (Math.abs(dx) > 3 || Math.abs(dy) > 3) hasPannedRef.current = true;
      setPan((p) => ({ x: p.x + dx, y: p.y + dy }));
    }
    lastMouseRef.current = { x: e.clientX, y: e.clientY };
  }

  function handleMouseUp() {
    isPanningRef.current = false;
  }

  async function handleDelete(id: string) {
    try {
      const res = await fetch(`/api/captures/${id}`, { method: "DELETE" });
      if (!res.ok) throw new Error("Failed to delete capture");
      setCaptures((c) => {
        const next = c.filter((cap) => cap.id !== id);
        if (selectedId === id) setSelectedId(next[0]?.id ?? null);
        return next;
      });
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to delete capture");
    }
  }

  async function handleSubmit() {
    try {
      setIsSubmitting(true);
      setError("");

      const dir =
        savedAnnotation?.imageType === "forward" ? "F"
        : savedAnnotation?.imageType === "downward" ? "D"
        : null;

      const descParts = [
        form.colour && `Colour: ${form.colour}`,
        dir && `Direction: ${dir}`,
        form.reference && `Reference Point: ${form.reference}`,
      ].filter(Boolean);

      const imageUrl = imagePair
        ? savedAnnotation?.imageType === "forward"
          ? imagePair.forwardUrl
          : imagePair.downwardUrl
        : null;

      const payload = {
        colour: form.colour || null,
        direction: dir,
        reference: form.reference || null,
        desc: descParts.length > 0 ? descParts.join(" | ") : null,
        green: savedAnnotation?.green ?? null,
        red: savedAnnotation?.red ?? null,
        imageUrl,
        time: new Date().toISOString(),
      };

      const res = await fetch("/api/captures", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });

      if (!res.ok) {
        const data = await res.json().catch(() => null);
        throw new Error(data?.message ?? "Failed to save capture");
      }

      const created: Capture = await res.json();
      setCaptures((c) => [created, ...c]);
      setSelectedId(created.id);

      setForm({ colour: "", reference: "" });
      setImagePair(null);
      setSavedAnnotation(null);
      setOutputPending(false);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to save capture");
      setOutputPending(false);
    } finally {
      setIsSubmitting(false);
    }
  }

  return (
    <main className="min-h-screen bg-white text-zinc-900 flex justify-center">
      {/* ── Popup ── */}
      {popup && (
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
                onClick={handleClosePopup}
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
            onMouseDown={handleMouseDown}
            onMouseMove={handleMouseMove}
            onMouseUp={handleMouseUp}
            onMouseLeave={handleMouseUp}
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
                  onClick={handlePopupClick}
                >
                  <AnnotationOverlay green={popup.green} red={popup.red} />
                </div>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* ── Single centered page wrapper ── */}
      <div className="max-w-5xl w-full px-6 py-6 flex flex-col gap-6">

      {/* Upper content */}
      <div className="w-full flex flex-col gap-6" style={{ maxWidth: '48rem', marginLeft: 'auto', marginRight: 'auto' }}>

        {/* CAPTURE button */}
        <div className="flex flex-col items-center gap-2">
          <button
            onClick={() => void handleFetchImages()}
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
                <kbd className="px-1 bg-zinc-100 rounded border border-zinc-300 text-zinc-600 font-mono text-xs">
                  F
                </kbd>
              </span>
              <img
                src={imagePair.forwardUrl}
                alt="Forward capture"
                className="w-full rounded-md cursor-pointer object-contain max-h-44 hover:ring-2 hover:ring-blue-300 transition-all"
                onClick={() => handleOpenPopup(imagePair.forwardUrl, "forward")}
              />
            </div>
            <div className="flex flex-col gap-1">
              <span className="text-xs text-zinc-500 text-center font-medium">
                Downward — click or press{" "}
                <kbd className="px-1 bg-zinc-100 rounded border border-zinc-300 text-zinc-600 font-mono text-xs">
                  D
                </kbd>
              </span>
              <img
                src={imagePair.downwardUrl}
                alt="Downward capture"
                className="w-full rounded-md cursor-pointer object-contain max-h-44 hover:ring-2 hover:ring-blue-300 transition-all"
                onClick={() => handleOpenPopup(imagePair.downwardUrl, "downward")}
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
                    void handleSubmit();
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
              void handleSubmit();
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
              <span className="text-amber-500">
                Fill in the fields above to generate output
              </span>
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

      {/* ── Capture History ── */}
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
                    onClick={() => setSelectedId(capture.id)}
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
                          void handleDelete(capture.id);
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

          {/* Image column — no scroll, crosshairs overlaid */}
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
                <p className="text-sm text-zinc-400">
                  Select a capture to view its image.
                </p>
              )}
            </div>
          </div>
        </div>
      </section>
      </div>{/* end page wrapper */}
    </main>
  );
}

export default App;