import { useCallback, useEffect, useRef, useState } from "react";
import type { Capture, ImagePair, PopupState, SavedAnnotation } from "./types";
import { ImagePopup } from "./components/ImagePopup";
import { CaptureForm } from "./components/CaptureForm";
import { CaptureHistory } from "./components/CaptureHistory";

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
      <ImagePopup
        popup={popup}
        zoom={zoom}
        pan={pan}
        popupContainerRef={popupContainerRef}
        onClose={handleClosePopup}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onClick={handlePopupClick}
      />

      <div className="max-w-5xl w-full px-6 py-6 flex flex-col gap-6">
        <CaptureForm
          imagePair={imagePair}
          isCapturing={isCapturing}
          onCapture={() => void handleFetchImages()}
          form={form}
          setForm={setForm}
          savedAnnotation={savedAnnotation}
          outputPending={outputPending}
          setOutputPending={setOutputPending}
          onSubmit={() => void handleSubmit()}
          isSubmitting={isSubmitting}
          output={output}
          colourRef={colourRef}
          referenceRef={referenceRef}
          outputBoxRef={outputBoxRef}
          error={error}
          onOpenPopup={handleOpenPopup}
        />

        <CaptureHistory
          captures={captures}
          selectedId={selectedId}
          onSelect={setSelectedId}
          isLoading={isLoading}
          selectedCapture={selectedCapture}
          onDelete={(id) => void handleDelete(id)}
        />
      </div>
    </main>
  );
}

export default App;