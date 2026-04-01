import { useCallback, useEffect, useRef, useState } from "react";
import type { Capture, ForwardAnnotation, DownwardAnnotation, ImagePair, PopupState } from "./types";
import { ImagePopup } from "./components/ImagePopup";
import { CaptureForm } from "./components/CaptureForm";
import { CaptureHistory } from "./components/CaptureHistory";

type CaptureImageResponse = {
  roll: number | null;
  pitch: number | null;
  downward_range: number | null;
  center_depth: number | null;
  arducam_image: string;  // base64 JPEG
  oakd_image: string;     // base64 JPEG
  pk: string;
};

type CaptureEntry = {
  ardufile_name: string;
  oakd_name: string;
  time: string;
  depth_map_name: string;
  desc: string | null;
  downward_range: number | null;
  direction: string | null;
  colour: string | null;
  ref_description: string | null;
  x_tar_pct: number | null;
  y_tar_pct: number | null;
  x_ref_pct: number | null;
  y_ref_pct: number | null;
  dist_up_m: number | null;
  dist_forward_m: number | null;
  dist_lateral_m: number | null;
};

type GenerateOutputResponse = {
  desc: string;
  oakd_image: string;   // base64 JPEG, annotated
  ardu_image: string;   // base64 JPEG, annotated
};

type ApiErrorResponse = { message: string };

// ─── API functions ─────────────────────────────────────────────────────────────

async function apiCaptureImage(): Promise<CaptureImageResponse> {
  const res = await fetch("/api/capture_image", { method: "POST" });
  const data = await res.json() as CaptureImageResponse | ApiErrorResponse;
  if (!res.ok) throw new Error((data as ApiErrorResponse).message ?? "Failed to capture image");
  return data as CaptureImageResponse;
}

async function apiFetchCaptures(): Promise<Record<string, CaptureEntry>> {
  const res = await fetch("/api/captures");
  if (!res.ok) throw new Error("Failed to load captures");
  return await res.json() as Record<string, CaptureEntry>;
}

async function apiGenerateOutput(payload: {
  pk: string;
  target_x_pct: number;
  target_y_pct: number;
  reference_x_pct: number;
  reference_y_pct: number;
  mode: "aided";
  color: string;
  ref_description: string;
  target_on_ground: boolean;
}): Promise<GenerateOutputResponse> {
  const res = await fetch("/api/generate_output", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await res.json() as GenerateOutputResponse | ApiErrorResponse;
  if (!res.ok) throw new Error((data as ApiErrorResponse).message ?? "Failed to generate output");
  return data as GenerateOutputResponse;
}

async function apiSaveById(pk: string): Promise<void> {
  const res = await fetch("/api/save_by_id", {
    method: "PATCH",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ pk }),
  });
  const data = await res.json() as ApiErrorResponse;
  if (!res.ok) throw new Error(data.message ?? "Failed to save");
}

async function apiSaveAll(): Promise<string> {
  const res = await fetch("/api/save_all", { method: "POST" });
  const data = await res.json() as ApiErrorResponse;
  if (!res.ok) throw new Error(data.message ?? "Failed to save all");
  return data.message;
}

async function apiDeleteById(pk: string): Promise<void> {
  const res = await fetch("/api/delete_by_id", {
    method: "DELETE",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ pk }),
  });
  const data = await res.json() as ApiErrorResponse;
  if (!res.ok) throw new Error(data.message ?? "Failed to delete");
}

// ─── Helpers ───────────────────────────────────────────────────────────────────

function parseCapturesDict(raw: Record<string, CaptureEntry>): Capture[] {
  const data: Capture[] = Object.entries(raw).map(([id, v]) => ({
    id,
    time: v.time,
    desc: v.desc ?? null,
    direction: v.direction ?? null,
    colour: v.colour ?? null,
    reference: v.ref_description ?? null,
    green: v.x_tar_pct != null && v.y_tar_pct != null ? { x: v.x_tar_pct, y: v.y_tar_pct } : null,
    red: v.x_ref_pct != null && v.y_ref_pct != null ? { x: v.x_ref_pct, y: v.y_ref_pct } : null,
    imageUrl: v.direction === "D" ? `/api/image/${id}/arducam` : `/api/image/${id}/oakd`,
  }));
  data.sort((a, b) => b.time.localeCompare(a.time));
  return data;
}

function getImageDimensions(src: string): Promise<{ width: number; height: number }> {
  return new Promise((resolve) => {
    const img = new window.Image();
    img.onload = () => resolve({ width: img.naturalWidth, height: img.naturalHeight });
    img.onerror = () => resolve({ width: 1, height: 1 });
    img.src = src;
  });
}

// ─── Component ─────────────────────────────────────────────────────────────────

function App() {
  const [captures, setCaptures] = useState<Capture[]>([]);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [imagePair, setImagePair] = useState<ImagePair>(null);
  const [popup, setPopup] = useState<PopupState>(null);
  const [zoom, setZoom] = useState(2.0);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [form, setForm] = useState({ colour: "", reference: "" });
  const [forwardAnnotation, setForwardAnnotation] = useState<ForwardAnnotation>(null);
  const [downwardAnnotation, setDownwardAnnotation] = useState<DownwardAnnotation>(null);
  const [generatedOutput, setGeneratedOutput] = useState<string | null>(null);
  const [outputPending, setOutputPending] = useState(false);
  const [pendingCoords, setPendingCoords] = useState<{ targetXPct: number; targetYPct: number; refXPct: number; refYPct: number } | null>(null);
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

  const selectedCapture = captures.find((c) => c.id === selectedId) ?? captures[0] ?? null;

  const targetOnGround = !!(downwardAnnotation?.target);
  const direction = targetOnGround ? "D" : forwardAnnotation?.target ? "F" : null;

  const localPreview = [
    form.colour && `Colour: ${form.colour}`,
    direction && `Direction: ${direction}`,
    form.reference && `Reference Point: ${form.reference}`,
  ].filter(Boolean).join(" | ");

  const output = generatedOutput ?? localPreview;

  async function loadCaptures() {
    try {
      setIsLoading(true);
      setError("");
      const raw = await apiFetchCaptures();
      const data = parseCapturesDict(raw);
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
    setPendingCoords(null);
    setForwardAnnotation(null);
    setDownwardAnnotation(null);
    setGeneratedOutput(null);
    setOutputPending(false);
    setImagePair(null);
    try {
      const data = await apiCaptureImage();
      const forwardUrl = `data:image/jpeg;base64,${data.oakd_image}`;
      const downwardUrl = `data:image/jpeg;base64,${data.arducam_image}`;
      const [oakdDims, arduDims] = await Promise.all([
        getImageDimensions(forwardUrl),
        getImageDimensions(downwardUrl),
      ]);
      setImagePair({
        forwardUrl,
        downwardUrl,
        pk: data.pk,
        oakdWidth: oakdDims.width,
        oakdHeight: oakdDims.height,
        arduWidth: arduDims.width,
        arduHeight: arduDims.height,
      });
    } catch (err) {
      // Fall back to demo mode with warg.jpg so the UI can be tested without the drone
      setError((err instanceof Error ? err.message : "Failed to capture images") + " — using demo image");
      const demoUrl = "/warg.jpg";
      const dims = await getImageDimensions(demoUrl);
      setImagePair({
        forwardUrl: demoUrl,
        downwardUrl: demoUrl,
        pk: "",  // empty pk = demo mode, skips backend calls
        oakdWidth: dims.width || 1280,
        oakdHeight: dims.height || 720,
        arduWidth: dims.width || 1280,
        arduHeight: dims.height || 720,
      });
    } finally {
      isCapturingRef.current = false;
      setIsCapturing(false);
      // setTimeout(() => colourRef.current?.focus(), 100);
    }
  }, []);

  const handleClosePopup = useCallback(() => {
    const p = popupRef.current;
    if (!p) return;
    if (p.imageType === "forward") {
      setForwardAnnotation({ target: p.green, ref: p.red });
    } else {
      setDownwardAnnotation({ target: p.green });
    }
    setPopup(null);
    setTimeout(() => colourRef.current?.focus(), 50);
  }, []);

  useEffect(() => {
    function onKeyDown(e: KeyboardEvent) {
      const inInput =
        e.target instanceof HTMLInputElement ||
        e.target instanceof HTMLTextAreaElement ||
        (e.target as HTMLElement)?.hasAttribute?.("tabindex");

      if (e.code === "Space" && !inInput && !popupRef.current) {
        e.preventDefault();
        void handleFetchImages();
      }
      if (e.code === "Enter" && popupRef.current) {
        handleClosePopup();
      }
      if (e.code === "KeyF" && !(e.target instanceof HTMLInputElement) && imagePairRef.current && !popupRef.current) {
        setPopup({ url: imagePairRef.current.forwardUrl, imageType: "forward", green: null, red: null });
        setZoom(2.0);
        setPan({ x: 0, y: 0 });
      }
      if (e.code === "KeyD" && !(e.target instanceof HTMLInputElement) && imagePairRef.current && !popupRef.current) {
        setPopup({ url: imagePairRef.current.downwardUrl, imageType: "downward", green: null, red: null });
        setZoom(2.0);
        setPan({ x: 0, y: 0 });
      }
    }
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [handleFetchImages, handleClosePopup]);

  function handleOpenPopup(url: string, imageType: "forward" | "downward") {
    setPopup({ url, imageType, green: null, red: null });
    setZoom(2.0);
    setPan({ x: 0, y: 0 });
  }

  function handlePopupClick(e: React.MouseEvent<HTMLDivElement>) {
    if (hasPannedRef.current) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const x = ((e.clientX - rect.left) / rect.width) * 100;
    const y = ((e.clientY - rect.top) / rect.height) * 100;
    const THRESHOLD = 5;
    setPopup((p) => {
      if (!p) return p;
      if (p.green && Math.abs(x - p.green.x) < THRESHOLD && Math.abs(y - p.green.y) < THRESHOLD)
        return { ...p, green: null };
      if (p.red && Math.abs(x - p.red.x) < THRESHOLD && Math.abs(y - p.red.y) < THRESHOLD)
        return { ...p, red: null };
      if (!p.green) return { ...p, green: { x, y } };
      if (!p.red) return { ...p, red: { x, y } };
      return p;
    });
  }

  async function handleEdit(id: string) {
    const capture = captures.find((c) => c.id === id);
    if (!capture) return;
    if (id.startsWith("demo-")) {
      const url = capture.imageUrl ?? "/warg.jpg";
      const dims = await getImageDimensions(url);
      setImagePair({ forwardUrl: url, downwardUrl: url, pk: id, oakdWidth: dims.width || 1280, oakdHeight: dims.height || 720, arduWidth: dims.width || 1280, arduHeight: dims.height || 720 });
    } else {
      const forwardUrl = `/api/image/${id}/oakd`;
      const downwardUrl = `/api/image/${id}/arducam`;
      const [oakdDims, arduDims] = await Promise.all([
        getImageDimensions(forwardUrl),
        getImageDimensions(downwardUrl),
      ]);
      setImagePair({ forwardUrl, downwardUrl, pk: id, oakdWidth: oakdDims.width, oakdHeight: oakdDims.height, arduWidth: arduDims.width, arduHeight: arduDims.height });
    }
    if (capture.direction === "D") {
      setDownwardAnnotation(capture.green ? { target: capture.green } : null);
      setForwardAnnotation(capture.red ? { target: null, ref: capture.red } : null);
    } else {
      setForwardAnnotation(capture.green ? { target: capture.green, ref: capture.red } : null);
      setDownwardAnnotation(null);
    }
    setForm({ colour: capture.colour ?? "", reference: capture.reference ?? "" });
    setGeneratedOutput(null);
    setOutputPending(false);
    setPendingCoords(null);
    setSelectedId(id);
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
      if (!id.startsWith("demo-")) {
        await apiDeleteById(id);
      }
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
    if (!imagePair || isSubmitting) return;

    if (!outputPending) {
      // Phase 1: generate output + get annotated images back from backend
      const tg = targetOnGround;
      let targetXPct: number, targetYPct: number, refXPct: number, refYPct: number;

      if (tg) {
        // Cross-camera: target on downward (arducam), reference on forward (oakd)
        const t = downwardAnnotation!.target!;
        const r = forwardAnnotation?.ref ?? forwardAnnotation?.target;
        if (!r) {
          setError("Place a reference point on the forward camera image (press F)");
          return;
        }
        targetXPct = t.x;
        targetYPct = t.y;
        refXPct = r.x;
        refYPct = r.y;
      } else {
        // Same-camera: both points on forward (oakd)
        const t = forwardAnnotation?.target;
        const r = forwardAnnotation?.ref;
        if (!t || !r) {
          setError("Open the forward camera (press F) and place target (green) then reference (red)");
          return;
        }
        targetXPct = t.x;
        targetYPct = t.y;
        refXPct = r.x;
        refYPct = r.y;
      }

      if (!form.colour || !form.reference) {
        setError("Fill in colour and reference description");
        return;
      }

      const isDemo = !imagePair.pk || imagePair.pk.startsWith("demo-");
      try {
        setIsSubmitting(true);
        setError("");
        let desc: string;
        if (isDemo) {
          const dirLabel = tg ? "ground" : "wall";
          desc = `[DEMO] The target is ${form.colour}, located on the ${dirLabel} relative to the ${form.reference}.`;
        } else {
          const result = await apiGenerateOutput({
            pk: imagePair.pk,
            target_x_pct: targetXPct,
            target_y_pct: targetYPct,
            reference_x_pct: refXPct,
            reference_y_pct: refYPct,
            mode: "aided",
            color: form.colour,
            ref_description: form.reference,
            target_on_ground: tg,
          });
          desc = result.desc;
          // Replace images with annotated versions from backend
          setImagePair((prev) => prev ? {
            ...prev,
            forwardUrl: `data:image/jpeg;base64,${result.oakd_image}`,
            downwardUrl: `data:image/jpeg;base64,${result.ardu_image}`,
          } : prev);
        }
        setPendingCoords({ targetXPct, targetYPct, refXPct, refYPct });
        setGeneratedOutput(desc);
        setOutputPending(true);
        setTimeout(() => outputBoxRef.current?.focus(), 50);
      } catch (err) {
        setError(err instanceof Error ? err.message : "Failed to generate output");
      } finally {
        setIsSubmitting(false);
      }
    } else {
      // Phase 2: write to descriptions.txt and reset
      const isDemo = !imagePair.pk || imagePair.pk.startsWith("demo-");
      try {
        setIsSubmitting(true);
        setError("");
        let savedId: string;
        if (isDemo) {
          const isEditingExisting = imagePair.pk.startsWith("demo-");
          savedId = isEditingExisting ? imagePair.pk : `demo-${Date.now()}`;
          const existingCapture = captures.find((c) => c.id === savedId);
          const newEntry: Capture = {
            id: savedId,
            time: existingCapture?.time ?? new Date().toISOString().replace(/[-:.TZ]/g, "").slice(0, 15),
            desc: generatedOutput,
            direction: targetOnGround ? "D" : "F",
            colour: form.colour,
            reference: form.reference,
            green: pendingCoords ? { x: pendingCoords.targetXPct, y: pendingCoords.targetYPct } : null,
            red: pendingCoords ? { x: pendingCoords.refXPct, y: pendingCoords.refYPct } : null,
            imageUrl: imagePair.forwardUrl,
          };
          if (isEditingExisting) {
            setCaptures((prev) => prev.map((c) => c.id === savedId ? newEntry : c));
          } else {
            setCaptures((prev) => [newEntry, ...prev]);
          }
        } else {
          await apiSaveById(imagePair.pk);
          savedId = imagePair.pk;
          const raw = await apiFetchCaptures();
          const data = parseCapturesDict(raw);
          setCaptures(data);
        }
        setSelectedId(savedId);
        setForm({ colour: "", reference: "" });
        setPendingCoords(null);
        setImagePair(null);
        setForwardAnnotation(null);
        setDownwardAnnotation(null);
        setOutputPending(false);
        setGeneratedOutput(null);
        setError("");
      } catch (err) {
        setError(err instanceof Error ? err.message : "Failed to save");
        setOutputPending(false);
      } finally {
        setIsSubmitting(false);
      }
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
          outputPending={outputPending}
          onSubmit={() => void handleSubmit()}
          isSubmitting={isSubmitting}
          output={output}
          colourRef={colourRef}
          referenceRef={referenceRef}
          outputBoxRef={outputBoxRef}
          error={error}
          onOpenPopup={handleOpenPopup}
          forwardGreen={targetOnGround ? null : (forwardAnnotation?.target ?? null)}
          forwardRed={forwardAnnotation?.ref ?? null}
          downwardGreen={downwardAnnotation?.target ?? null}
        />

        <CaptureHistory
          captures={captures}
          selectedId={selectedId}
          onSelect={setSelectedId}
          isLoading={isLoading}
          selectedCapture={selectedCapture}
          onDelete={(id) => void handleDelete(id)}
          onEdit={(id) => void handleEdit(id)}
        />

        <button
          onClick={() => void (async () => {
            try {
              const msg = await apiSaveAll();
              setError("");
              alert(msg);
            } catch (err) {
              setError(err instanceof Error ? err.message : "Failed to save descriptions");
            }
          })()}
          className="w-full py-2 bg-zinc-800 hover:bg-zinc-700 text-white text-sm font-medium rounded"
        >
          Save Descriptions
        </button>
      </div>
    </main>
  );
}

export default App;
