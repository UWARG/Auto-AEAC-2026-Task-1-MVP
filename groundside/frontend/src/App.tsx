import { useEffect, useState } from "react";
import type { SubmitEventHandler } from "react";

type Capture = {
  time: string;
  filename: string;
  desc: string;
};

const emptyForm = () => ({
  filename: "",
  desc: "",
  time: new Date().toISOString().slice(0, 16),
});

function App() {
  const [captures, setCaptures] = useState<Capture[]>([]);
  const [form, setForm] = useState(emptyForm);
  const [isLoading, setIsLoading] = useState(true);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [isResetting, setIsResetting] = useState(false);
  const [error, setError] = useState("");

  async function loadCaptures() {
    try {
      setIsLoading(true);
      setError("");

      const response = await fetch("/api/captures");
      if (!response.ok) {
        throw new Error("Failed to load captures");
      }

      const data: Capture[] = await response.json();
      setCaptures(data);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to load captures");
    } finally {
      setIsLoading(false);
    }
  }

  useEffect(() => {
    void loadCaptures();
  }, []);

  const handleSubmit: SubmitEventHandler<HTMLFormElement> = async (event) => {
    event.preventDefault();

    try {
      setIsSubmitting(true);
      setError("");

      const payload = {
        ...form,
        time: form.time ? new Date(form.time).toISOString() : "",
      };

      const response = await fetch("/api/captures", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        const data = await response.json().catch(() => null);
        throw new Error(data?.message ?? "Failed to save capture");
      }

      const created: Capture = await response.json();
      setCaptures((current) => [...current, created]);
      setForm(emptyForm());
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to save capture");
    } finally {
      setIsSubmitting(false);
    }
  };

  async function handleReset() {
    try {
      setIsResetting(true);
      setError("");

      const response = await fetch("/api/db_reset", {
        method: "POST",
      });

      if (!response.ok) {
        throw new Error("Failed to reset database");
      }

      setCaptures([]);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to reset database");
    } finally {
      setIsResetting(false);
    }
  }

  return (
    <main className="flex w-full max-w-xl flex-col gap-8 px-4 py-10">
      <header>
        <h1 className="font-[Inter,sans-serif] text-lg font-medium text-zinc-100">
          Groundside
        </h1>
      </header>

      <section className="rounded border border-zinc-800 bg-zinc-950 p-4">
        <form className="grid gap-4" onSubmit={handleSubmit}>
          <div className="grid gap-4 sm:grid-cols-2">
            <label className="grid min-w-0 gap-1">
              <span className="text-sm text-zinc-400">Filename</span>
              <input
                className="min-w-0 rounded border border-zinc-800 bg-zinc-900 px-3 py-2 text-sm text-zinc-100 outline-none focus:border-zinc-500"
                required
                value={form.filename}
                onChange={(event) =>
                  setForm((current) => ({
                    ...current,
                    filename: event.target.value,
                  }))
                }
                placeholder="capture_002.jpg"
              />
            </label>

            <label className="grid min-w-0 gap-1">
              <span className="text-sm text-zinc-400">Timestamp</span>
              <input
                className="min-w-0 rounded border border-zinc-800 bg-zinc-900 px-3 py-2 text-sm text-zinc-100 outline-none focus:border-zinc-500"
                type="datetime-local"
                value={form.time}
                onChange={(event) =>
                  setForm((current) => ({
                    ...current,
                    time: event.target.value,
                  }))
                }
              />
            </label>
          </div>

          <label className="grid gap-1">
            <span className="text-sm text-zinc-400">Description</span>
            <textarea
              className="min-h-24 rounded border border-zinc-800 bg-zinc-900 px-3 py-2 text-sm text-zinc-100 outline-none focus:border-zinc-500"
              required
              rows={2}
              value={form.desc}
              onChange={(event) =>
                setForm((current) => ({ ...current, desc: event.target.value }))
              }
              placeholder="Short note about the capture"
            />
          </label>

          <button
            className="w-fit rounded border border-zinc-700 bg-zinc-100 px-3 py-1.5 text-sm text-zinc-950 disabled:cursor-wait disabled:opacity-60"
            type="submit"
            disabled={isSubmitting}
          >
            {isSubmitting ? "Saving..." : "Add entry"}
          </button>
        </form>

        {error ? <p className="mt-3 text-sm text-red-400">{error}</p> : null}
      </section>

      <section className="rounded border border-zinc-800 bg-zinc-950 p-4">
        <div className="mb-4 flex items-center justify-between gap-3">
          <h2 className="font-['JetBrains_Mono',monospace] text-sm text-zinc-100">
            Entries
          </h2>
          <div className="flex gap-3 text-sm">
            <button
              type="button"
              className="text-zinc-400 disabled:cursor-wait disabled:opacity-60"
              onClick={() => void loadCaptures()}
              disabled={isLoading}
            >
              {isLoading ? "Refreshing..." : "Refresh"}
            </button>
            <button
              type="button"
              className="text-red-400 disabled:cursor-wait disabled:opacity-60"
              onClick={() => void handleReset()}
              disabled={isResetting}
            >
              {isResetting ? "Resetting..." : "Reset"}
            </button>
          </div>
        </div>

        {isLoading ? (
          <p className="text-sm text-zinc-400">Loading...</p>
        ) : captures.length === 0 ? (
          <p className="text-sm text-zinc-400">No entries found.</p>
        ) : (
          <ul className="grid gap-2">
            {captures.map((capture, index) => (
              <li
                key={`${capture.filename}-${capture.time}-${index}`}
                className="rounded border border-zinc-800 bg-zinc-900 px-3 py-2"
              >
                <p className="text-sm font-medium text-zinc-100">
                  {capture.filename}
                </p>
                <p className="mt-1 text-sm text-zinc-400">{capture.desc}</p>
                <p className="mt-1 font-['JetBrains_Mono',monospace] text-xs text-zinc-500">
                  {capture.time}
                </p>
              </li>
            ))}
          </ul>
        )}
      </section>
    </main>
  );
}

export default App;
