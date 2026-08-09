import { useEffect, useState } from "react";
import { getMap } from "../api";
import type { MapInfo } from "../types";

export function useMapInfo(name: string | undefined) {
  const [info, setInfo] = useState<MapInfo | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (!name) return;
    let cancelled = false;
    setLoading(true);
    setError(null);
    getMap(name)
      .then((i) => !cancelled && setInfo(i))
      .catch((e) => !cancelled && setError(String(e)))
      .finally(() => !cancelled && setLoading(false));
    return () => {
      cancelled = true;
    };
  }, [name]);

  return { info, error, loading };
}
