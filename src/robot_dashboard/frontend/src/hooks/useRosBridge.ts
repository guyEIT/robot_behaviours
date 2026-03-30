import { useEffect } from "react";
import { useConnectionStore } from "../stores/connection-store";

/** Initialize the rosbridge connection on mount. */
export function useRosBridge() {
  const init = useConnectionStore((s) => s.init);
  const connected = useConnectionStore((s) => s.connected);

  useEffect(() => {
    init();
  }, [init]);

  return connected;
}
