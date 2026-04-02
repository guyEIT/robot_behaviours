import { useCallback, useRef, useState } from "react";
import ROSLIB from "roslib";
import { toast } from "sonner";
import { getRos } from "../lib/rosbridge-client";

/**
 * Hook for calling a ROS2 service via rosbridge.
 * Returns { call, loading, error }.
 * Automatically shows a toast on service call failure.
 */
export function useServiceCall<Req, Res>(
  serviceName: string,
  serviceType: string
) {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const serviceRef = useRef<ROSLIB.Service | null>(null);

  const call = useCallback(
    (request: Req): Promise<Res> => {
      setLoading(true);
      setError(null);

      if (!serviceRef.current) {
        serviceRef.current = new ROSLIB.Service({
          ros: getRos(),
          name: serviceName,
          serviceType,
        });
      }

      return new Promise((resolve, reject) => {
        const req = new ROSLIB.ServiceRequest(request as any);
        serviceRef.current!.callService(
          req,
          (res: any) => {
            setLoading(false);
            resolve(res as Res);
          },
          (err: string) => {
            setLoading(false);
            setError(err);
            toast.error("Service call failed", {
              description: `${serviceName}: ${err}`,
              duration: 6000,
            });
            reject(new Error(err));
          }
        );
      });
    },
    [serviceName, serviceType]
  );

  return { call, loading, error };
}
