import { useState, useEffect, useCallback } from "react";
import ROSLIB from "roslib";
import { getRos } from "../../lib/rosbridge-client";
import { useConnectionStore } from "../../stores/connection-store";
import {
  PhoneCall,
  Search,
  Play,
  Loader2,
  ChevronRight,
  Clock,
} from "lucide-react";
import clsx from "clsx";

interface ServiceInfo {
  name: string;
  type: string;
}

interface CallRecord {
  id: number;
  service: string;
  request: string;
  response: string | null;
  error: string | null;
  durationMs: number | null;
  timestamp: Date;
}

export default function ServiceCallerPanel() {
  const connected = useConnectionStore((s) => s.connected);
  const [services, setServices] = useState<ServiceInfo[]>([]);
  const [search, setSearch] = useState("");
  const [selectedService, setSelectedService] = useState<ServiceInfo | null>(null);
  const [requestJson, setRequestJson] = useState("{}");
  const [calling, setCalling] = useState(false);
  const [history, setHistory] = useState<CallRecord[]>([]);
  const [nextId, setNextId] = useState(0);

  // Fetch service names only (types are resolved on selection)
  useEffect(() => {
    if (!connected) return;
    const ros = getRos();
    let cancelled = false;

    function fetch() {
      ros.getServices(
        (svcNames: string[]) => {
          if (cancelled) return;
          const infos: ServiceInfo[] = svcNames.map((name) => ({
            name,
            type: "",
          }));
          infos.sort((a, b) => {
            const aSk = a.name.includes("skill_server") ? 0 : 1;
            const bSk = b.name.includes("skill_server") ? 0 : 1;
            if (aSk !== bSk) return aSk - bSk;
            return a.name.localeCompare(b.name);
          });
          setServices(infos);
        },
        (err: string) => {
          console.warn("Failed to list services:", err);
        }
      );
    }

    fetch();
    const interval = setInterval(fetch, 30000);
    return () => {
      cancelled = true;
      clearInterval(interval);
    };
  }, [connected]);

  const filtered = search
    ? services.filter(
        (s) =>
          s.name.toLowerCase().includes(search.toLowerCase()) ||
          s.type.toLowerCase().includes(search.toLowerCase())
      )
    : services;

  const handleCall = useCallback(async () => {
    if (!selectedService) return;
    setCalling(true);
    const id = nextId;
    setNextId((prev) => prev + 1);

    const start = performance.now();
    let parsed: any;
    try {
      parsed = JSON.parse(requestJson);
    } catch {
      const record: CallRecord = {
        id,
        service: selectedService.name,
        request: requestJson,
        response: null,
        error: "Invalid JSON in request",
        durationMs: null,
        timestamp: new Date(),
      };
      setHistory((prev) => [record, ...prev].slice(0, 50));
      setCalling(false);
      return;
    }

    const service = new ROSLIB.Service({
      ros: getRos(),
      name: selectedService.name,
      serviceType: selectedService.type,
    });

    service.callService(
      new ROSLIB.ServiceRequest(parsed),
      (response: any) => {
        const record: CallRecord = {
          id,
          service: selectedService.name,
          request: requestJson,
          response: JSON.stringify(response, null, 2),
          error: null,
          durationMs: Math.round(performance.now() - start),
          timestamp: new Date(),
        };
        setHistory((prev) => [record, ...prev].slice(0, 50));
        setCalling(false);
      },
      (error: string) => {
        const record: CallRecord = {
          id,
          service: selectedService.name,
          request: requestJson,
          response: null,
          error,
          durationMs: Math.round(performance.now() - start),
          timestamp: new Date(),
        };
        setHistory((prev) => [record, ...prev].slice(0, 50));
        setCalling(false);
      }
    );
  }, [selectedService, requestJson, nextId]);

  // Resolve type lazily on selection, then apply template
  const handleSelectService = (svc: ServiceInfo) => {
    const applyTemplate = (resolved: ServiceInfo) => {
      const templates: Record<string, string> = {
        "robot_skills_msgs/srv/GetSkillDescriptions": JSON.stringify(
          { filter_categories: [], filter_tags: [], include_compounds: true, include_pddl: false },
          null, 2
        ),
        "robot_skills_msgs/srv/ComposeTask": JSON.stringify(
          {
            task_name: "my_task",
            task_description: "",
            steps: [{ skill_name: "move_to_named_config", parameters_json: '{"config_name":"home"}', input_blackboard_keys: [], output_blackboard_keys: [], retry_on_failure: false, max_retries: 0, condition_expression: "", description: "Go home" }],
            sequential: true,
            add_precondition_checks: false,
          },
          null, 2
        ),
      };
      setRequestJson(templates[resolved.type] || "{}");
    };

    if (svc.type) {
      setSelectedService(svc);
      applyTemplate(svc);
    } else {
      // Resolve type on first selection
      setSelectedService(svc);
      setRequestJson("{}");
      getRos().getServiceType(
        svc.name,
        (type: string) => {
          const resolved = { ...svc, type };
          setSelectedService(resolved);
          setServices((prev) =>
            prev.map((s) => (s.name === svc.name ? resolved : s))
          );
          applyTemplate(resolved);
        },
        () => {}
      );
    }
  };

  return (
    <div className="flex h-full">
      {/* Left: service list */}
      <div className="w-80 border-r border-gray-800 flex flex-col shrink-0">
        <div className="px-3 py-2 border-b border-gray-800">
          <div className="flex items-center gap-2 mb-2">
            <PhoneCall className="w-4 h-4 text-blue-400" />
            <h2 className="text-sm font-semibold">Services</h2>
            <span className="text-xs text-gray-500">{services.length}</span>
          </div>
          <div className="relative">
            <Search className="absolute left-2 top-1/2 -translate-y-1/2 w-3.5 h-3.5 text-gray-500" />
            <input
              type="text"
              placeholder="Filter services..."
              value={search}
              onChange={(e) => setSearch(e.target.value)}
              className="w-full pl-7 pr-3 py-1 text-xs bg-gray-800 border border-gray-700 rounded focus:border-blue-500 focus:outline-none text-gray-200 placeholder-gray-500"
            />
          </div>
        </div>

        <div className="flex-1 overflow-auto">
          {filtered.map((svc) => (
            <div
              key={svc.name}
              onClick={() => handleSelectService(svc)}
              className={clsx(
                "px-3 py-1.5 border-b border-gray-800/50 cursor-pointer hover:bg-gray-800/50 transition-colors flex items-center gap-2",
                selectedService?.name === svc.name && "bg-gray-800/80"
              )}
            >
              <div className="flex-1 min-w-0">
                <div className="text-[11px] text-gray-200 truncate font-mono">
                  {svc.name}
                </div>
                <div className="text-[9px] text-gray-500 truncate">
                  {svc.type || "unknown"}
                </div>
              </div>
              <ChevronRight className="w-3 h-3 text-gray-700 shrink-0" />
            </div>
          ))}
          {filtered.length === 0 && (
            <p className="text-xs text-gray-600 text-center py-8">
              {connected ? "No services found" : "Not connected"}
            </p>
          )}
        </div>
      </div>

      {/* Right: caller + history */}
      <div className="flex-1 flex flex-col overflow-hidden">
        {selectedService ? (
          <>
            {/* Service header */}
            <div className="px-4 py-2 border-b border-gray-800">
              <div className="text-xs font-mono text-gray-200">
                {selectedService.name}
              </div>
              <div className="text-[10px] text-gray-500">
                {selectedService.type}
              </div>
            </div>

            {/* Request editor */}
            <div className="p-3 border-b border-gray-800">
              <div className="text-[10px] text-gray-500 font-medium uppercase mb-1">
                Request
              </div>
              <textarea
                value={requestJson}
                onChange={(e) => setRequestJson(e.target.value)}
                rows={6}
                className="w-full px-2 py-1.5 text-[10px] font-mono bg-gray-800 border border-gray-700 rounded focus:border-blue-500 focus:outline-none text-gray-300 resize-y"
              />
              <button
                onClick={handleCall}
                disabled={calling}
                className="mt-1.5 flex items-center gap-1.5 px-3 py-1.5 rounded bg-blue-600 hover:bg-blue-500 text-xs font-semibold text-white disabled:bg-gray-700 disabled:text-gray-400"
              >
                {calling ? (
                  <Loader2 className="w-3 h-3 animate-spin" />
                ) : (
                  <Play className="w-3 h-3" />
                )}
                Call Service
              </button>
            </div>

            {/* History */}
            <div className="flex-1 overflow-auto p-3 space-y-2">
              <div className="text-[10px] text-gray-500 font-medium uppercase">
                Call History
              </div>
              {history
                .filter((r) => r.service === selectedService.name)
                .map((record) => (
                  <div
                    key={record.id}
                    className={clsx(
                      "p-2 rounded border text-[10px]",
                      record.error
                        ? "border-red-800/50 bg-red-950/20"
                        : "border-gray-800 bg-gray-900/50"
                    )}
                  >
                    <div className="flex items-center gap-2 mb-1">
                      <span className="text-gray-500">
                        {record.timestamp.toLocaleTimeString()}
                      </span>
                      {record.durationMs != null && (
                        <span className="flex items-center gap-0.5 text-gray-500">
                          <Clock className="w-2.5 h-2.5" />
                          {record.durationMs}ms
                        </span>
                      )}
                      <span
                        className={
                          record.error ? "text-red-400 font-bold" : "text-green-400 font-bold"
                        }
                      >
                        {record.error ? "ERROR" : "OK"}
                      </span>
                    </div>
                    {record.error && (
                      <pre className="text-red-300 whitespace-pre-wrap">{record.error}</pre>
                    )}
                    {record.response && (
                      <pre className="text-gray-300 whitespace-pre-wrap max-h-40 overflow-auto">
                        {record.response}
                      </pre>
                    )}
                  </div>
                ))}
            </div>
          </>
        ) : (
          <div className="flex flex-col items-center justify-center h-full text-gray-500">
            <PhoneCall className="w-8 h-8 mb-2 opacity-30" />
            <p className="text-sm">Select a service</p>
            <p className="text-xs">Click a service to call it</p>
          </div>
        )}
      </div>
    </div>
  );
}
