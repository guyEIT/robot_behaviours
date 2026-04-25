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
import { Button, Eyebrow, Textarea, Chip } from "../ui";
import type { ChipState } from "../ui";

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

  const handleSelectService = (svc: ServiceInfo) => {
    const applyTemplate = (resolved: ServiceInfo) => {
      const templates: Record<string, string> = {
        "robot_skills_msgs/srv/GetSkillDescriptions": JSON.stringify(
          { filter_categories: [], filter_tags: [], include_compounds: true, include_pddl: false },
          null,
          2,
        ),
        "robot_skills_msgs/srv/ComposeTask": JSON.stringify(
          {
            task_name: "my_task",
            task_description: "",
            steps: [
              {
                skill_name: "move_to_named_config",
                parameters_json: '{"config_name":"home"}',
                input_blackboard_keys: [],
                output_blackboard_keys: [],
                retry_on_failure: false,
                max_retries: 0,
                condition_expression: "",
                description: "Go home",
              },
            ],
            sequential: true,
            add_precondition_checks: false,
          },
          null,
          2,
        ),
      };
      setRequestJson(templates[resolved.type] || "{}");
    };

    if (svc.type) {
      setSelectedService(svc);
      applyTemplate(svc);
    } else {
      setSelectedService(svc);
      setRequestJson("{}");
      getRos().getServiceType(
        svc.name,
        (type: string) => {
          const resolved = { ...svc, type };
          setSelectedService(resolved);
          setServices((prev) => prev.map((s) => (s.name === svc.name ? resolved : s)));
          applyTemplate(resolved);
        },
        () => {},
      );
    }
  };

  return (
    <div className="flex h-full bg-paper">
      {/* Left: service list */}
      <div className="w-80 border-r border-hair flex flex-col shrink-0">
        <div className="px-4 py-3 border-b border-hair">
          <div className="flex items-center gap-2 mb-2">
            <PhoneCall className="w-4 h-4 text-terracotta" />
            <h2 className="text-[14px] font-medium text-ink">Services</h2>
            <Eyebrow size="sm" tone="muted">{services.length}</Eyebrow>
          </div>
          <div className="relative">
            <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-3.5 h-3.5 text-muted" />
            <input
              type="text"
              placeholder="Filter services…"
              value={search}
              onChange={(e) => setSearch(e.target.value)}
              className="w-full pl-9 pr-3 py-1.5 text-[12px] bg-paper border border-hair rounded-DEFAULT focus:border-terracotta focus:outline-none text-ink-soft placeholder:text-muted-2"
            />
          </div>
        </div>

        <div className="flex-1 overflow-auto">
          {filtered.map((svc) => (
            <div
              key={svc.name}
              onClick={() => handleSelectService(svc)}
              className={clsx(
                "px-3 py-2 border-b border-hair-soft border-l-2 cursor-pointer hover:bg-cream transition-colors flex items-center gap-2",
                selectedService?.name === svc.name
                  ? "border-l-terracotta bg-terracotta-tint"
                  : "border-l-transparent",
              )}
            >
              <div className="flex-1 min-w-0">
                <div className="text-[11.5px] text-ink truncate font-mono tracking-[0.04em]">
                  {svc.name}
                </div>
                <div className="text-[10px] text-muted truncate">{svc.type || "unknown"}</div>
              </div>
              <ChevronRight className="w-3 h-3 text-muted shrink-0" />
            </div>
          ))}
          {filtered.length === 0 && (
            <p className="text-[12px] text-muted text-center py-8">
              {connected ? "No services found" : "Not connected"}
            </p>
          )}
        </div>
      </div>

      {/* Right: caller + history */}
      <div className="flex-1 flex flex-col overflow-hidden">
        {selectedService ? (
          <>
            <div className="px-5 py-3 border-b border-hair">
              <div className="text-[13px] font-mono text-ink truncate tracking-[0.04em]">
                {selectedService.name}
              </div>
              <div className="text-[11px] text-muted font-mono mt-0.5 tracking-[0.04em]">
                {selectedService.type}
              </div>
            </div>

            <div className="p-4 border-b border-hair-soft bg-cream-deep">
              <Eyebrow size="sm" className="block mb-2">Request</Eyebrow>
              <Textarea
                value={requestJson}
                onChange={(e) => setRequestJson(e.target.value)}
                rows={6}
                mono
                className="resize-y"
              />
              <Button
                onClick={handleCall}
                disabled={calling}
                variant="primary"
                size="sm"
                leftIcon={
                  calling ? <Loader2 className="w-3 h-3 animate-spin" /> : <Play className="w-3 h-3" />
                }
                className="mt-3"
              >
                Call Service
              </Button>
            </div>

            <div className="flex-1 overflow-auto p-4 space-y-2">
              <Eyebrow size="sm" tone="muted" className="block">Call History</Eyebrow>
              {history
                .filter((r) => r.service === selectedService.name)
                .map((record) => {
                  const chipState: ChipState = record.error ? "failed" : "done";
                  return (
                    <div
                      key={record.id}
                      className={clsx(
                        "p-3 border text-[11px]",
                        record.error ? "border-err bg-err-soft" : "border-hair bg-paper",
                      )}
                    >
                      <div className="flex items-center gap-2 mb-2">
                        <span className="text-muted font-mono tracking-[0.04em]">
                          {record.timestamp.toLocaleTimeString()}
                        </span>
                        {record.durationMs != null && (
                          <span className="flex items-center gap-1 text-muted font-mono tracking-[0.04em]">
                            <Clock className="w-2.5 h-2.5" />
                            {record.durationMs}ms
                          </span>
                        )}
                        <Chip state={chipState}>{record.error ? "Error" : "OK"}</Chip>
                      </div>
                      {record.error && (
                        <pre className="text-err whitespace-pre-wrap font-mono tracking-[0.02em]">
                          {record.error}
                        </pre>
                      )}
                      {record.response && (
                        <pre className="sociius-code max-h-40 whitespace-pre-wrap text-[10.5px]">
                          {record.response}
                        </pre>
                      )}
                    </div>
                  );
                })}
            </div>
          </>
        ) : (
          <div className="flex flex-col items-center justify-center h-full text-muted bg-cream-deep">
            <PhoneCall className="w-8 h-8 mb-2 opacity-30" />
            <p className="text-[14px] text-ink-soft">Select a service</p>
            <p className="text-[12px]">Click a service to call it</p>
          </div>
        )}
      </div>
    </div>
  );
}
