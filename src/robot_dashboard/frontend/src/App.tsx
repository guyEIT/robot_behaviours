import { Toaster } from "sonner";
import { useRosBridge } from "./hooks/useRosBridge";
import { useTaskState } from "./hooks/useTaskState";
import { useBtRunnerStatus } from "./hooks/useBtRunnerStatus";
import { useSkillRegistry } from "./hooks/useSkillRegistry";
import { useRosout } from "./hooks/useRosout";
import { useDiagnostics } from "./hooks/useDiagnostics";
import { useTopicList } from "./hooks/useTopicList";
import { useTaskNotifications } from "./hooks/useTaskNotifications";
import { useHumanPrompts } from "./hooks/useHumanPrompts";
import DashboardShell from "./components/layout/DashboardShell";
import HumanPromptOverlay from "./components/human-prompts/HumanPromptOverlay";

export default function App() {
  const connected = useRosBridge();

  // Start all topic subscriptions
  useTaskState();
  useBtRunnerStatus();
  useSkillRegistry();
  useRosout();
  useDiagnostics();
  useTopicList();

  // Toast notifications + audio cues for task state changes
  useTaskNotifications();
  // Human interaction prompts from BT nodes
  useHumanPrompts();

  return (
    <>
      <DashboardShell connected={connected} />
      <HumanPromptOverlay />
      <Toaster
        position="bottom-right"
        theme="dark"
        richColors
        toastOptions={{
          style: {
            background: "#1a1a2e",
            border: "1px solid #374151",
          },
        }}
      />
    </>
  );
}
