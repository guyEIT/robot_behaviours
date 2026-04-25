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
        theme="light"
        toastOptions={{
          style: {
            background: "#FFFFFF",
            border: "1px solid #D7D7D7",
            color: "#1E1E1E",
            borderRadius: "6px",
            fontFamily: '"Source Sans 3", "Helvetica Neue", system-ui, sans-serif',
            fontSize: "14px",
          },
        }}
      />
    </>
  );
}
