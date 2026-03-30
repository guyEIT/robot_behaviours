import { useRosBridge } from "./hooks/useRosBridge";
import { useTaskState } from "./hooks/useTaskState";
import { useBtRunnerStatus } from "./hooks/useBtRunnerStatus";
import { useSkillRegistry } from "./hooks/useSkillRegistry";
import { useRosout } from "./hooks/useRosout";
import { useDiagnostics } from "./hooks/useDiagnostics";
import { useTopicList } from "./hooks/useTopicList";
import DashboardShell from "./components/layout/DashboardShell";

export default function App() {
  const connected = useRosBridge();

  // Start all topic subscriptions
  useTaskState();
  useBtRunnerStatus();
  useSkillRegistry();
  useRosout();
  useDiagnostics();
  useTopicList();

  return <DashboardShell connected={connected} />;
}
