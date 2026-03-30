import { useCallback, useEffect } from "react";
import { useServiceCall } from "./useServiceCall";
import { useTopicSubscription } from "./useTopicSubscription";
import { useSkillStore } from "../stores/skill-store";
import { useConnectionStore } from "../stores/connection-store";
import type {
  GetSkillDescriptionsRequest,
  GetSkillDescriptionsResponse,
  SkillDescription,
} from "../types/ros";

/** Fetch skills on connect and subscribe to live updates. */
export function useSkillRegistry() {
  const connected = useConnectionStore((s) => s.connected);
  const { setSkills, setLoading, setError } = useSkillStore();

  const { call } = useServiceCall<
    GetSkillDescriptionsRequest,
    GetSkillDescriptionsResponse
  >(
    "/skill_server/get_skill_descriptions",
    "robot_skills_msgs/srv/GetSkillDescriptions"
  );

  // Fetch all skills when connected
  useEffect(() => {
    if (!connected) return;

    setLoading(true);
    call({
      filter_categories: [],
      filter_tags: [],
      include_compounds: true,
      include_pddl: true,
    })
      .then((res) => {
        if (res.success) {
          setSkills(res.skills);
        } else {
          setError(res.message);
        }
      })
      .catch((err) => setError(err.message));
  }, [connected, call, setSkills, setLoading, setError]);

  // Subscribe to live updates (JSON string of skill names)
  const handleUpdate = useCallback(
    (_msg: { data: string }) => {
      // Re-fetch full descriptions when the skill list changes
      call({
        filter_categories: [],
        filter_tags: [],
        include_compounds: true,
        include_pddl: true,
      })
        .then((res) => {
          if (res.success) setSkills(res.skills);
        })
        .catch(() => {});
    },
    [call, setSkills]
  );

  useTopicSubscription<{ data: string }>(
    "/skill_server/available_skills",
    "std_msgs/msg/String",
    handleUpdate
  );
}
