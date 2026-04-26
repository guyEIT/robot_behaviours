# Latched skill advertisement payload published by any node that hosts
# one or more skill action servers, on topic <node_fqn>/skills.
#
# QoS: TRANSIENT_LOCAL durability + RELIABLE + KEEP_LAST(1) +
#      LIVELINESS_AUTOMATIC(10s) so SkillDiscovery can detect publisher
#      death without a heartbeat companion topic.

robot_skills_msgs/SkillAdvertisement[] skills
builtin_interfaces/Time published_at
string source_node              # FQN of the publishing node (diagnostics only)
