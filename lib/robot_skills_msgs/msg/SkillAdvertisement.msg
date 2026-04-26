# One self-advertised skill, published as part of a SkillManifest.
# Wraps an existing SkillDescription with the BT-binding fields the
# behavior tree executor needs to map an XML tag onto an action client.

robot_skills_msgs/SkillDescription description

# BT XML tag used to invoke this skill. Defaults to PascalCase(description.name)
# if left empty.
string bt_tag

# Default values for XML attributes when not present on the BT node.
robot_skills_msgs/KeyValue[] goal_defaults

# Result-field renames for cases where the action result field name differs
# from the XML attribute name written to the blackboard.
# key = action result field, value = blackboard / xml attr name.
robot_skills_msgs/KeyValue[] output_renames

# Identifier of a post-processor registered in tree_executor._POST_PROCESSORS.
# Empty string means none.
string post_process_id

# Explicit whitelist of XML attributes the BT executor should accept as
# inputs. Empty means: derive from action goal field names by reflection.
string[] input_xml_attrs
