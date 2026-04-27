"""Unit tests for the pragmatic PDDL plan validator.

Pure stdlib. No ROS deps, no skill_server fixtures.
"""

import os
import sys
import unittest

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from robot_skill_server.pddl_validator import (  # noqa: E402
    PlanValidationResult,
    SkillSpec,
    apply_effects,
    parse_effect,
    parse_precondition,
    validate_plan,
)


# ── String parsing ───────────────────────────────────────────────────────

class TestParseEffect(unittest.TestCase):
    def test_bare_fact_is_add(self):
        self.assertEqual(parse_effect("gripper_open"), ("gripper_open", True))

    def test_plus_prefix(self):
        self.assertEqual(parse_effect("+gripper_open"), ("gripper_open", True))
        self.assertEqual(parse_effect("add gripper_open"), ("gripper_open", True))

    def test_minus_prefix(self):
        self.assertEqual(parse_effect("-gripper_open"), ("gripper_open", False))
        self.assertEqual(parse_effect("del gripper_open"), ("gripper_open", False))
        self.assertEqual(parse_effect("not gripper_open"), ("gripper_open", False))

    def test_whitespace_normalised(self):
        self.assertEqual(parse_effect("  +  ready  "), ("ready", True))
        self.assertEqual(parse_effect("  not  ready  "), ("ready", False))

    def test_empty_yields_empty_fact(self):
        self.assertEqual(parse_effect(""), ("", True))
        self.assertEqual(parse_effect("   "), ("", True))


class TestApplyEffects(unittest.TestCase):
    def test_postconditions_add(self):
        out = apply_effects(set(), [], ["robot_initialized", "gripper_open"])
        self.assertEqual(out, {"robot_initialized", "gripper_open"})

    def test_effects_can_delete(self):
        out = apply_effects(
            {"gripper_open", "ready"},
            ["-gripper_open"],
            [],
        )
        self.assertEqual(out, {"ready"})

    def test_effects_override_postconditions(self):
        # postcondition adds X, effect removes X — net result: X gone.
        out = apply_effects(set(), ["-gripper_open"], ["gripper_open"])
        self.assertEqual(out, set())

    def test_returns_new_set(self):
        s = {"a"}
        out = apply_effects(s, [], ["b"])
        self.assertEqual(s, {"a"})       # original untouched
        self.assertEqual(out, {"a", "b"})


# ── End-to-end validation ────────────────────────────────────────────────

def _step(skill: str, **params) -> dict:
    return {"skill_name": skill, "parameters_json": "{}"}


class TestValidatePlan(unittest.TestCase):
    def test_empty_plan_is_valid(self):
        r = validate_plan([], {}, [])
        self.assertTrue(r.valid)
        self.assertEqual(r.first_failing_step, -1)
        self.assertEqual(r.final_state, [])

    def test_unknown_skill_fails(self):
        r = validate_plan([_step("phantom")], {}, [])
        self.assertFalse(r.valid)
        self.assertEqual(r.first_failing_step, 0)
        self.assertIn("unknown skill", r.message)

    def test_empty_skill_name_fails(self):
        r = validate_plan([{"skill_name": "", "parameters_json": "{}"}], {}, [])
        self.assertFalse(r.valid)
        self.assertEqual(r.first_failing_step, 0)
        self.assertIn("empty skill_name", r.message)

    def test_initial_state_satisfies_preconditions(self):
        skills = {
            "move_home": SkillSpec(
                name="move_home",
                preconditions=("robot_initialized",),
                postconditions=("at_home",),
            ),
        }
        r = validate_plan(
            [_step("move_home")], skills, ["robot_initialized"]
        )
        self.assertTrue(r.valid)
        self.assertIn("at_home", r.final_state)
        self.assertIn("robot_initialized", r.final_state)

    def test_missing_precondition_fails(self):
        skills = {
            "pick": SkillSpec(
                name="pick",
                preconditions=("gripper_open", "object_detected"),
                postconditions=("object_grasped",),
            ),
        }
        r = validate_plan(
            [_step("pick")], skills, ["gripper_open"]
        )
        self.assertFalse(r.valid)
        self.assertEqual(r.first_failing_step, 0)
        self.assertEqual(r.missing_preconditions, ["object_detected"])

    def test_chain_satisfies_via_postconditions(self):
        """A → B where B's precondition is A's postcondition."""
        skills = {
            "init": SkillSpec(
                name="init",
                postconditions=("robot_initialized",),
            ),
            "move_home": SkillSpec(
                name="move_home",
                preconditions=("robot_initialized",),
                postconditions=("at_home",),
            ),
        }
        r = validate_plan(
            [_step("init"), _step("move_home")], skills, []
        )
        self.assertTrue(r.valid)
        self.assertEqual(set(r.final_state), {"robot_initialized", "at_home"})

    def test_effect_deletes_breaks_chain(self):
        """A removes a fact that B needs."""
        skills = {
            "open_gripper": SkillSpec(
                name="open_gripper",
                postconditions=("gripper_open",),
            ),
            "close_gripper": SkillSpec(
                name="close_gripper",
                preconditions=("gripper_open",),
                effects=("-gripper_open",),
                postconditions=("gripper_closed",),
            ),
            "drop_object": SkillSpec(
                name="drop_object",
                preconditions=("gripper_open",),  # — but we just closed it
            ),
        }
        r = validate_plan(
            [_step("open_gripper"), _step("close_gripper"), _step("drop_object")],
            skills,
            [],
        )
        self.assertFalse(r.valid)
        self.assertEqual(r.first_failing_step, 2)
        self.assertEqual(r.missing_preconditions, ["gripper_open"])

    def test_negative_precondition_violated(self):
        """Skill demands `not failed`; state has `failed`."""
        skills = {
            "do_safe": SkillSpec(
                name="do_safe",
                preconditions=("not failed",),
            ),
        }
        r = validate_plan(
            [_step("do_safe")], skills, ["failed"]
        )
        self.assertFalse(r.valid)
        self.assertEqual(r.missing_preconditions, ["not failed"])

    def test_negative_precondition_satisfied(self):
        skills = {
            "do_safe": SkillSpec(
                name="do_safe",
                preconditions=("not failed",),
            ),
        }
        r = validate_plan([_step("do_safe")], skills, [])
        self.assertTrue(r.valid)

    def test_long_plan_failure_points_to_offending_step(self):
        skills = {
            "a": SkillSpec(name="a", postconditions=("p1",)),
            "b": SkillSpec(name="b", preconditions=("p1",), postconditions=("p2",)),
            "c": SkillSpec(name="c", preconditions=("p3",)),  # — missing p3
            "d": SkillSpec(name="d"),
        }
        r = validate_plan(
            [_step("a"), _step("b"), _step("c"), _step("d")],
            skills,
            [],
        )
        self.assertFalse(r.valid)
        self.assertEqual(r.first_failing_step, 2)
        self.assertEqual(r.missing_preconditions, ["p3"])

    def test_final_state_truncates_at_failure(self):
        skills = {
            "a": SkillSpec(name="a", postconditions=("p1",)),
            "b": SkillSpec(name="b", preconditions=("missing",)),
        }
        r = validate_plan(
            [_step("a"), _step("b")], skills, []
        )
        # State after a, before b's failed check, contains p1 — but not
        # b's would-be postconditions.
        self.assertIn("p1", r.final_state)


class TestPlanResultDataclass(unittest.TestCase):
    def test_default_is_invalid(self):
        # Sanity that the dataclass shape lets us return safely.
        r = PlanValidationResult(
            valid=False,
            message="x",
            first_failing_step=0,
            missing_preconditions=["a"],
            final_state=[],
        )
        self.assertFalse(r.valid)


if __name__ == "__main__":
    unittest.main()
