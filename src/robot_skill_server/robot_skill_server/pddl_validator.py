"""Pragmatic fact-based plan validator.

Walks a sequence of skill steps, looks each up in a `name → SkillDescription`
mapping, and checks every step's preconditions against the cumulative state
built from prior steps' postconditions / effects. Used by:

  - The `/skill_server/validate_plan` service — clients (LLMs, MCP, the
    dashboard's Compose panel) sanity-check a plan before submitting it.
  - `TaskComposer.compose_task` — auto-rejects plans whose preconditions
    aren't met, with a clear error pointing to the offending step.

Convention for `preconditions / postconditions / effects` strings:

  - Bare ``fact`` or prefix ``+`` / ``add `` → add fact
  - Prefix ``-`` / ``del `` / ``not `` → delete fact
  - Whitespace + case are normalised; the comparison is case-sensitive on
    the fact body itself

This is *not* a full PDDL parser: there are no typed objects, no
parameter binding, no quantifiers, no conditional effects. SkillDescription's
``pddl_action`` field is left untouched for callers that bring their own
planner. What this module does is the 80%-case "did the agent forget that
``gripper_open`` is needed before ``pick_object``?" check.

Pure stdlib — no ROS dependencies, so the validator can run in unit tests
and inside the synchronous service handler without an executor.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Mapping, Sequence


@dataclass(frozen=True)
class SkillSpec:
    """The slice of a SkillDescription the validator actually reads.

    Lets tests drive the validator without constructing a full ROS message.
    Producers (e.g. the service handler) build one of these per skill from
    the SkillRegistry / SkillDiscovery snapshot.
    """

    name: str
    preconditions: tuple[str, ...] = ()
    postconditions: tuple[str, ...] = ()
    effects: tuple[str, ...] = ()


@dataclass
class PlanStepResult:
    step_index: int
    skill_name: str
    missing: list[str] = field(default_factory=list)


@dataclass
class PlanValidationResult:
    valid: bool
    message: str
    first_failing_step: int               # -1 when valid
    missing_preconditions: list[str]      # at the failing step
    final_state: list[str]                # state after walking all steps
                                          # (or up to the failing one)


# ── String parsing ───────────────────────────────────────────────────────

_DEL_PREFIXES = ("-", "del ", "not ")
_ADD_PREFIXES = ("+", "add ")


def parse_effect(s: str) -> tuple[str, bool]:
    """Return ``(fact, is_add)``. Empty / blank input → empty fact + add."""
    s = (s or "").strip()
    if not s:
        return "", True
    lo = s.lower()
    for p in _DEL_PREFIXES:
        if lo.startswith(p):
            return s[len(p):].strip(), False
    for p in _ADD_PREFIXES:
        if lo.startswith(p):
            return s[len(p):].strip(), True
    return s, True


def parse_precondition(s: str) -> tuple[str, bool]:
    """Same syntax as effects. Returns ``(fact, must_be_true)``."""
    return parse_effect(s)


def apply_effects(
    state: set[str], effects: Iterable[str], postconditions: Iterable[str]
) -> set[str]:
    """Return a *new* state set with the given step's effects applied.

    Postconditions and effects compose: postconditions add what the skill
    *guarantees*, effects (the looser PDDL-style ones) can add or delete.
    Effects override postconditions when the same fact appears in both,
    so a skill that *closes* the gripper can list ``gripper_closed`` in
    postconditions and ``-gripper_open`` in effects without contradicting
    itself.
    """
    out = set(state)
    for raw in postconditions:
        fact, is_add = parse_effect(raw)
        if not fact:
            continue
        if is_add:
            out.add(fact)
        else:
            out.discard(fact)
    for raw in effects:
        fact, is_add = parse_effect(raw)
        if not fact:
            continue
        if is_add:
            out.add(fact)
        else:
            out.discard(fact)
    return out


# ── Validator ────────────────────────────────────────────────────────────

def validate_plan(
    steps: Sequence[Mapping],
    skills: Mapping[str, SkillSpec],
    initial_state: Iterable[str] = (),
) -> PlanValidationResult:
    """Walk the plan, return the first violation or success.

    Parameters
    ----------
    steps :
        Ordered list of step dicts. Each must have a ``skill_name`` field;
        other fields (parameters_json, etc.) are ignored for validation —
        we don't bind parameters into preconditions yet.
    skills :
        Resolution table keyed by skill name. Missing entries cause a
        validation error pointing at the unknown step.
    initial_state :
        Facts assumed true at plan start. Empty by default.
    """
    state = set(s.strip() for s in initial_state if s and s.strip())
    if not steps:
        return PlanValidationResult(
            valid=True,
            message="empty plan — nothing to validate",
            first_failing_step=-1,
            missing_preconditions=[],
            final_state=sorted(state),
        )

    for i, step in enumerate(steps):
        skill_name = (step.get("skill_name") or "").strip()
        if not skill_name:
            return PlanValidationResult(
                valid=False,
                message=f"step {i}: empty skill_name",
                first_failing_step=i,
                missing_preconditions=[],
                final_state=sorted(state),
            )
        spec = skills.get(skill_name)
        if spec is None:
            return PlanValidationResult(
                valid=False,
                message=f"step {i}: unknown skill {skill_name!r}",
                first_failing_step=i,
                missing_preconditions=[],
                final_state=sorted(state),
            )
        missing: list[str] = []
        for raw in spec.preconditions:
            fact, must_be_true = parse_precondition(raw)
            if not fact:
                continue
            present = fact in state
            if must_be_true and not present:
                missing.append(fact)
            elif not must_be_true and present:
                # Negative precondition violated.
                missing.append(f"not {fact}")
        if missing:
            return PlanValidationResult(
                valid=False,
                message=(
                    f"step {i} ({skill_name!r}) preconditions not met: "
                    f"{missing}"
                ),
                first_failing_step=i,
                missing_preconditions=missing,
                final_state=sorted(state),
            )
        state = apply_effects(state, spec.effects, spec.postconditions)

    return PlanValidationResult(
        valid=True,
        message=f"plan valid ({len(steps)} step(s))",
        first_failing_step=-1,
        missing_preconditions=[],
        final_state=sorted(state),
    )
