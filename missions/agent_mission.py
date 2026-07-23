"""First LLM-planned Marsy mission using Groq and the existing skills layer.

Generate and validate a plan without touching rover hardware:

    python -m missions.agent_mission \
      --goal "Look around, explore briefly, and stop" \
      --dry-run

Offline simulator smoke test:

    MARSY_MODE=sim python -m missions.agent_mission \
      --planner mock \
      --goal "Look around, make a cautious step, explore, and stop"

Groq-planned simulator mission:

    MARSY_MODE=sim python -m missions.agent_mission \
      --goal "Look around, choose a clear direction, explore for 30 seconds, and stop"
"""

from __future__ import annotations

import argparse
import json
import os
import sys

from marsy_agent import AgentExecutor, GroqPlanner, MockPlanner, PlanValidator
from marsy_agent.cache import PlanCache
from marsy_agent.journal import AgentJournal
from marsy_agent.models import jsonable

SOURCE = "agent_mission"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Marsy LLM agent mission")
    parser.add_argument("--goal", required=True, help="Natural-language mission goal")
    parser.add_argument(
        "--planner",
        choices=["groq", "mock"],
        default="groq",
        help="Use Groq or the deterministic offline test planner",
    )
    parser.add_argument(
        "--model",
        default=os.getenv("MARSY_LLM_MODEL", "openai/gpt-oss-20b"),
        help="Groq model ID",
    )
    parser.add_argument("--max-replans", type=int, default=1, choices=[0, 1, 2])
    parser.add_argument("--dry-run", action="store_true", help="Generate and validate only; do not load a rover backend")
    parser.add_argument("--journal-dir", default="artifacts/agent_runs")
    parser.add_argument("--cache-dir", default="artifacts/agent_cache")
    parser.add_argument(
        "--no-cache",
        action="store_true",
        help="Disable reading and writing the Groq plan cache",
    )
    parser.add_argument(
        "--refresh-plan",
        action="store_true",
        help="Ignore the cached plan, call Groq, and replace the cache",
    )
    parser.add_argument("--safe-distance", type=float, default=40.0)
    parser.add_argument("--danger-distance", type=float, default=24.0)
    parser.add_argument("--max-range", type=float, default=200.0)
    return parser


def _make_planner(args: argparse.Namespace):
    if args.planner == "mock":
        return MockPlanner()
    return GroqPlanner(model=args.model)


def _print_json(title: str, payload) -> None:
    print(f"\n{title}")
    print(json.dumps(jsonable(payload), ensure_ascii=False, indent=2))


def main() -> int:
    args = build_parser().parse_args()
    planner = _make_planner(args)
    validator = PlanValidator()
    journal = AgentJournal(
        goal=args.goal,
        planner=args.planner,
        model=planner.model,
        directory=args.journal_dir,
    )

    rover = None
    skills = None
    rover_initialized = False
    try:
        print(f"[AGENT] Goal: {args.goal}")
        print(f"[PLANNER] {args.planner} / {planner.model}")

        plan = None
        plan_source = args.planner
        cache = None
        if args.planner == "groq" and not args.no_cache:
            cache = PlanCache(directory=args.cache_dir)
            if not args.refresh_plan:
                cached_plan = cache.load(
                    goal=args.goal,
                    planner=args.planner,
                    model=planner.model,
                )
                if cached_plan is not None:
                    try:
                        plan = validator.validate(cached_plan)
                        plan_source = "cache"
                        print(f"[PLAN CACHE] Hit: {cache.path}")
                    except Exception:
                        cache.clear()
                        print("[PLAN CACHE] Cached plan was invalid and was removed")
                else:
                    print("[PLAN CACHE] Miss")
            else:
                print("[PLAN CACHE] Refresh requested")

        if plan is None:
            plan = validator.validate(planner.create_plan(args.goal))
            plan_source = args.planner
            if cache is not None:
                cache.save(
                    goal=args.goal,
                    planner=args.planner,
                    model=planner.model,
                    plan=plan,
                )
                print(f"[PLAN CACHE] Saved: {cache.path}")

        journal.set_plan_source(plan_source)
        journal.set_initial_plan(plan)
        _print_json(f"[PLAN] Validated plan ({plan_source})", plan)

        if args.dry_run:
            print(f"\n[DRY RUN] No rover backend was loaded. Journal: {journal.path}")
            return 0

        # Hardware/backend imports intentionally happen after --dry-run.
        from marsy_backends.loader import load_rover
        from marsy_core.motion import MarsyMotion
        from marsy_core.sensors import MarsySensors
        from skills import MarsySkills, SkillsConfig

        rover = load_rover()
        motion = MarsyMotion(rover)
        sensors = MarsySensors(rover)
        config = SkillsConfig(
            safe_distance_cm=args.safe_distance,
            danger_distance_cm=args.danger_distance,
            max_range_cm=args.max_range,
            default_speed=22,
            default_turn_speed=22,
        )
        skills = MarsySkills(rover, motion=motion, sensors=sensors, config=config)

        rover.init(0)
        rover_initialized = True
        motion.stop_and_reset()
        skills.sync_pose()

        executor = AgentExecutor(
            skills=skills,
            planner=planner,
            validator=validator,
            max_replans=args.max_replans,
        )
        report = executor.execute(goal=args.goal, plan=plan)
        journal.set_report(report)
        _print_json("[REPORT] Execution report", report)
        print(f"\n[JOURNAL] {journal.path}")
        return 0 if report.ok else 2

    except KeyboardInterrupt:
        journal.set_error("Interrupted")
        print("\n[AGENT] Interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        journal.set_error(exc)
        print(f"\n[AGENT ERROR] {type(exc).__name__}: {exc}", file=sys.stderr)
        print(f"[JOURNAL] {journal.path}", file=sys.stderr)
        return 1
    finally:
        if skills is not None:
            try:
                skills.stop(brake=False, reset_pose=True)
            except Exception:
                pass
        if rover is not None and rover_initialized:
            try:
                rover.cleanup()
            except Exception as exc:
                print(f"[CLEANUP WARNING] {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
