"""Atomic single-file JSON journal for the latest agent mission run."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from .models import jsonable


class AgentJournal:
    def __init__(
        self,
        *,
        goal: str,
        planner: str,
        model: str,
        directory: str | Path = "artifacts/agent_runs",
        filename: str = "agent_mission_latest.json",
        remove_legacy_runs: bool = True,
    ) -> None:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / filename

        if remove_legacy_runs:
            for old_path in directory.glob("agent_*.json"):
                if old_path != self.path:
                    try:
                        old_path.unlink()
                    except OSError:
                        pass

        self.data: dict[str, Any] = {
            "mission": "agent_mission",
            "goal": goal,
            "planner": planner,
            "model": model,
            "plan_source": None,
            "started_at": time.time(),
            "initial_plan": None,
            "report": None,
            "error": None,
        }
        self.flush()

    def set_plan_source(self, source: str) -> None:
        self.data["plan_source"] = str(source)
        self.flush()

    def set_initial_plan(self, plan: Any) -> None:
        self.data["initial_plan"] = jsonable(plan)
        self.flush()

    def set_report(self, report: Any) -> None:
        self.data["report"] = jsonable(report)
        self.data["finished_at"] = time.time()
        self.flush()

    def set_error(self, error: BaseException | str) -> None:
        self.data["error"] = str(error)
        self.data["finished_at"] = time.time()
        self.flush()

    def flush(self) -> None:
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(jsonable(self.data), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        temporary.replace(self.path)
