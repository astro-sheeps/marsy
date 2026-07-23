from __future__ import annotations

import unittest
from pathlib import Path

from marsy_web.server import MISSION_CATALOG


class AgentDashboardIntegrationTests(unittest.TestCase):
    def test_agent_mission_catalog_uses_large_goal_and_model_select(self):
        mission = next(item for item in MISSION_CATALOG if item["id"] == "agent_mission")
        parameters = {item["name"]: item for item in mission["parameters"]}

        self.assertEqual(parameters["goal"]["type"], "textarea")
        self.assertTrue(parameters["goal"]["wide"])
        self.assertGreaterEqual(parameters["goal"]["rows"], 5)

        self.assertEqual(parameters["model"]["type"], "select")
        option_values = {option["value"] for option in parameters["model"]["options"]}
        self.assertEqual(
            option_values,
            {"openai/gpt-oss-20b", "openai/gpt-oss-120b"},
        )
        self.assertTrue(mission["map_enabled"])

    def test_dashboard_javascript_supports_textarea_and_select_parameters(self):
        source = Path("marsy_web/static/missions.js").read_text(encoding="utf-8")
        self.assertIn("document.createElement('textarea')", source)
        self.assertIn("document.createElement('select')", source)
        self.assertIn("'textarea', 'select'", source)
        self.assertIn("payload[parameter.name] = raw", source)

    def test_agent_scan_is_published_to_camera_range_hud(self):
        server_source = Path("marsy_web/server.py").read_text(encoding="utf-8")
        app_source = Path("marsy_web/static/app.js").read_text(encoding="utf-8")

        self.assertIn('"source": "agent_mission"', server_source)
        self.assertIn('kwargs["on_sample"] = _on_sample', server_source)
        self.assertIn("_publish_agent_sweep(collected, scanning=True", server_source)
        self.assertIn("['explore_area', 'agent_mission']", app_source)

    def test_agent_goal_field_has_full_width_dashboard_style(self):
        source = Path("marsy_web/static/missions.css").read_text(encoding="utf-8")
        self.assertIn(".mission-field.wide-field", source)
        self.assertIn(".mission-field textarea", source)
        self.assertIn("min-height: 118px", source)


    def test_dashboard_publishes_and_renders_generated_agent_plan(self):
        server_source = Path("marsy_web/server.py").read_text(encoding="utf-8")
        executor_source = Path("marsy_agent/executor.py").read_text(encoding="utf-8")
        javascript = Path("marsy_web/static/missions.js").read_text(encoding="utf-8")
        css = Path("marsy_web/static/missions.css").read_text(encoding="utf-8")

        self.assertIn('"plan": plan.to_dict()', server_source)
        self.assertIn('"plan": current_plan.to_dict()', executor_source)
        self.assertIn("Generated plan", javascript)
        self.assertIn("data-agent-plan-panel", javascript)
        self.assertIn("updateAgentPlan(activeMission)", javascript)
        self.assertIn("mission-plan-step", javascript)
        self.assertIn(".mission-plan-panel", css)
        self.assertIn(".mission-plan-step.current", css)

    def test_real_dashboard_launcher_prefers_project_venv(self):
        source = Path("scripts/run_real_dashboard.sh").read_text(encoding="utf-8")
        self.assertIn('.venv/bin/python', source)
        self.assertIn('groq.env', source)


if __name__ == "__main__":
    unittest.main()
