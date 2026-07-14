"""
Sparse occupancy mapping for Marsy.

This is deliberately "SLAM-lite":
- ultrasonic rays update free/occupied cells;
- simulator pose is used when available;
- the real rover falls back to time-based dead reckoning;
- there is no loop closure or global pose correction yet.
"""

from __future__ import annotations

import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable, Optional

from .models import Pose2D, RangeSample, normalize_heading

Cell = tuple[int, int]


def _bresenham(start: Cell, end: Cell) -> list[Cell]:
    x0, y0 = start
    x1, y1 = end
    cells: list[Cell] = []
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return cells


class SparseOccupancyGrid:
    """A dependency-free sparse occupancy grid with JSON and SVG export."""

    def __init__(
        self,
        resolution_cm: float = 5.0,
        max_range_cm: float = 200.0,
        sensor_offset_cm: float = 9.0,
    ) -> None:
        if resolution_cm <= 0:
            raise ValueError("resolution_cm must be positive")
        self.resolution_cm = float(resolution_cm)
        self.max_range_cm = float(max_range_cm)
        self.sensor_offset_cm = float(sensor_offset_cm)
        self.free_hits: dict[Cell, int] = defaultdict(int)
        self.occupied_hits: dict[Cell, int] = defaultdict(int)
        self.visits: dict[Cell, int] = defaultdict(int)
        self.path: list[Pose2D] = []

    def world_to_cell(self, x_cm: float, y_cm: float) -> Cell:
        return (int(round(x_cm / self.resolution_cm)), int(round(y_cm / self.resolution_cm)))

    def cell_to_world(self, cell: Cell) -> tuple[float, float]:
        return (cell[0] * self.resolution_cm, cell[1] * self.resolution_cm)

    def state(self, cell: Cell) -> str:
        occupied = self.occupied_hits.get(cell, 0)
        free = self.free_hits.get(cell, 0)
        if occupied > 0 and occupied >= free:
            return "occupied"
        if free > 0:
            return "free"
        return "unknown"

    def mark_pose(self, pose: Pose2D) -> None:
        pose = pose.normalized()
        cell = self.world_to_cell(pose.x_cm, pose.y_cm)
        self.visits[cell] += 1
        self.free_hits[cell] += 2
        if not self.path or self.path[-1].distance_to(pose) >= self.resolution_cm / 2.0:
            self.path.append(pose)

    def observe_ray(
        self,
        pose: Pose2D,
        relative_angle_deg: float,
        distance_cm: Optional[float],
        *,
        valid_hit: bool,
        max_range_cm: Optional[float] = None,
    ) -> None:
        max_range = self.max_range_cm if max_range_cm is None else float(max_range_cm)
        global_angle = normalize_heading(pose.heading_deg + relative_angle_deg)
        angle_rad = math.radians(global_angle)
        direction_x = math.sin(angle_rad)
        direction_y = math.cos(angle_rad)

        origin_x = pose.x_cm + direction_x * self.sensor_offset_cm
        origin_y = pose.y_cm + direction_y * self.sensor_offset_cm
        # An invalid/no-echo/unstable reading is unknown, not evidence that the
        # entire ray is free. Skipping it avoids carving false corridors through
        # soft or acoustically absorbent obstacles such as sofas and curtains.
        if not valid_hit or distance_cm is None or float(distance_cm) <= 0:
            self.mark_pose(pose)
            return

        ray_length = min(float(distance_cm), max_range)

        end_x = origin_x + direction_x * ray_length
        end_y = origin_y + direction_y * ray_length
        cells = _bresenham(self.world_to_cell(origin_x, origin_y), self.world_to_cell(end_x, end_y))
        if not cells:
            return

        for cell in cells[:-1]:
            self.free_hits[cell] += 1

        self.occupied_hits[cells[-1]] += 3

        self.mark_pose(pose)

    def observe_samples(self, samples: Iterable[RangeSample]) -> None:
        for sample in samples:
            self.observe_ray(
                sample.pose,
                sample.mast_angle_deg,
                sample.distance_cm,
                valid_hit=sample.valid_hit,
            )

    def unknown_count_along_ray(
        self,
        pose: Pose2D,
        relative_angle_deg: float,
        distance_cm: float,
    ) -> int:
        global_angle = normalize_heading(pose.heading_deg + relative_angle_deg)
        angle_rad = math.radians(global_angle)
        end_x = pose.x_cm + math.sin(angle_rad) * distance_cm
        end_y = pose.y_cm + math.cos(angle_rad) * distance_cm
        cells = _bresenham(
            self.world_to_cell(pose.x_cm, pose.y_cm),
            self.world_to_cell(end_x, end_y),
        )
        return sum(1 for cell in cells if self.state(cell) == "unknown")

    def direction_score(
        self,
        pose: Pose2D,
        relative_angle_deg: float,
        clearance_cm: float,
    ) -> float:
        clearance = max(0.0, min(float(clearance_cm), self.max_range_cm))
        unknown = self.unknown_count_along_ray(pose, relative_angle_deg, clearance)
        angle_rad = math.radians(normalize_heading(pose.heading_deg + relative_angle_deg))
        probe_x = pose.x_cm + math.sin(angle_rad) * min(clearance, 40.0)
        probe_y = pose.y_cm + math.cos(angle_rad) * min(clearance, 40.0)
        visits = self.visits.get(self.world_to_cell(probe_x, probe_y), 0)
        turn_penalty = abs(float(relative_angle_deg)) * 0.025
        return unknown * 4.0 + clearance * 0.18 - visits * 8.0 - turn_penalty

    def bounds(self, margin_cells: int = 3) -> tuple[int, int, int, int]:
        cells = set(self.free_hits) | set(self.occupied_hits) | set(self.visits)
        cells.update(self.world_to_cell(p.x_cm, p.y_cm) for p in self.path)
        if not cells:
            return (-10, 10, -10, 10)
        xs = [cell[0] for cell in cells]
        ys = [cell[1] for cell in cells]
        return (
            min(xs) - margin_cells,
            max(xs) + margin_cells,
            min(ys) - margin_cells,
            max(ys) + margin_cells,
        )

    def to_dict(self, current_pose: Optional[Pose2D] = None, metadata: Optional[dict[str, Any]] = None) -> dict[str, Any]:
        cells = sorted(set(self.free_hits) | set(self.occupied_hits) | set(self.visits))
        return {
            "format": "marsy_sparse_occupancy_grid_v1",
            "mapping_mode": "occupancy_grid_plus_dead_reckoning_no_loop_closure",
            "resolution_cm": self.resolution_cm,
            "max_range_cm": self.max_range_cm,
            "sensor_offset_cm": self.sensor_offset_cm,
            "current_pose": None if current_pose is None else current_pose.normalized().__dict__,
            "path": [pose.normalized().__dict__ for pose in self.path],
            "cells": [
                {
                    "x": cell[0],
                    "y": cell[1],
                    "state": self.state(cell),
                    "free_hits": self.free_hits.get(cell, 0),
                    "occupied_hits": self.occupied_hits.get(cell, 0),
                    "visits": self.visits.get(cell, 0),
                }
                for cell in cells
            ],
            "metadata": metadata or {},
        }

    def save_json(self, path: str | Path, current_pose: Optional[Pose2D] = None, metadata: Optional[dict[str, Any]] = None) -> Path:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_name(f".{output.name}.tmp")
        temporary.write_text(
            json.dumps(self.to_dict(current_pose=current_pose, metadata=metadata), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        temporary.replace(output)
        return output

    def save_svg(self, path: str | Path, current_pose: Optional[Pose2D] = None, cell_px: int = 8) -> Path:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        min_x, max_x, min_y, max_y = self.bounds()
        width = (max_x - min_x + 1) * cell_px
        height = (max_y - min_y + 1) * cell_px

        def sx(cell_x: int) -> float:
            return (cell_x - min_x) * cell_px

        def sy(cell_y: int) -> float:
            return (max_y - cell_y) * cell_px

        lines = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            '<rect width="100%" height="100%" fill="#1a1d1f"/>',
        ]

        cells = sorted(set(self.free_hits) | set(self.occupied_hits) | set(self.visits))
        for cell in cells:
            state = self.state(cell)
            fill = "#ece8df" if state == "free" else "#272b2e"
            if state == "occupied":
                fill = "#e06b3c"
            lines.append(
                f'<rect x="{sx(cell[0]):.1f}" y="{sy(cell[1]):.1f}" width="{cell_px}" height="{cell_px}" fill="{fill}"/>'
            )

        if len(self.path) >= 2:
            points = []
            for pose in self.path:
                cell = self.world_to_cell(pose.x_cm, pose.y_cm)
                points.append(f"{sx(cell[0]) + cell_px / 2:.1f},{sy(cell[1]) + cell_px / 2:.1f}")
            lines.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="#6ca6c1" stroke-width="2"/>')

        if current_pose is not None:
            cell = self.world_to_cell(current_pose.x_cm, current_pose.y_cm)
            cx = sx(cell[0]) + cell_px / 2
            cy = sy(cell[1]) + cell_px / 2
            lines.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{max(3, cell_px / 2):.1f}" fill="#6fd08c"/>')

        lines.append("</svg>")
        temporary = output.with_name(f".{output.name}.tmp")
        temporary.write_text("\n".join(lines), encoding="utf-8")
        temporary.replace(output)
        return output
