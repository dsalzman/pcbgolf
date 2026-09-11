"""Deterministic connectivity-aware placement for the PCB Golf board."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Iterable


@dataclass(frozen=True)
class Rect:
    left: float
    top: float
    right: float
    bottom: float
    ref: str = ""

    @property
    def width(self) -> float:
        return self.right - self.left

    @property
    def height(self) -> float:
        return self.bottom - self.top

    @property
    def area(self) -> float:
        return self.width * self.height

    def expanded(self, amount: float) -> "Rect":
        return Rect(
            self.left - amount,
            self.top - amount,
            self.right + amount,
            self.bottom + amount,
            self.ref,
        )

    def overlaps(self, other: "Rect") -> bool:
        return (
            self.left < other.right
            and self.right > other.left
            and self.top < other.bottom
            and self.bottom > other.top
        )


@dataclass(frozen=True)
class FootprintShape:
    angle: int
    left: float
    top: float
    right: float
    bottom: float

    def at(self, x: float, y: float, ref: str) -> Rect:
        return Rect(
            x + self.left,
            y + self.top,
            x + self.right,
            y + self.bottom,
            ref,
        )


class SpatialIndex:
    def __init__(self, cell_size: float = 4.0) -> None:
        self.cell_size = cell_size
        self.cells: dict[tuple[int, int], list[Rect]] = defaultdict(list)
        self.rects: list[Rect] = []

    def _keys(self, rect: Rect) -> Iterable[tuple[int, int]]:
        x0 = math.floor(rect.left / self.cell_size)
        x1 = math.floor((rect.right - 1e-9) / self.cell_size)
        y0 = math.floor(rect.top / self.cell_size)
        y1 = math.floor((rect.bottom - 1e-9) / self.cell_size)
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                yield x, y

    def add(self, rect: Rect) -> None:
        self.rects.append(rect)
        for key in self._keys(rect):
            self.cells[key].append(rect)

    def collisions(self, rect: Rect) -> list[Rect]:
        seen: set[Rect] = set()
        result: list[Rect] = []
        for key in self._keys(rect):
            for other in self.cells.get(key, ()):
                if other not in seen and rect.overlaps(other):
                    seen.add(other)
                    result.append(other)
        return result


# Carefully floorplanned interfaces and large ICs. Coordinates are millimetres.
# Everything else is packed near its strongest already-placed electrical peers.
FIXED_PLACEMENT: dict[str, tuple[float, float, int]] = {
    "BH1": (102.5, 52.5, 0),
    "BH2": (159.5, 52.5, 0),
    "BH3": (102.5, 101.5, 0),
    "BH4": (159.5, 101.5, 0),
    "J1": (111.5, 58.0, 0),
    "J2": (120.0, 102.0, 0),
    "J3": (143.5, 94.0, 0),
    "J4": (156.5, 83.5, 0),
    "J5": (156.5, 57.2, 0),
    "J6": (156.5, 62.0, 0),
    "J7": (156.5, 66.8, 0),
    "J8": (156.5, 71.6, 0),
    "U1": (124.0, 63.0, 0),
    "U2": (124.0, 54.0, 0),
    "U3": (126.0, 78.0, 0),
    "U4": (143.5, 79.0, 0),
    "U5": (140.0, 54.0, 0),
    "U6": (140.0, 57.2, 0),
    "U7": (140.0, 60.4, 0),
    "U8": (140.0, 63.6, 0),
    "U9": (108.0, 69.0, 0),
    "U10": (108.0, 75.0, 0),
    "U11": (108.0, 81.0, 0),
    "U12": (108.0, 87.0, 0),
    "U13": (149.4, 57.2, 0),
    "U14": (149.4, 62.0, 0),
    "U15": (149.4, 66.8, 0),
    "U16": (149.4, 71.6, 0),
    "L1": (128.0, 63.0, 0),
    "L2": (128.0, 54.0, 0),
    "L5": (144.0, 54.0, 0),
    "L6": (144.0, 57.2, 0),
    "L7": (144.0, 60.4, 0),
    "L8": (144.0, 63.6, 0),
    "D1": (132.0, 58.5, 0),
    "D2": (132.0, 54.0, 0),
    "Q1": (124.0, 58.5, 0),
    "SW1": (154.5, 96.0, 0),
    "Y1": (127.0, 93.0, 0),
    "Y2": (143.5, 86.0, 0),
    "LED1": (132.0, 93.0, 0),
}


# The vertical OBD-C connector footprint only draws its pad tongue. Reserve
# the complete STEP-model body so adjacent connectors remain assemblable.
MODEL_RECTS: dict[str, tuple[float, float, float, float]] = {
    ref: (-4.370, -2.250, 4.370, 2.250) for ref in ("J5", "J6", "J7", "J8")
}
# Include the full microSD shell, which extends farther left than its graphics.
MODEL_RECTS["J2"] = (-14.271, -11.989, 0.329, 1.311)


GROUP_ANCHORS = {
    "/Power/": (120.0, 59.0),
    "/STM32H7/": (127.0, 79.0),
    "/USB + SD/": (140.0, 87.0),
    "/CAN-FD/": (141.0, 69.0),
    "/Channels/": (148.0, 69.0),
}

IGNORED_PLACEMENT_NETS = {
    "",
    "GND",
    "+3V3",
    "+5V",
    "+12V",
    "VBUS",
    "VDDA",
    "VDDLDO",
    "VLXSMPS",
}


def _bbox_shape(pcbnew: Any, footprint: Any, angle: int) -> FootprintShape:
    old_position = footprint.GetPosition()
    old_angle = footprint.GetOrientationDegrees()
    footprint.SetPosition(pcbnew.VECTOR2I_MM(0, 0))
    footprint.SetOrientationDegrees(angle)
    box = footprint.GetBoundingBox(False, False)
    shape = FootprintShape(
        angle=angle,
        left=pcbnew.ToMM(box.GetLeft()),
        top=pcbnew.ToMM(box.GetTop()),
        right=pcbnew.ToMM(box.GetRight()),
        bottom=pcbnew.ToMM(box.GetBottom()),
    )
    footprint.SetOrientationDegrees(old_angle)
    footprint.SetPosition(old_position)
    return shape


def _shape_options(pcbnew: Any, footprint: Any) -> list[FootprintShape]:
    first = _bbox_shape(pcbnew, footprint, 0)
    second = _bbox_shape(pcbnew, footprint, 90)
    if (
        abs(first.width - second.width) < 1e-6
        and abs(first.height - second.height) < 1e-6
    ):
        return [first]
    return [first, second]


def _union(left: Rect, right: Rect) -> Rect:
    return Rect(
        min(left.left, right.left),
        min(left.top, right.top),
        max(left.right, right.right),
        max(left.bottom, right.bottom),
        left.ref or right.ref,
    )


def _physical_rect(
    ref: str, shape: FootprintShape, x: float, y: float
) -> Rect:
    rect = shape.at(x, y, ref)
    if ref in MODEL_RECTS and shape.angle == 0:
        left, top, right, bottom = MODEL_RECTS[ref]
        rect = _union(rect, Rect(x + left, y + top, x + right, y + bottom, ref))
    return rect


def _inside(rect: Rect, bounds: Rect, edge_margin: float) -> bool:
    # Connector bodies may overhang, but all fixed choices are currently kept
    # inside too. Keeping one rule here makes the floorplan easy to audit.
    return (
        rect.left >= bounds.left + edge_margin
        and rect.top >= bounds.top + edge_margin
        and rect.right <= bounds.right - edge_margin
        and rect.bottom <= bounds.bottom - edge_margin
    )


def _ring(radius: int) -> Iterable[tuple[int, int]]:
    if radius == 0:
        yield 0, 0
        return
    for x in range(-radius, radius + 1):
        yield x, -radius
        yield x, radius
    for y in range(-radius + 1, radius):
        yield -radius, y
        yield radius, y


def _nets_by_ref(board: Any) -> tuple[dict[str, set[str]], dict[str, set[str]]]:
    by_ref: dict[str, set[str]] = {}
    by_net: dict[str, set[str]] = defaultdict(set)
    for footprint in board.GetFootprints():
        ref = str(footprint.GetReference())
        nets = {str(pad.GetNetname()) for pad in footprint.Pads()}
        by_ref[ref] = nets
        for net in nets:
            by_net[net].add(ref)
    return by_ref, by_net


def _desired_position(
    ref: str,
    footprint: Any,
    positions: dict[str, tuple[float, float]],
    by_ref: dict[str, set[str]],
    by_net: dict[str, set[str]],
) -> tuple[float, float]:
    points: list[tuple[float, float, float]] = []
    group = str(footprint.GetSheetname())
    anchor = GROUP_ANCHORS.get(group, (131.0, 77.0))
    points.append((anchor[0], anchor[1], 1.0))

    for net in by_ref[ref]:
        if net in IGNORED_PLACEMENT_NETS or net.startswith("unconnected-"):
            continue
        peers = by_net[net]
        weight = 8.0 / max(1, len(peers) - 1)
        for peer in peers:
            if peer != ref and peer in positions:
                x, y = positions[peer]
                points.append((x, y, weight))

    total = sum(point[2] for point in points)
    return (
        sum(point[0] * point[2] for point in points) / total,
        sum(point[1] * point[2] for point in points) / total,
    )


def _find_location(
    ref: str,
    shapes: list[FootprintShape],
    desired: tuple[float, float],
    bounds: Rect,
    index: SpatialIndex,
    *,
    grid: float,
    clearance: float,
    edge_margin: float,
) -> tuple[float, float, FootprintShape, Rect]:
    center_x = round(desired[0] / grid) * grid
    center_y = round(desired[1] / grid) * grid
    max_radius = math.ceil(max(bounds.width, bounds.height) / grid)

    best: tuple[float, float, FootprintShape, Rect, float] | None = None
    for radius in range(max_radius + 1):
        for dx, dy in _ring(radius):
            x = center_x + dx * grid
            y = center_y + dy * grid
            for shape in shapes:
                rect = _physical_rect(ref, shape, x, y)
                occupied = rect.expanded(clearance / 2)
                if not _inside(rect, bounds, edge_margin):
                    continue
                if index.collisions(occupied):
                    continue
                distance = (x - desired[0]) ** 2 + (y - desired[1]) ** 2
                # Prefer unrotated parts when equally good for deterministic,
                # readable placement and simpler escape routing.
                cost = distance + (0.001 if shape.angle else 0.0)
                if best is None or cost < best[4]:
                    best = (x, y, shape, rect, cost)
        if best is not None:
            return best[:4]

    raise RuntimeError(
        f"No legal {grid:.2f} mm-grid position remains for {ref}; "
        "increase the requested board dimensions."
    )


def _signal_hpwl(board: Any, pcbnew: Any) -> float:
    points: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            net = str(pad.GetNetname())
            if net in IGNORED_PLACEMENT_NETS or net.startswith("unconnected-"):
                continue
            pos = pad.GetPosition()
            points[net].append((pcbnew.ToMM(pos.x), pcbnew.ToMM(pos.y)))
    result = 0.0
    for net_points in points.values():
        if len(net_points) < 2:
            continue
        xs = [point[0] for point in net_points]
        ys = [point[1] for point in net_points]
        result += max(xs) - min(xs) + max(ys) - min(ys)
    return result


def _remove_tracks(board: Any) -> int:
    tracks = list(board.GetTracks())
    for track in tracks:
        board.Remove(track)
    return len(tracks)


def place_components(
    pcbnew: Any,
    board: Any,
    *,
    x0: float = 100.0,
    y0: float = 50.0,
    width: float = 62.0,
    height: float = 54.0,
    grid: float = 0.25,
    clearance: float = 0.15,
    edge_margin: float = 0.15,
) -> dict[str, Any]:
    footprints = {
        str(footprint.GetReference()): footprint
        for footprint in board.GetFootprints()
    }
    missing = sorted(set(FIXED_PLACEMENT) - set(footprints))
    if missing:
        raise RuntimeError(f"Floorplan references are missing: {', '.join(missing)}")

    removed_tracks = _remove_tracks(board)
    initial_hpwl = _signal_hpwl(board, pcbnew)
    options = {
        ref: _shape_options(pcbnew, footprint)
        for ref, footprint in footprints.items()
    }
    bounds = Rect(x0, y0, x0 + width, y0 + height, "board")
    index = SpatialIndex()
    positions: dict[str, tuple[float, float]] = {}
    angles: dict[str, int] = {}
    physical_rects: dict[str, Rect] = {}

    for ref, (x, y, angle) in FIXED_PLACEMENT.items():
        footprint = footprints[ref]
        shape = next(item for item in options[ref] if item.angle == angle)
        rect = _physical_rect(ref, shape, x, y)
        if not _inside(rect, bounds, edge_margin):
            raise RuntimeError(f"Fixed {ref} lies outside the floorplan: {rect}")
        collisions = index.collisions(rect.expanded(clearance / 2))
        if collisions:
            others = ", ".join(item.ref for item in collisions)
            raise RuntimeError(f"Fixed {ref} overlaps {others}")
        footprint.SetOrientationDegrees(angle)
        footprint.SetPosition(pcbnew.VECTOR2I_MM(x, y))
        positions[ref] = (x, y)
        angles[ref] = angle
        physical_rects[ref] = rect
        index.add(rect.expanded(clearance / 2))

    by_ref, by_net = _nets_by_ref(board)
    unplaced = [ref for ref in footprints if ref not in positions]

    def placement_priority(ref: str) -> tuple[int, float, str]:
        area = max(shape.at(0, 0, ref).area for shape in options[ref])
        passive = ref.startswith(("R", "C"))
        return (1 if passive else 0, -area, ref)

    for ref in sorted(unplaced, key=placement_priority):
        footprint = footprints[ref]
        desired = _desired_position(ref, footprint, positions, by_ref, by_net)
        x, y, shape, rect = _find_location(
            ref,
            options[ref],
            desired,
            bounds,
            index,
            grid=grid,
            clearance=clearance,
            edge_margin=edge_margin,
        )
        footprint.SetOrientationDegrees(shape.angle)
        footprint.SetPosition(pcbnew.VECTOR2I_MM(x, y))
        positions[ref] = (x, y)
        angles[ref] = shape.angle
        physical_rects[ref] = rect
        index.add(rect.expanded(clearance / 2))

    final_hpwl = _signal_hpwl(board, pcbnew)
    used = Rect(
        min(rect.left for rect in physical_rects.values()),
        min(rect.top for rect in physical_rects.values()),
        max(rect.right for rect in physical_rects.values()),
        max(rect.bottom for rect in physical_rects.values()),
        "used",
    )
    return {
        "components": len(positions),
        "fixed_components": len(FIXED_PLACEMENT),
        "removed_tracks": removed_tracks,
        "board_mm": {"x": x0, "y": y0, "width": width, "height": height},
        "used_component_bbox_mm": {
            "width": used.width,
            "height": used.height,
            "area": used.area,
        },
        "signal_hpwl_mm": {
            "before": initial_hpwl,
            "after": final_hpwl,
            "improvement_percent": 100.0 * (initial_hpwl - final_hpwl) / initial_hpwl,
        },
        "positions": {
            ref: {"x": positions[ref][0], "y": positions[ref][1], "angle": angles[ref]}
            for ref in sorted(positions)
        },
    }
