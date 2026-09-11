"""Deterministic completion of the last dense routes and ground islands."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Any, Iterable


GRID_MM = 0.10
TRACK_WIDTH_MM = 0.10
CLEARANCE_MM = 0.10
VIA_SIZE_MM = 0.25
VIA_DRILL_MM = 0.15


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class Box:
    left: float
    top: float
    right: float
    bottom: float


@dataclass(frozen=True)
class RouteSpec:
    net: str
    layer: int
    pad: tuple[float, float]
    start: tuple[float, float]
    goal: tuple[float, float]


def _grid(value: float) -> int:
    return round(value / GRID_MM)


def _mm(value: int) -> float:
    return value * GRID_MM


def _raster_circle(occupied: set[tuple[int, int]], circle: Circle) -> None:
    x0 = math.floor((circle.x - circle.radius) / GRID_MM)
    x1 = math.ceil((circle.x + circle.radius) / GRID_MM)
    y0 = math.floor((circle.y - circle.radius) / GRID_MM)
    y1 = math.ceil((circle.y + circle.radius) / GRID_MM)
    radius_squared = circle.radius * circle.radius
    for x in range(x0, x1 + 1):
        for y in range(y0, y1 + 1):
            if (_mm(x) - circle.x) ** 2 + (_mm(y) - circle.y) ** 2 <= radius_squared:
                occupied.add((x, y))


def _raster_box(occupied: set[tuple[int, int]], box: Box) -> None:
    for x in range(math.floor(box.left / GRID_MM), math.ceil(box.right / GRID_MM) + 1):
        for y in range(math.floor(box.top / GRID_MM), math.ceil(box.bottom / GRID_MM) + 1):
            occupied.add((x, y))


def _distance_to_segment(
    x: float,
    y: float,
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if dx == 0 and dy == 0:
        return math.hypot(x - start[0], y - start[1])
    fraction = max(
        0.0,
        min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy) / (dx * dx + dy * dy)),
    )
    return math.hypot(x - (start[0] + fraction * dx), y - (start[1] + fraction * dy))


def _raster_segment(
    occupied: set[tuple[int, int]],
    start: tuple[float, float],
    end: tuple[float, float],
    radius: float,
) -> None:
    x0 = math.floor((min(start[0], end[0]) - radius) / GRID_MM)
    x1 = math.ceil((max(start[0], end[0]) + radius) / GRID_MM)
    y0 = math.floor((min(start[1], end[1]) - radius) / GRID_MM)
    y1 = math.ceil((max(start[1], end[1]) + radius) / GRID_MM)
    for x in range(x0, x1 + 1):
        for y in range(y0, y1 + 1):
            if _distance_to_segment(_mm(x), _mm(y), start, end) <= radius:
                occupied.add((x, y))


def _compress(points: list[tuple[int, int]]) -> list[tuple[int, int]]:
    if len(points) < 3:
        return points
    result = [points[0]]
    old_direction: tuple[int, int] | None = None
    for index in range(1, len(points)):
        direction = (
            points[index][0] - points[index - 1][0],
            points[index][1] - points[index - 1][1],
        )
        if old_direction is not None and direction != old_direction:
            result.append(points[index - 1])
        old_direction = direction
    result.append(points[-1])
    return result


def find_path(
    occupied: set[tuple[int, int]],
    start_mm: tuple[float, float],
    goal_mm: tuple[float, float],
    bounds_mm: tuple[float, float, float, float],
) -> list[tuple[float, float]]:
    """Find an eight-direction grid path around plated-hole obstacles."""
    start = (_grid(start_mm[0]), _grid(start_mm[1]))
    goal = (_grid(goal_mm[0]), _grid(goal_mm[1]))
    occupied = occupied - {start, goal}
    min_x, min_y, max_x, max_y = (_grid(value) for value in bounds_mm)
    directions = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (1, 1, math.sqrt(2)),
    )
    queue: list[tuple[float, float, tuple[int, int]]] = [
        (math.dist(start, goal), 0.0, start)
    ]
    best = {start: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}

    while queue:
        _, cost, current = heapq.heappop(queue)
        if current == goal:
            path = [current]
            while current != start:
                current = parent[current]
                path.append(current)
            path.reverse()
            result = [start_mm]
            result.extend((_mm(x), _mm(y)) for x, y in _compress(path)[1:-1])
            result.append(goal_mm)
            return result
        if cost != best.get(current):
            continue
        for dx, dy, step_cost in directions:
            nxt = (current[0] + dx, current[1] + dy)
            if not (min_x <= nxt[0] <= max_x and min_y <= nxt[1] <= max_y):
                continue
            if nxt in occupied:
                continue
            if dx and dy and (
                (current[0] + dx, current[1]) in occupied
                or (current[0], current[1] + dy) in occupied
            ):
                continue
            new_cost = cost + step_cost
            if new_cost >= best.get(nxt, math.inf):
                continue
            best[nxt] = new_cost
            parent[nxt] = current
            heuristic = math.dist(nxt, goal)
            heapq.heappush(queue, (new_cost + heuristic, new_cost, nxt))
    raise RuntimeError(f"No completion path from {start_mm} to {goal_mm}")


def _add_track(
    pcbnew: Any,
    board: Any,
    net: Any,
    layer: int,
    points: Iterable[tuple[float, float]],
) -> None:
    points = list(points)
    for start, end in zip(points, points[1:]):
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(pcbnew.VECTOR2I_MM(*start))
        track.SetEnd(pcbnew.VECTOR2I_MM(*end))
        track.SetWidth(pcbnew.FromMM(TRACK_WIDTH_MM))
        track.SetLayer(layer)
        track.SetNet(net)
        board.Add(track)


def _add_blind_via(
    pcbnew: Any,
    board: Any,
    net: Any,
    position: tuple[float, float] | Any,
    bottom_layer: int,
) -> Any:
    point = (
        pcbnew.VECTOR2I_MM(*position)
        if isinstance(position, tuple)
        else position
    )
    via = pcbnew.PCB_VIA(board)
    via.SetViaType(pcbnew.VIATYPE_BLIND)
    via.SetPosition(point)
    via.SetWidth(pcbnew.FromMM(VIA_SIZE_MM))
    via.SetDrill(pcbnew.FromMM(VIA_DRILL_MM))
    via.SetLayerPair(pcbnew.F_Cu, bottom_layer)
    via.SetNet(net)
    board.Add(via)
    return via


def _add_ground_plane(pcbnew: Any, board: Any) -> Any:
    net = board.FindNet("GND")
    zone = pcbnew.ZONE(board)
    zone.SetLayer(pcbnew.In1_Cu)
    zone.SetNetCode(net.GetNetCode())
    zone.SetZoneName("GND inner plane")
    zone.SetLocalClearance(pcbnew.FromMM(CLEARANCE_MM))
    zone.SetMinThickness(pcbnew.FromMM(0.075))
    zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    points = pcbnew.VECTOR_VECTOR2I()
    for x, y in ((100.2, 50.2), (167.8, 50.2), (167.8, 109.8), (100.2, 109.8)):
        points.append(pcbnew.VECTOR2I_MM(x, y))
    zone.AddPolygon(points)
    board.Add(zone)
    return zone


def _stitch_front_ground(pcbnew: Any, board: Any) -> int:
    net = board.FindNet("GND")
    front = next(
        zone for zone in board.Zones() if str(zone.GetZoneName()) == "GND front plane"
    )
    polygons = front.GetFilledPolysList(pcbnew.F_Cu)
    pads: list[tuple[int, Any]] = []
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            if str(pad.GetNetname()) != "GND" or not pad.FlashLayer(pcbnew.F_Cu):
                continue
            box = pad.GetBoundingBox()
            pads.append((box.GetWidth() * box.GetHeight(), pad))

    connected = [
        item.GetPosition()
        for item in board.GetTracks()
        if item.Type() == pcbnew.PCB_VIA_T and str(item.GetNetname()) == "GND"
    ]
    connected.extend(
        pad.GetPosition()
        for _, pad in pads
        if pad.GetAttribute() != pcbnew.PAD_ATTRIB_SMD
    )

    added = 0
    for index in range(polygons.OutlineCount()):
        if any(polygons.Contains(position, index) for position in connected):
            continue
        candidates = [
            (area, pad)
            for area, pad in pads
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_SMD
            and polygons.Contains(pad.GetPosition(), index)
        ]
        if not candidates:
            raise RuntimeError(f"No grounded SMD pad in front-plane island {index}")
        pad = max(candidates, key=lambda candidate: candidate[0])[1]
        _add_blind_via(pcbnew, board, net, pad.GetPosition(), pcbnew.In1_Cu)
        added += 1
    return added


def _obstacles(
    pcbnew: Any,
    board: Any,
    layer: int,
    route_net: str,
    extra_segments: Iterable[tuple[tuple[float, float], tuple[float, float]]] = (),
) -> set[tuple[int, int]]:
    occupied: set[tuple[int, int]] = set()
    track_margin = CLEARANCE_MM + TRACK_WIDTH_MM / 2
    for item in board.GetTracks():
        item_net = str(item.GetNetname())
        if item_net == route_net:
            continue
        if item.Type() == pcbnew.PCB_VIA_T and item.GetLayerSet().Contains(layer):
            position = item.GetPosition()
            _raster_circle(
                occupied,
                Circle(
                    pcbnew.ToMM(position.x),
                    pcbnew.ToMM(position.y),
                    pcbnew.ToMM(item.GetWidth()) / 2 + track_margin,
                ),
            )
        elif item.GetLayer() == layer:
            start = item.GetStart()
            end = item.GetEnd()
            _raster_segment(
                occupied,
                (pcbnew.ToMM(start.x), pcbnew.ToMM(start.y)),
                (pcbnew.ToMM(end.x), pcbnew.ToMM(end.y)),
                pcbnew.ToMM(item.GetWidth()) / 2 + track_margin,
            )
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            if str(pad.GetNetname()) == route_net or not pad.FlashLayer(layer):
                continue
            box = pad.GetBoundingBox()
            _raster_box(
                occupied,
                Box(
                    pcbnew.ToMM(box.GetLeft()) - track_margin,
                    pcbnew.ToMM(box.GetTop()) - track_margin,
                    pcbnew.ToMM(box.GetRight()) + track_margin,
                    pcbnew.ToMM(box.GetBottom()) + track_margin,
                ),
            )
    for start, end in extra_segments:
        _raster_segment(occupied, start, end, 2 * track_margin)
    return occupied


def finish_route(pcbnew: Any, board: Any) -> dict[str, Any]:
    """Complete the deterministic 20-pass route on four freed inner layers."""
    board.SetCopperLayerCount(10)
    board.GetDesignSettings().SetBoardThickness(pcbnew.FromMM(1.0))

    moved_power_tracks = 0
    for item in board.GetTracks():
        if (
            item.Type() == pcbnew.PCB_VIA_T
            or str(item.GetNetname()) != "+12V"
            or item.GetLayer() != pcbnew.F_Cu
        ):
            continue
        endpoints = (item.GetStart(), item.GetEnd())
        if all(
            123.0 <= pcbnew.ToMM(point.x) <= 124.7
            and 51.5 <= pcbnew.ToMM(point.y) <= 55.6
            for point in endpoints
        ):
            item.SetLayer(pcbnew.In7_Cu)
            moved_power_tracks += 1

    moved_tracks = 0
    for item in board.GetTracks():
        if item.GetLayer() == pcbnew.In1_Cu:
            item.SetLayer(pcbnew.In5_Cu)
            moved_tracks += 1

    _add_ground_plane(pcbnew, board)
    if not pcbnew.ZONE_FILLER(board).Fill(board.Zones()):
        raise RuntimeError("Could not fill initial ground planes")
    ground_vias = _stitch_front_ground(pcbnew, board)

    specs = (
        RouteSpec(
            "CH1_IMON",
            pcbnew.In6_Cu,
            (153.9306, 56.4333),
            (153.9306, 56.4333),
            (147.6154, 51.2566),
        ),
        RouteSpec(
            "CH1_D_P",
            pcbnew.In1_Cu,
            (161.7177, 57.5033),
            (161.7177, 57.5033),
            (162.1573, 59.9847),
        ),
        RouteSpec(
            "Net-(U3-PC3_C)",
            pcbnew.In1_Cu,
            (142.5, 52.45),
            (142.5, 52.45),
            (124.5261, 86.25),
        ),
        RouteSpec(
            "CH2_SBU1",
            pcbnew.In8_Cu,
            (162.7177, 63.7811),
            (162.7177, 63.7811),
            (155.4578, 93.5952),
        ),
    )
    layer_segments: dict[
        int, list[tuple[tuple[float, float], tuple[float, float]]]
    ] = {}
    route_points: dict[str, list[tuple[float, float]]] = {}
    for spec in specs:
        net = board.FindNet(spec.net)
        _add_blind_via(pcbnew, board, net, spec.start, spec.layer)
        if spec.pad != spec.start:
            _add_track(pcbnew, board, net, pcbnew.F_Cu, (spec.pad, spec.start))
        occupied = _obstacles(
            pcbnew,
            board,
            spec.layer,
            spec.net,
            layer_segments.get(spec.layer, ()),
        )
        path = find_path(occupied, spec.start, spec.goal, (100.4, 50.4, 167.6, 109.6))
        _add_track(pcbnew, board, net, spec.layer, path)
        route_points[spec.net] = path
        layer_segments.setdefault(spec.layer, []).extend(zip(path, path[1:]))

    _add_blind_via(
        pcbnew,
        board,
        board.FindNet("+12V"),
        (123.75, 55.45),
        pcbnew.In7_Cu,
    )

    if not pcbnew.ZONE_FILLER(board).Fill(board.Zones()):
        raise RuntimeError("Could not refill completion ground planes")
    return {
        "moved_inner_tracks": moved_tracks,
        "moved_conflicting_power_tracks": moved_power_tracks,
        "ground_stitch_vias": ground_vias,
        "completed_signal_nets": list(route_points),
        "completion_route_points": route_points,
        "copper_layers": board.GetCopperLayerCount(),
        "board_thickness_mm": pcbnew.ToMM(
            board.GetDesignSettings().GetBoardThickness()
        ),
    }
