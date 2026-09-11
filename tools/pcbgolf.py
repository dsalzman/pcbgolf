#!/usr/bin/env python3
"""Reproducible prepare, route, DRC, and scoring tools for PCB Golf."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BOARD = ROOT / "pcbgolf.kicad_pcb"
DEFAULT_BUILD_DIR = ROOT / ".pcbgolf-build"
FREEROUTING_VERSION = "2.4.1"
FREEROUTING_SHA256 = (
    "251101c3eeac22d7e7dfcf6796603279e5d1000283eb82d8f093780f7afc6aa9"
)
DEFAULT_TRACK_WIDTH_MM = 0.10
DEFAULT_CLEARANCE_MM = 0.10
DEFAULT_VIA_SIZE_MM = 0.50
DEFAULT_VIA_DRILL_MM = 0.30
DEFAULT_BOARD_THICKNESS_MM = 0.40


class PcbGolfError(RuntimeError):
    """A user-facing workflow failure."""


def run(
    command: Iterable[str | os.PathLike[str]],
    *,
    cwd: Path = ROOT,
    capture: bool = False,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    argv = [os.fspath(part) for part in command]
    print("+", " ".join(argv), file=sys.stderr)
    return subprocess.run(
        argv,
        cwd=cwd,
        check=check,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture else None,
    )


def import_pcbnew() -> Any:
    try:
        import pcbnew  # type: ignore[import-not-found]
    except ImportError as exc:
        raise PcbGolfError(
            "KiCad's pcbnew Python module is unavailable; run this in the PCB Golf "
            "Cloud environment or install KiCad 10."
        ) from exc
    return pcbnew


def load_board(path: Path) -> tuple[Any, Any]:
    pcbnew = import_pcbnew()
    board = pcbnew.LoadBoard(os.fspath(path.resolve()))
    if board is None:
        raise PcbGolfError(f"KiCad could not load {path}")
    return pcbnew, board


def find_freerouting_jar() -> Path:
    candidates = [
        os.environ.get("FREEROUTING_JAR"),
        f"/opt/freerouting/freerouting-{FREEROUTING_VERSION}.jar",
        str(
            Path.home()
            / ".local"
            / "share"
            / "pcbgolf-tools"
            / f"freerouting-{FREEROUTING_VERSION}.jar"
        ),
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return Path(candidate)
    raise PcbGolfError(
        f"Freerouting {FREEROUTING_VERSION} was not found. "
        "Use the repository's Cloud environment."
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def command_env_check(_: argparse.Namespace) -> int:
    failures: list[str] = []
    versions: dict[str, str] = {}

    if not shutil.which("kicad-cli"):
        failures.append("kicad-cli is missing")
    else:
        result = run(["kicad-cli", "version"], capture=True, check=False)
        versions["kicad"] = result.stdout.strip()
        if result.returncode or not versions["kicad"].startswith("10."):
            failures.append(f"KiCad 10 is required (found {versions['kicad']!r})")

    if not shutil.which("java"):
        failures.append("Java is missing")
    else:
        result = run(["java", "--version"], capture=True, check=False)
        java_output = (result.stdout or result.stderr).strip()
        versions["java"] = java_output.splitlines()[0] if java_output else ""
        match = re.search(r"\b(\d+)(?:\.\d+)*\b", versions["java"])
        if result.returncode or not match or int(match.group(1)) < 25:
            failures.append(f"Java 25+ is required (found {versions['java']!r})")

    try:
        pcbnew = import_pcbnew()
        versions["pcbnew"] = str(pcbnew.Version())
        if not versions["pcbnew"].startswith("10."):
            failures.append(
                f"KiCad 10 pcbnew bindings are required (found {versions['pcbnew']!r})"
            )
    except PcbGolfError as exc:
        failures.append(str(exc))

    try:
        jar = find_freerouting_jar()
        versions["freerouting"] = FREEROUTING_VERSION
        versions["freerouting_jar"] = os.fspath(jar)
        actual_hash = sha256(jar)
        if actual_hash != FREEROUTING_SHA256:
            failures.append(
                f"Freerouting checksum mismatch: expected {FREEROUTING_SHA256}, "
                f"found {actual_hash}"
            )
    except PcbGolfError as exc:
        failures.append(str(exc))

    print(json.dumps({"versions": versions, "failures": failures}, indent=2))
    return 1 if failures else 0


def configure_routing_rules(pcbnew: Any, board: Any) -> None:
    netclasses = board.GetAllNetClasses()
    if "Default" not in netclasses:
        raise PcbGolfError("The board has no Default netclass")
    netclass = netclasses["Default"]
    netclass.SetClearance(pcbnew.FromMM(DEFAULT_CLEARANCE_MM))
    netclass.SetTrackWidth(pcbnew.FromMM(DEFAULT_TRACK_WIDTH_MM))
    netclass.SetViaDiameter(pcbnew.FromMM(DEFAULT_VIA_SIZE_MM))
    netclass.SetViaDrill(pcbnew.FromMM(DEFAULT_VIA_DRILL_MM))


def normalize_stacked_connector_pads(board: Any) -> int:
    """Join duplicated through-hole/top contacts representing one connector pin."""
    changed = 0
    for footprint in board.GetFootprints():
        by_number = {str(pad.GetNumber()): pad for pad in footprint.Pads()}
        for number, pad in by_number.items():
            if not number.endswith("T") or number[:-1] not in by_number:
                continue
            base = by_number[number[:-1]]
            base_net = str(base.GetNetname())
            pad_net = str(pad.GetNetname())
            if (
                base_net
                and pad_net
                and base_net != pad_net
                and base_net.startswith("unconnected-")
                and pad_net.startswith("unconnected-")
            ):
                pad.SetNet(base.GetNet())
                changed += 1
    return changed


def remove_edge_cuts(pcbnew: Any, board: Any) -> int:
    removed = 0
    for drawing in list(board.GetDrawings()):
        if drawing.GetLayer() == pcbnew.Edge_Cuts:
            board.Remove(drawing)
            removed += 1
    return removed


def pad_bounds(pcbnew: Any, board: Any) -> tuple[int, int, int, int]:
    boxes = [
        pad.GetBoundingBox()
        for footprint in board.GetFootprints()
        for pad in footprint.Pads()
    ]
    if not boxes:
        raise PcbGolfError("The board contains no pads")
    return (
        min(box.GetLeft() for box in boxes),
        min(box.GetTop() for box in boxes),
        max(box.GetRight() for box in boxes),
        max(box.GetBottom() for box in boxes),
    )


def add_rectangular_outline(
    pcbnew: Any, board: Any, *, margin_mm: float
) -> tuple[float, float]:
    x0, y0, x1, y1 = pad_bounds(pcbnew, board)
    margin = pcbnew.FromMM(margin_mm)
    x0 -= margin
    y0 -= margin
    x1 += margin
    y1 += margin

    outline = pcbnew.PCB_SHAPE(board)
    outline.SetShape(pcbnew.SHAPE_T_RECT)
    outline.SetStart(pcbnew.VECTOR2I(x0, y0))
    outline.SetEnd(pcbnew.VECTOR2I(x1, y1))
    outline.SetLayer(pcbnew.Edge_Cuts)
    outline.SetWidth(pcbnew.FromMM(0.05))
    board.Add(outline)
    return pcbnew.ToMM(x1 - x0), pcbnew.ToMM(y1 - y0)


def add_ground_plane(
    pcbnew: Any,
    board: Any,
    *,
    x0: float,
    y0: float,
    width: float,
    height: float,
    inset: float = 0.20,
) -> None:
    net = board.FindNet("GND")
    if net is None:
        raise PcbGolfError("The board has no GND net for its ground plane")
    zone = pcbnew.ZONE(board)
    zone.SetLayer(pcbnew.F_Cu)
    zone.SetNetCode(net.GetNetCode())
    zone.SetZoneName("GND front plane")
    zone.SetLocalClearance(pcbnew.FromMM(DEFAULT_CLEARANCE_MM))
    zone.SetMinThickness(pcbnew.FromMM(DEFAULT_TRACK_WIDTH_MM))
    zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    points = pcbnew.VECTOR_VECTOR2I()
    for x, y in (
        (x0 + inset, y0 + inset),
        (x0 + width - inset, y0 + inset),
        (x0 + width - inset, y0 + height - inset),
        (x0 + inset, y0 + height - inset),
    ):
        points.append(pcbnew.VECTOR2I_MM(x, y))
    zone.AddPolygon(points)
    board.Add(zone)
    if not pcbnew.ZONE_FILLER(board).Fill(board.Zones()):
        raise PcbGolfError("KiCad could not fill the GND plane")


def prepare_board(
    source: Path,
    output: Path,
    *,
    margin_mm: float,
    thickness_mm: float,
) -> dict[str, Any]:
    pcbnew, board = load_board(source)
    configure_routing_rules(pcbnew, board)
    normalized_pads = normalize_stacked_connector_pads(board)
    removed_edges = remove_edge_cuts(pcbnew, board)
    width_mm, height_mm = add_rectangular_outline(
        pcbnew, board, margin_mm=margin_mm
    )
    board.GetDesignSettings().SetBoardThickness(pcbnew.FromMM(thickness_mm))

    output.parent.mkdir(parents=True, exist_ok=True)
    if not pcbnew.SaveBoard(os.fspath(output.resolve()), board):
        raise PcbGolfError(f"KiCad could not save {output}")
    return {
        "source": os.fspath(source),
        "output": os.fspath(output),
        "outline_mm": {"width": width_mm, "height": height_mm},
        "thickness_mm": thickness_mm,
        "normalized_stacked_pads": normalized_pads,
        "removed_edge_items": removed_edges,
    }


def command_prepare(args: argparse.Namespace) -> int:
    result = prepare_board(
        args.input,
        args.output,
        margin_mm=args.margin,
        thickness_mm=args.thickness,
    )
    print(json.dumps(result, indent=2))
    return 0


def command_place(args: argparse.Namespace) -> int:
    try:
        from tools.placement import place_components
    except ModuleNotFoundError:
        from placement import place_components  # type: ignore[no-redef]

    thickness = args.thickness
    if thickness is None:
        thickness = 0.8 if args.layers >= 6 else DEFAULT_BOARD_THICKNESS_MM

    pcbnew, board = load_board(args.input)
    configure_routing_rules(pcbnew, board)
    normalized_pads = normalize_stacked_connector_pads(board)
    removed_edges = remove_edge_cuts(pcbnew, board)
    result = place_components(
        pcbnew,
        board,
        x0=args.x,
        y0=args.y,
        width=args.width,
        height=args.height,
        grid=args.grid,
        clearance=args.component_clearance,
        edge_margin=args.edge_margin,
    )
    board.SetCopperLayerCount(args.layers)

    outline = pcbnew.PCB_SHAPE(board)
    outline.SetShape(pcbnew.SHAPE_T_RECT)
    outline.SetStart(pcbnew.VECTOR2I_MM(args.x, args.y))
    outline.SetEnd(
        pcbnew.VECTOR2I_MM(args.x + args.width, args.y + args.height)
    )
    outline.SetLayer(pcbnew.Edge_Cuts)
    outline.SetWidth(pcbnew.FromMM(0.05))
    board.Add(outline)
    board.GetDesignSettings().SetBoardThickness(pcbnew.FromMM(thickness))
    if not args.no_ground_plane:
        add_ground_plane(
            pcbnew,
            board,
            x0=args.x,
            y0=args.y,
            width=args.width,
            height=args.height,
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not pcbnew.SaveBoard(os.fspath(args.output.resolve()), board):
        raise PcbGolfError(f"KiCad could not save placed board {args.output}")
    result.update(
        {
            "source": os.fspath(args.input),
            "output": os.fspath(args.output),
            "thickness_mm": thickness,
            "normalized_stacked_pads": normalized_pads,
            "removed_edge_items": removed_edges,
            "copper_layers": args.layers,
            "ground_plane": not args.no_ground_plane,
        }
    )
    print(json.dumps(result, indent=2))
    return 0


def route_board(
    source: Path,
    output: Path,
    work_dir: Path,
    *,
    passes: int,
    threads: int,
    improvement_threshold: float,
    selection_strategy: str,
    update_strategy: str,
    fanout: bool,
    fanout_passes: int,
    via_cost: int,
    ripup_cost: int,
) -> dict[str, Any]:
    pcbnew, board = load_board(source)
    configure_routing_rules(pcbnew, board)
    work_dir.mkdir(parents=True, exist_ok=True)
    dsn = work_dir / f"{source.stem}.dsn"
    ses = work_dir / f"{source.stem}.ses"

    if not pcbnew.ExportSpecctraDSN(board, os.fspath(dsn.resolve())):
        raise PcbGolfError(f"KiCad failed to export Specctra DSN to {dsn}")

    jar = find_freerouting_jar()
    command = [
        "java",
        "-Xmx12g",
        "-jar",
        jar,
        "--gui.enabled=false",
        "--api_server.enabled=false",
        "--usage_and_diagnostic_data.disable_analytics=true",
        "--logging.file.enabled=false",
        f"--router.copperToEdgeClearanceUm={int(DEFAULT_CLEARANCE_MM * 1000)}",
        "--router.hole_clearance_um=200",
        f"--router.fanout.enabled={str(fanout).lower()}",
        f"--router.fanout.max_passes={fanout_passes}",
        f"--router.fanout.start_via_diameter_mm={DEFAULT_VIA_SIZE_MM}",
        f"--router.fanout.end_via_diameter_mm={DEFAULT_VIA_SIZE_MM}",
        f"--router.scoring.via_costs={via_cost}",
        f"--router.scoring.plane_via_costs={via_cost}",
        f"--router.scoring.start_ripup_costs={ripup_cost}",
        "-de",
        dsn,
        "-do",
        ses,
        "-mp",
        str(passes),
        "-mt",
        str(threads),
        "-oit",
        str(improvement_threshold),
        "-is",
        selection_strategy,
        "-us",
        update_strategy,
    ]
    run(command)
    if not ses.is_file() or ses.stat().st_size == 0:
        raise PcbGolfError("Freerouting completed without creating a session file")

    routed = pcbnew.LoadBoard(os.fspath(source.resolve()))
    configure_routing_rules(pcbnew, routed)
    if not pcbnew.ImportSpecctraSES(routed, os.fspath(ses.resolve())):
        raise PcbGolfError(f"KiCad failed to import the routed session {ses}")
    via_type = getattr(pcbnew, "PCB_VIA_T", None)
    for item in routed.GetTracks():
        if via_type is not None and item.Type() == via_type:
            item.SetWidth(pcbnew.FromMM(DEFAULT_VIA_SIZE_MM))
            item.SetDrill(pcbnew.FromMM(DEFAULT_VIA_DRILL_MM))
    if len(list(routed.Zones())) and not pcbnew.ZONE_FILLER(routed).Fill(
        routed.Zones()
    ):
        raise PcbGolfError("KiCad could not refill copper zones after routing")
    output.parent.mkdir(parents=True, exist_ok=True)
    if not pcbnew.SaveBoard(os.fspath(output.resolve()), routed):
        raise PcbGolfError(f"KiCad could not save routed board {output}")

    tracks = list(routed.GetTracks())
    via_type = getattr(pcbnew, "PCB_VIA_T", None)
    vias = sum(1 for item in tracks if via_type is not None and item.Type() == via_type)
    segments = len(tracks) - vias
    return {
        "source": os.fspath(source),
        "output": os.fspath(output),
        "dsn": os.fspath(dsn),
        "ses": os.fspath(ses),
        "segments": segments,
        "vias": vias,
    }


def command_route(args: argparse.Namespace) -> int:
    result = route_board(
        args.input,
        args.output,
        args.work_dir,
        passes=args.passes,
        threads=args.threads,
        improvement_threshold=args.improvement_threshold,
        selection_strategy=args.selection_strategy,
        update_strategy=args.update_strategy,
        fanout=args.fanout,
        fanout_passes=args.fanout_passes,
        via_cost=args.via_cost,
        ripup_cost=args.ripup_cost,
    )
    print(json.dumps(result, indent=2))
    return 0


def run_drc(board: Path, report: Path, *, include_warnings: bool) -> dict[str, Any]:
    report.parent.mkdir(parents=True, exist_ok=True)
    project = board.with_suffix(".kicad_pro")
    root_project = ROOT / "pcbgolf.kicad_pro"
    if board.resolve() != DEFAULT_BOARD.resolve() and root_project.is_file():
        shutil.copy2(root_project, project)
    command: list[str | Path] = [
        "kicad-cli",
        "pcb",
        "drc",
        "--format",
        "json",
        "--severity-error",
        "--refill-zones",
        "--save-board",
    ]
    if include_warnings:
        command.append("--severity-warning")
    command.extend(["--output", report, board])
    result = run(command, capture=True, check=False)
    if not report.is_file():
        detail = (result.stderr or result.stdout).strip()
        raise PcbGolfError(f"KiCad DRC did not create {report}: {detail}")

    payload = json.loads(report.read_text())
    violations = payload.get("violations", [])
    errors = [item for item in violations if item.get("severity") == "error"]
    warnings = [item for item in violations if item.get("severity") == "warning"]
    unconnected = payload.get("unconnected_items", [])
    by_type: dict[str, int] = {}
    for item in violations:
        kind = item.get("type", "unknown")
        by_type[kind] = by_type.get(kind, 0) + 1
    return {
        "board": os.fspath(board),
        "report": os.fspath(report),
        "errors": len(errors),
        "warnings": len(warnings),
        "unconnected_items": len(unconnected),
        "violations_by_type": dict(sorted(by_type.items())),
    }


def command_test(args: argparse.Namespace) -> int:
    result = run_drc(
        args.input, args.report, include_warnings=args.strict_warnings
    )
    print(json.dumps(result, indent=2))
    failed = result["errors"] or result["unconnected_items"]
    if args.strict_warnings:
        failed = failed or result["warnings"]
    return 1 if failed else 0


def stl_bounds(path: Path) -> tuple[list[float], list[float]]:
    data = path.read_bytes()
    points: list[tuple[float, float, float]] = []
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * triangle_count == len(data):
            for index in range(triangle_count):
                values = struct.unpack_from("<12fH", data, 84 + 50 * index)
                points.extend(
                    tuple(values[3 + vertex * 3 : 6 + vertex * 3])
                    for vertex in range(3)
                )
    if not points:
        vertex_pattern = re.compile(
            rb"\bvertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)"
        )
        points = [
            tuple(float(value) for value in match)
            for match in vertex_pattern.findall(data)
        ]
    if not points:
        raise PcbGolfError(f"No mesh vertices found in {path}")
    lower = [min(point[axis] for point in points) for axis in range(3)]
    upper = [max(point[axis] for point in points) for axis in range(3)]
    return lower, upper


def project_root_for(board: Path) -> Path:
    candidates = [board.resolve().parent, ROOT]
    for candidate in candidates:
        if (candidate / "pcbgolf.kicad_pro").is_file():
            return candidate
    return ROOT


def board_metrics(board_path: Path) -> dict[str, Any]:
    pcbnew, board = load_board(board_path)
    tracks = list(board.GetTracks())
    via_type = getattr(pcbnew, "PCB_VIA_T", None)
    vias = [item for item in tracks if via_type is not None and item.Type() == via_type]
    segments = [item for item in tracks if item not in vias]
    return {
        "copper_layers": board.GetCopperLayerCount(),
        "vias": len(vias),
        "segments": len(segments),
        "track_length_mm": sum(pcbnew.ToMM(item.GetLength()) for item in segments),
        "board_thickness_mm": pcbnew.ToMM(
            board.GetDesignSettings().GetBoardThickness()
        ),
    }


def score_board(
    board: Path,
    artifact_dir: Path,
    *,
    export_step: bool,
) -> dict[str, Any]:
    artifact_dir.mkdir(parents=True, exist_ok=True)
    project_root = project_root_for(board)
    stl = artifact_dir / f"{board.stem}-assembly.stl"
    common: list[str | Path] = [
        "--force",
        "--subst-models",
        "-D",
        f"KIPRJMOD={project_root}",
    ]
    stl_result = run(
        [
            "kicad-cli",
            "pcb",
            "export",
            "stl",
            *common,
            "--output",
            stl,
            board,
        ],
        capture=True,
        check=False,
    )
    stl_log = (stl_result.stdout or "") + (stl_result.stderr or "")
    if stl_result.returncode or not stl.is_file():
        raise PcbGolfError(f"Assembly STL export failed:\n{stl_log.strip()}")
    missing_models = stl_log.count("Could not add 3D model")
    if missing_models:
        raise PcbGolfError(
            f"Assembly export omitted {missing_models} component models; "
            "the volume would not be a valid challenge score."
        )

    lower, upper = stl_bounds(stl)
    dimensions = [upper[index] - lower[index] for index in range(3)]
    volume = math.prod(dimensions)
    metrics = board_metrics(board)
    score = (
        volume
        + 50 * metrics["vias"]
        + 5000 * metrics["copper_layers"]
    )

    step: Path | None = None
    if export_step:
        step = artifact_dir / f"{board.stem}-assembly.step"
        run(
            [
                "kicad-cli",
                "pcb",
                "export",
                "step",
                *common,
                "--output",
                step,
                board,
            ]
        )

    result = {
        "board": os.fspath(board),
        "score": score,
        "formula": {
            "assembly_volume_mm3": volume,
            "via_penalty": 50 * metrics["vias"],
            "layer_penalty": 5000 * metrics["copper_layers"],
        },
        "assembly_bbox_mm": {
            "x": dimensions[0],
            "y": dimensions[1],
            "z": dimensions[2],
        },
        **metrics,
        "artifacts": {
            "stl": os.fspath(stl),
            "step": os.fspath(step) if step else None,
        },
    }
    (artifact_dir / f"{board.stem}-score.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    return result


def command_score(args: argparse.Namespace) -> int:
    result = score_board(
        args.input, args.artifact_dir, export_step=not args.no_step
    )
    print(json.dumps(result, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    env_check = commands.add_parser("env-check", help="validate routing toolchain")
    env_check.set_defaults(func=command_env_check)

    prepare = commands.add_parser(
        "prepare", help="normalize rules and add a manufacturable board outline"
    )
    prepare.add_argument("--input", type=Path, default=DEFAULT_BOARD)
    prepare.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_BUILD_DIR / "pcbgolf-prepared.kicad_pcb",
    )
    prepare.add_argument("--margin", type=float, default=0.30)
    prepare.add_argument("--thickness", type=float, default=DEFAULT_BOARD_THICKNESS_MM)
    prepare.set_defaults(func=command_prepare)

    place = commands.add_parser(
        "place", help="apply the connectivity-aware compact floorplan"
    )
    place.add_argument("--input", type=Path, default=DEFAULT_BOARD)
    place.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_BUILD_DIR / "pcbgolf-placed.kicad_pcb",
    )
    place.add_argument("--x", type=float, default=100.0)
    place.add_argument("--y", type=float, default=50.0)
    place.add_argument("--width", type=float, default=62.0)
    place.add_argument("--height", type=float, default=54.0)
    place.add_argument("--grid", type=float, default=0.25)
    place.add_argument("--component-clearance", type=float, default=0.15)
    place.add_argument("--edge-margin", type=float, default=0.15)
    place.add_argument(
        "--thickness",
        type=float,
        default=None,
        help="board thickness in mm (default: 0.4 for 2/4 layers, 0.8 otherwise)",
    )
    place.add_argument("--layers", type=int, choices=(2, 4, 6, 8), default=4)
    place.add_argument(
        "--no-ground-plane",
        action="store_true",
        help="omit the solid front GND plane",
    )
    place.set_defaults(func=command_place)

    route = commands.add_parser("route", help="route a board with Freerouting")
    route.add_argument("--input", type=Path, required=True)
    route.add_argument("--output", type=Path, required=True)
    route.add_argument("--work-dir", type=Path, default=DEFAULT_BUILD_DIR / "route")
    route.add_argument("--passes", type=int, default=20)
    route.add_argument("--threads", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    route.add_argument("--improvement-threshold", type=float, default=0.1)
    route.add_argument(
        "--selection-strategy",
        choices=("prioritized", "sequential", "random"),
        default="prioritized",
    )
    route.add_argument(
        "--update-strategy",
        choices=("greedy", "global", "hybrid"),
        default="greedy",
    )
    route.add_argument(
        "--fanout",
        action="store_true",
        help="fan out every SMD pin before routing (uses many more vias)",
    )
    route.add_argument(
        "--fanout-passes",
        type=int,
        default=4,
        help="maximum fanout passes when --fanout is enabled",
    )
    route.add_argument(
        "--via-cost",
        type=int,
        default=500,
        help="Freerouting cost assigned to each via",
    )
    route.add_argument(
        "--ripup-cost",
        type=int,
        default=100,
        help="initial cost of ripping an existing route",
    )
    route.set_defaults(func=command_route)

    test = commands.add_parser("test", help="run KiCad DRC and connectivity checks")
    test.add_argument("--input", type=Path, required=True)
    test.add_argument(
        "--report", type=Path, default=DEFAULT_BUILD_DIR / "drc.json"
    )
    test.add_argument("--strict-warnings", action="store_true")
    test.set_defaults(func=command_test)

    score = commands.add_parser("score", help="compute the PCB Golf challenge score")
    score.add_argument("--input", type=Path, required=True)
    score.add_argument(
        "--artifact-dir", type=Path, default=DEFAULT_BUILD_DIR / "score"
    )
    score.add_argument("--no-step", action="store_true")
    score.set_defaults(func=command_score)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except (PcbGolfError, OSError, subprocess.SubprocessError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
