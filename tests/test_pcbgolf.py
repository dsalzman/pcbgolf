from __future__ import annotations

from pathlib import Path
import struct
import tempfile
import unittest

from tools.pcbgolf import stl_bounds
from tools.finish_route import Box, _raster_box, find_path
from tools.placement import Rect, SpatialIndex


class StlBoundsTest(unittest.TestCase):
    def test_ascii_stl(self) -> None:
        text = """solid board
  facet normal 0 0 1
    outer loop
      vertex -1.5 2 0
      vertex 4 2 0
      vertex 4 8.25 3
    endloop
  endfacet
endsolid board
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "assembly.stl"
            path.write_text(text)
            lower, upper = stl_bounds(path)

        self.assertEqual(lower, [-1.5, 2.0, 0.0])
        self.assertEqual(upper, [4.0, 8.25, 3.0])

    def test_binary_stl(self) -> None:
        normal_and_vertices = (
            0.0,
            0.0,
            1.0,
            -2.0,
            -3.0,
            -4.0,
            5.0,
            6.0,
            7.0,
            1.0,
            2.0,
            3.0,
            0,
        )
        data = b"PCB Golf".ljust(80, b"\0")
        data += struct.pack("<I", 1)
        data += struct.pack("<12fH", *normal_and_vertices)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "assembly.stl"
            path.write_bytes(data)
            lower, upper = stl_bounds(path)

        self.assertEqual(lower, [-2.0, -3.0, -4.0])
        self.assertEqual(upper, [5.0, 6.0, 7.0])

    def test_rejects_empty_mesh(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "empty.stl"
            path.write_text("solid empty\nendsolid empty\n")
            with self.assertRaisesRegex(RuntimeError, "No mesh vertices"):
                stl_bounds(path)


class PlacementGeometryTest(unittest.TestCase):
    def test_touching_rectangles_do_not_overlap(self) -> None:
        left = Rect(0.0, 0.0, 2.0, 2.0, "left")
        right = Rect(2.0, 0.0, 4.0, 2.0, "right")
        self.assertFalse(left.overlaps(right))
        self.assertTrue(left.expanded(0.1).overlaps(right))

    def test_spatial_index_reports_each_collision_once(self) -> None:
        index = SpatialIndex(cell_size=1.0)
        placed = Rect(0.0, 0.0, 3.0, 3.0, "U1")
        index.add(placed)
        collisions = index.collisions(Rect(1.0, 1.0, 2.0, 2.0, "R1"))
        self.assertEqual(collisions, [placed])


class CompletionRouterTest(unittest.TestCase):
    def test_grid_router_detours_around_obstacle(self) -> None:
        occupied: set[tuple[int, int]] = set()
        _raster_box(occupied, Box(1.0, -0.2, 2.0, 0.2))
        path = find_path(occupied, (0.0, 0.0), (3.0, 0.0), (0.0, -1.0, 3.0, 1.0))
        self.assertEqual(path[0], (0.0, 0.0))
        self.assertEqual(path[-1], (3.0, 0.0))
        self.assertTrue(any(abs(y) > 0.2 for _, y in path))


if __name__ == "__main__":
    unittest.main()
