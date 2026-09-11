from __future__ import annotations

from pathlib import Path
import struct
import tempfile
import unittest

from tools.pcbgolf import stl_bounds


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


if __name__ == "__main__":
    unittest.main()
