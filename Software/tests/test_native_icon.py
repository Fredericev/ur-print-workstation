"""Pure asset checks: no app, backend, ROS node or controller is launched."""
from pathlib import Path
import plistlib
import struct
import unittest


SOFTWARE = Path(__file__).resolve().parents[1]


class NativeIconTests(unittest.TestCase):
    def test_declared_mac_icon_has_valid_packaged_artwork(self):
        native = SOFTWARE / "native_app"
        info = plistlib.loads((native / "Info.plist").read_bytes())
        icon = native / info["CFBundleIconFile"]
        self.assertTrue(icon.is_file(), "The declared Mac icon must exist before packaging")
        data = icon.read_bytes()
        self.assertEqual(data[:4], b"icns")
        self.assertEqual(struct.unpack(">I", data[4:8])[0], len(data))
        offset, frames = 8, []
        while offset < len(data):
            kind, size = struct.unpack(">4sI", data[offset:offset + 8])
            self.assertGreater(size, 8)
            self.assertLessEqual(offset + size, len(data))
            frames.append(kind)
            offset += size
        self.assertEqual(offset, len(data))
        self.assertIn(b"ic08", frames, "Keep the source artwork's native 256 px frame")


if __name__ == "__main__":
    unittest.main()
