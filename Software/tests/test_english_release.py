from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"
spec = importlib.util.spec_from_file_location("english_release_audit", SCRIPTS / "verify_english_release.py")
assert spec and spec.loader
audit = importlib.util.module_from_spec(spec)
previous = list(sys.path)
try:
    sys.path.insert(0, str(SCRIPTS))
    spec.loader.exec_module(audit)
finally:
    sys.path[:] = previous


class EnglishReleaseTests(unittest.TestCase):
    def test_checks_copy_and_filenames_but_preserves_binary_assets(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            english = root / "guide.md"
            english.write_text("# Getting started\nUse the full Windows package.\n", encoding="utf-8")
            binary = root / "icon.png"
            binary.write_bytes(b"\x89PNG\xff\x00")
            translated = root / "message.txt"
            translated.write_text("English\n\u4e2d\u6587\n", encoding="utf-8")
            filename = root / "\u8bf4\u660e.txt"
            filename.write_text("English copy", encoding="utf-8")
            self.assertEqual(audit.non_english_copy([english, binary], root), [])
            findings = audit.non_english_copy([translated, filename], root)
            self.assertEqual(len(findings), 2)
            self.assertIn("message.txt:2:", findings[0])
            self.assertIn("non-English filename", findings[1])


if __name__ == "__main__":
    unittest.main()
