from __future__ import annotations

from contextlib import redirect_stdout
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def load_script(name: str):
    spec = importlib.util.spec_from_file_location(name, PROJECT_ROOT / "scripts" / f"{name}.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


exporter = load_script("export_github_source")
verifier = load_script("verify_team_release")


class TeamReleaseVerifierTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "project"
        self.root.mkdir()
        for relative in exporter.REQUIRED_FILES:
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("release fixture\n", encoding="utf-8")
        (self.root / "Software/control_station.py").write_text(
            f'VERSION = "{verifier.VERSION}"\n', encoding="utf-8"
        )
        (self.root / "scripts/extra_runtime.py").write_text("print('runtime')\n", encoding="utf-8")
        (self.root / "scripts/verify_team_release.py").write_text("print('source only')\n", encoding="utf-8")
        output = self.root / "Software/dist/source"
        self.source_folder, self.source_zip = exporter.export_source(self.root, output)

    def write_checksum(self, archive: Path) -> None:
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        archive.with_name(archive.name + ".sha256").write_text(
            f"{digest}  {archive.name}\n", encoding="ascii"
        )

    def package(self, category: str, suffix: str, project_prefix: str, launcher: str, omit: set[str] = frozenset()) -> Path:
        folder = f"UR-Print-Workstation-{verifier.VERSION}-{suffix}"
        archive_path = self.root / "Software" / category / f"{folder}.zip"
        archive_path.parent.mkdir(parents=True, exist_ok=True)
        selected, _ = verifier.source_selection(self.root)
        with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            archive.writestr(f"{folder}/{launcher}", b"native launcher")
            for relative, source in selected.items():
                if verifier.shared_runtime_file(relative) and relative not in omit:
                    archive.write(source, f"{folder}/{project_prefix}{relative}")
        self.write_checksum(archive_path)
        return archive_path

    def all_packages(self, omit: set[str] = frozenset()) -> None:
        self.package(
            "dist/mac", "mac-arm64", "",
            "Software/Mac/UR Print Workstation.app/Contents/MacOS/URPrintWorkstation", omit,
        )
        for arch in ("x64", "arm64"):
            self.package("Windows", f"win-{arch}", "project/", "UR Print Workstation.exe", omit)

    def rewrite_source_member(self, member: str, content: bytes) -> None:
        replacement = self.source_zip.with_name("replacement.zip")
        with zipfile.ZipFile(self.source_zip) as original, zipfile.ZipFile(replacement, "w") as rewritten:
            for info in original.infolist():
                rewritten.writestr(info, content if info.filename == member else original.read(info))
        replacement.replace(self.source_zip)
        self.write_checksum(self.source_zip)

    def test_source_zip_checksum_manifest_and_runtime_snapshot_pass(self) -> None:
        with patch.object(verifier, "ROOT", self.root), patch.object(verifier, "SOFTWARE", self.root / "Software"):
            self.assertEqual(
                verifier.release_zip(self.source_folder.name, "dist/source").resolve(), self.source_zip.resolve()
            )
            hashes, omitted = verifier.inspect_source(self.source_zip, self.source_folder.name, self.root)
            self.assertIn("scripts/extra_runtime.py", hashes)
            self.assertNotIn("scripts/verify_team_release.py", hashes)
            self.assertEqual(omitted, set())
            self.all_packages()
            with redirect_stdout(io.StringIO()) as output:
                self.assertEqual(verifier.main(), 0)
            self.assertIn("files match source ZIP and all platforms", output.getvalue())

    def test_missing_file_in_every_package_is_caught_against_source_zip(self) -> None:
        self.all_packages({"scripts/extra_runtime.py"})
        with patch.object(verifier, "ROOT", self.root), patch.object(verifier, "SOFTWARE", self.root / "Software"):
            with self.assertRaisesRegex(ValueError, "Package differs from source ZIP snapshot.*extra_runtime.py"):
                verifier.main()

    def test_report_records_complete_verified_snapshot_and_package_checksums(self) -> None:
        self.all_packages()
        report_path = self.root / "reports/parity.json"
        with patch.object(verifier, "ROOT", self.root), patch.object(verifier, "SOFTWARE", self.root / "Software"):
            with redirect_stdout(io.StringIO()):
                self.assertEqual(verifier.main(report_path), 0)
        report = json.loads(report_path.read_text(encoding="utf-8"))
        self.assertEqual(report["version"], verifier.VERSION)
        self.assertEqual(report["result"], "passed")
        self.assertEqual(report["shared_file_count"], len(report["shared_files"]))
        self.assertIn("scripts/extra_runtime.py", report["shared_files"])
        self.assertEqual(len(report["packages"]), 4)
        for item in report["packages"]:
            archive = next((self.root / "Software").rglob(item["file"]))
            self.assertEqual(item["sha256"], hashlib.sha256(archive.read_bytes()).hexdigest())

    def test_source_manifest_hash_must_match_zip_member(self) -> None:
        member = f"{self.source_folder.name}/SOURCE_MANIFEST.json"
        with zipfile.ZipFile(self.source_zip) as archive:
            manifest = json.loads(archive.read(member))
        manifest["files"][0]["sha256"] = "0" * 64
        self.rewrite_source_member(member, (json.dumps(manifest) + "\n").encode("utf-8"))
        with self.assertRaisesRegex(ValueError, "Source manifest hash differs from ZIP"):
            verifier.inspect_source(self.source_zip, self.source_folder.name, self.root)

    def test_source_zip_rejects_unlisted_private_file(self) -> None:
        replacement = self.source_zip.with_name("replacement.zip")
        private = self.root / "Software/config.json"
        private.write_text('{"private": true}\n', encoding="utf-8")
        with zipfile.ZipFile(self.source_zip) as original, zipfile.ZipFile(replacement, "w") as rewritten:
            for info in original.infolist():
                rewritten.writestr(info, original.read(info))
            rewritten.write(private, f"{self.source_folder.name}/Software/config.json")
        replacement.replace(self.source_zip)
        with self.assertRaisesRegex(ValueError, "Source ZIP file list differs from clean export"):
            verifier.inspect_source(self.source_zip, self.source_folder.name, self.root)

    def test_source_checksum_rejects_tampering(self) -> None:
        self.source_zip.write_bytes(self.source_zip.read_bytes() + b"tampering")
        with patch.object(verifier, "SOFTWARE", self.root / "Software"):
            with self.assertRaisesRegex(ValueError, "Checksum mismatch"):
                verifier.release_zip(self.source_folder.name, "dist/source")


if __name__ == "__main__":
    unittest.main()
