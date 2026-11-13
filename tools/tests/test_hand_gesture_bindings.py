"""Validate the actual C++ binding inventory against the bundled Khronos registry.

Run after building hand_gestures_test. No runtime, controllers or game required.
"""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=REPO / "build/hand_gestures/bin/hand_gestures_test.exe")
    args = parser.parse_args()
    registry = ET.parse(REPO / "third_party/openxr-sdk/specification/registry/xr.xml")
    profiles = {p.attrib["name"]: p for p in registry.findall(".//interaction_profile") if p.findall("component")}
    names = ["oculus/touch_controller", "facebook/touch_controller_pro", "meta/touch_controller_plus",
             "valve/index_controller", "bytedance/pico4_controller", "htc/vive_controller",
             "microsoft/motion_controller", "hp/mixed_reality_controller", "khr/simple_controller"]
    checked = 0
    for name in names:
        profile = "/interaction_profiles/" + name
        assert profile in profiles, f"Missing profile in bundled registry: {profile}"
        native = {}
        for component in profiles[profile].findall("component"):
            hands = [component.get("user_path")] if component.get("user_path") else ["/user/hand/left", "/user/hand/right"]
            for hand in hands:
                native[hand + component.attrib["subpath"]] = component.attrib["type"]
        lines = subprocess.check_output([str(args.binary.resolve()), "--bindings", profile], text=True).splitlines()
        assert lines, f"No bindings for {profile}"
        seen = set()
        for line in lines:
            sensor, path = line.split()
            assert path in native, f"Unsupported sensor source: {profile} {path}"
            expected = {"XR_ACTION_TYPE_BOOLEAN_INPUT", "XR_ACTION_TYPE_FLOAT_INPUT"} if int(sensor) < 2 else {"XR_ACTION_TYPE_BOOLEAN_INPUT"}
            assert native[path] in expected, f"Wrong action type: {profile} {path}"
            assert (sensor, path) not in seen, f"Duplicate binding: {profile} {path}"
            seen.add((sensor, path))
            checked += 1
        print(f"{name}: {len(lines)} valid bindings")
    unknown = subprocess.check_output([str(args.binary.resolve()), "--bindings", "/interaction_profiles/unknown/controller"], text=True)
    assert not unknown, "Unknown profiles must not get invented sources"
    print(f"PASS: {checked} sensor bindings across {len(names)} profiles")


if __name__ == "__main__":
    main()
