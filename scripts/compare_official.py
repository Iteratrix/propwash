"""Compare propwash output against Betaflight's official blackbox_decode CSV.

Run from the propwash workspace root:
    python3 compare_official.py

Requires target/release/propwash built and blackbox_decode CSVs already generated.
"""

import csv
import json
import os
import subprocess
import sys

FIXTURES = "propwash-core/tests/fixtures"
PROPWASH = "target/release/propwash"

INT_FIELDS = {
    "loopIteration", "time (us)",
    "axisP[0]", "axisP[1]", "axisP[2]",
    "axisI[0]", "axisI[1]", "axisI[2]",
    "axisD[0]", "axisD[1]",
    "axisF[0]", "axisF[1]", "axisF[2]",
    "rcCommand[0]", "rcCommand[1]", "rcCommand[2]", "rcCommand[3]",
    "setpoint[0]", "setpoint[1]", "setpoint[2]", "setpoint[3]",
    "rssi",
    "gyroADC[0]", "gyroADC[1]", "gyroADC[2]",
    "accSmooth[0]", "accSmooth[1]", "accSmooth[2]",
    "motor[0]", "motor[1]", "motor[2]", "motor[3]",
}

FIELD_REMAP = {"time (us)": "time"}


def load_csv_frames(path):
    frames = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = {}
            for key, val in row.items():
                key = key.strip()
                if key in INT_FIELDS:
                    mapped = FIELD_REMAP.get(key, key)
                    try:
                        frame[mapped] = int(val.strip())
                    except ValueError:
                        pass
            frames.append(frame)
    return frames


def load_propwash_frames(log_path, session_idx):
    result = subprocess.run(
        [PROPWASH, "dump", log_path, "--session", str(session_idx)],
        capture_output=True, text=True
    )
    if result.returncode != 0 or not result.stdout.strip():
        return []
    data = json.loads(result.stdout)
    if not data["sessions"]:
        return []
    return [{k: int(v) for k, v in f["values"].items()} for f in data["sessions"][0]["frames"]]


def find_csv_sessions(log_path):
    base = os.path.splitext(log_path)[0]
    csvs = []
    for i in range(1, 100):
        path = f"{base}.{i:02d}.csv"
        if os.path.exists(path) and os.path.getsize(path) > 0:
            csvs.append((i, path))
        else:
            break
    return csvs


def compare():
    if not os.path.exists(PROPWASH):
        print("Build first: cargo build --release")
        sys.exit(1)

    log_files = []
    for root, dirs, files in os.walk(FIXTURES):
        dirs.sort()
        for f in sorted(files):
            if f.endswith((".bbl", ".BFL", ".TXT")) and not f.endswith(".csv"):
                log_files.append(os.path.join(root, f))

    grand_frames = 0
    grand_fields = 0
    grand_mismatches = 0

    for log_path in log_files:
        rel = os.path.relpath(log_path, FIXTURES)
        csv_sessions = find_csv_sessions(log_path)
        if not csv_sessions:
            continue

        file_mismatches = 0
        file_frames = 0

        for session_idx, csv_path in csv_sessions:
            ref_frames = load_csv_frames(csv_path)
            pw_frames = load_propwash_frames(log_path, session_idx)

            if not ref_frames or not pw_frames:
                continue

            min_frames = min(len(ref_frames), len(pw_frames))
            common_fields = set(ref_frames[0].keys()) & set(pw_frames[0].keys())

            for i in range(min_frames):
                for field in common_fields:
                    ref_val = ref_frames[i].get(field)
                    pw_val = pw_frames[i].get(field)
                    if ref_val is not None and pw_val is not None and ref_val != pw_val:
                        if file_mismatches < 10:
                            print(f"  {rel} s{session_idx} frame {i} [{field}]: official={ref_val} propwash={pw_val} (diff={pw_val - ref_val})")
                        file_mismatches += 1
                        grand_mismatches += 1
                    grand_fields += 1

            file_frames += min_frames

        if file_frames > 0:
            if file_mismatches > 10:
                print(f"  ... and {file_mismatches - 10} more")
            status = "MATCH" if file_mismatches == 0 else f"DIFF ({file_mismatches})"
            print(f"{status:30s} {rel}: {file_frames} frames")
        grand_frames += file_frames

    print(f"\n{'=' * 60}")
    print(f"Total: {grand_frames} frames, {grand_fields} field comparisons")
    print(f"Mismatches: {grand_mismatches}")
    if grand_mismatches == 0:
        print("ALL MATCH")


if __name__ == "__main__":
    compare()
