"""Turn the ClaudeBot assembly STEP into AdvantageScope's robot model, one GLB per motion group.

    python tools/step_to_glb.py <robotA_assembly_complete.step> ascope/Robot_ClaudeBot

Needs build123d and numpy (the CAD source folder's .venv has both). Writes model.glb (everything that
does not move on the robot) and model_0.glb to model_5.glb, in the order of Mechanism/Components: stage A,
stage B, carriage, wrist, climber fork, funnel tray. Each moving group is written with its joint at the
origin, the wrist turned to level, the fork as stowed and the tray as closed, which are the zero poses
SimulatedRobot publishes. config.json and layout.json are kept by hand.

GLB is Y up, in metres; AdvantageScope turns it back with the 90 degree X rotation in config.json.
"""
import json
import math
import os
import re
import struct
import sys

import numpy as np
from build123d import import_step

MM_PER_IN = 25.4
TOLERANCE_MM = 0.6
ANGULAR_TOLERANCE = 0.6
SMALLEST_PART_IN = 0.6  # screws, nuts and rivets below this are left out

GROUPS = ["base", "stage A", "stage B", "carriage", "wrist", "fork", "tray"]
# Joint position in inches, and the turn about +Y in degrees that takes the CAD pose to the zero pose.
JOINTS = {
    "wrist": ((4.5, 0.0, 24.5), -90.0),
    "fork": ((-13.5, 0.0, 16.0), 0.0),
    "tray": ((0.0, -7.1, 27.5), 0.0),
}

TRAY_PARTS = re.compile(r"^F-(201|202|203|102|104|107|108|112)")
# Parts of the wrist joint that turn with the wrist; the rest of that group rides on the carriage.
TURNS_WITH_WRIST = re.compile(r"^(W-102|W-104|W-103|Shaft_end_screw.*wrist_stub|WCP-0800)")
SKIP_GROUPS = ("Elevator_Rev_G:_Reference", "Wrist_reference")

# First match wins: (pattern, RGBA, roughness).
COLOURS = [
    (r"polycarbonate", (0.75, 0.85, 0.92, 0.35), 0.2),
    (r"padding|noodle", (0.10, 0.22, 0.60, 1), 0.9),
    (r"plywood", (0.72, 0.58, 0.40, 1), 0.8),
    (r"flex_wheel|claw_roller__|roller_\(|sleeve.*rubber|neoprene", (0.16, 0.45, 0.24, 1), 0.9),
    (r"Kraken|motor", (0.10, 0.10, 0.11, 1), 0.5),
    (r"Limelight", (0.15, 0.50, 0.20, 1), 0.6),
    (r"printed|Onyx", (0.17, 0.17, 0.18, 1), 0.9),
    (r"belt|tyre|tread|battery|EL-|servo|webbing", (0.13, 0.13, 0.14, 1), 0.8),
    (r"gear|sprocket|chain|steel|spring|screw|SHCS|BHCS|bearing|shaft|axle", (0.55, 0.56, 0.58, 1), 0.4),
    (r"", (0.76, 0.77, 0.79, 1), 0.45),
]


def leaves(node):
    kids = list(node.children)
    if not kids:
        return [node]
    return [s for k in kids for s in leaves(k)]


def motion_group(group_label, label):
    if group_label.startswith("Elevator_Rev_G:_Stage_A"):
        return "stage A"
    if group_label.startswith("Elevator_Rev_G:_Stage_B"):
        return "stage B"
    if group_label.startswith("Elevator_Rev_G:_Carriage"):
        return "carriage"
    if group_label.startswith(("Elevator_Rev_G:_Wrist_joint", "Wrist:_carriage_side")):
        return "wrist" if TURNS_WITH_WRIST.search(label) else "carriage"
    if group_label.startswith("Wrist_and_claw"):
        return "wrist"
    if group_label.startswith("Climber") and "fork" in group_label:
        return "fork"
    if group_label.startswith("Funnel") and TRAY_PARTS.search(label):
        return "tray"
    return "base"


def colour_of(label):
    for i, (pattern, _, _) in enumerate(COLOURS):
        if re.search(pattern, label, re.IGNORECASE):
            return i
    raise AssertionError


def mesh(shape):
    verts, tris = shape.tessellate(TOLERANCE_MM, ANGULAR_TOLERANCE)
    v = np.array([(p.X, p.Y, p.Z) for p in verts], dtype=np.float64) / MM_PER_IN
    t = np.array(tris, dtype=np.uint32)
    return v, t


def to_zero_pose(v, group):
    if group not in JOINTS:
        return v
    origin, turn_deg = JOINTS[group]
    v = v - np.array(origin)
    if turn_deg:
        a = math.radians(turn_deg)
        c, s = math.cos(a), math.sin(a)
        x, z = v[:, 0].copy(), v[:, 2].copy()
        v[:, 0] = x * c + z * s
        v[:, 2] = -x * s + z * c
    return v


def write_glb(path, by_colour):
    """by_colour: colour index -> list of (vertices in inches, triangles)."""
    blob = bytearray()
    views, accessors, primitives, materials = [], [], [], []
    for colour in sorted(by_colour):
        vs, ts, base = [], [], 0
        for v, t in by_colour[colour]:
            vs.append(v)
            ts.append(t + base)
            base += len(v)
        v = np.vstack(vs) * 0.0254
        v = np.column_stack([v[:, 0], v[:, 2], -v[:, 1]]).astype(np.float32)  # Z up to Y up
        t = np.vstack(ts).astype(np.uint32)
        _, rgba, rough = COLOURS[colour]
        material = {
            "doubleSided": True,
            "pbrMetallicRoughness": {"baseColorFactor": list(rgba), "metallicFactor": 0.0, "roughnessFactor": rough},
        }
        if rgba[3] < 1:
            material["alphaMode"] = "BLEND"
        for data, target in ((v, 34962), (t, 34963)):
            views.append({"buffer": 0, "byteOffset": len(blob), "byteLength": data.nbytes, "target": target})
            blob += data.tobytes()
        accessors.append({
            "bufferView": len(views) - 2, "componentType": 5126, "count": len(v), "type": "VEC3",
            "min": v.min(axis=0).tolist(), "max": v.max(axis=0).tolist()})
        accessors.append({"bufferView": len(views) - 1, "componentType": 5125, "count": int(t.size), "type": "SCALAR"})
        primitives.append({
            "attributes": {"POSITION": len(accessors) - 2}, "indices": len(accessors) - 1, "material": len(materials)})
        materials.append(material)
    doc = {
        "asset": {"version": "2.0", "generator": "ClaudeBot step_to_glb.py"}, "scene": 0, "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}], "meshes": [{"primitives": primitives}], "materials": materials,
        "accessors": accessors, "bufferViews": views, "buffers": [{"byteLength": len(blob)}]}
    text = json.dumps(doc, separators=(",", ":")).encode()
    text += b" " * (-len(text) % 4)
    blob += b"\0" * (-len(blob) % 4)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(text) + 8 + len(blob)))
        f.write(struct.pack("<II", len(text), 0x4E4F534A) + text)
        f.write(struct.pack("<II", len(blob), 0x004E4942) + bytes(blob))


def main():
    step, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    assembly = import_step(step)
    parts = {g: {} for g in GROUPS}
    count = {g: 0 for g in GROUPS}
    dropped = 0
    total = 0
    for group in assembly.children:
        if group.label.startswith(SKIP_GROUPS):
            continue
        for part in leaves(group):
            total += 1
            size = part.bounding_box().diagonal / MM_PER_IN
            if size < SMALLEST_PART_IN:
                dropped += 1
                continue
            g = motion_group(group.label, part.label)
            v, t = mesh(part)
            if len(t) == 0:
                dropped += 1
                continue
            parts[g].setdefault(colour_of(part.label), []).append((to_zero_pose(v, g), t))
            count[g] += 1
    assert sum(count.values()) + dropped == total, "a part was lost"
    for i, g in enumerate(GROUPS):
        name = "model.glb" if i == 0 else "model_%d.glb" % (i - 1)
        path = os.path.join(out, name)
        write_glb(path, parts[g])
        allv = np.vstack([v for c in parts[g].values() for v, _ in c])
        tris = sum(len(t) for c in parts[g].values() for _, t in c)
        lo, hi = allv.min(axis=0), allv.max(axis=0)
        print("%-11s %-9s %4d parts %7d triangles %5.1f MB   X %6.1f..%5.1f  Y %6.1f..%5.1f  Z %6.1f..%5.1f in" % (
            name, g, count[g], tris, os.path.getsize(path) / 1e6, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    print("%d parts, %d too small to draw" % (total, dropped))


if __name__ == "__main__":
    main()
