# Builds an animator control rig on top of the UE5-style deform skeleton in
# Rig_BaseFemale.blend.
#
# The UE bones keep their names, rest pose and hierarchy, so the exported
# skeleton stays compatible with Unreal / Godot retargeting. The script adds
# non-deform bones:
#   CTRL_*  animator controls (custom shapes, bone collections, colors)
#   MCH_*   mechanism (IK chains, IK/FK blend chain, twist helpers)
#   TGT_*   one per driven UE bone, carrying that bone's exact rest matrix
# Every driven UE bone gets a single Copy Transforms constraint to its TGT_
# bone, which sidesteps the FBX-imported bone orientations.
#
# Run it from Blender's Text Editor (or over MCP). Re-running removes the
# previous build first. IK/FK switches are custom properties on the armature
# object (0 = FK, 1 = IK) and in the 3D View sidebar "Rig" tab, which also has
# IK/FK snap buttons and the parent-space switches (head, FK arm, poles).
#
# Export: glTF "Deform Bones Only" / FBX "Only Deform Bones", with animation
# sampling/baking enabled.

import math

import bpy
from mathutils import Vector

ARMATURE = "Armature"
BODY = "BodyCombined"
WGT_COLLECTION = "WGT_BaseFemale"
RIG_PREFIXES = ("CTRL_", "MCH_", "TGT_")
CON_PREFIX = "RIG_"
UI_TEXT = "rig_ui_basefemale.py"
SIDES = ("l", "r")

# Custom properties on the armature object: name -> (default, description).
SWITCHES = {
    "ik_arm_l": (0.0, "0 = FK, 1 = IK"),
    "ik_arm_r": (0.0, "0 = FK, 1 = IK"),
    "ik_leg_l": (1.0, "0 = FK, 1 = IK"),
    "ik_leg_r": (1.0, "0 = FK, 1 = IK"),
    "head_follow": (0.0, "Head rotation: 0 = follow neck, 1 = follow torso"),
    "arm_fk_follow_l": (0.0, "FK arm rotation: 0 = follow clavicle, 1 = follow torso"),
    "arm_fk_follow_r": (0.0, "FK arm rotation: 0 = follow clavicle, 1 = follow torso"),
    "pole_follow_arm_l": (0.0, "Elbow pole: 0 = follow root, 1 = follow hand IK control"),
    "pole_follow_arm_r": (0.0, "Elbow pole: 0 = follow root, 1 = follow hand IK control"),
    "pole_follow_leg_l": (0.0, "Knee pole: 0 = follow root, 1 = follow foot IK control"),
    "pole_follow_leg_r": (0.0, "Knee pole: 0 = follow root, 1 = follow foot IK control"),
}

COLLECTIONS = {
    "torso": "Root & Torso",
    "spine": "Spine & Head",
    "face": "Face",
    "arm_ik_l": "Arm.L IK",
    "arm_fk_l": "Arm.L FK",
    "arm_ik_r": "Arm.R IK",
    "arm_fk_r": "Arm.R FK",
    "fingers_l": "Fingers.L",
    "fingers_r": "Fingers.R",
    "leg_ik_l": "Leg.L IK",
    "leg_fk_l": "Leg.L FK",
    "leg_ik_r": "Leg.R IK",
    "leg_fk_r": "Leg.R FK",
    "def": "DEF (UE5)",
    "mch": "MCH",
}
HIDDEN_COLLECTIONS = {"def", "mch", "arm_ik_l", "arm_ik_r", "leg_fk_l", "leg_fk_r"}

# Center yellow, left blue, right red, face green.
COLORS = {"c": "THEME09", "l": "THEME04", "r": "THEME01", "face": "THEME03"}

FINGER_CHAINS = (
    ("thumb_01", "thumb_02", "thumb_03"),
    ("index_metacarpal", "index_01", "index_02", "index_03"),
    ("middle_metacarpal", "middle_01", "middle_02", "middle_03"),
    ("ring_metacarpal", "ring_01", "ring_02", "ring_03"),
    ("pinky_metacarpal", "pinky_01", "pinky_02", "pinky_03"),
)


# ---------------------------------------------------------------------------
# Context helpers

def _view3d_override():
    for window in bpy.context.window_manager.windows:
        for area in window.screen.areas:
            if area.type == 'VIEW_3D':
                region = next(r for r in area.regions if r.type == 'WINDOW')
                return {"window": window, "area": area, "region": region}
    return {}


def set_mode(arm, mode):
    override = _view3d_override()
    active = bpy.context.view_layer.objects.active
    if active is not None and active != arm and active.mode != 'OBJECT':
        with bpy.context.temp_override(**override):
            bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.view_layer.objects.active = arm
    arm.select_set(True)
    if arm.mode != mode:
        with bpy.context.temp_override(**override):
            bpy.ops.object.mode_set(mode=mode)


# ---------------------------------------------------------------------------
# Cleanup of a previous build

def cleanup(arm):
    if arm.animation_data:
        for fc in list(arm.animation_data.drivers):
            if CON_PREFIX in fc.data_path:
                arm.animation_data.drivers.remove(fc)
    for pb in arm.pose.bones:
        for con in list(pb.constraints):
            if con.name.startswith(CON_PREFIX):
                pb.constraints.remove(con)
        if pb.name.startswith(RIG_PREFIXES):
            pb.custom_shape = None

    set_mode(arm, 'EDIT')
    edit_bones = arm.data.edit_bones
    for eb in list(edit_bones):
        if eb.name.startswith(RIG_PREFIXES):
            edit_bones.remove(eb)
    set_mode(arm, 'OBJECT')

    for name in COLLECTIONS.values():
        coll = arm.data.collections_all.get(name)
        if coll is not None:
            arm.data.collections.remove(coll)

    for ob in list(bpy.data.objects):
        if ob.name.startswith("WGT_CTRL_"):
            mesh = ob.data
            bpy.data.objects.remove(ob)
            if mesh is not None and mesh.users == 0:
                bpy.data.meshes.remove(mesh)
    wgt = bpy.data.collections.get(WGT_COLLECTION)
    if wgt is not None:
        bpy.data.collections.remove(wgt)


# ---------------------------------------------------------------------------
# Geometry helpers (everything is in armature space)

class Frame:
    """World directions and units expressed in armature space."""

    def __init__(self, arm):
        self.mw = arm.matrix_world.copy()
        self.inv = self.mw.inverted()
        rot = self.inv.to_3x3()
        self.up = (rot @ Vector((0, 0, 1))).normalized()
        self.fwd = (rot @ Vector((0, -1, 0))).normalized()
        self.left = (rot @ Vector((1, 0, 0))).normalized()
        # Armature units per metre (the FBX import uses a 0.01 object scale).
        self.m = 1.0 / self.mw.to_scale().x

    def ground(self, p, z):
        w = self.mw @ p
        w.z = z
        return self.inv @ w

    def world_z(self, p):
        return (self.mw @ p).z


def body_points(arm, body, threshold=0.3):
    """Vertex positions (armature space) per vertex group, weight >= threshold."""
    to_arm = arm.matrix_world.inverted() @ body.matrix_world
    names = {g.index: g.name for g in body.vertex_groups}
    out = {}
    for v in body.data.vertices:
        co = None
        for g in v.groups:
            if g.weight >= threshold:
                if co is None:
                    co = to_arm @ v.co
                out.setdefault(names[g.group], []).append(co)
    return out


def bend_dir(a, b, c):
    """Unit vector from the a-c line toward b (the elbow/knee direction)."""
    axis = c - a
    t = (b - a).dot(axis) / axis.length_squared
    d = b - (a + axis * t)
    return d.normalized() if d.length > 1e-4 else None


def perp(v, axis):
    axis = axis.normalized()
    return (v - axis * v.dot(axis)).normalized()


def _basis(axis, hint=None):
    axis = axis.normalized()
    if hint is not None:
        u = hint - axis * hint.dot(axis)
        if u.length > 1e-6:
            u.normalize()
            return u, axis.cross(u)
    ref = Vector((1, 0, 0)) if abs(axis.x) < 0.9 else Vector((0, 1, 0))
    u = axis.cross(ref).normalized()
    return u, axis.cross(u)


def arc(c, u, v, r, a0=0.0, a1=2 * math.pi, seg=24):
    closed = abs((a1 - a0) - 2 * math.pi) < 1e-6
    count = seg if closed else seg + 1
    pts = [c + (u * math.cos(a0 + (a1 - a0) * i / seg) + v * math.sin(a0 + (a1 - a0) * i / seg)) * r
           for i in range(count)]
    return (pts, closed)


def ring(c, axis, r, seg=24, hint=None):
    u, v = _basis(axis, hint)
    return arc(c, u, v, r, seg=seg)


def ellipse(c, u, v, ru, rv, seg=32):
    return ([c + u * ru * math.cos(2 * math.pi * i / seg) + v * rv * math.sin(2 * math.pi * i / seg)
             for i in range(seg)], True)


def box(c, x, y, z, hx, hy, hz):
    def corner(sx, sy, sz):
        return c + x * (hx * sx) + y * (hy * sy) + z * (hz * sz)
    loops = []
    for sz in (-1, 1):
        loops.append(([corner(-1, -1, sz), corner(1, -1, sz), corner(1, 1, sz), corner(-1, 1, sz)], True))
    for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        loops.append(([corner(sx, sy, -1), corner(sx, sy, 1)], False))
    return loops


def diamond(c, r, frame):
    loops = []
    for u, v in ((frame.left, frame.up), (frame.left, frame.fwd), (frame.fwd, frame.up)):
        loops.append(arc(c, u, v, r, seg=4))
    return loops


# ---------------------------------------------------------------------------
# Rig builder (edit-mode part)

class Builder:
    def __init__(self, arm, frame, points):
        self.arm = arm
        self.eb = arm.data.edit_bones
        self.F = frame
        self.points = points
        self.meta = {}       # bone -> collection / color / rotation mode / locks
        self.widgets = {}    # bone -> list of (points, closed)
        self.follow = []     # (deform bone, TGT bone)
        self.blend = []      # (MCH blend bone, FK source, IK source, switch property)
        self.ik = []         # (owner, target, pole)
        self.damped = []     # (owner, target)
        self.twist = []      # (owner, twist source, source space, factor)
        self.spaces = []     # (MCH parent bone, socket bone, constraint type, switch property)

    def H(self, name):
        return self.eb[name].head.copy()

    def bone(self, name, head, tail, parent=None, roll_to=None, coll="mch", color=None,
             connect=False, rot='QUATERNION', lock_loc=False):
        assert name not in self.eb, name
        e = self.eb.new(name)
        e.head = head
        e.tail = tail
        if roll_to is not None:
            e.align_roll(roll_to)
        if parent is not None:
            e.parent = self.eb[parent]
            e.use_connect = connect and (self.eb[parent].tail - e.head).length < 1e-4
        e.use_deform = False
        self.meta[name] = {"coll": coll, "color": color, "rot": rot, "lock_loc": lock_loc}
        return e

    def tgt(self, deform, parent):
        src = self.eb[deform]
        name = "TGT_" + deform
        e = self.eb.new(name)
        e.head = src.head.copy()
        e.tail = src.tail.copy()
        e.roll = src.roll
        e.parent = self.eb[parent]
        e.use_deform = False
        self.meta[name] = {"coll": "mch", "color": None, "rot": 'QUATERNION', "lock_loc": False}
        self.follow.append((deform, name))

    def space_switch(self, ctrl, name, socket_parent, prop, kind='COPY_ROTATION'):
        """Re-parent ctrl under an MCH parent bone that blends toward a second space.

        name is a pattern with one %s ("parent" / "socket"). Both bones copy ctrl's
        rest matrix: the parent stays under ctrl's original parent, the socket goes
        under socket_parent, and a constraint driven by prop pulls the parent onto
        the socket. At rest both coincide, so the switch never moves the rest pose.
        """
        e = self.eb[ctrl]
        for suffix, parent in (("parent", e.parent.name), ("socket", socket_parent)):
            b = self.eb.new(name % suffix)
            b.head, b.tail, b.roll = e.head.copy(), e.tail.copy(), e.roll
            b.parent = self.eb[parent]
            b.use_deform = False
            self.meta[b.name] = {"coll": "mch", "color": None, "rot": 'QUATERNION', "lock_loc": False}
        e.use_connect = False
        e.parent = self.eb[name % "parent"]
        self.spaces.append((name % "parent", name % "socket", kind, prop))

    # Widget sizing from the skinned mesh ------------------------------------

    def ring_radius(self, groups, c, axis, band, scale=1.3, fallback=0.05):
        axis = axis.normalized()
        dists = []
        for g in groups:
            for p in self.points.get(g, ()):
                d = p - c
                h = d.dot(axis)
                if abs(h) <= band:
                    dists.append((d - axis * h).length)
        if len(dists) < 8:
            return fallback * self.F.m
        dists.sort()
        return dists[int(len(dists) * 0.9)] * scale

    def body_ellipse(self, groups, c, band, su=1.2, sv=1.3, fallback=0.15):
        F = self.F
        pts = [p for g in groups for p in self.points.get(g, ()) if abs((p - c).dot(F.up)) <= band]
        if len(pts) < 8:
            return ellipse(c, F.left, F.fwd, fallback * F.m, fallback * F.m * 0.7)
        ls = sorted((p - c).dot(F.left) for p in pts)
        fs = sorted((p - c).dot(F.fwd) for p in pts)
        lo_l, hi_l = ls[int(len(ls) * 0.03)], ls[int(len(ls) * 0.97)]
        lo_f, hi_f = fs[int(len(fs) * 0.03)], fs[int(len(fs) * 0.97)]
        center = c + F.left * ((lo_l + hi_l) / 2) + F.fwd * ((lo_f + hi_f) / 2)
        return ellipse(center, F.left, F.fwd, (hi_l - lo_l) / 2 * su, (hi_f - lo_f) / 2 * sv)

    def limb_ring(self, name, groups, head, tail, t=0.5, band=0.3, scale=1.3, fallback=0.05, hint=None):
        axis = tail - head
        c = head.lerp(tail, t)
        r = self.ring_radius(groups, c, axis, axis.length * band, scale, fallback)
        self.widgets[name] = [ring(c, axis, r, hint=hint)]

    # Body regions -------------------------------------------------------------

    def build_torso(self):
        F, H, m = self.F, self.H, self.F.m

        root = H("root")
        self.bone("CTRL_root", root, root + F.fwd * 0.35 * m, roll_to=F.up, coll="torso", color="c")
        tip = root + F.fwd * 0.62 * m
        self.widgets["CTRL_root"] = [
            ring(root, F.up, 0.45 * m, seg=48, hint=F.left),
            ([root + F.fwd * 0.47 * m + F.left * 0.07 * m, tip, root + F.fwd * 0.47 * m - F.left * 0.07 * m], True),
        ]
        self.tgt("root", "CTRL_root")

        pel = H("pelvis")
        self.bone("CTRL_torso", pel, pel + F.up * 0.18 * m, parent="CTRL_root", roll_to=F.fwd,
                  coll="torso", color="c")
        self.widgets["CTRL_torso"] = [
            self.body_ellipse(("pelvis", "spine_01", "spine_02"), pel + F.up * 0.03 * m, 0.05 * m, 1.45, 1.6)]

        self.bone("CTRL_hips", pel, pel - F.up * 0.15 * m, parent="CTRL_torso", roll_to=F.fwd,
                  coll="torso", color="c", lock_loc=True)
        self.widgets["CTRL_hips"] = [
            self.body_ellipse(("pelvis", "thigh_l", "thigh_r"), pel - F.up * 0.06 * m, 0.04 * m, 1.2, 1.3)]
        self.tgt("pelvis", "CTRL_hips")

        chain = ("spine_01", "spine_02", "spine_03", "spine_04", "spine_05", "neck_01", "neck_02", "head")
        parent = "CTRL_torso"
        for name, nxt in zip(chain[:-1], chain[1:]):
            ctrl = "CTRL_" + name
            h, t = H(name), H(nxt)
            self.bone(ctrl, h, t, parent=parent, roll_to=F.fwd, coll="spine", color="c", lock_loc=True)
            if name.startswith("spine"):
                self.widgets[ctrl] = [self.body_ellipse((name,), h.lerp(t, 0.5), 0.035 * m, 1.15, 1.25)]
            else:
                self.widgets[ctrl] = [self.body_ellipse(("neck_01", "neck_02"), h.lerp(t, 0.5), 0.02 * m, 1.35, 1.35)]
            self.tgt(name, ctrl)
            parent = ctrl

        head = H("head")
        self.bone("CTRL_head", head, head + F.up * 0.18 * m, parent="CTRL_neck_02", roll_to=F.fwd,
                  coll="spine", color="c", lock_loc=True)
        self.widgets["CTRL_head"] = [self.body_ellipse(("face",), head + F.up * 0.06 * m, 0.02 * m, 1.3, 1.3)]
        self.space_switch("CTRL_head", "MCH_head_%s", "CTRL_torso", "head_follow")
        self.tgt("head", "CTRL_head")

    def build_arm(self, s):
        F, H, m = self.F, self.H, self.F.m
        outward = F.left if s == "l" else -F.left

        # Clavicle and breast.
        clav = "CTRL_clavicle_" + s
        a, b = H("clavicle_" + s), H("upperarm_" + s)
        self.bone(clav, a, b, parent="CTRL_spine_05", roll_to=F.up, coll="spine", color=s, lock_loc=True)
        self.widgets[clav] = [([a.lerp(b, 0.15 + 0.85 * i / 10) + F.up * (0.05 + 0.015 * math.sin(math.pi * i / 10)) * m
                                for i in range(11)], False)]
        self.tgt("clavicle_" + s, clav)

        breast = "CTRL_breast_" + s
        bh = H("breast_" + s)
        self.bone(breast, bh, bh + F.fwd * 0.08 * m, parent="CTRL_spine_05", roll_to=F.up, coll="spine", color=s)
        front = max([(p - bh).dot(F.fwd) for p in self.points.get("breast_" + s, ())] or [0.0])
        self.widgets[breast] = [arc(bh + F.fwd * (front + 0.015 * m), F.left, F.up, 0.045 * m)]
        self.tgt("breast_" + s, breast)

        # Arm chain geometry: shoulder, elbow, wrist, middle knuckle.
        S, E, W = H("upperarm_" + s), H("lowerarm_" + s), H("hand_" + s)
        M = H("middle_01_" + s)
        pole = bend_dir(S, E, W) or -F.fwd
        dorsal = (H("index_01_" + s) - W).cross(H("pinky_01_" + s) - W).normalized()
        if dorsal.dot(outward) < 0:
            dorsal = -dorsal
        geo = {"upperarm": (S, E, pole), "lowerarm": (E, W, pole), "hand": (W, M, dorsal)}
        rings = {
            "upperarm": ("upperarm_" + s, "upperarm_twist_01_" + s, "upperarm_twist_02_" + s),
            "lowerarm": ("lowerarm_" + s, "lowerarm_twist_01_" + s, "lowerarm_twist_02_" + s),
            "hand": ("hand_" + s,),
        }

        # FK.
        parent = clav
        for part in ("upperarm", "lowerarm", "hand"):
            h, t, r = geo[part]
            ctrl = "CTRL_%s_fk_%s" % (part, s)
            self.bone(ctrl, h, t, parent=parent, roll_to=r, coll="arm_fk_" + s, color=s,
                      connect=part != "upperarm", lock_loc=True)
            self.limb_ring(ctrl, rings[part], h, t, t=0.35 if part == "hand" else 0.5, scale=1.35)
            parent = ctrl
        self.space_switch("CTRL_upperarm_fk_" + s, "MCH_upperarm_fk_%s_" + s, "CTRL_torso", "arm_fk_follow_" + s)

        # IK.
        self.bone("MCH_upperarm_ik_" + s, S, E, parent=clav, roll_to=pole)
        self.bone("MCH_lowerarm_ik_" + s, E, W, parent="MCH_upperarm_ik_" + s, roll_to=pole, connect=True)
        hand_ik = "CTRL_hand_ik_" + s
        self.bone(hand_ik, W, M, parent="CTRL_root", roll_to=dorsal, coll="arm_ik_" + s, color=s)
        y = (M - W).normalized()
        z = perp(dorsal, y)
        self.widgets[hand_ik] = box(W.lerp(M, 0.45), y.cross(z), y, z, 0.05 * m, (M - W).length * 0.8, 0.03 * m)
        elbow = "CTRL_elbow_pole_" + s
        p = E + pole * 0.35 * m
        self.bone(elbow, p, p + pole * 0.05 * m, parent="CTRL_root", roll_to=F.up, coll="arm_ik_" + s, color=s)
        self.widgets[elbow] = diamond(p, 0.03 * m, F)
        self.space_switch(elbow, "MCH_elbow_pole_%s_" + s, hand_ik, "pole_follow_arm_" + s, 'COPY_TRANSFORMS')
        self.ik.append(("MCH_lowerarm_ik_" + s, hand_ik, elbow))

        # IK/FK blend chain, which the UE bones follow.
        sources = {
            "upperarm": ("CTRL_upperarm_fk_" + s, "MCH_upperarm_ik_" + s),
            "lowerarm": ("CTRL_lowerarm_fk_" + s, "MCH_lowerarm_ik_" + s),
            "hand": ("CTRL_hand_fk_" + s, hand_ik),
        }
        parent = clav
        for part in ("upperarm", "lowerarm", "hand"):
            h, t, r = geo[part]
            mch = "MCH_%s_%s" % (part, s)
            self.bone(mch, h, t, parent=parent, roll_to=r, connect=part != "upperarm")
            self.blend.append((mch, sources[part][0], sources[part][1], "ik_arm_" + s))
            self.tgt(part + "_" + s, mch)
            parent = mch

        # Twist: upper arm counter-twists against the shoulder, forearm follows the hand.
        notwist = "MCH_upperarm_notwist_" + s
        self.bone(notwist, S, E, parent=clav, roll_to=pole)
        self.damped.append((notwist, "MCH_lowerarm_" + s))
        axis = (E - S).normalized()
        for twist, factor in (("upperarm_twist_01_" + s, -0.66), ("upperarm_twist_02_" + s, -0.33)):
            h = H(twist)
            self.bone("MCH_" + twist, h, h + axis * 0.04 * m, parent="MCH_upperarm_" + s, roll_to=pole)
            self.twist.append(("MCH_" + twist, "MCH_upperarm_" + s, notwist, factor))
            self.tgt(twist, "MCH_" + twist)

        wrist = "MCH_wrist_twist_" + s
        axis = (W - E).normalized()
        self.bone(wrist, W, W + axis * 0.04 * m, parent="MCH_hand_" + s, roll_to=pole)
        for twist, factor in (("lowerarm_twist_01_" + s, 0.66), ("lowerarm_twist_02_" + s, 0.33)):
            h = H(twist)
            self.bone("MCH_" + twist, h, h + axis * 0.04 * m, parent="MCH_lowerarm_" + s, roll_to=pole)
            self.twist.append(("MCH_" + twist, wrist, "MCH_lowerarm_" + s, factor))
            self.tgt(twist, "MCH_" + twist)

        self.tgt("ik_hand_" + s, "MCH_hand_" + s)
        if s == "r":
            self.tgt("ik_hand_gun", "MCH_hand_r")

        self.build_fingers(s, dorsal)

    def build_fingers(self, s, dorsal):
        m = self.F.m
        for chain in FINGER_CHAINS:
            names = [n + "_" + s for n in chain]
            heads = [self.H(n) for n in names]
            tails = heads[1:] + [heads[-1] + (heads[-1] - heads[-2]) * 0.8]
            joints = heads + [tails[-1]]
            # Hinge axis from the curl of the chain; falls back to the back of the hand.
            hinge = Vector()
            for a, b, c in zip(joints, joints[1:], joints[2:]):
                hinge += (b - a).cross(c - b)
            parent = "MCH_hand_" + s
            for i, (name, h, t) in enumerate(zip(names, heads, tails)):
                y = (t - h).normalized()
                z = hinge.normalized().cross(y) if hinge.length > 1e-6 else perp(dorsal, y)
                if z.length < 1e-6:
                    z = perp(dorsal, y)
                if z.dot(dorsal) < 0:
                    z = -z
                ctrl = "CTRL_" + name
                self.bone(ctrl, h, t, parent=parent, roll_to=z, coll="fingers_" + s, color=s,
                          connect=i > 0, rot='XYZ', lock_loc=True)
                if "metacarpal" in name:
                    self.widgets[ctrl] = [ring(h.lerp(t, 0.5) + perp(z, y) * 0.012 * m, z, 0.005 * m, seg=10)]
                else:
                    r = 0.011 if name.startswith("thumb") else 0.009
                    self.widgets[ctrl] = [ring(h.lerp(t, 0.5), y, r * m, seg=12, hint=z)]
                self.tgt(name, ctrl)
                parent = ctrl

    def build_leg(self, s):
        F, H, m = self.F, self.H, self.F.m
        Hp, K, A, B = H("thigh_" + s), H("calf_" + s), H("foot_" + s), H("ball_" + s)

        # Foot frame and contact points from the skinned mesh.
        foot_pts = list(self.points.get("foot_" + s, ())) + list(self.points.get("ball_" + s, ()))
        fdir = perp(B - A, F.up)
        fside = F.up.cross(fdir).normalized()
        back = max((A - p).dot(fdir) for p in foot_pts)
        front = max((p - B).dot(fdir) for p in foot_pts)
        lat = sorted((p - A).dot(fside) for p in foot_pts)
        gz = min(F.world_z(p) for p in foot_pts)
        heel = F.ground(A - fdir * back, gz)
        toe = F.ground(B + fdir * front, gz)
        toe_tip = B + fdir * front

        knee = bend_dir(Hp, K, A)
        if knee is None or knee.dot(fdir) < 0.3:
            knee = perp(fdir, A - Hp)
        geo = {
            "thigh": (Hp, K, knee),
            "calf": (K, A, knee),
            "foot": (A, B, F.up),
            "toe": (B, toe_tip, F.up),
        }
        rings = {
            "thigh": ("thigh_" + s, "thigh_twist_01_" + s, "thigh_twist_02_" + s),
            "calf": ("calf_" + s, "calf_twist_01_" + s, "calf_twist_02_" + s),
            "foot": ("foot_" + s,),
            "toe": ("ball_" + s,),
        }

        # FK.
        parent = "CTRL_hips"
        for part in ("thigh", "calf", "foot", "toe"):
            h, t, r = geo[part]
            ctrl = "CTRL_%s_fk_%s" % (part, s)
            self.bone(ctrl, h, t, parent=parent, roll_to=r, coll="leg_fk_" + s, color=s,
                      connect=part != "thigh", lock_loc=True, rot='XYZ' if part == "toe" else 'QUATERNION')
            self.limb_ring(ctrl, rings[part], h, t, t=0.3 if part == "foot" else 0.5, scale=1.3,
                           hint=F.up if part in ("foot", "toe") else None)
            parent = ctrl

        # IK chain.
        self.bone("MCH_thigh_ik_" + s, Hp, K, parent="CTRL_hips", roll_to=knee)
        self.bone("MCH_calf_ik_" + s, K, A, parent="MCH_thigh_ik_" + s, roll_to=knee, connect=True)

        # Foot control with a reverse foot: heel -> toe tip -> ball roll -> ankle.
        # The three pivots point backward, so +X rolls the foot forward and -X rocks it back.
        foot_ik = "CTRL_foot_ik_" + s
        coll = "leg_ik_" + s
        self.bone(foot_ik, A, A + fdir * 0.12 * m, parent="CTRL_root", roll_to=F.up, coll=coll, color=s)
        pad = 0.012 * m
        lift = F.up * 0.002 * m
        corners = [
            F.ground(A + fdir * (-back - pad) + fside * (lat[0] - pad), gz) + lift,
            F.ground(A + fdir * (-back - pad) + fside * (lat[-1] + pad), gz) + lift,
            F.ground(B + fdir * (front + pad) + fside * (lat[-1] + pad), gz) + lift,
            F.ground(B + fdir * (front + pad) + fside * (lat[0] - pad), gz) + lift,
        ]
        self.widgets[foot_ik] = [(corners, True)]

        heel_c = "CTRL_heel_" + s
        self.bone(heel_c, heel, heel - fdir * 0.05 * m, parent=foot_ik, roll_to=F.up, coll=coll, color=s,
                  rot='XYZ', lock_loc=True)
        self.widgets[heel_c] = [arc(heel + F.up * 0.03 * m, fdir, F.up, 0.03 * m, math.pi / 2, 3 * math.pi / 2, seg=12)]

        tip_c = "CTRL_toe_tip_" + s
        self.bone(tip_c, toe, toe - fdir * 0.04 * m, parent=heel_c, roll_to=F.up, coll=coll, color=s,
                  rot='XYZ', lock_loc=True)
        self.widgets[tip_c] = [arc(toe + F.up * 0.025 * m, fdir, F.up, 0.025 * m, -math.pi / 2, math.pi / 2, seg=12)]

        ball_c = "CTRL_ball_roll_" + s
        self.bone(ball_c, B, B - fdir * 0.06 * m, parent=tip_c, roll_to=F.up, coll=coll, color=s,
                  rot='XYZ', lock_loc=True)
        self.widgets[ball_c] = [arc(B, fdir, F.up, 0.05 * m, 0.0, math.pi, seg=16)]

        ankle = "MCH_ankle_ik_" + s
        self.bone(ankle, A, A + F.up * 0.05 * m, parent=ball_c)
        self.bone("MCH_foot_ik_" + s, A, B, parent=ball_c, roll_to=F.up)

        toe_ik = "CTRL_toe_ik_" + s
        self.bone(toe_ik, B, toe_tip, parent=tip_c, roll_to=F.up, coll=coll, color=s, rot='XYZ', lock_loc=True)
        self.limb_ring(toe_ik, rings["toe"], B, toe_tip, scale=1.4, hint=F.up)

        knee_c = "CTRL_knee_pole_" + s
        p = K + knee * 0.5 * m
        self.bone(knee_c, p, p + knee * 0.05 * m, parent="CTRL_root", roll_to=F.up, coll=coll, color=s)
        self.widgets[knee_c] = diamond(p, 0.035 * m, F)
        self.space_switch(knee_c, "MCH_knee_pole_%s_" + s, foot_ik, "pole_follow_leg_" + s, 'COPY_TRANSFORMS')
        self.ik.append(("MCH_calf_ik_" + s, ankle, knee_c))

        # IK/FK blend chain, which the UE bones follow.
        sources = {
            "thigh": ("CTRL_thigh_fk_" + s, "MCH_thigh_ik_" + s),
            "calf": ("CTRL_calf_fk_" + s, "MCH_calf_ik_" + s),
            "foot": ("CTRL_foot_fk_" + s, "MCH_foot_ik_" + s),
            "toe": ("CTRL_toe_fk_" + s, toe_ik),
        }
        deform = {"thigh": "thigh_", "calf": "calf_", "foot": "foot_", "toe": "ball_"}
        parent = "CTRL_hips"
        for part in ("thigh", "calf", "foot", "toe"):
            h, t, r = geo[part]
            mch = "MCH_%s_%s" % (part, s)
            self.bone(mch, h, t, parent=parent, roll_to=r, connect=part != "thigh")
            self.blend.append((mch, sources[part][0], sources[part][1], "ik_leg_" + s))
            self.tgt(deform[part] + s, mch)
            parent = mch

        # Twist: thigh counter-twists against the hips, shin follows the foot.
        notwist = "MCH_thigh_notwist_" + s
        self.bone(notwist, Hp, K, parent="CTRL_hips", roll_to=knee)
        self.damped.append((notwist, "MCH_calf_" + s))
        axis = (K - Hp).normalized()
        for twist, factor in (("thigh_twist_01_" + s, -0.66), ("thigh_twist_02_" + s, -0.33)):
            h = H(twist)
            self.bone("MCH_" + twist, h, h + axis * 0.04 * m, parent="MCH_thigh_" + s, roll_to=knee)
            self.twist.append(("MCH_" + twist, "MCH_thigh_" + s, notwist, factor))
            self.tgt(twist, "MCH_" + twist)

        ankle_tw = "MCH_ankle_twist_" + s
        axis = (A - K).normalized()
        self.bone(ankle_tw, A, A + axis * 0.04 * m, parent="MCH_foot_" + s, roll_to=knee)
        for twist, factor in (("calf_twist_01_" + s, 0.66), ("calf_twist_02_" + s, 0.33)):
            h = H(twist)
            self.bone("MCH_" + twist, h, h + axis * 0.04 * m, parent="MCH_calf_" + s, roll_to=knee)
            self.twist.append(("MCH_" + twist, ankle_tw, "MCH_calf_" + s, factor))
            self.tgt(twist, "MCH_" + twist)

        self.tgt("ik_foot_" + s, "MCH_foot_" + s)

    def build_face(self):
        F, H, m = self.F, self.H, self.F.m

        j1, j2 = H("jaw_01"), H("jaw_02")
        jd = (j2 - j1).normalized()
        self.bone("CTRL_jaw", j1, j2 + jd * 0.02 * m, parent="CTRL_head", roll_to=F.up, coll="face",
                  color="face", rot='XYZ')
        self.widgets["CTRL_jaw"] = [arc(j2 - F.up * 0.01 * m, F.left, F.fwd, 0.035 * m, 0.0, math.pi, seg=16)]
        self.tgt("jaw_01", "CTRL_jaw")

        tongue = ("tongue_1", "tongue_2", "tongue_3", "tongue_4")
        parent = "CTRL_jaw"
        for i, name in enumerate(tongue):
            h = H(name)
            t = H(tongue[i + 1]) if i + 1 < len(tongue) else h + (h - H(tongue[i - 1]))
            ctrl = "CTRL_" + name
            self.bone(ctrl, h, t, parent=parent, roll_to=F.up, coll="face", color="face", rot='XYZ', lock_loc=True)
            self.widgets[ctrl] = [ring(h.lerp(t, 0.5), t - h, 0.006 * m, seg=10, hint=F.up)]
            self.tgt(name, ctrl)
            parent = ctrl

        el, er = H("eye_l"), H("eye_r")
        dist = 0.35 * m
        mid = el.lerp(er, 0.5) + F.fwd * dist
        self.bone("CTRL_eyes", mid, mid + F.fwd * 0.04 * m, parent="CTRL_head", roll_to=F.up, coll="face",
                  color="face")
        self.widgets["CTRL_eyes"] = [ellipse(mid, F.left, F.up, (el - er).length / 2 + 0.03 * m, 0.022 * m)]
        for s, eye in (("l", el), ("r", er)):
            target = "CTRL_eye_target_" + s
            p = eye + F.fwd * dist
            self.bone(target, p, p + F.fwd * 0.03 * m, parent="CTRL_eyes", roll_to=F.up, coll="face", color=s)
            self.widgets[target] = [arc(p, F.left, F.up, 0.012 * m, seg=16)]

            aim = "MCH_eye_" + s
            self.bone(aim, eye, eye + F.fwd * 0.03 * m, parent="CTRL_head", roll_to=F.up)
            self.damped.append((aim, target))
            self.tgt("eye_" + s, aim)

            lid = "CTRL_eyelid_" + s
            self.bone(lid, eye, eye + F.fwd * 0.03 * m, parent="CTRL_head", roll_to=F.up, coll="face",
                      color="face", rot='XYZ', lock_loc=True)
            self.widgets[lid] = [arc(eye + F.fwd * 0.022 * m, F.left, F.up, 0.017 * m, 0.0, math.pi, seg=12)]
            self.tgt("eye_%s_lid" % s, lid)

            parent = "CTRL_head"
            for k in (1, 2, 3):
                name = "eyebrow_%d_%s" % (k, s)
                h = H(name)
                ctrl = "CTRL_" + name
                self.bone(ctrl, h, h + F.fwd * 0.012 * m, parent=parent, roll_to=F.up, coll="face", color=s,
                          rot='XYZ')
                self.widgets[ctrl] = [arc(h + F.fwd * 0.008 * m, F.left, F.up, 0.005 * m, seg=10)]
                self.tgt(name, ctrl)
                parent = ctrl


# ---------------------------------------------------------------------------
# Pose-level setup

def add_constraint(pb, kind, name, **props):
    con = pb.constraints.new(kind)
    con.name = CON_PREFIX + name
    for key, value in props.items():
        setattr(con, key, value)
    return con


def drive_influence(arm, con, prop):
    fc = con.driver_add("influence")
    for mod in list(fc.modifiers):
        fc.modifiers.remove(mod)
    driver = fc.driver
    driver.type = 'AVERAGE'
    for var in list(driver.variables):
        driver.variables.remove(var)
    var = driver.variables.new()
    var.name = "switch"
    var.type = 'SINGLE_PROP'
    var.targets[0].id_type = 'OBJECT'
    var.targets[0].id = arm
    var.targets[0].data_path = '["%s"]' % prop


def setup_pose(arm, builder):
    pose = arm.pose.bones
    for deform, tgt in builder.follow:
        add_constraint(pose[deform], 'COPY_TRANSFORMS', "follow", target=arm, subtarget=tgt)
    for mch, fk, ik, prop in builder.blend:
        add_constraint(pose[mch], 'COPY_TRANSFORMS', "fk", target=arm, subtarget=fk)
        con = add_constraint(pose[mch], 'COPY_TRANSFORMS', "ik", target=arm, subtarget=ik)
        drive_influence(arm, con, prop)
    # Space switches: the constraint copies world-space rotation (or the full
    # transform, for poles) from the socket, so the influence blends between parents.
    for owner, socket, kind, prop in builder.spaces:
        con = add_constraint(pose[owner], kind, "space", target=arm, subtarget=socket)
        drive_influence(arm, con, prop)
    for owner, target in builder.damped:
        add_constraint(pose[owner], 'DAMPED_TRACK', "track", target=arm, subtarget=target, track_axis='TRACK_Y')
    for owner, target, pole in builder.ik:
        add_constraint(pose[owner], 'IK', "ik", target=arm, subtarget=target, pole_target=arm,
                       pole_subtarget=pole, chain_count=2, use_tail=True, use_stretch=False)
    for owner, source, space, factor in builder.twist:
        add_constraint(
            pose[owner], 'TRANSFORM', "twist", target=arm, subtarget=source,
            target_space='CUSTOM', space_object=arm, space_subtarget=space, owner_space='LOCAL',
            map_from='ROTATION', map_to='ROTATION', from_rotation_mode='SWING_TWIST_Y',
            map_to_x_from='Y', map_to_y_from='Y', map_to_z_from='Y', mix_mode_rot='REPLACE',
            from_min_y_rot=-math.pi, from_max_y_rot=math.pi,
            to_min_y_rot=-math.pi * factor, to_max_y_rot=math.pi * factor,
        )


def setup_collections(arm, builder):
    colls = {}
    for key, name in COLLECTIONS.items():
        coll = arm.data.collections.new(name)
        coll.is_visible = key not in HIDDEN_COLLECTIONS
        colls[key] = coll
    for bone in arm.data.bones:
        if not bone.name.startswith(RIG_PREFIXES):
            colls["def"].assign(bone)
    for name, meta in builder.meta.items():
        bone = arm.data.bones[name]
        colls[meta["coll"]].assign(bone)
        if meta["color"]:
            bone.color.palette = COLORS[meta["color"]]
        pb = arm.pose.bones[name]
        pb.rotation_mode = meta["rot"]
        pb.lock_location = (meta["lock_loc"],) * 3


def setup_widgets(arm, builder):
    # Widget objects are kept out of every scene (the custom shape reference keeps
    # them saved), so exporters never pick them up.
    for name, loops in builder.widgets.items():
        bone = arm.data.bones[name]
        to_bone = bone.matrix_local.inverted()
        scale = 1.0 / bone.length
        verts, edges = [], []
        for pts, closed in loops:
            start = len(verts)
            verts.extend((to_bone @ p) * scale for p in pts)
            edges.extend((start + i, start + i + 1) for i in range(len(pts) - 1))
            if closed and len(pts) > 2:
                edges.append((start + len(pts) - 1, start))
        mesh = bpy.data.meshes.new("WGT_" + name)
        mesh.from_pydata(verts, edges, [])
        ob = bpy.data.objects.new("WGT_" + name, mesh)
        pb = arm.pose.bones[name]
        pb.custom_shape = ob
        if hasattr(pb, "custom_shape_wire_width"):
            pb.custom_shape_wire_width = 2.0


def setup_switches(arm):
    for key, (default, description) in SWITCHES.items():
        if key not in arm:
            arm[key] = default
        arm.id_properties_ui(key).update(min=0.0, max=1.0, soft_min=0.0, soft_max=1.0,
                                         description=description)
    arm["basefemale_rig"] = 1


def _chain_error(pbs):
    bpy.context.view_layer.update()
    return sum(pb.matrix.to_quaternion().rotation_difference(pb.bone.matrix_local.to_quaternion()).angle
               for pb in pbs)


def calibrate_poles(arm, builder):
    """Pick the pole angle that keeps each IK chain in its rest pose."""
    angles = {}
    for owner, _target, _pole in builder.ik:
        pb = arm.pose.bones[owner]
        con = pb.constraints[CON_PREFIX + "ik"]
        chain = (pb, pb.parent)
        best = None
        for deg in (0.0, 90.0, -90.0, 180.0):
            con.pole_angle = math.radians(deg)
            err = _chain_error(chain)
            if best is None or err < best[1]:
                best = (deg, err)
        deg0 = best[0]
        if best[1] > 1e-3:
            for i in range(-40, 41):
                deg = deg0 + i * 0.25
                con.pole_angle = math.radians(deg)
                err = _chain_error(chain)
                if err < best[1]:
                    best = (deg, err)
        con.pole_angle = math.radians(best[0])
        angles[owner] = (best[0], round(math.degrees(best[1]), 4))
    return angles


def rest_error(arm):
    bpy.context.view_layer.update()
    worst_rot, worst_loc = ("", 0.0), ("", 0.0)
    for pb in arm.pose.bones:
        if pb.name.startswith(RIG_PREFIXES) or not pb.constraints:
            continue
        rest = pb.bone.matrix_local
        rot = math.degrees(pb.matrix.to_quaternion().rotation_difference(rest.to_quaternion()).angle)
        loc = (pb.matrix.translation - rest.translation).length
        if rot > worst_rot[1]:
            worst_rot = (pb.name, round(rot, 4))
        if loc > worst_loc[1]:
            worst_loc = (pb.name, round(loc, 4))
    return {"max_rot_deg": worst_rot, "max_loc_units": worst_loc}


UI_CODE = '''import bpy

# FK controls and the IK bones they line up with, in chain order. The third IK
# bone is the end of the chain the IK control carries rigidly (the hand control
# itself, or the reverse foot's MCH_foot_ik); any later IK bones are controls
# that copy their FK partner directly.
LIMBS = {
    "arm": {
        "fk": ("CTRL_upperarm_fk_%s", "CTRL_lowerarm_fk_%s", "CTRL_hand_fk_%s"),
        "ik": ("MCH_upperarm_ik_%s", "MCH_lowerarm_ik_%s", "CTRL_hand_ik_%s"),
        "control": "CTRL_hand_ik_%s",
        "pivots": (),
        "pole": "CTRL_elbow_pole_%s",
        "switch": "ik_arm_%s",
    },
    "leg": {
        "fk": ("CTRL_thigh_fk_%s", "CTRL_calf_fk_%s", "CTRL_foot_fk_%s", "CTRL_toe_fk_%s"),
        "ik": ("MCH_thigh_ik_%s", "MCH_calf_ik_%s", "MCH_foot_ik_%s", "CTRL_toe_ik_%s"),
        "control": "CTRL_foot_ik_%s",
        "pivots": ("CTRL_heel_%s", "CTRL_toe_tip_%s", "CTRL_ball_roll_%s"),
        "pole": "CTRL_knee_pole_%s",
        "switch": "ik_leg_%s",
    },
}

SPACES = (
    ("head_follow", "Head", "Neck / Torso"),
    ("arm_fk_follow_l", "FK Arm L", "Clavicle / Torso"),
    ("arm_fk_follow_r", "FK Arm R", "Clavicle / Torso"),
    ("pole_follow_arm_l", "Elbow Pole L", "Root / Hand"),
    ("pole_follow_arm_r", "Elbow Pole R", "Root / Hand"),
    ("pole_follow_leg_l", "Knee Pole L", "Root / Foot"),
    ("pole_follow_leg_r", "Knee Pole R", "Root / Foot"),
)


def _rot_path(pb):
    return {"QUATERNION": "rotation_quaternion", "AXIS_ANGLE": "rotation_axis_angle"}.get(
        pb.rotation_mode, "rotation_euler")


def _place(context, pb, matrix, keyed):
    """Give pb the armature-space matrix, honouring fully locked location."""
    pb.matrix = matrix
    if all(pb.lock_location):
        pb.location = (0.0, 0.0, 0.0)
    context.view_layer.update()
    keyed.append(pb)


def _place_pole(context, pole, upper, lower, keyed):
    """Put the pole control in the bend plane of upper/lower, at its rest distance."""
    s, e, w = upper.head, lower.head, lower.tail
    axis = w - s
    d = (e - s) - axis * ((e - s).dot(axis) / axis.length_squared)
    if d.length < 1e-4 * upper.length:
        # Straight limb: fall back to the bone's Z axis, which was rolled toward the pole.
        d = upper.matrix.col[2].xyz
    rest = (pole.bone.head_local - lower.bone.head_local).length
    m = pole.matrix.copy()
    m.translation = e + d.normalized() * rest
    _place(context, pole, m, keyed)


def snap(context, ob, limb, side, to_fk):
    """Match one chain to the other. Returns the pose bones that changed."""
    spec = LIMBS[limb]
    pose = ob.pose.bones
    fk = [pose[n % side] for n in spec["fk"]]
    ik = [pose[n % side] for n in spec["ik"]]
    keyed = []
    context.view_layer.update()
    if to_fk:
        # The FK and IK chains never depend on each other, so read every target first.
        targets = [pb.matrix.copy() for pb in ik]
        for pb, mat in zip(fk, targets):
            _place(context, pb, mat, keyed)
        return keyed

    for name in spec["pivots"]:
        pb = pose[name % side]
        pb.location = (0.0, 0.0, 0.0)
        pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
        pb.rotation_euler = (0.0, 0.0, 0.0)
        pb.rotation_axis_angle = (0.0, 0.0, 1.0, 0.0)
        keyed.append(pb)
    context.view_layer.update()
    targets = [pb.matrix.copy() for pb in fk]
    ctrl = pose[spec["control"] % side]
    end = ik[2]
    # The end bone rides rigidly on the control, so solve the control from its rest offset.
    offset = end.bone.matrix_local.inverted() @ ctrl.bone.matrix_local
    _place(context, ctrl, targets[2] @ offset, keyed)
    for pb, mat in zip(ik[3:], targets[3:]):
        _place(context, pb, mat, keyed)
    _place_pole(context, pose[spec["pole"] % side], fk[0], fk[1], keyed)
    return keyed


class BASEFEMALE_OT_snap_ikfk(bpy.types.Operator):
    """Match one chain to the other and flip the IK/FK switch"""
    bl_idname = "basefemale.snap_ikfk"
    bl_label = "Snap IK/FK"
    bl_options = {'REGISTER', 'UNDO'}

    limb: bpy.props.EnumProperty(items=(("arm", "Arm", ""), ("leg", "Leg", "")))
    side: bpy.props.EnumProperty(items=(("l", "Left", ""), ("r", "Right", "")))
    to_fk: bpy.props.BoolProperty(name="To FK", description="Snap FK controls to the IK pose")
    switch: bpy.props.BoolProperty(name="Switch", default=True,
                                   description="Also set the IK/FK switch to the snapped chain")

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.type == 'ARMATURE' and "basefemale_rig" in ob

    @classmethod
    def description(cls, context, props):
        if props.to_fk:
            return "Snap the FK controls to the current IK pose and switch to FK"
        return "Snap the IK control, pole and foot to the current FK pose and switch to IK"

    def execute(self, context):
        ob = context.active_object
        keyed = snap(context, ob, self.limb, self.side, self.to_fk)
        prop = LIMBS[self.limb]["switch"] % self.side
        if self.switch:
            ob[prop] = 0.0 if self.to_fk else 1.0
            # Setting an ID property from Python does not re-run the drivers by itself.
            ob.update_tag()
        if context.scene.tool_settings.use_keyframe_insert_auto:
            for pb in keyed:
                pb.keyframe_insert("location", group=pb.name)
                pb.keyframe_insert(_rot_path(pb), group=pb.name)
            if self.switch:
                ob.keyframe_insert('["%s"]' % prop)
        return {'FINISHED'}


class BASEFEMALE_PT_rig(bpy.types.Panel):
    bl_label = "BaseFemale Rig"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Rig"

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.type == 'ARMATURE' and "basefemale_rig" in ob

    def draw(self, context):
        ob = context.active_object
        layout = self.layout
        col = layout.column(align=True)
        col.label(text="IK / FK  (1 = IK)")
        for limb, side, label in (("arm", "l", "Arm L"), ("arm", "r", "Arm R"),
                                  ("leg", "l", "Leg L"), ("leg", "r", "Leg R")):
            split = col.split(factor=0.25, align=True)
            split.label(text=label)
            row = split.row(align=True)
            row.prop(ob, '["%s"]' % (LIMBS[limb]["switch"] % side), text="", slider=True)
            for to_fk, text in ((True, "FK"), (False, "IK")):
                op = row.operator("basefemale.snap_ikfk", text=text)
                op.limb, op.side, op.to_fk = limb, side, to_fk
        layout.label(text="Parent Spaces  (0 / 1)")
        col = layout.column(align=True)
        for key, label, spaces in SPACES:
            col.prop(ob, '["%s"]' % key, text="%s: %s" % (label, spaces), slider=True)
        layout.label(text="Bone Collections")
        grid = layout.grid_flow(columns=2, align=True)
        for coll in ob.data.collections_all:
            grid.prop(coll, "is_visible", text=coll.name, toggle=True)


for _cls in (BASEFEMALE_OT_snap_ikfk, BASEFEMALE_PT_rig):
    _old = getattr(bpy.types, _cls.__name__, None)
    if _old is not None:
        bpy.utils.unregister_class(_old)
    bpy.utils.register_class(_cls)
'''


def setup_ui():
    text = bpy.data.texts.get(UI_TEXT) or bpy.data.texts.new(UI_TEXT)
    text.from_string(UI_CODE)
    text.use_module = True
    exec(compile(UI_CODE, UI_TEXT, 'exec'), {"__name__": "__main__"})


# ---------------------------------------------------------------------------

def main():
    arm = bpy.data.objects[ARMATURE]
    body = bpy.data.objects[BODY]

    cleanup(arm)

    frame = Frame(arm)
    points = body_points(arm, body)

    set_mode(arm, 'EDIT')
    builder = Builder(arm, frame, points)
    builder.build_torso()
    for s in SIDES:
        builder.build_arm(s)
        builder.build_leg(s)
    builder.build_face()
    set_mode(arm, 'OBJECT')

    setup_switches(arm)
    setup_collections(arm, builder)
    setup_widgets(arm, builder)
    setup_pose(arm, builder)
    poles = calibrate_poles(arm, builder)

    arm.show_in_front = True
    if hasattr(arm.data, "show_bone_colors"):
        arm.data.show_bone_colors = True
    setup_ui()
    set_mode(arm, 'POSE')

    report = {
        "controls": sum(1 for n in builder.meta if n.startswith("CTRL_")),
        "mechanism": sum(1 for n in builder.meta if n.startswith("MCH_")),
        "driven_ue_bones": len(builder.follow),
        "pole_angles": poles,
        "rest_error": rest_error(arm),
    }
    print("BaseFemale control rig:", report)
    return report


result = main()
