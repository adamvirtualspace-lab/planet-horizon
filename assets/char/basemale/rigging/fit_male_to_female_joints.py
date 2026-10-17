# Reshapes the imported male body parts so they fit the BaseFemale joint
# placement (the UE5 Quinn positions), then skins them to the female rig.
#
# Rig_BaseMale.blend starts as a copy of Rig_BaseFemale.blend with the male
# parts imported under an Empty: each part is skinned to its own UE-named
# skeleton ("root", "root.001", "root.002" - identical joint positions).
#
# How the mesh is moved (all in world space, so the Empty's offset and scale
# are baked in):
#   * First a uniform scale about the ground puts the male eye line at the
#     female eye height (the imported male is a few cm taller). This keeps his
#     proportions; without it the whole height difference would end up
#     squeezing the neck.
#   * Every limb and finger bone gets a map that carries its rest frame onto
#     the female bone of the same name. Chain bones are also rotated and
#     stretched along their length so the male child joint lands exactly on
#     the female child joint, which keeps limbs continuous at the joints.
#     Twist bones reuse their parent's map. Vertices blend the maps with the
#     male skin weights.
#   * The spine and neck are matched as one curve instead (pelvis to pelvis,
#     neck base to neck base, joints in between by distance along the back),
#     because the two skeletons space their spine joints very differently.
#   * The head is moved as one rigid piece: the transform that best fits the
#     male face joints (eyes, lids, jaw, brows, tongue) onto the female ones.
#     The face keeps its shape and the neck absorbs the difference, eased
#     evenly from the collarbones up to the jaw line (see NeckZone).
#   * A small smooth correction field then pulls the face onto the exact
#     female face joints; the eyeballs are translated so their centres sit
#     exactly on the female eye joints.
#
# Output (parented to the female "Armature", same object transform and
# armature modifier as the female meshes, so build_control_rig.py works
# unchanged):
#   BodyCombined  body + head skin       Eyes  eyeballs       Eyebrows
# The female meshes are renamed *_Female and, with the male source objects,
# moved to collections excluded from the view layer.
#
# Re-running rebuilds the outputs from the untouched male sources.

import bpy
import numpy as np
from mathutils import Matrix, Vector

ARMATURE = "Armature"            # female rig (UE deform bones + controls)
MALE_SKELETON = "root"           # any of the three male skeletons; they match
MALE_ROOT = "Empty"              # parent of the imported male hierarchy
PARTS = ("SKM_BodyA", "SKM_HeadA_01", "SKM_EyebrowsA_01")
EYE_MATERIAL = "MI_Eyes_01_a"
FEMALE_MESHES = ("BodyCombined", "Eyes", "Eyebrows")
FEMALE_COLLECTION = "Female_Reference"
SOURCE_COLLECTION = "Male_Source"
OUTPUT_TAG = "male_fit"

FACE = ("eye_l", "eye_r", "eye_l_lid", "eye_r_lid", "jaw_01", "jaw_02",
        "eyebrow_1_l", "eyebrow_2_l", "eyebrow_3_l", "eyebrow_1_r", "eyebrow_2_r", "eyebrow_3_r",
        "tongue_1", "tongue_2", "tongue_3", "tongue_4")
HEAD_BONES = ("head",) + FACE

# Correction field: Gaussian radius and smoothing (metres).
FIELD_SIGMA = 0.035
FIELD_SMOOTH = 0.05


def chain_children():
    """Bone -> the child joint its segment should reach."""
    chain = {}
    spine = ("spine_01", "spine_02", "spine_03", "spine_04", "spine_05", "neck_01", "neck_02", "head")
    chain.update(zip(spine[:-1], spine[1:]))
    for s in ("l", "r"):
        for seq in (("clavicle", "upperarm", "lowerarm", "hand"), ("thigh", "calf", "foot", "ball"),
                    ("thumb_01", "thumb_02", "thumb_03")):
            names = [n + "_" + s for n in seq]
            chain.update(zip(names[:-1], names[1:]))
        for finger in ("index", "middle", "ring", "pinky"):
            names = ["%s_%s_%s" % (finger, k, s) for k in ("metacarpal", "01", "02", "03")]
            chain.update(zip(names[:-1], names[1:]))
    return chain


def world(arm, name):
    return arm.matrix_world @ arm.data.bones[name].matrix_local


def joint(arm, name):
    return arm.matrix_world @ arm.data.bones[name].head_local


def best_fit_rigid(src, dst):
    """Rotation + translation (Kabsch) taking the src points onto dst, as a 4x4."""
    P, Q = np.array(src), np.array(dst)
    pc, qc = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - pc).T @ (Q - qc))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    m = Matrix.Identity(4)
    for i in range(3):
        for j in range(3):
            m[i][j] = R[i, j]
    m.translation = Vector(qc - R @ pc)
    return m


def align_stretch(dm, df):
    """Linear map turning direction dm into df: rotate onto it, stretch along it."""
    n = dm.normalized()
    k = df.length / dm.length
    stretch = Matrix.Identity(3) + (k - 1.0) * Matrix([[n.x * n.x, n.x * n.y, n.x * n.z],
                                                      [n.y * n.x, n.y * n.y, n.y * n.z],
                                                      [n.z * n.x, n.z * n.y, n.z * n.z]])
    return (n.rotation_difference(df.normalized()).to_matrix() @ stretch).to_4x4()


def polyline_point(points, t):
    """Point at normalised arc length t along a polyline."""
    lengths = [(b - a).length for a, b in zip(points, points[1:])]
    target = t * sum(lengths)
    for a, b, seg in zip(points, points[1:], lengths):
        if target <= seg or b is points[-1]:
            return a.lerp(b, min(target / seg, 1.0) if seg > 0 else 0.0)
        target -= seg
    return points[-1].copy()


def spine_maps(male, female, head_map):
    """Maps for the spine and neck bones, following the torso as one curve.

    The two skeletons space their spine joints very differently (the male
    spine_04 alone is a third of his back), so bone-for-bone matching would
    stretch the chest unevenly. Instead pelvis meets pelvis, neck base meets
    neck base, the head follows the head fit, and every joint in between keeps
    its relative distance along the spine. Rotations ease from the pelvis fit
    to the head fit.
    """
    torso = ("pelvis", "spine_01", "spine_02", "spine_03", "spine_04", "spine_05", "neck_01")
    neck = ("neck_01", "neck_02", "head")
    m_torso = [joint(male, n) for n in torso]
    f_torso = [joint(female, n) for n in torso]
    m_neck = [joint(male, n) for n in neck]
    f_neck = [joint(female, "neck_01"), head_map @ joint(male, "head")]

    def cumulative(points):
        out = [0.0]
        for a, b in zip(points, points[1:]):
            out.append(out[-1] + (b - a).length)
        return out

    c_torso, c_neck = cumulative(m_torso), cumulative(m_neck)
    total = c_torso[-1] + c_neck[-1]
    pelvis_rot = (world(female, "pelvis") @ world(male, "pelvis").inverted()).to_quaternion()
    head_rot = head_map.to_quaternion()

    maps = {}
    for names, points, cum, target, offset in ((torso, m_torso, c_torso, f_torso, 0.0),
                                               (neck, m_neck, c_neck, f_neck, c_torso[-1])):
        for name, p, c in zip(names, points, cum):
            if name in ("pelvis", "head") or name in maps:
                continue
            dest = polyline_point(target, c / cum[-1])
            rot = pelvis_rot.slerp(head_rot, (offset + c) / total).to_matrix().to_4x4()
            maps[name] = Matrix.Translation(dest) @ rot @ Matrix.Translation(-p)
    return maps


def bone_maps(male, female):
    """World-space 4x4 per male bone, carrying the male rest pose onto the female one."""
    head_map = best_fit_rigid([joint(male, n) for n in FACE], [joint(female, n) for n in FACE])
    chain = chain_children()
    maps = {name: head_map for name in HEAD_BONES}
    maps.update(spine_maps(male, female, head_map))
    for bone in male.data.bones:
        name = bone.name
        if name in maps or name not in female.data.bones or "_twist_" in name:
            continue
        rm, rf = world(male, name), world(female, name)
        a = Matrix.Identity(4)
        child = chain.get(name)
        if child:
            cm = joint(male, child)
            target = head_map @ cm if child == "head" else joint(female, child)
            a = align_stretch(rm.inverted() @ cm, rf.inverted() @ target)
        maps[name] = rf @ a @ rm.inverted()
    for side in ("l", "r"):
        maps["clavicle_" + side] = clavicle_map(male, female, side, maps["spine_05"])
    for bone in male.data.bones:
        if "_twist_" in bone.name and bone.parent is not None:
            maps[bone.name] = maps[bone.parent.name]
    return maps, head_map


def clavicle_map(male, female, side, chest):
    """Clavicle map that starts where the chest already goes.

    The female clavicle joints sit ~2 cm closer to the centre line, and the male
    shoulders are ~5 cm wider per side. Snapping the clavicle root onto the
    female joint would shear the trapezius away from the neck, so the root
    follows the chest fit and the narrowing is spread along the clavicle
    instead; the shoulder (upperarm joint) still lands exactly.
    """
    root_m = joint(male, "clavicle_" + side)
    shoulder_m, shoulder_f = joint(male, "upperarm_" + side), joint(female, "upperarm_" + side)
    root_f = chest @ root_m
    turn = chest.to_3x3().normalized()
    d_m = turn @ (shoulder_m - root_m)
    d_f = shoulder_f - root_f
    n = d_m.normalized()
    k = d_f.length / d_m.length
    stretch = Matrix.Identity(3) + (k - 1.0) * Matrix([[n.x * n.x, n.x * n.y, n.x * n.z],
                                                      [n.y * n.x, n.y * n.y, n.y * n.z],
                                                      [n.z * n.x, n.z * n.y, n.z * n.z]])
    linear = n.rotation_difference(d_f.normalized()).to_matrix() @ stretch @ turn
    return Matrix.Translation(root_f) @ linear.to_4x4() @ Matrix.Translation(-root_m)


def correction_field(male, female, head_map):
    """Smooth displacement pulling the rigidly moved face joints onto the female ones."""
    src, disp = [], []
    for name in FACE:
        s = head_map @ joint(male, name)
        if any((s - p).length < 1e-4 for p in src):
            continue  # Lids share their eye's position.
        src.append(s)
        disp.append(joint(female, name) - s)
    # Anchors that must not move: the neck below the head.
    for anchor in (head_map @ joint(male, "head"), joint(female, "neck_02")):
        src.append(anchor)
        disp.append(Vector())
    S = np.array(src)
    K = np.exp(-((S[:, None] - S[None]) ** 2).sum(-1) / (2 * FIELD_SIGMA ** 2))
    coeff = np.linalg.solve(K + FIELD_SMOOTH * np.eye(len(S)), np.array(disp))

    def field(points):
        d2 = ((points[:, None] - S[None]) ** 2).sum(-1)
        return np.exp(-d2 / (2 * FIELD_SIGMA ** 2)) @ coeff
    return field


def smoothstep(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


class NeckZone:
    """Where the head drop is spread evenly instead of following the skin weights.

    The male throat is skinned mostly to the jaw and head, so with plain weight
    blending the whole drop would land in the thin neck-weighted band and fold
    into a collar. Inside the neck column, between the neck base and a plane
    under the jaw (chin crease to skull base), the move instead eases from the
    torso fit at the bottom to the head fit at the top.
    """
    def __init__(self, male):
        neck_01, jaw_02, head = (joint(male, n) for n in ("neck_01", "jaw_02", "head"))
        self.z_lo = neck_01.z - 0.035                         # collarbone level
        self.front = (jaw_02.y, jaw_02.z - 0.02)              # crease under the chin
        self.back = (head.y + 0.07, head.z + 0.01)            # base of the skull
        self.axis_y = (self.front[0] + self.back[0]) / 2
        self.r_in, self.r_out = 0.09, 0.15                    # neck column, fading out to the shoulders

    def factors(self, p):
        """(how much the zone applies, how far up the neck) for a world point."""
        (yf, zf), (yb, zb) = self.front, self.back
        z_hi = zf + (p.y - yf) * (zb - zf) / (yb - yf)
        if p.z < self.z_lo:
            return 0.0, 0.0
        r = (p.x ** 2 + (p.y - self.axis_y) ** 2) ** 0.5
        column = 1.0 - smoothstep((r - self.r_in) / (self.r_out - self.r_in))
        # Above the plane the column moves with the head outright, even where
        # the skin weights still carry some neck.
        return column, smoothstep((p.z - self.z_lo) / (z_hi - self.z_lo))


def fitted_positions(ob, maps, head_map, field, male, female):
    """New world positions for one male part."""
    names = {g.index: g.name for g in ob.vertex_groups}
    mw = ob.matrix_world
    eye_verts = set()
    mat_index = [i for i, m in enumerate(ob.data.materials) if m and m.name == EYE_MATERIAL]
    for poly in ob.data.polygons:
        if poly.material_index in mat_index:
            eye_verts.update(poly.vertices)
    eye_shift = {s: joint(female, "eye_" + s) - head_map @ joint(male, "eye_" + s) for s in ("l", "r")}
    zone = NeckZone(male)
    neck_top = maps["neck_02"]

    out = np.empty((len(ob.data.vertices), 3))
    for v in ob.data.vertices:
        p = mw @ v.co
        acc, torso, total = Vector(), Vector(), 0.0
        for g in v.groups:
            name = names[g.group]
            m = maps.get(name)
            if m is not None and g.weight > 0.0:
                acc += (m @ p) * g.weight
                # The same blend with the head and face read as the top of the neck.
                torso += ((neck_top if name in HEAD_BONES else m) @ p) * g.weight
                total += g.weight
        if total <= 0:
            out[v.index] = p
            continue
        skinned, torso = acc / total, torso / total
        column, up = zone.factors(p)
        if column > 0.0:
            eased = torso.lerp(head_map @ p, up)
            skinned = skinned.lerp(eased, column)
        out[v.index] = skinned
    eyes = np.array(sorted(eye_verts), dtype=int)
    rest = np.setdiff1d(np.arange(len(out)), eyes)
    out[rest] += field(out[rest])
    for i in eyes:
        # Eyeballs move rigidly with the head, then sit exactly on the female eye joints.
        p = head_map @ (mw @ ob.data.vertices[i].co)
        out[i] = p + eye_shift["l" if (p - joint(female, "eye_l")).length < (p - joint(female, "eye_r")).length else "r"]
    return out, eyes


class Scaled:
    """Stands in for a male object as if it had been uniformly scaled about the
    ground origin first, so the rest of the fit only handles proportions."""
    def __init__(self, ob, scale):
        self._ob = ob
        self.matrix_world = scale @ ob.matrix_world

    def __getattr__(self, name):
        return getattr(self._ob, name)


def match_height(male, female):
    """Uniform scale (about the ground origin) that puts the male eye line at
    the female eye height, so the head does not have to drop onto the neck."""
    m = (joint(male, "eye_l") + joint(male, "eye_r")) / 2
    f = (joint(female, "eye_l") + joint(female, "eye_r")) / 2
    return Matrix.Scale(f.z / m.z, 4)


def collection(name, scene):
    coll = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    if coll.name not in scene.collection.children:
        scene.collection.children.link(coll)
    return coll


def move_to(obs, coll):
    for ob in obs:
        for c in list(ob.users_collection):
            c.objects.unlink(ob)
        coll.objects.link(ob)


def exclude(scene, coll):
    for lc in bpy.context.view_layer.layer_collection.children:
        if lc.collection == coll:
            lc.exclude = True


def main():
    scene = bpy.context.scene
    female = bpy.data.objects[ARMATURE]
    male = bpy.data.objects[MALE_SKELETON]
    template = bpy.data.objects.get("BodyCombined_Female") or bpy.data.objects["BodyCombined"]

    # Previous outputs go; female originals are kept aside once.
    for ob in [o for o in bpy.data.objects if OUTPUT_TAG in o]:
        mesh = ob.data
        bpy.data.objects.remove(ob)
        if mesh is not None and mesh.users == 0:
            bpy.data.meshes.remove(mesh)
    fem_coll = collection(FEMALE_COLLECTION, scene)
    for name in FEMALE_MESHES:
        ob = bpy.data.objects.get(name)
        if ob is not None and OUTPUT_TAG not in ob:
            ob.name = name + "_Female"
            move_to([ob], fem_coll)

    scale = match_height(male, female)
    male = Scaled(male, scale)
    maps, head_map = bone_maps(male, female)
    field = correction_field(male, female, head_map)

    to_local = template.matrix_world.inverted()
    built, report = [], {"height_scale": round(scale[2][2], 4)}
    for part in PARTS:
        src = bpy.data.objects[part]
        pos, eyes = fitted_positions(Scaled(src, scale), maps, head_map, field, male, female)
        me = src.data.copy()
        for v, p in zip(me.vertices, pos):
            v.co = to_local @ Vector(p)
        ob = bpy.data.objects.new(part + "_fit", me)
        scene.collection.objects.link(ob)
        ob.parent = female
        ob.matrix_world = template.matrix_world
        ob[OUTPUT_TAG] = 1
        for g in [g for g in ob.vertex_groups if g.name not in female.data.bones]:
            ob.vertex_groups.remove(g)
        mod = ob.modifiers.new("Armature", 'ARMATURE')
        mod.object = female
        mod.use_vertex_groups, mod.use_bone_envelopes = True, False
        shift = np.linalg.norm(pos - np.array([src.matrix_world @ v.co for v in src.data.vertices]), axis=1)
        report[part] = {"verts": len(pos), "max_shift_cm": round(float(shift.max()) * 100, 2),
                        "mean_shift_cm": round(float(shift.mean()) * 100, 2), "eyeball_verts": len(eyes)}
        built.append(ob)

    body, head, brows = built
    # Eyeballs become their own object, like the female rig.
    view_layer = bpy.context.view_layer
    active = view_layer.objects.active
    if active is not None and active.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    view_layer.objects.active = head
    eye_slot = next(i for i, m in enumerate(head.data.materials) if m and m.name == EYE_MATERIAL)
    me = head.data
    eye_poly = [p.material_index == eye_slot for p in me.polygons]
    eye_vert = [False] * len(me.vertices)
    for poly, is_eye in zip(me.polygons, eye_poly):
        poly.select = is_eye
        if is_eye:
            for i in poly.vertices:
                eye_vert[i] = True
    for v, sel in zip(me.vertices, eye_vert):
        v.select = sel
    for e in me.edges:
        e.select = eye_vert[e.vertices[0]] and eye_vert[e.vertices[1]]
    before = set(bpy.data.objects)
    with bpy.context.temp_override(active_object=head, object=head, selected_objects=[head],
                                   selected_editable_objects=[head]):
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.separate(type='SELECTED')
        bpy.ops.object.mode_set(mode='OBJECT')
    eyes = next(o for o in bpy.data.objects if o not in before)
    eyes.data.materials.clear()
    eyes.data.materials.append(bpy.data.materials[EYE_MATERIAL])
    for poly in eyes.data.polygons:
        poly.material_index = 0
    head.data.materials.pop(index=eye_slot)
    for poly in head.data.polygons:
        if poly.material_index > eye_slot:
            poly.material_index -= 1

    # Body and head skin share the neck seam, so they become one mesh like the female one.
    with bpy.context.temp_override(active_object=body, object=body, selected_objects=[body, head],
                                   selected_editable_objects=[body, head]):
        bpy.ops.object.join()
    for ob, name in ((body, "BodyCombined"), (eyes, "Eyes"), (brows, "Eyebrows")):
        ob.name = name
        ob.data.name = name + "_Male"
        ob[OUTPUT_TAG] = 1
        used = {vg.group for v in ob.data.vertices for vg in v.groups if vg.weight > 0.0}
        for g in [g for g in ob.vertex_groups if g.index not in used]:
            ob.vertex_groups.remove(g)

    src_coll = collection(SOURCE_COLLECTION, scene)
    root = bpy.data.objects[MALE_ROOT]
    move_to([root] + list(root.children_recursive), src_coll)
    exclude(scene, src_coll)
    exclude(scene, fem_coll)

    eye_mid = (joint(male, "eye_l") + joint(male, "eye_r")) / 2
    report["head_move"] = {"eye_line_shift_cm": [round(x * 100, 2) for x in head_map @ eye_mid - eye_mid],
                           "rot_deg": round(head_map.to_quaternion().angle * 57.29578, 2)}
    print("Male fit:", report)
    return report


result = main()
