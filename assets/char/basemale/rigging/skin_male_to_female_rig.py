# Skins the hand-fitted male parts (children of "Empty.tobeskinned") to the
# female rig "Armature" in this file, by copying the female skin weights.
#
#   * Body and head take the weights of the nearest point on the female
#     BodyCombined, interpolated across that triangle. The female rest shape is
#     read from the mesh data, so whatever pose the rig is in does not matter.
#   * Eyebrows copy from the female Eyebrows mesh the same way.
#   * Eyeballs (the eye material in the head mesh) go 100% to eye_l / eye_r.
#   * Each part is re-parented to the Armature (world transform kept) and gets
#     an Armature modifier like the female meshes.
#
# Re-running replaces the copied weights and the modifier.

import bpy
from mathutils import interpolate
from mathutils.bvhtree import BVHTree

ARMATURE = "Armature"
PARTS = {                      # male part -> female mesh to copy weights from
    "SKM_BodyA": "BodyCombined",
    "SKM_HeadA_01": "BodyCombined",
    "SKM_EyebrowsA_01": "Eyebrows",
}
EYE_MATERIAL = "MI_Eyes_01_a"
MIN_WEIGHT = 0.01              # influences below this are dropped
MAX_INFLUENCES = 8


class Source:
    """Female mesh in its rest shape: triangles + per-vertex weights."""

    def __init__(self, ob):
        me = ob.data
        me.calc_loop_triangles()
        mw = ob.matrix_world
        self.co = [mw @ v.co for v in me.vertices]
        self.tris = [tuple(t.vertices) for t in me.loop_triangles]
        self.bvh = BVHTree.FromPolygons(self.co, self.tris)
        names = {g.index: g.name for g in ob.vertex_groups}
        self.weights = [{names[g.group]: g.weight for g in v.groups if g.weight > 0.0} for v in me.vertices]

    def sample(self, p):
        hit, _, tri, dist = self.bvh.find_nearest(p)
        corners = self.tris[tri]
        bary = interpolate.poly_3d_calc([self.co[i] for i in corners], hit)
        out = {}
        for i, b in zip(corners, bary):
            for name, w in self.weights[i].items():
                out[name] = out.get(name, 0.0) + w * b
        return out, dist


def clean(weights):
    top = sorted(((w, n) for n, w in weights.items() if w >= MIN_WEIGHT), reverse=True)[:MAX_INFLUENCES]
    total = sum(w for w, _ in top)
    return {n: w / total for w, n in top} if total > 0 else {}


def skin(ob, source, arm):
    me = ob.data
    mw = ob.matrix_world
    eye_slot = [i for i, m in enumerate(me.materials) if m and m.name == EYE_MATERIAL]
    eye_verts = {i for p in me.polygons if p.material_index in eye_slot for i in p.vertices}

    ob.vertex_groups.clear()
    groups = {}

    def group(name):
        if name not in groups:
            groups[name] = ob.vertex_groups.new(name=name)
        return groups[name]

    far = 0
    for v in me.vertices:
        p = mw @ v.co
        if v.index in eye_verts:
            weights = {"eye_l" if p.x > 0 else "eye_r": 1.0}
        else:
            raw, dist = source.sample(p)
            weights = clean({n: w for n, w in raw.items() if n in arm.data.bones})
            far += dist > 0.015
        for name, w in weights.items():
            group(name).add([v.index], w, 'REPLACE')

    for m in [m for m in ob.modifiers if m.type == 'ARMATURE']:
        ob.modifiers.remove(m)
    mod = ob.modifiers.new("Armature", 'ARMATURE')
    mod.object = arm
    mod.use_vertex_groups, mod.use_bone_envelopes = True, False

    world = ob.matrix_world.copy()
    ob.parent = arm
    ob.matrix_parent_inverse = arm.matrix_world.inverted()
    ob.matrix_world = world
    return {"verts": len(me.vertices), "groups": len(ob.vertex_groups), "eyeball_verts": len(eye_verts),
            "verts_over_15mm_from_female": far}


def main():
    arm = bpy.data.objects[ARMATURE]
    sources = {}
    report = {}
    for part, src in PARTS.items():
        if src not in sources:
            sources[src] = Source(bpy.data.objects[src])
        report[part] = skin(bpy.data.objects[part], sources[src], arm)
    print("Male skin:", report)
    return report


result = main()
