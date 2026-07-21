"""
FineTune: translation-only packing of visible instances.

When only a subset of instances is visible in the Web UI, FineTune shifts
(x/y translation only; orientation and flip are left unchanged) the visible
non-base instances so that they abut each other on as many sides as possible,
without violating margin requirements against any instance — including the
fixed (invisible) ones.
"""

from .models import DesignModel, ChipletDef, AABB
from .placer import SPACING_EPSILON, ABUT_TOLERANCE


def _margin(def_: ChipletDef) -> float:
    return max(def_.seal_ring) + max(def_.scribe_line)


def finetune_visible(design: DesignModel) -> int:
    """Pack the visible non-base instances together by translation only.

    The leftmost (then bottom-most) visible instance anchors the pack and is
    never moved. Every other visible instance is placed at the feasible
    candidate position that abuts the most already-placed visible instances
    (pocket corners win), tie-broken by smaller (y, x). Fixed obstacles are
    all invisible instances and the base layer; nothing may come closer than
    the pairwise margin to any of them.

    Returns the number of instances that actually moved.
    """
    movable = [i for i in design.instances
               if i.visible and not design.is_base_instance(i)]
    if not movable:
        return 0

    def z_overlap(inst_i, def_i, inst_o, def_o) -> bool:
        a0, a1 = inst_i.pose.z, inst_i.pose.z + def_i.thickness
        b0, b1 = inst_o.pose.z, inst_o.pose.z + def_o.thickness
        return not (a1 <= b0 or b1 <= a0)

    def key_aabb(inst):
        def_ = design.get_def(inst.reference)
        a = inst.global_aabb(def_) if def_ else AABB(0, 0, 0, 0)
        return (a.x1, a.y1, inst.name)

    movable.sort(key=key_aabb)
    anchor = movable[0]
    placed = [anchor]
    moved = 0

    for inst in movable[1:]:
        def_i = design.get_def(inst.reference)
        if not def_i:
            continue
        a_i = inst.global_aabb(def_i)
        w, h = a_i.width, a_i.height
        m_i = _margin(def_i)

        # Obstacles: every other non-base instance on an overlapping Z layer
        # (the base layer is the substrate, not a blocker — chiplets sit on
        # top of it). Entries are (aabb, margin, is_placed_visible); only
        # placed visible neighbors count toward the abutment score.
        obstacles = []
        for other in design.instances:
            if other.name == inst.name or design.is_base_instance(other):
                continue
            od = design.get_def(other.reference)
            if not od or not z_overlap(inst, def_i, other, od):
                continue
            is_placed_visible = any(other is p for p in placed)
            obstacles.append((other.global_aabb(od), _margin(od), is_placed_visible))

        cur_x, cur_y = a_i.x1, a_i.y1
        xs = {cur_x}
        ys = {cur_y}
        for a, m_o, _ in obstacles:
            mm = m_i + m_o + SPACING_EPSILON
            xs.update([a.x1 - mm - w, a.x2 + mm])
            ys.update([a.y1 - mm - h, a.y2 + mm])

        best = None  # (-abut_count, cy, cx)
        for cx in xs:
            for cy in ys:
                cand = AABB(cx, cy, cx + w, cy + h)
                ok = True
                abut_count = 0
                for a, m_o, is_placed_visible in obstacles:
                    mm = m_i + m_o + SPACING_EPSILON
                    if cand.inflate(mm).overlaps(a):
                        ok = False
                        break
                    if is_placed_visible:
                        x_ov = cand.x1 < a.x2 and a.x1 < cand.x2
                        y_ov = cand.y1 < a.y2 and a.y1 < cand.y2
                        if (y_ov and (abs((cand.x2 + mm) - a.x1) <= ABUT_TOLERANCE or
                                      abs((a.x2 + mm) - cand.x1) <= ABUT_TOLERANCE)) or \
                           (x_ov and (abs((cand.y2 + mm) - a.y1) <= ABUT_TOLERANCE or
                                      abs((a.y2 + mm) - cand.y1) <= ABUT_TOLERANCE)):
                            abut_count += 1
                if not ok:
                    continue
                key = (-abut_count, cy, cx)
                if best is None or key < best[0]:
                    best = (key, cx, cy)

        if best is not None:
            _, bx, by = best
            dx = bx - a_i.x1
            dy = by - a_i.y1
            if dx or dy:
                inst.pose.x += dx
                inst.pose.y += dy
                moved += 1
        placed.append(inst)

    return moved
