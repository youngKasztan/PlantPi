"""PlantPi peristaltic pump for a 28BYJ-48 stepper - Autodesk Fusion script.

Run in Fusion: Utilities > Add-Ins > Scripts > "+" > pick this file's folder > Run.
It builds a new design with 5 components:

  Housing        1x  print flat, plate down
  Rotor base     1x  print disc down, pins up
  Rotor top      1x  print flat
  Roller         3x  print standing (axis vertical)
  28BYJ-48 (ref) -   motor model for checking the fit, DO NOT PRINT

Change the numbers in P below (tube size, occlusion) and run again.
All dimensions in mm. z = 0 is the motor's flange face (the face with the shaft).
"""
import math

import adsk.core
import adsk.fusion

P = dict(
    # tube: soft silicone, e.g. 2 mm inside / 4 mm outside diameter
    tube_od=4.0,
    tube_id=2.0,
    occlusion=0.85,      # 1.0 = squeeze the tube walls exactly together; lower = gentler (less torque)
    # rotor
    roller_circle_r=10.0,  # roller centres on this radius (smaller = more force, less flow)
    roller_od=8.0,
    roller_bore=4.5,
    pin_d=4.0,
    pin_hole_top=4.1,    # rotor-top holes: snug on the 4 mm pins (add a drop of glue)
    # housing
    wall=2.5,
    plate=3.0,
    leg_len=22.0,        # how far the straight tube channels run below the shaft
    grip=0.8,            # exit slot width = grip x tube_od (holds the tube so it is not dragged)
    # 28BYJ-48 (datasheet / measured)
    motor_d=28.0, motor_h=19.0, shaft_offset=8.0, ear_spacing=35.0, ear_hole=4.2,
    boss_d=9.0, boss_h=1.5, shaft_d=5.0, shaft_flats=3.0, shaft_len=10.0, flat_len=6.0,
    screw_pilot=2.8,     # M3 screw cuts its own thread in the plastic
)


def mm(v):
    return v / 10.0


def pt(x, y):
    return adsk.core.Point3D.create(mm(x), mm(y), 0)


class Builder:
    def __init__(self, design):
        self.design = design
        self.root = design.rootComponent

    def component(self, name):
        occ = self.root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = name
        return occ.component

    def sketch(self, comp, z):
        if abs(z) < 1e-9:
            plane = comp.xYConstructionPlane
        else:
            pi = comp.constructionPlanes.createInput()
            pi.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
            plane = comp.constructionPlanes.add(pi)
            plane.isLightBulbOn = False
        return comp.sketches.add(plane)

    def extrude(self, comp, sk, h, op, profiles=None, body=None):
        coll = adsk.core.ObjectCollection.create()
        for p in (profiles if profiles is not None else list(sk.profiles)):
            coll.add(p)
        inp = comp.features.extrudeFeatures.createInput(coll, op)
        inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(h)))
        if body is not None and op != adsk.fusion.FeatureOperations.NewBodyFeatureOperation:
            inp.participantBodies = [body]
        f = comp.features.extrudeFeatures.add(inp)
        sk.isVisible = False
        return f

    def prism(self, comp, z0, z1, draw, op, body=None, pick=None):
        sk = self.sketch(comp, z0)
        draw(sk)
        profs = list(sk.profiles)
        if pick:
            profs = pick(profs)
        f = self.extrude(comp, sk, z1 - z0, op, profs, body)
        return f.bodies.item(0) if f.bodies.count else body


# ---------------------------------------------------------------- 2D shapes
def u_shape(r, L):
    """Semicircle of radius r above the x axis + rectangle down to y = -L (one closed loop)."""
    def draw(sk):
        a, l = sk.sketchCurves.sketchArcs, sk.sketchCurves.sketchLines
        a.addByThreePoints(pt(r, 0), pt(0, r), pt(-r, 0))
        l.addByTwoPoints(pt(-r, 0), pt(-r, -L))
        l.addByTwoPoints(pt(-r, -L), pt(r, -L))
        l.addByTwoPoints(pt(r, -L), pt(r, 0))
    return draw


def circle(x, y, d):
    def draw(sk):
        sk.sketchCurves.sketchCircles.addByCenterRadius(pt(x, y), mm(d / 2))
    return draw


def circles(centres, d):
    def draw(sk):
        for (x, y) in centres:
            sk.sketchCurves.sketchCircles.addByCenterRadius(pt(x, y), mm(d / 2))
    return draw


def rect(x0, y0, x1, y1):
    def draw(sk):
        sk.sketchCurves.sketchLines.addTwoPointRectangle(pt(x0, y0), pt(x1, y1))
    return draw


def double_d(d, flats):
    """Shaft hole: circle of diameter d with two flats `flats` apart (flats parallel to y)."""
    r, f = d / 2, flats / 2
    yy = math.sqrt(r * r - f * f)

    def draw(sk):
        a, l = sk.sketchCurves.sketchArcs, sk.sketchCurves.sketchLines
        a.addByThreePoints(pt(f, yy), pt(0, r), pt(-f, yy))
        l.addByTwoPoints(pt(-f, yy), pt(-f, -yy))
        a.addByThreePoints(pt(-f, -yy), pt(0, -r), pt(f, -yy))
        l.addByTwoPoints(pt(f, -yy), pt(f, yy))
    return draw


def largest(profs):
    return [max(profs, key=lambda p: p.areaProperties().area)]


# ---------------------------------------------------------------- geometry
def derived(p):
    g = dict(p)
    wall_t = (p["tube_od"] - p["tube_id"]) / 2
    g["roller_out"] = p["roller_circle_r"] + p["roller_od"] / 2
    g["gap"] = 2 * wall_t * p["occlusion"]                  # roller-to-track gap = squeezed tube
    g["r_track"] = g["roller_out"] + g["gap"]
    g["r_out"] = g["r_track"] + p["wall"]
    g["r_flange"] = g["r_track"] - 0.6
    # z stack (motor face = 0)
    g["z_rotor"] = p["plate"] + 0.5                         # rotor base disc bottom
    g["disc"] = 2.0
    g["collar"] = 0.4
    g["roller_len"] = max(p["tube_od"] * math.pi / 2 + 0.9, 7.0)
    g["z_roll0"] = g["z_rotor"] + g["disc"] + g["collar"]
    g["z_roll1"] = g["z_roll0"] + g["roller_len"]
    g["z_top0"] = g["z_roll1"] + g["collar"]
    g["z_top1"] = g["z_top0"] + g["disc"]
    g["z_wall"] = g["z_top1"] + 1.0
    g["z_tube"] = (g["z_roll0"] + g["z_roll1"]) / 2
    g["ears"] = [(-p["ear_spacing"] / 2, p["shaft_offset"]), (p["ear_spacing"] / 2, p["shaft_offset"])]
    g["pins"] = [(p["roller_circle_r"] * math.cos(math.radians(90 + 120 * k)),
                  p["roller_circle_r"] * math.sin(math.radians(90 + 120 * k))) for k in range(3)]
    g["grip_half"] = g["r_track"] - p["grip"] * p["tube_od"]
    g["flow_ml_per_rev"] = (math.pi * (p["tube_id"] / 2) ** 2) * 2 * math.pi * (g["roller_out"] + g["gap"] / 2) / 1000
    assert g["r_flange"] < p["leg_len"] - 6 - 0.5, "rotor would hit the tube grip block: increase leg_len"
    assert p["roller_circle_r"] - p["roller_od"] / 2 > 5.5 + 0.4, "rollers hit the hub"
    return g


def build(app):
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    g = derived(P)
    B = Builder(design)
    NEW, JOIN, CUT = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                      adsk.fusion.FeatureOperations.JoinFeatureOperation,
                      adsk.fusion.FeatureOperations.CutFeatureOperation)
    for k in ("tube_od", "tube_id", "occlusion", "roller_circle_r", "roller_od"):
        unit = "" if k == "occlusion" else "mm"
        design.userParameters.add(k, adsk.core.ValueInput.createByString(f"{P[k]} {unit}".strip()), unit,
                                  "PlantPi pump (documentation only - edit P in the script and re-run)")

    # ---------------- housing
    hc = B.component("Housing")
    L, ro, rt = P["leg_len"], g["r_out"], g["r_track"]
    body = B.prism(hc, 0, P["plate"], u_shape(ro, L), NEW)
    body.name = "Housing"
    # ear lobes with screw pillars
    pillar_h = 8.0
    for (x, y) in g["ears"]:
        B.prism(hc, 0, pillar_h, circle(x, y, 9.0), JOIN, body)
        sx = 1 if x > 0 else -1
        B.prism(hc, 0, P["plate"], rect(sx * (ro - 3), y - 4.5, x, y + 4.5), JOIN, body)
    # walls
    B.prism(hc, P["plate"], g["z_wall"], u_shape(ro, L), JOIN, body)
    B.prism(hc, P["plate"], g["z_wall"], u_shape(rt, L + 1), CUT, body)
    # tube grip block between the two exit channels
    gh = g["grip_half"]
    B.prism(hc, P["plate"], g["z_wall"], rect(-gh, -L, gh, -L + 6), JOIN, body)
    # holes: shaft boss, screws
    B.prism(hc, 0, P["plate"], circle(0, 0, P["boss_d"] + 1.0), CUT, body)
    B.prism(hc, 0, pillar_h, circles(g["ears"], P["screw_pilot"]), CUT, body)

    # ---------------- rotor base: disc + hub + pins
    rc = B.component("Rotor base")
    z0 = g["z_rotor"]
    rb = B.prism(rc, z0, z0 + g["disc"], circle(0, 0, 2 * g["r_flange"]), NEW)
    rb.name = "Rotor base"
    hub_top = P["shaft_len"] + 1.0
    B.prism(rc, z0 + g["disc"], hub_top, circle(0, 0, 10.0), JOIN, rb)
    B.prism(rc, z0 + g["disc"], z0 + g["disc"] + g["collar"], circles(g["pins"], P["pin_d"] + 1.6), JOIN, rb)
    B.prism(rc, z0 + g["disc"], g["z_top1"], circles(g["pins"], P["pin_d"]), JOIN, rb)
    # shaft hole: round where the shaft is round, double-D where it has flats
    z_flat = P["shaft_len"] - P["flat_len"]
    B.prism(rc, z0, max(z_flat, z0) + 0.5, circle(0, 0, P["shaft_d"] + 0.3), CUT, rb)
    B.prism(rc, max(z_flat, z0) + 0.5, P["shaft_len"] + 0.3, double_d(P["shaft_d"] + 0.3, P["shaft_flats"] + 0.2),
            CUT, rb)

    # ---------------- rotor top
    tc = B.component("Rotor top")
    tb = B.prism(tc, g["z_top0"], g["z_top1"], circle(0, 0, 2 * g["r_flange"]), NEW)
    tb.name = "Rotor top"
    B.prism(tc, g["z_top0"] - g["collar"], g["z_top0"], circles(g["pins"], P["pin_d"] + 1.6), JOIN, tb)
    B.prism(tc, g["z_top0"] - g["collar"], g["z_top1"], circles(g["pins"], P["pin_hole_top"]), CUT, tb)

    # ---------------- rollers (3 identical bodies, export one, print 3)
    lc = B.component("Roller x3")
    for k, (x, y) in enumerate(g["pins"]):
        def ring(sk, x=x, y=y):
            c = sk.sketchCurves.sketchCircles
            c.addByCenterRadius(pt(x, y), mm(P["roller_od"] / 2))
            c.addByCenterRadius(pt(x, y), mm(P["roller_bore"] / 2))
        rl = B.prism(lc, g["z_roll0"], g["z_roll1"], ring, NEW,
                     pick=lambda ps: [p for p in ps if p.profileLoops.count == 2])
        rl.name = f"Roller {k + 1}"

    # ---------------- 28BYJ-48 reference model (do not print)
    mc = B.component("28BYJ-48 (reference - do not print)")
    cy = P["shaft_offset"]
    mb = B.prism(mc, -P["motor_h"], 0, circle(0, cy, P["motor_d"]), NEW)
    mb.name = "Motor"
    ex = P["ear_spacing"] / 2

    def ears(sk):
        a, l = sk.sketchCurves.sketchArcs, sk.sketchCurves.sketchLines
        a.addByThreePoints(pt(-ex, cy + 3.5), pt(-ex - 3.5, cy), pt(-ex, cy - 3.5))
        l.addByTwoPoints(pt(-ex, cy - 3.5), pt(ex, cy - 3.5))
        a.addByThreePoints(pt(ex, cy - 3.5), pt(ex + 3.5, cy), pt(ex, cy + 3.5))
        l.addByTwoPoints(pt(ex, cy + 3.5), pt(-ex, cy + 3.5))
    B.prism(mc, -0.8, 0, ears, JOIN, mb, pick=largest)
    B.prism(mc, -0.8, 0, circles(g["ears"], P["ear_hole"]), CUT, mb)
    B.prism(mc, 0, P["boss_h"], circle(0, 0, P["boss_d"]), JOIN, mb)
    B.prism(mc, P["boss_h"], z_flat, circle(0, 0, P["shaft_d"]), JOIN, mb)
    B.prism(mc, z_flat, P["shaft_len"], double_d(P["shaft_d"], P["shaft_flats"]), JOIN, mb)
    return g


def run(context):
    app = adsk.core.Application.get()
    try:
        g = build(app)
        app.userInterface.messageBox(
            f"PlantPi pump built.\nTrack radius {g['r_track']:.2f} mm, theoretical flow "
            f"{g['flow_ml_per_rev']:.2f} ml/rev.\nExport Housing, Rotor base, Rotor top, Roller 1 as STL.")
    except Exception:
        import traceback
        app.userInterface.messageBox(traceback.format_exc())
