# Mass properties from the actual meshes (uniform density scaled to each part's real mass),
# expressed in each link's frame (= after the same pose the visual uses).
import os, sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mesh_inertia import in_link, combine, inertial_xml
I3=np.eye(3); Z3=np.zeros(3)
IN_CHASSIS = inertial_xml(*in_link("chassis_only.stl", 0.6683, _R(1.5708,0,0), Z3)[:3])
for _n,_c in CORNERS.items():
    _c['_in_leg'] = inertial_xml(*in_link(_c['leg_mesh'], 0.04344, _R(0,-1.5708,0), np.array([0,0,-0.1156]))[:3])
IN_N20   = inertial_xml(*in_link("N20 DC MOTOR.stl", 0.01902, I3, Z3)[:3])
IN_WHEEL = inertial_xml(*in_link("Wheel_morpho.stl", 0.04944, I3, Z3)[:3])
IN_SERVO = inertial_xml(*in_link("Mg995.stl", 0.097, I3, Z3)[:3])
# rotor = BLDC (34.65 g) + Gemfan 51466 prop (4.5 g) = 39.15 g
def _balanced(m, c, I):
    """Put the rotor's centre of mass on its spin axis (link Z) and drop the products of inertia.
    The CAD meshes leave the COM ~0.16 mm off-axis; spinning at ~1800 rad/s that is ~20 N of
    rotating force per rotor (a badly unbalanced prop), which showed up as VIBE X/Y ~15 m/s^2.
    Real props are balanced; motor vibration, if wanted, is simulated with SIM_VIB_* instead."""
    c = np.array([0.0, 0.0, c[2]])
    ixy = 0.5 * (I[0, 0] + I[1, 1])
    return m, c, np.diag([ixy, ixy, I[2, 2]])
IN_ROTOR = inertial_xml(*_balanced(*combine([in_link("brushless emax series eco ii 2207 v23.stl", 0.03465, I3, Z3),
                                  in_link("gemfan propeller 51466.stl", 0.0045, _R(1.5708,0,0), np.array([0,0,0.027]))])))
