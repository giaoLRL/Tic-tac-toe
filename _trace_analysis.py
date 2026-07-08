"""
Comprehensive Z-axis scaling analysis for 4-DOF robotic arm IK.
Traces three test cases through committed (HEAD) and working tree code.
"""
import math

def ik_solve_head(x, y, z):
    """Committed HEAD: L1=105, L2=100, s2=500+(theta2-pi/2)*254.65"""
    L1=105.0; L2=100.0; SH=65.0
    BS=424.41; SS=254.65; ES=636.62; WS=636.62
    BO=1500; SO=500; EO=1900; WO=1600
    SERVO_MIN=500; SERVO_MAX=2490

    r = math.sqrt(x*x + y*y)
    theta1 = math.atan2(y, x)
    z_rel = z - SH
    d = math.sqrt(r*r + z_rel*z_rel)
    if d > L1 + L2:
        sc = (L1+L2)*0.95/d
        r*=sc; z_rel*=sc; d=(L1+L2)*0.95

    cos_t3 = (d*d - L1*L1 - L2*L2)/(2*L1*L2)
    cos_t3 = max(-1,min(1,cos_t3))
    theta3 = math.acos(cos_t3)
    theta2 = math.atan2(z_rel,r) - math.atan2(L2*math.sin(theta3), L1+L2*math.cos(theta3))
    theta4 = -(theta2+theta3)-math.pi/2

    s1_raw = BO + theta1*BS
    s2_raw = SO + (theta2-math.pi/2)*SS
    s3_raw = EO - theta3*ES
    s4_raw = WO + theta4*WS

    s1 = max(SERVO_MIN, min(SERVO_MAX, int(s1_raw)))
    s2 = max(SERVO_MIN, min(SERVO_MAX, int(s2_raw)))
    s3 = max(SERVO_MIN, min(SERVO_MAX, int(s3_raw)))
    s4 = max(SERVO_MIN, min(SERVO_MAX, int(s4_raw)))

    # Actual angle after clamping
    # Inverse of: s2 = SO + (theta2 - pi/2)*SS  -> theta2 = (s2-SO)/SS + pi/2
    theta2_actual = (s2 - SO)/SS + math.pi/2

    # Forward kinematics
    xr = L1*math.cos(theta2_actual) + L2*math.cos(theta2_actual+theta3)
    zr = L1*math.sin(theta2_actual) + L2*math.sin(theta2_actual+theta3)
    z_actual = zr + SH

    # Also compute what FK would look like if using COMPUTED theta2 (ideal)
    xr_ideal = L1*math.cos(theta2) + L2*math.cos(theta2+theta3)
    zr_ideal = L1*math.sin(theta2) + L2*math.sin(theta2+theta3)

    return dict(L1=L1, L2=L2, r=r, z_rel=z_rel, d=d,
                t1=math.degrees(theta1), t2=math.degrees(theta2),
                t3=math.degrees(theta3), t4=math.degrees(theta4),
                s1_raw=s1_raw, s2_raw=s2_raw, s3_raw=s3_raw, s4_raw=s4_raw,
                s1=s1, s2=s2, s3=s3, s4=s4,
                clamps=[n for n,v in zip(['s1','s2','s3','s4'],[s1_raw,s2_raw,s3_raw,s4_raw]) if v<SERVO_MIN or v>SERVO_MAX],
                t2_actual=math.degrees(theta2_actual),
                fk_z_actual=z_actual, fk_z_ideal=zr_ideal+SH,
                fk_r_actual=xr, fk_r_ideal=xr_ideal)


def ik_solve_working(x, y, z):
    """Working tree: L1=60, L2=135, s2=500+theta2*560"""
    L1=60.0; L2=135.0; SH=65.0
    BS=424.41; SS=560.0; ES=636.62; WS=636.62
    BO=1500; SO=500; EO=1900; WO=1600
    SERVO_MIN=500; SERVO_MAX=2490

    r = math.sqrt(x*x + y*y)
    theta1 = math.atan2(y, x)
    z_rel = z - SH
    d = math.sqrt(r*r + z_rel*z_rel)
    if d > L1 + L2:
        sc = (L1+L2)*0.95/d
        r*=sc; z_rel*=sc; d=(L1+L2)*0.95

    cos_t3 = (d*d - L1*L1 - L2*L2)/(2*L1*L2)
    cos_t3 = max(-1,min(1,cos_t3))
    theta3 = math.acos(cos_t3)
    theta2 = math.atan2(z_rel,r) - math.atan2(L2*math.sin(theta3), L1+L2*math.cos(theta3))
    theta4 = -(theta2+theta3)-math.pi/2

    s1_raw = BO + theta1*BS
    s2_raw = SO + theta2*SS
    s3_raw = EO - theta3*ES
    s4_raw = WO + theta4*WS

    s1 = max(SERVO_MIN, min(SERVO_MAX, int(s1_raw)))
    s2 = max(SERVO_MIN, min(SERVO_MAX, int(s2_raw)))
    s3 = max(SERVO_MIN, min(SERVO_MAX, int(s3_raw)))
    s4 = max(SERVO_MIN, min(SERVO_MAX, int(s4_raw)))

    # Inverse: s2 = SO + theta2*SS  -> theta2 = (s2-SO)/SS
    theta2_actual = (s2 - SO)/SS

    xr = L1*math.cos(theta2_actual) + L2*math.cos(theta2_actual+theta3)
    zr = L1*math.sin(theta2_actual) + L2*math.sin(theta2_actual+theta3)
    z_actual = zr + SH

    xr_ideal = L1*math.cos(theta2) + L2*math.cos(theta2+theta3)
    zr_ideal = L1*math.sin(theta2) + L2*math.sin(theta2+theta3)

    return dict(L1=L1, L2=L2, r=r, z_rel=z_rel, d=d,
                t1=math.degrees(theta1), t2=math.degrees(theta2),
                t3=math.degrees(theta3), t4=math.degrees(theta4),
                s1_raw=s1_raw, s2_raw=s2_raw, s3_raw=s3_raw, s4_raw=s4_raw,
                s1=s1, s2=s2, s3=s3, s4=s4,
                clamps=[n for n,v in zip(['s1','s2','s3','s4'],[s1_raw,s2_raw,s3_raw,s4_raw]) if v<SERVO_MIN or v>SERVO_MAX],
                t2_actual=math.degrees(theta2_actual),
                fk_z_actual=z_actual, fk_z_ideal=zr_ideal+SH,
                fk_r_actual=xr, fk_r_ideal=xr_ideal)


def print_trace(res, label):
    v = res
    print(f"  {'='*68}")
    print(f"  {v['version']}")
    print(f"  {label}")
    print(f"  {'='*68}")
    print(f"  L1={v['L1']:.0f}  L2={v['L2']:.0f}")
    print(f"  r={v['r']:.2f}  z_rel={v['z_rel']:.2f}  d={v['d']:.2f}  (reach={v['L1']+v['L2']:.0f})")
    print(f"  theta2={v['t2']:.2f}  theta3={v['t3']:.2f}  theta4={v['t4']:.2f}")
    print(f"  s1={v['s1_raw']:.1f}->{v['s1']}  s2={v['s2_raw']:.1f}->{v['s2']}  s3={v['s3_raw']:.1f}->{v['s3']}  s4={v['s4_raw']:.1f}->{v['s4']}")
    print(f"  Clamped: {', '.join(v['clamps']) if v['clamps'] else '(none)'}")
    print(f"  theta2_actual (after clamp) = {v['t2_actual']:.2f} deg")
    print(f"  FK actual  : z={v['fk_z_actual']:.1f} mm, r={v['fk_r_actual']:.1f} mm")
    print(f"  FK ideal   : z={v['fk_z_ideal']:.1f} mm (what theta2 SHOULD produce)")
    print(f"  FK actual vs target: Z_err={v['fk_z_actual']-v['z_target']:.1f} mm")
    print()


tests = [
    (165, 0, 50,  "#POS,165,0,50  (complaint)"),
    (165, 0, 200, "#POS,165,0,200 (Z=200)"),
    (165, 0, 20,  "#POS,165,0,20  (ground-hit)"),
]

for x, y, z, label in tests:
    rh = ik_solve_head(x, y, z)
    rh['version'] = 'COMMITTED (HEAD) — on hardware'
    rh['z_target'] = z
    rw = ik_solve_working(x, y, z)
    rw['version'] = 'WORKING TREE (uncommitted)'
    rw['z_target'] = z

    print_trace(rh, label)
    print_trace(rw, label)

# Summary
print(f"  {'='*68}")
print(f"  SUMMARY TABLE")
print(f"  {'='*68}")
print(f"  {'Command':<20} {'HEAD actual Z':<16} {'WT actual Z':<16} {'HEAD theta2':<16} {'WT theta2':<16}")
print(f"  {'-'*18} {'-'*14} {'-'*14} {'-'*14} {'-'*14}")
for x, y, z, label in tests:
    rh = ik_solve_head(x, y, z)
    rw = ik_solve_working(x, y, z)
    cmd = f"#POS,{x},{y},{z}"
    print(f"  {cmd:<20} {rh['fk_z_actual']:<16.1f} {rw['fk_z_actual']:<16.1f} {rh['t2_actual']:<16.1f} {rw['t2_actual']:<16.1f}")

# The real problem: theta2 goes NEGATIVE for any z < ~130mm
print(f"""
  {'='*68}
  CORE INSIGHT: theta2 SIGN
  {'='*68}
  For BOTH committed and working tree code, when the target z is below
  the reachable mid-range, theta2 goes NEGATIVE (shoulder points DOWN):

   HEAD  #POS,165,0,50  -> theta2 = -40.3 deg  -> s2 CLAMPS to 500
   WT    #POS,165,0,50  -> theta2 = -55.0 deg  -> s2 CLAMPS to 500
   HEAD  #POS,165,0,20  -> theta2 = -47.8 deg  -> s2 CLAMPS to 500
   WT    #POS,165,0,20  -> theta2 = -59.8 deg  -> s2 CLAMPS to 500

  But s2=500 = SERVO_MIN for the PLUS-sign formulas, so the arm
  PHYSICALLY stays at:
    HEAD: theta2_actual = 90 deg (vertical up), Z ~200-210 mm
    WT:   theta2_actual = 0 deg (horizontal), Z ~185-192 mm

  For Z=200 (above shoulder), theta2 is slightly positive:
    HEAD: theta2 = +21.5 deg -> STILL clamps because theta2 < 90 deg!
           (formula has -(theta2-pi/2), so theta2=21.5 < 90 means
            theta2-pi/2 is negative, s2 < 500, CLAMPS)
    WT:   theta2 = +11.7 deg -> s2 = 613 (no clamp, still positive)
""")
