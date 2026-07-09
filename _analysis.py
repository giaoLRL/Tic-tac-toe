import math

# ========== Current code parameters ==========
L1 = 60.0
L2 = 135.0
SH = 65.0
BASE_SCALE = 424.41
SHOULDER_SCALE = 254.65
ELBOW_SCALE = 636.62
WRIST_SCALE = 636.62
BASE_OFF = 1500
SHOULDER_OFF = 500
ELBOW_OFF = 1900
WRIST_OFF = 1600

def ik_pwm(x, y, z):
    r = math.sqrt(x*x + y*y)
    theta1 = math.atan2(y, x)
    z_rel = z - SH
    d = math.sqrt(r*r + z_rel*z_rel)
    if d > (L1 + L2):
        scale = (L1 + L2) * 0.95 / d
        r *= scale; z_rel *= scale; d = (L1+L2)*0.95

    cos_t3 = (d*d - L1*L1 - L2*L2) / (2*L1*L2)
    cos_t3 = max(-1, min(1, cos_t3))
    theta3 = math.acos(cos_t3)
    sin_t3 = math.sin(theta3)

    alpha = math.atan2(z_rel, r)
    beta = math.atan2(L2*sin_t3, L1+L2*cos_t3)
    theta2 = alpha - beta
    theta4 = -(theta2 + theta3) - math.pi/2

    s1 = int(BASE_OFF + theta1 * BASE_SCALE)
    s2 = int(SHOULDER_OFF + theta2 * SHOULDER_SCALE)
    s3 = int(ELBOW_OFF - theta3 * ELBOW_SCALE)
    s4 = int(WRIST_OFF + theta4 * WRIST_SCALE)

    return (s1,s2,s3,s4), (theta1, theta2, theta3, theta4)

def fk(theta2, theta3):
    x_rel = L1*math.cos(theta2) + L2*math.cos(theta2+theta3)
    z_rel = L1*math.sin(theta2) + L2*math.sin(theta2+theta3)
    return x_rel, z_rel

print("="*65)
print("Current code: L1=%.0f, L2=%.0f, SH=%.0f" % (L1, L2, SH))
print("="*65)

tests = [
    (100, 0, 210, "user example"),
    (165, 0, 35,  "measured data"),
]

for x, y, z, label in tests:
    pwm, angles = ik_pwm(x, y, z)
    t1, t2, t3, t4 = angles
    print(f"\n--- Input: ({x}, {y}, {z}) [{label}] ---")
    print(f"  Angles: t1={math.degrees(t1):.1f}  t2={math.degrees(t2):.1f}  t3={math.degrees(t3):.1f}  t4={math.degrees(t4):.1f}")
    print(f"  PWM:    s1={pwm[0]}  s2={pwm[1]}  s3={pwm[2]}  s4={pwm[3]}")

    xr, zr = fk(t2, t3)
    print(f"  FK:     r={xr:.1f}  z_rel={zr:.1f}  z={zr+SH:.1f}  (target: r={math.sqrt(x*x+y*y):.1f}, z={z:.1f})")

    for i, (s, name) in enumerate(zip(pwm, ['Base','Shoulder','Elbow','Wrist'])):
        if s < 500:
            print(f"  ** CLAMP: {name} PWM={s} < 500!")
        elif s > 2490:
            print(f"  ** CLAMP: {name} PWM={s} > 2490!")

print("\n" + "="*65)
print("DIAGNOSIS")
print("="*65)

# Issue 1: s2 scale
print("\n[Issue 1: Shoulder s2 formula]")
print(f"  offset={SHOULDER_OFF}, scale={SHOULDER_SCALE}")
t2_range = math.pi  # -pi/2 to pi/2 typical
pwm_range = t2_range * SHOULDER_SCALE
print(f"  Full theoretical PWM range for t2 in [-pi/2, pi/2]: {pwm_range:.0f}")
print(f"  Expected range for 270deg servo: 1990 (500-2490)")
print(f"  SHOULDER_SCALE=254.65 = 800/pi (legacy IK_RadToPWM), NOT standard 270deg scale")
print(f"  Standard 270deg scale = 1990/(3*pi/2) = 424.41")

# s2 values at key angles
for deg in [-90, -45, 0, 45, 90]:
    rad = math.radians(deg)
    pwm = SHOULDER_OFF + rad * SHOULDER_SCALE
    print(f"  t2={deg:4.0f} deg -> s2={pwm:.0f}")

# Issue 2: s3 back-calc from measured data
print("\n[Issue 2: Elbow s3 back-calculation from measured data]")
# Measured: (165,0,35) -> PWM=(1500,800,1000,1500)
_, (t1_m, t2_m, t3_m, t4_m) = ik_pwm(165, 0, 35)
print(f"  (165,0,35) computed angles: t2={math.degrees(t2_m):.1f} deg, t3={math.degrees(t3_m):.1f} deg")
print(f"  Measured PWM: s3=1000")
print(f"  Code computes: s3 = {ELBOW_OFF} - {math.degrees(t3_m):.1f}*636.62/57.3 = {int(ELBOW_OFF - t3_m*ELBOW_SCALE)}")
print(f"  Deviation: {int(ELBOW_OFF - t3_m*ELBOW_SCALE) - 1000} PWM")
# Back-calc what scale would match measured
scale_needed = (ELBOW_OFF - 1000) / t3_m
print(f"  Scale needed to match measured: {scale_needed:.1f} (vs code 636.62)")

# Issue 3: s4
print("\n[Issue 3: Wrist s4 formula]")
print(f"  Computed t4 = {math.degrees(t4_m):.1f} deg")
print(f"  s4 = {WRIST_OFF} + {math.degrees(t4_m):.1f}*636.62/57.3 = {int(WRIST_OFF + t4_m*WRIST_SCALE)}")
print(f"  Measured s4 = 1500")
print(f"  t4 = -(t2+t3) - pi/2 always produces angles << -90 deg for typical reaches")

# Issue 4: Link length effect
print("\n[Issue 4: Link length sensitivity]")
print(f"  Current uncommitted: L1={L1}, L2={L2}")
print(f"  Committed:          L1=105, L2=100")
# Show how much difference L1 makes
for (l1, l2, tag) in [(105, 100, "committed"), (60, 135, "current-uncommitted")]:
    old_L1, old_L2 = L1, L2  # hack for analysis
    # We can't easily change globals, just note
    print(f"  {tag}: L1={l1}, L2={l2}")

print("\n" + "="*65)
print("ROOT CAUSE SUMMARY")
print("="*65)
print("""
The systematic error comes from 4 compounding issues:

1. SHOULDER SCALE (s2): Using 254.65 (800/pi from legacy IK_RadToPWM)
   instead of 424.41 (standard 270deg servo). This makes s2 change
   too slowly with t2 angle, compressing the PWM range to ~1/2 of needed.

2. SHOULDER OFFSET (s2): offset=500 sets t2=0(horizontal) at PWM=500.
   With the small scale, s2 quickly hits SERVO_MIN=500 for any t2<0,
   making the arm unable to reach below shoulder height.

3. WRIST FORMULA (s4): t4 = -(t2+t3) - pi/2 produces angles of -150 to -180 deg
   for most positions, exceeding 180deg servo range. PWM always clamps to 500.

4. LINK LENGTHS: L1=60, L2=135 (uncommitted) vs L1=105, L2=100 (committed)
   significantly changes t2, t3 decomposition, cascading to all PWMs.

Fix strategy: calibrate each joint independently with physical measurements
rather than deriving all formulas from geometry alone.
""")
