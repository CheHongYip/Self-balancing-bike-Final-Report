"""
LQR Gain Calculator for Self-Balancing Bicycle Controller
Computes optimal gains for hardcoding into ESP32 Arduino sketch
"""

import numpy as np
import control
import matplotlib.pyplot as plt

# ============================================================================
# PHYSICAL PARAMETERS (from param module)
# ============================================================================

# Geometric parameters
L1 = 0.55      # Distance from lean pivot (tyre contact) to bicycle CoM (m)
L2 = 0.22      # Distance from lean pivot to reaction wheel CoM (m)

# Mass parameters
m1 = 12        # Bicycle + rider effective mass (kg)
m2 = 5         # Reaction wheel mass (kg)
# I1_cm: bicycle body moment of inertia about its own CoM
# Modelled as a uniform rod of total height 2*L1 rotating about its centre:
#   I_cm = (1/12)*m*(2*L1)^2 = (1/3)*m*L1^2
I1_cm = (1/3) * m1 * L1**2   # Bicycle CM inertia (kg·m²)

# Physical constants
g = 9.81       # Gravitational acceleration (m/s²)
dp = 0.01      # Bicycle lean viscous damping coefficient (N·m·s/rad)

# Motor / reaction-wheel parameters
r_rw = 0.09    # Reaction wheel radius (m)
J  = 0.03126   # Reaction wheel moment of inertia about spin axis (kg·m²)
Ng = 1         # Gear ratio (motor shaft to wheel shaft, 1 = direct drive)
ke = 1.019     # Back-EMF constant (V·s/rad)
kt = 0.882     # Motor torque constant (N·m/A)
R  = 0.46      # Motor winding resistance (Ω)
B  = 0.049     # Motor viscous damping at wheel shaft (N·m·s/rad)


# ============================================================================
# LQR TUNING WEIGHTS --- TUNE THESE ---
# ============================================================================

# Q matrix penalizes states: [qp (angle), qp_d (angular velocity), qr (wheel angle), qr_d (wheel velocity)]
Q_LQR = np.diag([
    100.0,    # qp: pendulum angle error (rad) - HIGH priority
    50.0,     # qp_d: pendulum angular velocity (rad/s) - MEDIUM priority
    1.0,      # qr: reaction wheel angle (rad) - LOW priority  
    10.0       # qr_d: reaction wheel angular velocity (rad/s) - LOW priority
])

R_LQR = np.array([[1000.0]])  # Control effort penalty (Vin voltage) - keep small for aggressive control

# Cross term (typically zero unless coupling is desired)
N_LQR = np.zeros((4, 1))

print("=" * 60)
print("LQR GAIN COMPUTATION FOR SELF-BALANCING BICYCLE")
print("=" * 60)

# ============================================================================
# STATE-SPACE MATRIX CONSTRUCTION
# ============================================================================

# ── Effective lean inertia about the pivot (parallel-axis theorem) ──────────
# Total = (bicycle CM inertia) + (m1 offset) + (reaction wheel mass offset)
a = I1_cm + m1 * L1**2 + m2 * L2**2   # [kg·m²]

# Gravitational torque coefficient (both masses above pivot → destabilising)
b = (m1 * L1 + m2 * L2) * g           # [N·m/rad]

# ── Combined motor back-EMF + friction damping ───────────────────────────────
# Motor torque: τ = (kt*Ng/R)*Vin - motor_damp/R * qr_d
# (B captures viscous friction at the wheel shaft)
motor_damp = kt * ke * Ng**2 + B * R  # [N·m·s/rad]

# ── State-matrix coefficients (derived via Lagrangian, qr = relative angle) ──
a21 =  b / a                               # θ̈ / θ    (gravity, destabilising)
a22 = -dp / a                              # θ̈ / θ̇   (lean damping) ← was missing
a24 =  motor_damp / (a * R)               # θ̈ / φ̇   (motor back-EMF on lean)
a41 = -b / a                               # φ̈ / θ    (gravity coupling to wheel)
a42 =  dp / a                              # φ̈ / θ̇   (damping coupling) ← was missing
a44 = -(a + J) * motor_damp / (a * J * R) # φ̈ / φ̇   (motor + friction damping)

b2  = -(kt * Ng) / (a * R)                # θ̈ / Vin
b4  =  (a + J) * (kt * Ng) / (a * J * R)  # φ̈ / Vin

# Construct state-space matrices
# States: x = [qp, qp_d, qr, qr_d]  (lean angle, lean rate, wheel rel. angle, wheel rel. vel.)
A_matrix = np.array([
    [  0,   1,   0,    0  ],
    [a21, a22,   0,  a24  ],   # a22 = lean damping (dp); a24 = back-EMF reaction on lean
    [  0,   0,   0,    1  ],
    [a41, a42,   0,  a44  ],   # a42 = damping coupling; a44 = motor + friction damping
])

B_matrix = np.array([
    [ 0 ],
    [ b2],
    [ 0 ],
    [ b4],
])

print("\n--- Physical Intermediate Values ---")
print(f"  Lean inertia a   = {a:.5f} kg.m^2")
print(f"  Gravity coeff b  = {b:.5f} N.m/rad")
print(f"  motor_damp coeff = {motor_damp:.5f} N.m.s/rad  (back-EMF + B)")
print("\n--- State-Space Matrices ---")
print("\nA_matrix (System Dynamics):")
print(np.array2string(A_matrix, precision=6, suppress_small=True))
print("\nB_matrix (Input Matrix):")
print(np.array2string(B_matrix, precision=6, suppress_small=True))

# ============================================================================
# CONTROLLABILITY & OBSERVABILITY CHECK  (must pass before LQR is meaningful)
# ============================================================================

print("\n--- System Properties ---")
ctrb_rank = np.linalg.matrix_rank(control.ctrb(A_matrix, B_matrix))
is_controllable = (ctrb_rank == 4)
print(f"  Controllability rank: {ctrb_rank}/4  ->  {'CONTROLLABLE [OK]' if is_controllable else 'NOT CONTROLLABLE [FAIL]'}")

# Realistic C: IMU gives lean angle + lean rate only
C_sense = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0]])
obsv_rank = np.linalg.matrix_rank(control.obsv(A_matrix, C_sense))
is_observable = (obsv_rank == 4)
print(f"  Observability rank:   {obsv_rank}/4  (C = lean angle + lean rate)  ->  {'OBSERVABLE [OK]' if is_observable else 'NOT FULLY OBSERVABLE [WARN] - consider encoder feedback'}")

if not is_controllable:
    print("\n[FATAL] System not controllable -- check physical parameters. Aborting.")
    raise SystemExit(1)

# ============================================================================
# LQR SOLVE
# ============================================================================

print("\n--- LQR Solution ---")
print(f"Q_LQR diagonal: [{Q_LQR[0,0]:.2f}, {Q_LQR[1,1]:.2f}, {Q_LQR[2,2]:.2f}, {Q_LQR[3,3]:.2f}]")
print(f"R_LQR: {R_LQR[0,0]:.4f}")

# Compute optimal gain matrix
K, S, E = control.lqr(A_matrix, B_matrix, Q_LQR, R_LQR, N_LQR)

print("\n--- Full K Matrix ---")
print(np.array2string(K, precision=6, suppress_small=True))

# Extract all four gains for Arduino
K00 = K[0, 0]   # Lean angle gain           (qp)
K01 = K[0, 1]   # Lean rate gain            (qp_d)
K02 = K[0, 2]   # Wheel relative angle gain (qr)   — often small but keeps wheel wound
K03 = K[0, 3]   # Wheel velocity gain       (qr_d) — critical for wheel damping

print("\n--- Gains for Arduino ---")
print(f"  K00 (lean angle):          {K00:.6f}")
print(f"  K01 (lean rate):           {K01:.6f}")
print(f"  K02 (wheel rel. angle):    {K02:.6f}")
print(f"  K03 (wheel rel. velocity): {K03:.6f}")

# ============================================================================
# STABILITY VERIFICATION
# ============================================================================

print("\n--- Stability Analysis ---")

# Compute closed-loop system matrix
A_cl = A_matrix - B_matrix @ K
closed_loop_poles = np.linalg.eigvals(A_cl)

print("\nClosed-loop poles (x = [qp, qp_d, qr, qr_d]):")
print(f"  {'#':>2}  {'Real':>12}  {'Imag':>12}  {'|wn| rad/s':>12}  {'Damp z':>9}  Status")
for i, pole in enumerate(closed_loop_poles):
    real_part = np.real(pole)
    imag_part = np.imag(pole)
    wn   = abs(pole)
    zeta = (-real_part / wn) if wn > 1e-10 else float('nan')
    status = "STABLE" if real_part < 0 else ("MARGINAL" if real_part == 0 else "UNSTABLE")
    print(f"  {i+1:>2}  {real_part:>+12.5f}  {imag_part:>+12.5f}j  {wn:>12.5f}  {zeta:>9.4f}  {status}")

# Check stability (strictly negative real parts)
if np.all(np.real(closed_loop_poles) < 0):
    print("\n[OK] SYSTEM IS STABLE: All poles have strictly negative real parts")
else:
    print("\n[WARNING] System is UNSTABLE. Re-tune Q_LQR and R_LQR before deploying.")
    unstable_poles = [p for p in closed_loop_poles if np.real(p) >= 0]
    print(f"  Found {len(unstable_poles)} pole(s) with non-negative real parts")

# ============================================================================
# ARDUINO-READY OUTPUT
# ============================================================================

print("\n" + "=" * 60)
print("ARDUINO CONSTANTS (Copy these into your sketch)")
print("=" * 60)
print(f"\n// LQR gains -- state: [qp, qp_d, qr, qr_d] (all in radians / rad/s)")
print(f"#define K00 {K00:.6f}f   // Lean angle gain          (qp)")
print(f"#define K01 {K01:.6f}f   // Lean rate gain            (qp_d)")
print(f"#define K02 {K02:.6f}f   // Wheel relative angle gain (qr)")
print(f"#define K03 {K03:.6f}f   // Wheel relative velocity gain (qr_d)")
print(f"\n// Full K row: K = [{K[0,0]:.6f}, {K[0,1]:.6f}, {K[0,2]:.6f}, {K[0,3]:.6f}]")

# ============================================================================
# CONTROL AUTHORITY CHECK
# ============================================================================

print("\n--- Control Authority Check ---")
# Worst-case: 3° lean, 5°/s lean rate, wheel at rest
worst_case_state = np.array([[np.deg2rad(3)], [np.deg2rad(5)], [0.0], [0.0]])
max_Vin = float(np.abs(K @ worst_case_state))
print(f"  |Vin| at 3 deg tilt + 5 deg/s lean rate: {max_Vin:.2f} V")
V_SUPPLY = 48.0   # <-- Set to your actual motor supply voltage
if max_Vin > V_SUPPLY:
    print(f"  [WARNING] Vin ({max_Vin:.2f} V) > supply ({V_SUPPLY} V). Increase R_LQR or reduce Q weights.")
else:
    print(f"  [OK] Vin within supply voltage ({V_SUPPLY} V).")

print("\n" + "=" * 60)
print("ESP32 control law (implement in your loop):")
print("  float Vin = -(K00*qp + K01*qp_d + K02*qr + K03*qr_d);")
print("  Vin = constrain(Vin, -V_SUPPLY, V_SUPPLY); // clamp to supply rail")
print("Ensure all states are in SI units (rad, rad/s).")
print("=" * 60)