#!/usr/bin/env python3
"""Snubber-consistent EMT init from the CLASSICAL load flow -- no re-LF.

This is the production-realistic version of solve_snub.py. Dynawo receives an
IIDM that already carries a classical, snubber-UNAWARE load flow and cannot add
components and re-run a load flow. But it does NOT need to: the EMT snubber
current at each node is the analytic quantity i = jwC*V, fully known from the
classical LF voltage and the model's own CPu. Closing KCL at t=0 is therefore a
small INTERNAL nodal correction

    solve the branch network (Ybus) + snubber shunts for the voltage shift that
    supplies i_snub, starting from the classical LF point,

using only data Dynawo already has at init time (network admittances + CPu).
No external load flow, no network modification.

This script demonstrates that correction on the SMIB case: it takes the
classical LF voltages (as carried by smib_dynawo.xiidm) and the branch/snubber
admittances, and produces exactly the snubber-consistent operating point that
solve_snub.py obtained by re-running the AC load flow -- proving the strategy is
portable to Dynawo's internal init.

The x3 on the snubber susceptance is the single-line <-> 3-phase-peak pu bridge:
EMT shunt admittance = single-line/3, so the single-line susceptance is 3*wC
(the inverse of the r/x *3 rule solve.py uses for series impedances).
"""
import cmath, math
import numpy as np

w = 2 * math.pi * 50.0
CPu = 1e-4                      # EmtBus snubber per-phase pu capacitance (= bus_CPu)
b_single = 3.0 * w * CPu       # single-line snubber susceptance (x3 pu bridge)

# ---- classical, snubber-unaware LF (what Dynawo receives in the IIDM) ----
Vg0 = cmath.rect(0.92,               math.radians(12.433850130739353))  # GEN (PV, |V| held by AVR)
Vh0 = cmath.rect(0.9129771049840923, math.radians(8.337508799355973))   # HV  (PQ, S = 0)
Vi  = cmath.rect(0.9, 0.0)                                               # INF (slack)
Pgen = 0.60                                                              # machine active injection

# ---- single-line branch admittances (physical smib.xiidm; EMT values are 3x) ----
y_tfo  = 1.0 / complex(0.0,  0.1)    # tfo  X = 0.1  (EMT 0.3)
y_line = 1.0 / complex(0.02, 0.2)    # line R,X = 0.02,0.2 (EMT 0.06,0.6)
ysh = complex(0.0, b_single)         # snubber shunt (capacitor: +jb)

# ---- internal nodal correction: solve KCL WITH snubbers from the classical LF ----
# Unknowns: GEN angle (|V| fixed by AVR), HV complex voltage. INF is the slack.
Y = np.array([[y_tfo + ysh,  -y_tfo              ],
              [-y_tfo,        y_tfo + y_line + ysh]], dtype=complex)
Yinf = np.array([0.0, -y_line], dtype=complex)      # coupling to the slack column

def mismatch(x):
    Vg = cmath.rect(0.92, x[0]); Vh = complex(x[1], x[2])
    I = Y @ np.array([Vg, Vh]) + Yinf * Vi
    Sg = Vg * np.conj(I[0]); Sh = Vh * np.conj(I[1])
    return np.array([Sg.real - Pgen, Sh.real, Sh.imag])   # GEN P ; HV P=0,Q=0

x = np.array([cmath.phase(Vg0), Vh0.real, Vh0.imag])      # seed from the classical LF
for _ in range(30):
    f = mismatch(x)
    J = np.column_stack([(mismatch(x + np.eye(3)[j] * 1e-7) - f) / 1e-7 for j in range(3)])
    x = x - np.linalg.solve(J, f)

Vg = cmath.rect(0.92, x[0]); Vh = complex(x[1], x[2])
SQ2 = math.sqrt(2.0)
print("snubber-consistent seeds from classical LF (no re-LF):")
print(f"  GEN: Upu={abs(Vg):.6f}  Theta={cmath.phase(Vg):+.6f} rad")
print(f"  HV : Upu={abs(Vh):.6f}  Theta={cmath.phase(Vh):+.6f} rad")
print("  --- par-ready peak-pu branch seeds ---")
print(f"  tfo_UpMagPu={abs(Vg)*SQ2:.6f} tfo_UpPhaseRad={cmath.phase(Vg):.6f}")
print(f"  tfo_UsMagPu={abs(Vh)*SQ2:.6f} tfo_UsPhaseRad={cmath.phase(Vh):.6f}")
print(f"  line_UpMagPu={abs(Vh)*SQ2:.6f} line_UpPhaseRad={cmath.phase(Vh):.6f}")
