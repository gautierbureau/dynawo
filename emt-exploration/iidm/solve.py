#!/usr/bin/env python3
"""Solve the SMIB IIDM load flow (pypowsybl) and print the EMT init values.

The EMT cases need a *solved* operating point: bus v/angle and injector p/q.
This computes it from the topology + setpoints in smib.xiidm, with the slack
forced at the infinite bus so power flows generator -> tfo -> line -> infinite
bus. It then derives the per-node EMT start values (peak abc voltages, branch
currents) and the machine's receptor/RMS (P0,Q0,U0,UPhase0), which map directly
onto Dynawo's IIDM references p_pu/q_pu/v_pu/angle_pu.
"""
import pypowsybl as pp, pypowsybl.loadflow as lf, math, cmath

n = pp.network.load('smib.xiidm')
lf.run_ac(n, lf.Parameters(distributed_slack=False,
    provider_parameters={'slackBusSelectionMode': 'NAME', 'slackBusesIds': 'VL_INF_0'}))
b = n.get_buses(); g = n.get_generators()
tfo = n.get_2_windings_transformers().loc['TR']; line = n.get_lines().loc['LINE']

def bus(v):
    r = b.loc[v + '_0']; return r['v_mag'], math.radians(r['v_angle'])
vg, ag = bus('VL_GEN'); vh, ah = bus('VL_HV'); vi, ai = bus('VL_INF')

def abc(vrms, ang):                      # peak phase voltages at t = 0
    vp = vrms * math.sqrt(2); return [vp * math.cos(ang - k * 2 * math.pi / 3) for k in range(3)]
def branch_I(p, q, vrms, ang):           # peak phase currents at t = 0 (S = 1.5 Vpk conj(Ipk))
    S = complex(p / 100, q / 100); V = cmath.rect(vrms * math.sqrt(2), ang)
    Ip = (S / (1.5 * V)).conjugate(); m = abs(Ip); a0 = cmath.phase(Ip)
    return [m * math.cos(a0 - k * 2 * math.pi / 3) for k in range(3)]

print(f"machine (receptor/RMS): P0Pu={g.loc['SM','p']/100:+.6f} Q0Pu={g.loc['SM','q']/100:+.6f} "
      f"U0Pu={vg:.6f} UPhase0={ag:+.6f}")
print("capGen U0[abc]:", [round(x, 9) for x in abc(vg, ag)])
print("capBus U0[abc]:", [round(x, 9) for x in abc(vh, ah)])
print("tfo  I0[abc]  :", [round(x, 9) for x in branch_I(tfo['p1'], tfo['q1'], vg, ag)])
print("line I0[abc]  :", [round(x, 9) for x in branch_I(line['p1'], line['q1'], vh, ah)])
print(f"infBus UPeakPu={vi*math.sqrt(2):.6f} Phase0={ai:+.6f}")

# --- emit a Dynawo-ready IIDM: drop the infinite-bus slack generator (the EMT
#     VoltageSource overrides INF_BUS directly) and the slackTerminal/reference
#     extension blocks Dynawo does not need ---
import re
n.save('smib_solved.xiidm', format='XIIDM', parameters={'iidm.export.xml.version': '1.5'})
raw = open('smib_solved.xiidm').read()
raw = re.sub(r'[ \t]*<iidm:generator id="INF".*?</iidm:generator>\n', '', raw, flags=re.DOTALL)
raw = re.sub(r'[ \t]*<iidm:extension\b.*?</iidm:extension>\n', '', raw, flags=re.DOTALL)
raw = re.sub(r'\s+xmlns:(slt|reft)="[^"]*"', '', raw)

# Per-unit convention bridge: pypowsybl stores branch r/x in SINGLE-LINE pu (P = V*I),
# but the EMT abc models use the THREE-PHASE instantaneous convention (P = sum(v_k*i_k) =
# (3/2)*Vpk*Ipk = 3*Vrms*Irms -- the same 1.5*Vpk factor branch_I uses above). For an
# identical voltage drop the EMT per-phase current is 1/3 of the single-line current, so the
# EMT per-unit impedance is 3x the single-line one. Scale r/x by 3 on the line/transformer of
# the Dynawo-ready IIDM so that Dynawo's r_pu/x_pu references resolve directly to the EMT-base
# reactance (no manual factor in the .par). The canonical smib(_solved).xiidm keep the physical
# single-line values.
def scale_rx(m):
    head = m.group(0)
    head = re.sub(r'(\br=")([-+0-9.eE]+)(")', lambda a: a.group(1) + repr(float(a.group(2)) * 3.0) + a.group(3), head)
    head = re.sub(r'(\bx=")([-+0-9.eE]+)(")', lambda a: a.group(1) + repr(float(a.group(2)) * 3.0) + a.group(3), head)
    return head
raw = re.sub(r'<iidm:(line|twoWindingsTransformer)\b[^>]*', scale_rx, raw)
open('smib_dynawo.xiidm', 'w').write(raw)
print("wrote smib_dynawo.xiidm (Dynawo-ready; branch r/x scaled x3 single-line->EMT 3-phase pu)")
