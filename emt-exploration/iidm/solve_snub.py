#!/usr/bin/env python3
"""PROTOTYPE: snubber-consistent EMT seed.

Same as solve.py, but BEFORE the load flow it adds, at every EMT-snubber node
(the EmtBus nodes GEN/HV, each CPu = 1e-4), a shunt whose susceptance equals the
snubber's, b_pu = omega*CPu (=> Q = omega*CPu*V^2 reactive). The load flow then
solves an operating point in which the branch currents already supply each node's
snubber reactive current, so KCL closes at t=0 and the EMT start is flat -- the
fix the iidm/README flagged. The composite case uses explicit EmtBus models and
ignores IIDM shunts, so the added shunt only shifts the exported v/angle that seed
the branch _INITs; there is no double-count.
"""
import pypowsybl as pp, pypowsybl.loadflow as lf, math, cmath, pandas as pd, re

SNREF = 100.0          # MVA (solve.py divides p/q by 100)
FNOM = 50.0
CPU = 1e-4             # EmtBus snubber per-unit capacitance (matches bus_CPu)
w = 2 * math.pi * FNOM
b_emt = w * CPU        # snubber susceptance in EMT 3-phase-peak pu (~0.0314)
# pu bridge: EMT shunt admittance = single-line/3 (the inverse of the series r/x *3
# rule solve.py uses), so the single-line susceptance the LF needs is 3x the EMT one.
b_pu = 3.0 * b_emt     # single-line pu susceptance fed to pypowsybl (~0.0942)

n = pp.network.load('smib.xiidm')

# --- add a snubber-matching shunt at each EmtBus node (GEN, HV) ---
vls = n.get_voltage_levels()
snub_nodes = [('SNUB_GEN', 'VL_GEN', 'GEN_BUS'), ('SNUB_HV', 'VL_HV', 'HV_BUS')]
sh, lin = [], []
for sid, vl, bbid in snub_nodes:
    vnom = vls.loc[vl, 'nominal_v']            # kV
    b_siemens = b_pu * SNREF / (vnom ** 2)     # b_pu -> Siemens (Q[MVAr]=b[S]*V[kV]^2)
    sh.append({'id': sid, 'voltage_level_id': vl, 'bus_id': bbid,
               'section_count': 1, 'target_v': vnom, 'target_deadband': 0., 'model_type': 'LINEAR'})
    lin.append({'id': sid, 'g_per_section': 0.0, 'b_per_section': b_siemens, 'max_section_count': 1})
n.create_shunt_compensators(pd.DataFrame.from_records(index='id', data=sh),
                            linear_model_df=pd.DataFrame.from_records(index='id', data=lin))
print(f"added snubber shunts b_pu={b_pu:.5f} at {[v for _, v, _ in snub_nodes]}")

lf.run_ac(n, lf.Parameters(distributed_slack=False,
    provider_parameters={'slackBusSelectionMode': 'NAME', 'slackBusesIds': 'VL_INF_0'}))
b = n.get_buses(); g = n.get_generators()
tfo = n.get_2_windings_transformers().loc['TR']; line = n.get_lines().loc['LINE']

def bus(v):
    r = b.loc[v + '_0']; return r['v_mag'], math.radians(r['v_angle'])
vg, ag = bus('VL_GEN'); vh, ah = bus('VL_HV'); vi, ai = bus('VL_INF')

def abc(vrms, ang):
    vp = vrms * math.sqrt(2); return [vp * math.cos(ang - k * 2 * math.pi / 3) for k in range(3)]
def branch_I(p, q, vrms, ang):
    S = complex(p / 100, q / 100); V = cmath.rect(vrms * math.sqrt(2), ang)
    Ip = (S / (1.5 * V)).conjugate(); m = abs(Ip); a0 = cmath.phase(Ip)
    return [m * math.cos(a0 - k * 2 * math.pi / 3) for k in range(3)]

print(f"machine: P0Pu={g.loc['SM','p']/100:+.6f} Q0Pu={g.loc['SM','q']/100:+.6f} U0Pu={vg:.6f} UPhase0={ag:+.6f}")
print("tfo  I0[abc]:", [round(x, 9) for x in branch_I(tfo['p1'], tfo['q1'], vg, ag)])
print("line I0[abc]:", [round(x, 9) for x in branch_I(line['p1'], line['q1'], vh, ah)])

SQ2 = math.sqrt(2)
print("--- par-ready peak-pu seeds (snubber-aware) ---")
print(f"tfo_UpMagPu={vg*SQ2:.6f} tfo_UpPhaseRad={ag:.6f}")
print(f"tfo_UsMagPu={vh*SQ2:.6f} tfo_UsPhaseRad={ah:.6f}")
print(f"line_UpMagPu={vh*SQ2:.6f} line_UpPhaseRad={ah:.6f}")
print(f"line_UnMagPu={vi*SQ2:.6f} line_UnPhaseRad={ai:.6f}")
print(f"source_UPeakPu={vi*SQ2:.9f} source_Phase0={ai:.6f}")
print(f"BUSGEN Upu={vg:.6f} Theta={ag:.6f} | BUSHV Upu={vh:.6f} Theta={ah:.6f}")

n.save('smib_solved_snub.xiidm', format='XIIDM', parameters={'iidm.export.xml.version': '1.5'})
raw = open('smib_solved_snub.xiidm').read()
raw = re.sub(r'[ \t]*<iidm:generator id="INF".*?</iidm:generator>\n', '', raw, flags=re.DOTALL)
raw = re.sub(r'[ \t]*<iidm:shunt id="SNUB_.*?</iidm:shunt>\n', '', raw, flags=re.DOTALL)
raw = re.sub(r'[ \t]*<iidm:extension\b.*?</iidm:extension>\n', '', raw, flags=re.DOTALL)
raw = re.sub(r'\s+xmlns:(slt|reft)="[^"]*"', '', raw)
def scale_rx(m):
    head = m.group(0)
    head = re.sub(r'(\br=")([-+0-9.eE]+)(")', lambda a: a.group(1) + repr(float(a.group(2)) * 3.0) + a.group(3), head)
    head = re.sub(r'(\bx=")([-+0-9.eE]+)(")', lambda a: a.group(1) + repr(float(a.group(2)) * 3.0) + a.group(3), head)
    return head
raw = re.sub(r'<iidm:(line|twoWindingsTransformer)\b[^>]*', scale_rx, raw)
open('smib_dynawo_snub.xiidm', 'w').write(raw)
print("wrote smib_dynawo_snub.xiidm (snubber-consistent operating point)")
