# Classical EMT models Dynawo does not have yet

Scope: a catalogue of the models a full electromagnetic-transient (EMT) tool
(EMTP / EMTP-RV / PSCAD class) carries that the Dynawo EMT library does **not**
have today, organised by category and by the *study* each one enables. It is a
roadmap, not a commitment: each entry says what the model is, which study needs
it, what stand-in (if any) exists in Dynawo now, whether a phasor analogue exists
(so an EMT port may be feasible, as was done for the synchronous machine), and a
rough priority.

The companion notes are `EMT_methods.tex` (the DAE-vs-companion formulation) and
`EMT_NETWORK_PARITY.md` (the C++ network vs the phasor network). This file is
about *model coverage*, not solver method or network parity.

## What the Dynawo EMT library has today

Passive: `Resistor`, `Inductor`, `Capacitor`, `SeriesRL`, `CoupledRL` (mutual
3×3), `Bus` (shunt-C node), `Ground`. Switching: `Breaker` / ideal switch.
Branches: lumped R–L only (no shunt C distribution, no travelling wave).
Transformers: `TransformerYgYg`, `TransformerYgD` (ideal ratio + leakage R–L, **no
magnetising / saturation branch**). Sources: `VoltageSource`, `InfiniteBus`.
Faults: `Fault` (phase–ground), `LineToLineFault`, `LineFault`. Machines:
`SynchronousMachine` (2-axis), `SynchronousMachineFull` / `GeneratorSynchronous`
(full d/f/D, q/Q1/Q2, Canay mutual, Shackshaft saturation). C++ network
(`DYNModelNetworkEMT`): constant-Z load, constant-PQ injector, shunt compensator,
frozen-B SVC, decoupled-PQ HVDC, dangling line, current-limit automaton.

Everything below is **absent**.

---

## A. Transmission lines & cables  (the single biggest classical-EMT gap)

Dynawo EMT represents a line only as a lumped R–L (or mutual `CoupledRL`) branch:
no shunt capacitance distribution and, crucially, **no wave propagation delay**.
That is adequate for short lines / fundamental dynamics but wrong for surges,
energisation, TRV, lightning and high-order harmonic resonances.

| Missing model | What / why | Stand-in today | Priority |
|---|---|---|---|
| Constant-parameter distributed (Bergeron / CP) line | travelling-wave, lossless or with lumped loss; propagation delay τ | lumped R–L (no delay) | **High** |
| Frequency-dependent line (J. Martí, FD-mode) | per-mode frequency-dependent Z_c(ω), A(ω); accurate switching/lightning | none | **High** |
| Wideband / universal line & cable (vector-fitted) | full freq-dependent phase-domain, unbalanced/untransposed | none | Med |
| Multi-conductor cable (coaxial / pipe-type) | core/sheath/armour, bonding, induced sheath currents | none | Med |
| Nominal-π / exact-π / cascaded-π section | lumped line *with* shunt C (and optional cascade) | R–L only | **High** (cheap) |
| Line/cable constants (geometry → Z,Y, modal transform) | builds the above from tower/cable geometry | none | Med |

Note: the Bergeron (constant-parameter travelling-wave) and wideband line models
are the classical EMTP *companion* form, not a Dynawo DAE model — see
`EMT_methods.tex` §9 for how they contrast with the DAE line of §7.

---

## B. Rotating machines  (you asked specifically)

We have synchronous machines only. The classical EMT machine library is wider;
the gaps matter for motor-dominated, converter-interfaced and torsional studies.
See the machine deep-dive in §J.

| Missing model | What / why | Phasor analogue in Dynawo? | Priority |
|---|---|---|---|
| Induction machine (single-cage) | motor start/stall/reaccel, dynamic load | **none** (new model) | **High** |
| Induction machine (double-cage / deep-bar) | accurate starting torque & rotor freq dependence | none | Med |
| Wound-rotor / DFIG rotor circuit | type-3 wind, slip-ring machines | partial (Wind IEC, phasor) | Med |
| PMSM | full-converter drivetrains, direct-drive wind | none | Low–Med |
| Multi-mass shaft / torsional model | SSR, sub-synchronous torsional interaction (SSTI) | none | **High** for SSR |
| Universal / generalised machine | one model, many machine types | none | Low |
| DC machine | traction, legacy drives | none | Low |
| Synchronous-machine extras | explicit hysteresis, extra damper circuits, frequency-dependent rotor | partial (we have saturation) | Low |

---

## C. Power electronics & converters  (the modern-EMT core)

Dynawo has these only as **phasor / averaged** RMS models (HVDC `HvdcPV`/`HvdcVsc`,
SVC `SVarCPV…`, Wind IEC, PV WECC). A true EMT study needs switched or
averaged-EMT converters with their inner controls — a substantial new class.

| Missing model | What / why | Phasor analogue? | Priority |
|---|---|---|---|
| Switch devices: diode, thyristor (+snubber/firing), GTO, IGBT/MOSFET (+anti-parallel diode) | the building blocks of every converter | none (EMT) | **High** |
| Line-commutated converter (6/12-pulse bridge) | LCC-HVDC, large drives, harmonic source | HVDC phasor | Med |
| 2-level / 3-level (NPC) VSC | STATCOM, drives, small HVDC | VSC phasor | **High** |
| MMC (modular multilevel) — detailed & averaged (CIGRE Type 4–7) | modern HVDC/FACTS; arm/submodule dynamics | none (EMT) | **High** |
| Averaged-value converter (AVM) | faster EMT when switching detail not needed | partial | Med |
| Detailed HVDC (LCC & VSC/MMC) with controls | DC faults, commutation failure, control interaction | HVDC phasor | **High** |
| FACTS: STATCOM, TCSC, SSSC, UPFC, detailed SVC (TCR/TSC) | sub-cycle FACTS behaviour, harmonics | SVC phasor | Med |
| Grid-forming / grid-following IBR (EMT) | inverter-based-resource stability, PLL/PWM, fault ride-through | Wind/PV phasor | **High** |

---

## D. Nonlinear & saturable elements  (inrush, ferroresonance, GIC, harmonics)

| Missing model | What / why | Stand-in | Priority |
|---|---|---|---|
| Saturable transformer (magnetising branch + saturation/hysteresis) | inrush, ferroresonance, GIC half-cycle saturation, harmonic generation | linear leakage only | **High** |
| Nonlinear / saturable reactor | shunt-reactor saturation, ferroresonance | linear `Inductor` | Med |
| Surge arrester / ZnO metal-oxide varistor (MOV) | nonlinear V–I clamp, energy duty; every surge study | none | **High** |
| Hysteresis model (Jiles–Atherton / preisach / piecewise) | residual flux, inrush asymmetry | none | Med |
| Nonlinear (piecewise) resistor | generic nonlinearity, gaps | none | Low |
| Spark gap / protective gap | flashover, crowbar | none | Low |

---

## E. Harmonic-study models  (you asked specifically)

See the harmonic deep-dive in §K. EMT harmonic studies need *sources* of
harmonics, accurate *frequency-dependent* network representation, and the
nonlinear *loads* that create distortion.

| Missing model | What / why | Stand-in | Priority |
|---|---|---|---|
| Harmonic voltage/current source (multi-tone, spectrum) | inject a measured/standard spectrum | single-frequency source only | **High** |
| Nonlinear harmonic loads: arc furnace, rectifier/ASD load | the dominant real distortion sources | constant-Z load | Med |
| Frequency-Dependent Network Equivalent (FDNE, vector-fit) | accurate boundary impedance vs frequency | none | Med |
| Harmonic filter banks (single/double-tuned, C-type, damped) | mitigation; resonance interaction | buildable from R/L/C (no template) | Med |
| Frequency scan / harmonic-impedance scan | resonance identification (study feature, not a model) | none | Med |
| Distributed / FD lines | accurate high-order resonance (overlaps §A) | lumped R–L | **High** |

---

## F. Protection & instrument models

| Missing model | What / why | Stand-in | Priority |
|---|---|---|---|
| CT / VT / CCVT with core saturation | relay EMT, CT saturation, CCVT ferroresonance | none | Med |
| Detailed relays (distance, differential, overcurrent curves) | protection coordination in EMT | current-limit automaton | Med |
| Circuit-breaker arc model (Mayr/Cassie) + TRV | interruption, restrike, TRV withstand | ideal breaker | Med |

---

## G. Loads

| Missing model | What / why | Stand-in | Priority |
|---|---|---|---|
| Dynamic motor load (induction-equivalent) / composite ZIP+motor | realistic load recovery, stalling | constant-Z | **High** |
| Harmonic / nonlinear load | see §E | constant-Z | Med |

Note: a *voltage-/frequency-dependent ZIP* load is **not** a classical EMT
primitive — its defining variable is RMS `U`, which in EMT must be *measured*
(RMS/dq filter). It belongs to a measurement-based Modelica model, not a network
primitive (see `EMT_NETWORK_PARITY.md` §2). The motor-equivalent dynamic load
above *is* a proper EMT model and is the right way to get load dynamics.

---

## H. Sources & equivalents

| Missing model | What / why | Stand-in | Priority |
|---|---|---|---|
| Thévenin source behind sequence impedance (Z1/Z0) | short-circuit-equivalent grid feed | infinite bus behind small Rs | **High** (cheap) |
| Controlled V/I sources (signal-driven) | test benches, HIL-style injection | none | Med |
| FDNE (frequency-dependent equivalent) | see §E | none | Med |

---

## I. Grounding / insulation  (lightning & fast-front surge)

| Missing model | What / why | Stand-in | Priority |
|---|---|---|---|
| Lightning current source (Heidler / CIGRE waveform) | stroke injection | none | Med |
| Tower model (surge impedance, multistory) + footing R with soil ionisation | backflashover, GPR | ideal ground | Med |
| Insulator flashover (volt–time / leader) | flashover rate | none | Low |
| Corona model | attenuation/distortion of fast fronts | none | Low |

---

## J. Machine deep-dive (your question)

**What we have.** Three synchronous machines: `SynchronousMachine` (reduced
two-axis / subtransient), and `SynchronousMachineFull` / `GeneratorSynchronous`
(full equivalent circuit — d/f/D and q/Q1/Q2 windings, Canay differential leakage,
Shackshaft cross-saturation on the instantaneous air-gap flux, zero-sequence
winding, EMT stator transients `der(λ)/ω_N`). For synchronous-generator EMT
(faults, machine swing, unbalance, stator transients) this is already
full-detail.

**What classical EMT adds, and why you might need it:**

- **Induction machine** — *the* missing machine. Needed for motor starting/stall
  studies, dynamic (motor) loads, and as the rotor of a DFIG. No Dynawo analogue
  at all (phasor or EMT), so it is a genuinely new model. Single-cage first, then
  double-cage / deep-bar for accurate starting torque and rotor-frequency effects.
- **Multi-mass shaft (torsional) model** on the synchronous machine — couples the
  electrical torque to a spring–mass shaft (turbine stages, generator, exciter).
  Required for **sub-synchronous resonance (SSR)** and sub-synchronous torsional
  interaction with series caps or converters. Our machine has a single lumped
  inertia `2H`; the torsional modes need the multi-mass extension.
- **PMSM** — for direct-drive wind / full-converter drivetrains (though usually
  studied behind its converter). Lower priority unless converter-machine EMT is in
  scope.
- **Universal / DC machines** — completeness; low priority for transmission EMT.
- **Synchronous-machine refinements** — explicit magnetic *hysteresis* (residual
  flux for re-energisation), additional rotor damper branches, and
  frequency-dependent rotor impedance for very-fast-front accuracy.

**Recommendation.** For machine coverage the high-value additions are (1) the
**induction machine** (single- then double-cage) and (2) the **multi-mass shaft**
on the existing synchronous machine for SSR/torsional studies. Both are
classical, well-documented, and fit the existing Park-`dq0` + `der(λ)` EMT style
of `SynchronousMachineFull`.

---

## K. Harmonic-study deep-dive (your question)

EMT is a natural harmonic tool because it resolves the instantaneous waveform, but
a *harmonic study* needs three things we lack:

1. **Sources of harmonics.** Either explicit **multi-tone harmonic
   voltage/current sources** (inject a standard or measured spectrum) or the
   **nonlinear loads/converters** that physically create harmonics (rectifiers,
   ASDs, arc furnaces, LCC bridges, saturated transformers). Today the only EMT
   source is a single-frequency `VoltageSource`/`InfiniteBus`.
2. **Frequency-accurate network.** Harmonic resonances live at hundreds–thousands
   of Hz, where the **distributed / frequency-dependent line** (§A) and the
   **saturable transformer** (§D) dominate the impedance. Lumped R–L lines and
   linear transformers misplace resonance peaks. These two are the highest-value
   harmonic enablers and overlap the surge gaps.
3. **A boundary equivalent and a way to read impedance.** A **FDNE**
   (vector-fitted frequency-dependent equivalent) for the external grid, and a
   **frequency-scan / harmonic-impedance** capability to find resonances. Filter
   bank templates (single/double-tuned, C-type, damped) round out mitigation work.

**Recommendation.** The harmonic must-haves are the **distributed/FD line** and
the **saturable transformer** (shared with surge studies), plus a **harmonic
current/voltage source**. With those three, EMT harmonic-injection studies become
possible; FDNE, frequency-scan and filter templates are follow-ups.

---

## L. Suggested priorities (cross-cutting)

Ranked by value × breadth of studies unlocked, cheapest-first within a tier:

1. **Nominal-π line with shunt C** and a **Thévenin-behind-Z source** — small,
   immediately improve every existing case.
2. **Distributed (Bergeron) + frequency-dependent line** — unlocks switching
   surges, TRV, lightning and harmonic resonance; the biggest single gap.
3. **Saturable transformer (magnetising + hysteresis)** and **surge arrester
   (MOV)** — inrush, ferroresonance, GIC, every surge study.
4. **Induction machine** (single- then double-cage) and **multi-mass shaft** on
   the synchronous machine — motor and SSR/torsional studies.
5. **Converter/IBR EMT** (switch devices → VSC/MMC → grid-forming/following) —
   the largest effort; the modern inverter-dominated-grid studies.
6. **Harmonic sources / FDNE / frequency-scan** and **protection/instrument
   (CT/VT saturation, arc breaker)** — specialised study support.

None of these is required for the EMT *network parity* already achieved; they
extend the EMT library toward the full classical-EMT study set (switching surge,
lightning, SSR/torsional, harmonics, ferroresonance/inrush, power-electronic and
inverter-dominated systems).
