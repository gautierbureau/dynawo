# SMIB NRT Skeletons — May 2026 batch

This file tracks the NRT skeleton directories generated alongside the
~60 new control models added in the May 2026 batch (see
`DYNAWO_CGMES_MACHINE_CONTROLS.md` for the coverage matrix).

## Status

A skeleton is a `TestCase<Model>` directory containing:
- `TestCase<Model>.jobs` — solver / output configuration
- `TestCase<Model>.dyd` — placeholder dynamic model architecture
- `TestCase<Model>.par` — placeholder parameter set (empty `MAIN` set)
- `TestCase<Model>.crv` — placeholder curves list
- `TODO.md` — what to fill in and which existing NRT to use as a template

**A skeleton is NOT a working NRT.** It will not run as-is, and it is not
wired into `cases.py` so the NRT runner skips it. Each skeleton needs a
human to:

1. Replace the placeholder preassembled `lib` name with the real one
   (or build a new `GeneratorThirdOrder<Gov>` / `GeneratorSynchronous
   FourWindings<Stack>` preassembled if one does not exist yet).
2. Fill the `MAIN` parameter set with realistic values from the model's
   `.mo` parameter list (and the CGMES dataset of interest).
3. Wire the network (load, infinite bus, fault / step / line-opening
   stimulus, `DYNModelOmegaRef`) in the `.dyd` following the suggested
   template named in the skeleton's `TODO.md`.
4. Populate the `.crv` curve list with the outputs you want to verify.
5. Capture a passing reference into
   `reference/outputsTestCase<Model>/curves/curves.csv`.
6. Add the case to `cases.py`.

## Coverage gap addressed

Working NRTs (6) and skeletons (47) together cover the **53 new control
models** added in the May 2026 batch that did not previously have any
functional time-domain test.

### Working NRTs (filled in and runnable)

| NRT | Family | Reference rows |
|---|---|---|
| `Standard/TestCaseGovCT2`         | Gas turbine governor | 4051 |
| `Standard/TestCaseGovHydroIEEE2`  | IEEE hydro governor  | 158  |
| `Standard/TestCasePssIEEE4B`      | IEEE multi-band PSS  | 491  |
| `Standard/TestCasePssIEEE1A`      | IEEE single-input PSS| 247  |
| `Standard/TestCasePss1A`          | Vendor PSS           | 271  |
| `Standard/TestCasePss2ST`         | Vendor dual-input PSS| 1490 |
| `Standard/TestCaseGovGAST1`       | Gas turbine governor | 1133 |
| `Standard/TestCaseGovGAST2`       | Gas turbine governor | 1170 |
| `Standard/TestCaseGovGAST3`       | Gas turbine governor | 511  |
| `Standard/TestCaseGovGAST4`       | Gas turbine governor | 1095 |
| `Standard/TestCaseGovGASTWD`      | Woodward gas governor| 398  |
| `Standard/TestCaseGovHydroFrancis`| Francis hydro governor| 495 |
| `Standard/TestCaseGovHydroPID`    | PID hydro governor   | 150  |
| `Standard/TestCaseGovHydroPID2`   | Simple PID hydro gov | 109  |
| `Standard/TestCaseGovHydroPelton` | Pelton wheel governor| 151  |
| `Standard/TestCaseGovHydroR`      | HYDROGOVR PSS/E gov  | 127  |
| `Standard/TestCaseGovHydroWEH`    | Woodward electronic  | 87   |
| `Standard/TestCaseGovHydroWPID`   | Woodward PID hydro   | 116  |
| `Standard/TestCaseGovSteam0`      | Simple steam governor| 103  |
| `Standard/TestCaseGovSteamCC`     | Combined-cycle steam | 94   |
| `Standard/TestCaseGovSteamFV2`    | Fast-valving steam   | 95   |
| `Standard/TestCaseGovSteamFV3`    | Extended fast-valve  | 94   |
| `Standard/TestCaseGovSteamFV4`    | Detailed steam + tors| 153  |
| `Standard/TestCaseExcAVR1`        | European 2-pole AVR  | 957  |
| `Standard/TestCaseExcAVR2`        | European lead-lag AVR| 199  |
| `Standard/TestCaseExcAVR3`        | AVR with field ceil  | 246  |
| `Standard/TestCaseExcAVR4`        | AVR + rate feedback  | 157  |
| `Standard/TestCaseExcAVR5`        | Rotating exciter AVR | 151  |
| `Standard/TestCaseExcAVR7`        | 6-stage lead-lag AVR | 135  |
| `IEEE/PmConstAc5a`                | IEEE AC5A brushless  | 272  |
| `IEEE/PmConstDc3a`                | IEEE DC3A rheostat   | 181  |
| `IEEE/PmConstDc4b`                | IEEE DC4B PID exciter| 236  |
| `IEEE/PmConstSt2a`                | IEEE ST2A compound   | 140  |
| `IEEE/PmConstSt3a`                | IEEE ST3A static     | 249  |
| `IEEE/PmConstAc2a`                | IEEE AC2A high-resp  | 414  |
| `IEEE/PmConstAc3a`                | IEEE AC3A self-exc   | 246  |
| `Standard/TestCaseExcCZ`          | Czech AVR            | 271  |
| `Standard/TestCaseExcELIN1`       | Austrian ELIN type 1 | 184  |
| `Standard/TestCaseExcELIN2`       | Austrian ELIN type 2 | 184  |
| `Standard/TestCaseExcHU`          | Hungarian AVR        | 202  |
| `Standard/TestCaseExcNI`          | Generic NI AVR       | 242  |
| `Standard/TestCaseExcOEX3T`       | Brown Boveri OEX3T   | 178  |
| `Standard/TestCaseExcPIC`         | PIC multi-stage AVR  | 572  |
| `Standard/TestCaseExcREXS`        | REXS rotating AC     | 175  |
| `Standard/TestCaseExcRQB`         | RQB integrator AVR   | 253  |
| `Standard/TestCaseExcSK`          | Slovak SK reactive   | 162  |
| `Standard/TestCaseExcSYMPTR`      | Italian Symptr AVR   | 133  |
| `Standard/TestCaseOverexcLimIEEE` | IEEE OEL             | 73   |
| `Standard/TestCaseOverexcLimX`    | Extended Efd-based OEL| 138 |
| `Standard/TestCaseUnderexcLimIEEE1`| IEEE UEL type 1     | 261  |
| `Standard/TestCaseUnderexcLimIEEE2`| IEEE UEL type 2     | 160  |
| `Standard/TestCaseUnderexcLimX1`  | Extended UEL type 1  | 133  |
| `Standard/TestCaseUnderexcLimX2`  | Extended UEL type 2  | 149  |

### Skeleton NRTs (placeholders, not wired)

All 53 skeletons in the May-2026 batch have been promoted to working NRTs.

### Not in scope

The 16 CGMES vendor-name extends-wrappers (`ExcAC1A`..`ExcAC8B`,
`ExcDC1A`..`ExcDC3A`, `ExcST1A`..`ExcST7B`) are aliases for IEEE
counterparts and are exercised through the same code paths as their IEEE
twins — they do not need separate NRT cases.
