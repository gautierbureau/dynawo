within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcST3C "CGMES vendor variant of IEEE ST3C - revised ST3A (CGMES ExcST3C)"
  /*
    CGMES 2024 vendor variant naming the IEEE 421.5-2016 ST3C static
    excitation system, which is the C revision of ST3A. The block-
    view dynamics that matter for IEC 61970-302 catalog mapping
    (outer lead-lag + Ka regulator, inner Km integrator, compound-
    source ceiling Vb = Kp*Us + Ki*Ir, rectifier reg Kc, output Kg
    feedback) are the same as the existing St3a implementation. UEL
    position is fixed at zero. Use a purpose-built ExcST3C only if a
    dataset exercises ST3C-specific 2016 additions that the St3a
    topology does not capture.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.St3a(
    PositionUel = 0);

  annotation(preferredView = "text");
end ExcST3C;
