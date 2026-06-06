within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcST2C "CGMES vendor variant of IEEE ST2C - revised ST2A (CGMES ExcST2C)"
  /*
    CGMES 2024 vendor variant naming the IEEE 421.5-2016 ST2C
    compound-source rectifier excitation system, which is the C
    revision of ST2A. The block-view dynamics that matter for IEC
    61970-302 catalog mapping are the same Ka / (1 + sTa) regulator
    feeding the normalised exciter integrator scaled by the compound-
    source ceiling Vb = Kp*Us + Ki*Ir already implemented in the
    St2a model. UEL position is fixed at zero. Use a purpose-built
    ExcST2C only if a dataset exercises ST2C-specific 2016 features
    (e.g. additional pre-filter stages) that the St2a topology does
    not capture.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.St2a(
    PositionUel = 0);

  annotation(preferredView = "text");
end ExcST2C;
