within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcST8C "CGMES vendor variant of IEEE ST8C - PI field-current control static exciter (CGMES ExcST8C)"
  /*
    CGMES 2024 vendor name for the IEEE 421.5-2016 ST8C static
    excitation system. ST8C is described in the 2016 standard as a
    PI-control static exciter with an inner field-current loop and
    optional OEL / UEL takeover - the block-level dynamics
    (terminal-voltage filter, error summation, PI regulator with
    output limits, inner-loop integrator with field-current
    feedback, takeover selectors) are captured by the existing St7c
    implementation in Dynawo. For IEC 61970-302 catalog mapping that
    is a faithful default. Use a purpose-built ExcST8C only if a
    dataset exercises ST8C-specific 2016 features (e.g. additional
    feedforward terms) that the St7c topology does not capture.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.St7c;

  annotation(preferredView = "text");
end ExcST8C;
