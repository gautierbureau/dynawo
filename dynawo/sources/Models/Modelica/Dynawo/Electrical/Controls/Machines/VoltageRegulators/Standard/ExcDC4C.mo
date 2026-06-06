within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcDC4C "CGMES vendor variant of the IEEE Dc4 family (CGMES ExcDC4C)"
  /*
    Vendor variant of the IEEE Type-4 DC excitation system (PID-regulator
    DC commutator exciter). CGMES 2024 adds a 'C' alias; the only Dc4
    implementation in the library is Dc4b, which already covers the
    Dc4c parameter set (the C-variant in the CGMES 2024 namespace
    introduces no new dynamics versus Dc4b).
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Dc4b;

  annotation(preferredView = "text");
end ExcDC4C;
