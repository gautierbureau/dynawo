within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcDC3C "CGMES vendor variant of the IEEE Dc3 family (CGMES ExcDC3C)"
  /*
    Vendor variant of the IEEE Type-3 DC excitation system. CGMES 2024
    adds a 'C' alias for the manual-rheostat IEEE Dc3 family; the only
    Dc3 implementation in the library is Dc3a, which already covers the
    Dc3c parameter set (the C-variant in the CGMES 2024 namespace
    introduces no new dynamics versus Dc3a).
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Dc3a;

  annotation(preferredView = "text");
end ExcDC3C;
