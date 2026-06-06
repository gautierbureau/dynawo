within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAC5A "CGMES vendor variant of IEEE AC5A (CGMES ExcAC5A)"
  /*
    Vendor variant of the IEEE AC5A simplified rotating brushless
    excitation system. Parameter set is identical to the IEEE version
    (Ac5a) - this is a name alias for the CGMES vendor namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac5a;

  annotation(preferredView = "text");
end ExcAC5A;
