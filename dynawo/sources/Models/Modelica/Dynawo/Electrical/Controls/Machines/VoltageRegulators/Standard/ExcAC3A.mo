within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAC3A "CGMES vendor variant of IEEE AC3A (CGMES ExcAC3A)"
  /*
    Vendor variant of the IEEE AC3A self-excited alternator excitation
    system. Parameter set is identical to the IEEE version (Ac3a) minus
    the Kn / EfdN high-field gain reduction parameters; those are bound
    to neutral defaults (Kn = 1, EfdN = 999) here.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac3a(
    Kn = 1, EfdNPu = 999);

  annotation(preferredView = "text");
end ExcAC3A;
