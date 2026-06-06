within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAC2A "CGMES vendor variant of IEEE AC2A (CGMES ExcAC2A)"
  /*
    Vendor variant of the IEEE AC2A excitation system. Parameter set is
    identical to the IEEE version (Ac2a) - this is a name alias for the
    CGMES vendor namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac2a;

  annotation(preferredView = "text");
end ExcAC2A;
