within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcDC3A "CGMES vendor variant of IEEE DC3A (CGMES ExcDC3A)"
  /*
    Vendor variant of the IEEE DC3A non-continuously-acting dc commutator
    exciter. Parameter set is identical to the IEEE version (Dc3a) - this
    is a name alias for the CGMES vendor namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Dc3a;

  annotation(preferredView = "text");
end ExcDC3A;
