within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcST2A "CGMES vendor variant of IEEE ST2A (CGMES ExcST2A)"
  /*
    Vendor variant of the IEEE ST2A compound-source rectifier excitation
    system. Parameter set is identical to the IEEE version (St2a) - this
    is a name alias for the CGMES vendor namespace. UEL position is fixed
    at zero (the IEEE ST2A uelin field is absent in the vendor variant).
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.St2a(
    PositionUel = 0);

  annotation(preferredView = "text");
end ExcST2A;
