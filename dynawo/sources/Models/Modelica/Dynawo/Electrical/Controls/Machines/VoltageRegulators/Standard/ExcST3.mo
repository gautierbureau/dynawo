within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcST3 "CGMES vendor variant of IEEE ST3A (CGMES ExcST3)"
  /*
    Vendor variant of the IEEE ST3A potential-or-compound-source thyristor
    excitation system. Parameter set is identical to the IEEE version
    (St3a) minus VgMax and uelin - both are bound to neutral defaults here
    (VgMax = 999, PositionUel = 0).
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.St3a(
    VgMaxPu = 999, PositionUel = 0);

  annotation(preferredView = "text");
end ExcST3;
