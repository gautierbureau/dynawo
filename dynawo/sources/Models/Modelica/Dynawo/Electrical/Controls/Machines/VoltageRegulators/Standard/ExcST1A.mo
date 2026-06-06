within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcST1A "CGMES vendor variant of IEEE St1a (CGMES ExcST1A)"
  /*
    Vendor variant of the IEEE St1a excitation system. Identical parameter
    set and topology - this is a name alias for the CGMES vendor namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.St1a;

  annotation(preferredView = "text");
end ExcST1A;
