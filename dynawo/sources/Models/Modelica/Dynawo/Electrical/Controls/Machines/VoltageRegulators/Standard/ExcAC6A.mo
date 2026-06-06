within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAC6A "CGMES vendor variant of IEEE Ac6a (CGMES ExcAC6A)"
  /*
    Vendor variant of the IEEE Ac6a excitation system. Identical parameter
    set and topology - this is a name alias for the CGMES vendor namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac6a;

  annotation(preferredView = "text");
end ExcAC6A;
