within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAC4A "CGMES vendor variant of IEEE Ac4a (CGMES ExcAC4A)"
  /*
    Vendor variant of the IEEE Ac4a excitation system. Identical parameter
    set and topology - this is a name alias for the CGMES vendor namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac4a;

  annotation(preferredView = "text");
end ExcAC4A;
