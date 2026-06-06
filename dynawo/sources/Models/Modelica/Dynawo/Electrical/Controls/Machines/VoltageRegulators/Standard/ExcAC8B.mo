within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAC8B "CGMES vendor variant of IEEE Ac8b (CGMES ExcAC8B)"
  /*
    Vendor variant of the IEEE Ac8b excitation system. Identical parameter
    set and topology - this is a name alias for the CGMES vendor namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac8b;

  annotation(preferredView = "text");
end ExcAC8B;
