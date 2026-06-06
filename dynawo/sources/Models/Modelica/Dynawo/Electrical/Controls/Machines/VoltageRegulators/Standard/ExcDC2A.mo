within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcDC2A "CGMES vendor variant of IEEE Dc2a (CGMES ExcDC2A)"
  /*
    Vendor variant of the IEEE Dc2a excitation system. Identical parameter
    set and topology - this is a name alias for the CGMES vendor namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Dc2a;

  annotation(preferredView = "text");
end ExcDC2A;
