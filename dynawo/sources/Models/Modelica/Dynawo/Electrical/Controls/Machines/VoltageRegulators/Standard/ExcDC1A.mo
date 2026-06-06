within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcDC1A "CGMES vendor variant of IEEE Dc1a (CGMES ExcDC1A)"
  /*
    Vendor variant of the IEEE Dc1a excitation system. Identical parameter
    set and topology - this is a name alias for the CGMES vendor namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Dc1a;

  annotation(preferredView = "text");
end ExcDC1A;
