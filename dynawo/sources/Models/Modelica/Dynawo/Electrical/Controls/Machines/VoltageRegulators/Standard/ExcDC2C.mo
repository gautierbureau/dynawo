within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcDC2C "CGMES vendor variant of IEEE Dc2c (CGMES ExcDC2C)"
  /*
    Vendor variant of the IEEE Dc2c excitation system. Identical parameter
    set and topology - this is a name alias for the CGMES 2024 vendor
    namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Dc2c;

  annotation(preferredView = "text");
end ExcDC2C;
