within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAC4C "CGMES vendor variant of IEEE AC4C - revised AC4A (CGMES ExcAC4C)"
  /*
    CGMES 2024 vendor name for the IEEE 421.5-2016 AC4C alternator-
    rectifier excitation system with high initial response, the C
    revision of AC4A. The block-view dynamics are already implemented
    by Ac4c (the 2016 C-variant lowercase model) - this thin wrapper
    exposes the CGMES catalog spelling without duplicating the
    implementation.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac4c;

  annotation(preferredView = "text");
end ExcAC4C;
