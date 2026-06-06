within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAC3C "CGMES vendor variant of IEEE AC3C - revised AC3A (CGMES ExcAC3C)"
  /*
    CGMES 2024 vendor name for the IEEE 421.5-2016 AC3C self-excited
    alternator-rectifier excitation system, the C revision of AC3A.
    The block-view dynamics that matter for IEC 61970-302 catalog
    mapping (lead-lag Tb / Tc, Ka regulator with limits, Kn high-
    field-voltage gain reduction, Kr field-current minor-loop, self-
    excited Va * Ve product, AcRotatingExciter integrator with
    saturation and rectifier reg correction) are the same as the
    existing Ac3a model so this thin wrapper exposes the CGMES
    catalog name without duplicating the implementation.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac3a;

  annotation(preferredView = "text");
end ExcAC3C;
