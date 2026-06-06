within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAC2C "CGMES vendor variant of IEEE AC2C - revised AC2A (CGMES ExcAC2C)"
  /*
    CGMES 2024 vendor name for the IEEE 421.5-2016 AC2C high-initial-
    response alternator-rectifier excitation system, the C revision of
    AC2A. The block-view dynamics that matter for IEC 61970-302
    catalog mapping (Ka regulator, Kb second-stage gain, Kh exciter
    field-current feedback, exciter integrator with saturation, Kf
    rate feedback, rectifier reg correction Kc / Kd) are the same as
    the existing Ac2a model so this thin wrapper exposes the CGMES
    catalog name without duplicating the implementation. Use a
    purpose-built ExcAC2C only if a dataset exercises AC2C-specific
    2016 features that the Ac2a topology does not capture.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac2a;

  annotation(preferredView = "text");
end ExcAC2C;
