within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAC5C "CGMES vendor variant of IEEE AC5C - revised AC5A (CGMES ExcAC5C)"
  /*
    CGMES 2024 vendor name for the IEEE 421.5-2016 AC5C simplified
    rotating brushless excitation system, the C revision of AC5A.
    The block-view dynamics that matter for IEC 61970-302 catalog
    mapping (Ka regulator, exciter integrator 1 / sTe with VeMin
    floor, Ke + AEx*exp(BEx*Ve) saturation, three-stage Kf rate
    feedback through Tf1 / Tf2 / Tf3) are the same as the existing
    Ac5a model so this thin wrapper exposes the CGMES catalog name
    without duplicating the implementation.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac5a;

  annotation(preferredView = "text");
end ExcAC5C;
