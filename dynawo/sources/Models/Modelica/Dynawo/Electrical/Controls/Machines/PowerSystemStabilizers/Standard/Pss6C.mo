within Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model Pss6C "IEEE power system stabilizer type 6C (CGMES Pss6C)"
  /*
    CGMES 2024 vendor-name alias (uppercase 'C') for the existing
    Dynawo Pss6c model. Topology: PGen path with low-pass filter,
    speed path with low-pass + derivative, sum with washout, four
    cascaded integrators with K0..K4 weights tapped on each
    integrator output, overall Ks gain, output limiter and a PGen-
    hysteresis activation switch. Identical dynamics to Pss6c; this
    thin wrapper exposes the CGMES catalog name (uppercase C) without
    duplicating the implementation.
  */
  extends Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard.Pss6c;

  annotation(preferredView = "text");
end Pss6C;
