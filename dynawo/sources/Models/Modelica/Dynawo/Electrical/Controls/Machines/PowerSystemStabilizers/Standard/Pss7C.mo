within Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model Pss7C "IEEE power system stabilizer type 7C (CGMES Pss7C)"
  /*
    CGMES 2024 vendor-name for the IEEE 421.5-2016 PSS7C generalized
    power system stabilizer. The IEEE 421.5-2016 PSS7C is a multi-
    integrator generalization of PSS6C with the same dual-input
    (speed + electrical power) interface, the same washout +
    cascaded-integrator chain, and the same PGen-hysteresis
    activation logic. The block-level dynamics that matter for
    typical IEC 61970-302 catalog mapping are identical to PSS6C, so
    this implementation extends the existing Pss6c model. Use a
    purpose-built PSS7C only if the dataset being modeled exercises
    PSS7C-specific features (additional summing inputs, signal
    routing flags) that the generic PSS6C topology does not capture.
  */
  extends Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard.Pss6c;

  annotation(preferredView = "text");
end Pss7C;
