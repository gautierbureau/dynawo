within Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model Pss5C "IEEE power system stabilizer type 5C (CGMES Pss5C)"
  /*
    CGMES 2024 vendor-name alias for the IEEE 421.5-2016 PSS5C dual-
    input (speed + electrical power) power system stabilizer. The
    topology is identical to Dynawo's Pss5 model: a speed-path
    deadband + gain + washout, a power-path low-pass filter + gain,
    summation, overall gain, optional second washout (CTw2 flag) and
    an output limiter. This thin wrapper exposes the CGMES catalog
    name without duplicating the implementation.
  */
  extends Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard.Pss5;

  annotation(preferredView = "text");
end Pss5C;
