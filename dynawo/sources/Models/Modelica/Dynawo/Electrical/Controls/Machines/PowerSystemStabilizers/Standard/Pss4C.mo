within Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model Pss4C "IEEE power system stabilizer type 4C (CGMES Pss4C, IEEE 421.5-2016 revision of PSS4B)"
  /*
    CGMES 2024 vendor-name alias for the IEEE multi-band PSS4B model
    already implemented as PssIEEE4B. The IEEE 421.5-2016 Annex C "C"
    revision keeps the three-band parallel structure (high H,
    intermediate I, low L) acting on the speed deviation, with each
    band feeding a differentiator-washout pair plus a chain of four
    lead-lag stages and a per-band gain + limiter; the band outputs
    are summed and the total is clamped. The dynamics are identical
    to the existing PssIEEE4B model so this thin wrapper exposes the
    CGMES catalog name without duplicating the implementation.
  */
  extends Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard.PssIEEE4B;

  annotation(preferredView = "text");
end Pss4C;
