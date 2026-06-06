within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovSteamIEEE1 "IEEE type 1 steam governor (CGMES GovSteamIEEE1, IEEEG1 2005 update alias)"
  /*
    CGMES 2024 vendor-name alias for the existing IEEEG1 model. The
    CGMES GovSteamIEEE1 is IEEE Std 421.5-2005 / IEEE PES-TR1 (2013)
    type-1 steam turbine governor: 1/(1+sT1)/(1+sT2) lead-lag input
    filter, K = 1/R droop gain into a 1/sT3 valve-positioner
    integrator (Uo, Uc velocity limited and Pmin..Pmax position
    limited), then a four-stage steam expansion (T4 HP bowl, T5
    reheater, T6 crossover, T7 double-reheater) with K1..K8 HP / LP
    power fractions. The dynamics are identical to the IEEEG1 model
    Dynawo already implements; this wrapper just exposes the CGMES
    name for catalog completeness.
  */
  extends Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam.IEEEG1;

  annotation(preferredView = "text");
end GovSteamIEEE1;
