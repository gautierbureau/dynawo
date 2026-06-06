within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAC9C "CGMES vendor name for IEEE AC9C alternator-rectifier (CGMES ExcAC9C)"
  /*
    CGMES 2024 vendor name for the IEEE 421.5-2016 AC9C alternator-
    rectifier excitation system. AC9C is the newest entry in the AC
    family and adds field-shorting and additional rectifier-loading
    paths to the AC8B baseline. For IEC 61970-302 catalog mapping the
    AC8C topology already implemented in Dynawo as Ac8c is the
    closest available match (same Kp / Ki / Kd PID, same exciter
    integrator, same rectifier regulation and saturation). Use a
    purpose-built ExcAC9C only if a dataset exercises AC9C-specific
    2016 features (field-shorting, extended OEL/UEL routing) that
    the Ac8c topology does not capture.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac8c;

  annotation(preferredView = "text");
end ExcAC9C;
