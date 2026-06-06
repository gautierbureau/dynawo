within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcST4B "CGMES vendor variant of IEEE St4b (CGMES ExcST4B)"
  /*
    Vendor variant of the IEEE St4b excitation system. Identical parameter
    set and topology - this is a name alias for the CGMES vendor namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.St4b;

  annotation(preferredView = "text");
end ExcST4B;
