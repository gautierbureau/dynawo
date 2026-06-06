within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcST7B "CGMES vendor variant of IEEE St7b (CGMES ExcST7B)"
  /*
    Vendor variant of the IEEE St7b excitation system. Identical parameter
    set and topology - this is a name alias for the CGMES vendor namespace.
  */
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.St7b;

  annotation(preferredView = "text");
end ExcST7B;
