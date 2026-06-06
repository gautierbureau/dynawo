within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model St2a_INIT "IEEE excitation system type ST2A initialization model"
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Exciter_INIT;

  Dynawo.Connectors.CurrentModulePuConnector Ir0Pu "Initial rotor current in pu (base SNom, user-selected base voltage)";

  annotation(preferredView = "text");
end St2a_INIT;
