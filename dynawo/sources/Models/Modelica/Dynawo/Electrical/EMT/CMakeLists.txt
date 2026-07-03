# Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.

add_subdirectory(Functions EMf)
add_subdirectory(Examples EMx)

set(MODEL_FILES
  # Electrical EMT (instantaneous three-phase) models
  package.mo
  package.order
  EmtTerminal.mo
  BaseEmtTwoTerminal.mo
  Ground.mo
  Ground.extvar
  VoltageSource.mo
  VoltageSource.extvar
  InfiniteBus.mo
  InfiniteBus.extvar
  Resistor.mo
  Resistor.extvar
  Inductor.mo
  Inductor.extvar
  Capacitor.mo
  Capacitor.extvar
  Capacitor_INIT.mo
  Bus.mo
  Bus.extvar
  SeriesRL.mo
  SeriesRL.extvar
  SeriesRL_INIT.mo
  CoupledRL.mo
  CoupledRL.extvar
  CoupledRL_INIT.mo
  Breaker.mo
  Breaker.extvar
  Fault.mo
  Fault.extvar
  LineToLineFault.mo
  LineToLineFault.extvar
  LineFault.mo
  LineFault.extvar
  TransformerYgYg.mo
  TransformerYgYg.extvar
  TransformerYgYg_INIT.mo
  TransformerYgD.mo
  TransformerYgD.extvar
  TransformerYgD_INIT.mo
  SynchronousMachine.mo
  SynchronousMachine.extvar
  SynchronousMachine_INIT.mo
  GeneratorSynchronous.mo
  GeneratorSynchronous.extvar
  GeneratorSynchronous_INIT.mo
  SynchronousMachineFull.mo
  SynchronousMachineFull.extvar
  SynchronousMachineFull_INIT.mo
  FixedExcitationMechanicalPower.mo
  FixedExcitationMechanicalPower_INIT.mo
  )

#
# Modelica models install
#
foreach(MODEL_FILE ${MODEL_FILES})
  install_model_file(${MODEL_FILE})
endforeach(MODEL_FILE)
