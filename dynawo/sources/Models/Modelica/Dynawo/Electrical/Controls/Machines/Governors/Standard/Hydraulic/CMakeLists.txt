# Copyright (c) 2023, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source suite
# of simulation tools for power systems.

# Standard hydraulic governor models

set(MODEL_FILES
  package.mo
  package.order
  GovHydro0.mo
  GovHydro0.extvar
  GovHydro1.mo
  GovHydro1.extvar
  GovHydro2.mo
  GovHydro2.extvar
  GovHydro3.mo
  GovHydro3.extvar
  GovHydroDD.mo
  GovHydroDD.extvar
  GovHydro4.mo
  GovHydro4.extvar
  GovHydroFrancis.mo
  GovHydroFrancis.extvar
  GovHydroIEEE2.mo
  GovHydroIEEE2.extvar
  GovHydroPID.mo
  GovHydroPID.extvar
  GovHydroPID2.mo
  GovHydroPID2.extvar
  GovHydroPelton.mo
  GovHydroPelton.extvar
  GovHydroR.mo
  GovHydroR.extvar
  GovHydroWEH.mo
  GovHydroWEH.extvar
  GovHydroWPID.mo
  GovHydroWPID.extvar
  HyGov.mo
  HyGov.extvar
  )

#
# Modelica models install
#
foreach(MODEL_FILE ${MODEL_FILES})
  install_model_file(${MODEL_FILE})
endforeach(MODEL_FILE)
