# Copyright (c) 2021, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.

# Standard steam governor models

set(MODEL_FILES
  package.mo
  package.order
  Gast.mo
  Gast.extvar
  GovCT1.mo
  GovCT1.extvar
  GovCT2.mo
  GovCT2.extvar
  GovGAST1.mo
  GovGAST1.extvar
  GovGAST2.mo
  GovGAST2.extvar
  GovGAST3.mo
  GovGAST3.extvar
  GovGAST4.mo
  GovGAST4.extvar
  GovGASTWD.mo
  GovGASTWD.extvar
  GovSteamBB.mo
  GovSteamBB.extvar
  GovSteamIEEE1.mo
  GovSteamIEEE1.extvar
  GovSteam0.mo
  GovSteam0.extvar
  GovSteam1.mo
  GovSteam1.extvar
  GovSteam1_INIT.mo
  GovSteam2.mo
  GovSteam2.extvar
  GovSteamCC.mo
  GovSteamCC.extvar
  GovSteamEu.mo
  GovSteamEu.extvar
  GovSteamFV2.mo
  GovSteamFV2.extvar
  GovSteamFV3.mo
  GovSteamFV3.extvar
  GovSteamFV4.mo
  GovSteamFV4.extvar
  GovSteamSGO.mo
  GovSteamSGO.extvar
  IEEEG1.mo
  IEEEG1.extvar
  IEEEG2.mo
  IEEEG2.extvar
  TGov1.mo
  TGov1.extvar
  TGov3.mo
  TGov3.extvar
  TGov3_INIT.mo
  )

#
# Modelica models install
#
foreach(MODEL_FILE ${MODEL_FILES})
  install_model_file(${MODEL_FILE})
endforeach(MODEL_FILE)
