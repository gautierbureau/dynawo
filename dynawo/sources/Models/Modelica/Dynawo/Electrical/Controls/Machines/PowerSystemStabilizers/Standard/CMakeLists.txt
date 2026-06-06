# Copyright (c) 2021, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.

# IEEE power system stabilizers models

add_subdirectory(BaseClasses B)

set(MODEL_FILES
    package.mo
    package.order
    Pss1.mo
    Pss1.extvar
    Pss1A.mo
    Pss1A.extvar
    Pss1aOmega.mo
    Pss1aOmega.extvar
    Pss1aPGen.mo
    Pss1aPGen.extvar
    Pss2a.mo
    Pss2a.extvar
    Pss2b.mo
    Pss2b.extvar
    Pss2c.mo
    Pss2c.extvar
    Pss2ST.mo
    Pss2ST.extvar
    Pss3b.mo
    Pss3b.extvar
    Pss3c.mo
    Pss3c.extvar
    Pss4C.mo
    Pss4C.extvar
    Pss5.mo
    Pss5.extvar
    Pss5C.mo
    Pss5C.extvar
    Pss6c.mo
    Pss6c.extvar
    Pss6C.mo
    Pss6C.extvar
    Pss7C.mo
    Pss7C.extvar
    PssIEEE1A.mo
    PssIEEE1A.extvar
    PssIEEE4B.mo
    PssIEEE4B.extvar
    PssKundur.mo
    PssKundur.extvar
    PssPTIST1.mo
    PssPTIST1.extvar
    PssPTIST3.mo
    PssPTIST3.extvar
    PssRQB.mo
    PssRQB.extvar
    PssWECC.mo
    PssWECC.extvar
    PssELIN2.mo
    PssELIN2.extvar
    PssSB4.mo
    PssSB4.extvar
    PssSH.mo
    PssSH.extvar
    PssSK.mo
    PssSK.extvar
    )

#
# Modelica models install
#
foreach(MODEL_FILE ${MODEL_FILES})
  install_model_file(${MODEL_FILE})
endforeach(MODEL_FILE)
