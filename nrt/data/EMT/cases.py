# -*- coding: utf-8 -*-

# Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source time domain
# simulation tool for power systems.

# All the EMT (three-phase instantaneous) non-regression cases are grouped under this
# single EMT folder. The NRT harness (nrt.py) discovers only one cases.py per top-level
# data directory, so this aggregator loads every per-case cases.py in the sub-folders and
# merges their test_cases. Each sub-folder's cases.py stays self-contained (it computes its
# own job_file from its __file__ and keeps its own reference/), so adding an EMT case is just
# creating a new sub-folder with its own cases.py -- it is picked up here automatically.

import os
import importlib.util

test_cases = []

_base = os.path.dirname(__file__)
for _sub in sorted(os.listdir(_base)):
    _cases_py = os.path.join(_base, _sub, "cases.py")
    if not os.path.isfile(_cases_py):
        continue
    _spec = importlib.util.spec_from_file_location("emt_cases_" + _sub, _cases_py)
    _mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_mod)
    test_cases.extend(_mod.test_cases)
