# Copyright (c) 2026, RTE (http://www.rte-france.com)
# SPDX-License-Identifier: MPL-2.0

set(MODEL_FILES
  package.mo
  package.order
  DiscExcContIEEEDEC1A.mo
  DiscExcContIEEEDEC1A.extvar
  DiscExcContIEEEDEC2A.mo
  DiscExcContIEEEDEC2A.extvar
  DiscExcContIEEEDEC3A.mo
  DiscExcContIEEEDEC3A.extvar
  )

foreach(MODEL_FILE ${MODEL_FILES})
  install_model_file(${MODEL_FILE})
endforeach(MODEL_FILE)
