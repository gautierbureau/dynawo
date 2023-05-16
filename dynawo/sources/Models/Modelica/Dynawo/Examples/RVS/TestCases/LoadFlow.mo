within Dynawo.Examples.RVS.TestCases;

/*
* Copyright (c) 2023, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite
* of simulation tools for power systems.
*/

model LoadFlow "Model of load flow calculation for the RVS test system"
  import Dynawo;

  extends Icons.Example;
  extends Dynawo.Examples.RVS.Grid.FullStatic(
    UNom_lower = 138,
    UNom_upper = 230,
    URef0Pu_bus_101 = 1.0342,
    URef0Pu_bus_102 = 1.0358,
    URef0Pu_bus_107 = 1.0286,
    URef0Pu_bus_113 = 1.02,
    URef0Pu_bus_115 = 1.0113,
    URef0Pu_bus_116 = 1.0164,
    URef0Pu_bus_118 = 1.0432,
    URef0Pu_bus_122 = 1.05,
    URef0Pu_bus_123 = 1.0499,
    PRef0Pu_gen_10101 = 0.10000000000012331,
    PRef0Pu_gen_10102 = 0.10000000000001656,
    PRef0Pu_gen_10107 = 0.7999999999999469,
    PRef0Pu_gen_10113 = 1.6250000000182763,
    PRef0Pu_gen_10115 = 0.12000000000000558,
    PRef0Pu_gen_10116 = 1.5500000000057312,
    PRef0Pu_gen_10118 = 4.000000000000116,
    PRef0Pu_gen_10122 = 0.5000000000000253,
    PRef0Pu_gen_10123 = 1.5500000000002288,
    PRef0Pu_gen_20101 = 0.10000000000012331,
    PRef0Pu_gen_20102 = 0.10000000000001656,
    PRef0Pu_gen_20107 = 0.7999999999999469,
    PRef0Pu_gen_20113 = 1.625000000000306,
    PRef0Pu_gen_20115 = 0.12000000000000558,
    PRef0Pu_gen_20122 = 0.5000000000000254,
    PRef0Pu_gen_20123 = 1.5500000000002285,
    PRef0Pu_gen_30101 = 0.7600000000004553,
    PRef0Pu_gen_30102 = 0.7600000000000327,
    PRef0Pu_gen_30107 = 0.7999999999999469,
    PRef0Pu_gen_30113 = 1.625000000000306,
    PRef0Pu_gen_30115 = 0.12000000000000558,
    PRef0Pu_gen_30122 = 0.5000000000000254,
    PRef0Pu_gen_30123 = 3.5000000000005014,
    PRef0Pu_gen_40101 = 0.7600000000004553,
    PRef0Pu_gen_40102 = 0.7600000000000327,
    PRef0Pu_gen_40115 = 0.12000000000000558,
    PRef0Pu_gen_40122 = 0.5000000000000254,
    PRef0Pu_gen_50115 = 0.12000000000000558,
    PRef0Pu_gen_50122 = 0.5000000000000254,
    PRef0Pu_gen_60115 = 1.5500000000000716,
    PRef0Pu_gen_60122 = 0.5000000000000254,
    U0Pu_gen_10101 = 1.0298133177242317,
    U0Pu_gen_10102 = 1.045699670338049,
    U0Pu_gen_10107 = 1.0392480430917328,
    U0Pu_gen_10113 = 1.017778247138224,
    U0Pu_gen_10115 = 1.0194959441376978,
    U0Pu_gen_10116 = 1.013626716640669,
    U0Pu_gen_10118 = 1.0484086458065356,
    U0Pu_gen_10122 = 1.0038557904723289,
    U0Pu_gen_10123 = 1.0505246698144304,
    U0Pu_gen_20101 = 1.0298133177242317,
    U0Pu_gen_20102 = 1.0456996703380488,
    U0Pu_gen_20107 = 1.0392480430917328,
    U0Pu_gen_20113 = 1.0188760820803218,
    U0Pu_gen_20115 = 1.0194959441376976,
    U0Pu_gen_20122 = 1.0038557904723286,
    U0Pu_gen_20123 = 1.0505246698144302,
    U0Pu_gen_30101 = 1.016343574912408,
    U0Pu_gen_30102 = 1.0123906132652394,
    U0Pu_gen_30107 = 1.0392480430917328,
    U0Pu_gen_30113 = 1.0188760820803218,
    U0Pu_gen_30115 = 1.0194959441376976,
    U0Pu_gen_30122 = 1.0038557904723286,
    U0Pu_gen_30123 = 1.0413279094389931,
    U0Pu_gen_40101 = 1.016343574912408,
    U0Pu_gen_40102 = 1.0123906132652394,
    U0Pu_gen_40115 = 1.0194959441376976,
    U0Pu_gen_40122 = 1.0038557904723286,
    U0Pu_gen_50115 = 1.0194959441376976,
    U0Pu_gen_50122 = 1.0038557904723286,
    U0Pu_gen_60115 = 1.0211248001666335,
    U0Pu_gen_60122 = 1.0038557904723286,
    Q0Pu_line_reactor_106 = 0.7886505779740544,
    Q0Pu_line_reactor_110 = 0.7642449840596226,
    U0Pu_line_reactor_106 = 1,
    U0Pu_line_reactor_110 = 1,
    UPhase0_line_reactor_106 = 0,
    UPhase0_line_reactor_110 = 0,
    P0Pu_load_1101_ABEL = 1.1880000305126062,
    P0Pu_load_1102_ADAMS = 1.0669999694032049,
    P0Pu_load_1103_ADLER = 1.9799999999965128,
    P0Pu_load_1104_AGRICOLA = 0.8140000152415523,
    P0Pu_load_1105_AIKEN = 0.7809999847285005,
    P0Pu_load_1106_ALBER = 1.4960000610362467,
    P0Pu_load_1107_ALDER = 1.3749999999783746,
    P0Pu_load_1108_ALGER = 1.8810000610320796,
    P0Pu_load_1109_ALI = 1.9249999999952647,
    P0Pu_load_1110_ALLEN = 2.144999999966342,
    P0Pu_load_1113_ARNE = 2.9149999999494955,
    P0Pu_load_1114_ARNOLD = 2.1339999389236377,
    P0Pu_load_1115_ARTHUR = 3.4870001220067244,
    P0Pu_load_1116_ASSER = 1.0999999999349863,
    P0Pu_load_1118_ASTOR = 3.662999877912528,
    P0Pu_load_1119_ATTAR = 1.9910000610193985,
    P0Pu_load_1120_ATTILA = 1.408000030459867,
    Q0Pu_load_1101_ABEL = 0.24200000762794033,
    Q0Pu_load_1102_ADAMS = 0.21999999997414654,
    Q0Pu_load_1103_ADLER = 0.407000007625965,
    Q0Pu_load_1104_AGRICOLA = 0.16499999998875836,
    Q0Pu_load_1105_AIKEN = 0.15399999617863952,
    Q0Pu_load_1106_ALBER = 0.3079999923711575,
    Q0Pu_load_1107_ALDER = 0.27499999999505487,
    Q0Pu_load_1108_ALGER = 0.3849999999997419,
    Q0Pu_load_1109_ALI = 0.3959999847378253,
    Q0Pu_load_1110_ALLEN = 0.4399999999884504,
    Q0Pu_load_1113_ARNE = 0.5940000152398809,
    Q0Pu_load_1114_ARNOLD = 0.42900001524297804,
    Q0Pu_load_1115_ARTHUR = 0.7040000152345325,
    Q0Pu_load_1116_ASSER = 0.21999999997646183,
    Q0Pu_load_1118_ASTOR = 0.7480000305135921,
    Q0Pu_load_1119_ATTAR = 0.40700000762424227,
    Q0Pu_load_1120_ATTILA = 0.286000003796252,
    U0Pu_load_1101_ABEL = 1,
    U0Pu_load_1102_ADAMS = 1,
    U0Pu_load_1103_ADLER = 1,
    U0Pu_load_1104_AGRICOLA = 1,
    U0Pu_load_1105_AIKEN = 1,
    U0Pu_load_1106_ALBER = 1,
    U0Pu_load_1107_ALDER = 1,
    U0Pu_load_1108_ALGER = 1,
    U0Pu_load_1109_ALI = 1,
    U0Pu_load_1110_ALLEN = 1,
    U0Pu_load_1113_ARNE = 1,
    U0Pu_load_1114_ARNOLD = 1,
    U0Pu_load_1115_ARTHUR = 1,
    U0Pu_load_1116_ASSER = 1,
    U0Pu_load_1118_ASTOR = 1,
    U0Pu_load_1119_ATTAR = 1,
    U0Pu_load_1120_ATTILA = 1,
    UPhase0_load_1101_ABEL = 0,
    UPhase0_load_1102_ADAMS = 0,
    UPhase0_load_1103_ADLER = 0,
    UPhase0_load_1104_AGRICOLA = 0,
    UPhase0_load_1105_AIKEN = 0,
    UPhase0_load_1106_ALBER = 0,
    UPhase0_load_1107_ALDER = 0,
    UPhase0_load_1108_ALGER = 0,
    UPhase0_load_1109_ALI = 0,
    UPhase0_load_1110_ALLEN = 0,
    UPhase0_load_1113_ARNE = 0,
    UPhase0_load_1114_ARNOLD = 0,
    UPhase0_load_1115_ARTHUR = 0,
    UPhase0_load_1116_ASSER = 0,
    UPhase0_load_1118_ASTOR = 0,
    UPhase0_load_1119_ATTAR = 0,
    UPhase0_load_1120_ATTILA = 0,
    Q0Pu_sVarC_10106 = -0.6157864880236029,
    Q0Pu_sVarC_10114 = -1.2514318403502432);

equation
  line_106_110.switchOffSignal1.value = false;
  line_106_110.switchOffSignal2.value = false;
  line_reactor_110.switchOffSignal1.value = false;
  line_reactor_110.switchOffSignal2.value = false;
  line_reactor_106.switchOffSignal1.value = false;
  line_reactor_106.switchOffSignal2.value = false;

  annotation(preferredView = "diagram",
    experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.01),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --daeMode",
    __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "ida", nls = "hybrid"),
    Diagram(coordinateSystem(extent = {{-340, -340}, {340, 340}})));
end LoadFlow;
