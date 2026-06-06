within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2024, RTE (http://www.rte-france.com)
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

model Ac4a "IEEE exciter type AC4A model (2005 standard)"
  /*
    IEEE type AC4A alternator-supplied controlled-rectifier excitation system.
    Unlike the other AC type exciters it behaves like a controlled-rectifier
    system: no exciter saturation, no rate feedback and no rectifier
    regulation are represented. The signal path is a transient gain reduction
    lead-lag followed by the voltage regulator gain and time constant, between
    an input limiter and an output limiter. The underexcitation limitation
    input is routed through its PositionUel parameter.
  */

  //Regulation parameters
  parameter Types.PerUnit Ka "Voltage regulator gain";
  parameter Integer PositionUel "Input location : (0) none, (1) voltage error summation, (2) take-over at AVR input";
  parameter Types.Time tA "Voltage regulator time constant in s";
  parameter Types.Time tB "Voltage regulator lag time constant in s";
  parameter Types.Time tC "Voltage regulator lead time constant in s";
  parameter Types.Time tR "Stator voltage filter time constant in s";
  parameter Types.VoltageModulePu VImaxPu "Maximum voltage regulator input in pu (base UNom)";
  parameter Types.VoltageModulePu VIminPu "Minimum voltage regulator input in pu (base UNom)";
  parameter Types.VoltageModulePu VrMaxPu "Maximum output voltage of voltage regulator in pu (user-selected base voltage)";
  parameter Types.VoltageModulePu VrMinPu "Minimum output voltage of voltage regulator in pu (user-selected base voltage)";

  //Input variables
  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0) "Power system stabilizer output voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-200, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu) "Stator voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-200, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu) "Reference stator voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-200, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput UUelPu(start = 0) "Underexcitation limitation output voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-200, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  //Output variable
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu) "Excitation voltage in pu (user-selected base voltage)" annotation(
    Placement(visible = true, transformation(origin = {190, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Continuous.FirstOrder firstOrder(T = tR, y_start = Us0Pu) annotation(
    Placement(visible = true, transformation(origin = {-150, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add3 add3(k2 = -1) annotation(
    Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sum sum1(nin = 2) annotation(
    Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax = VImaxPu, uMin = VIminPu) annotation(
    Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.NonLinear.Max3 max1 annotation(
    Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction transferFunction(a = {tB, 1}, b = {tC, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Efd0Pu / Ka) annotation(
    Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder limitedFirstOrder(K = Ka, Y0 = Efd0Pu, YMax = VrMaxPu, YMin = VrMinPu, tFilter = tA) annotation(
    Placement(visible = true, transformation(origin = {120, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  //Generator initial parameters
  parameter Types.VoltageModulePu Efd0Pu "Initial excitation voltage in pu (user-selected base voltage)";
  parameter Types.VoltageModulePu Us0Pu "Initial stator voltage in pu (base UNom)";

  final parameter Types.VoltageModulePu UsRef0Pu = Efd0Pu / Ka + Us0Pu "Initial reference stator voltage in pu (base UNom)";

equation
  if PositionUel == 1 then
    sum1.u[2] = UUelPu;
    max1.u[2] = max1.u[1];
  elseif PositionUel == 2 then
    sum1.u[2] = 0;
    max1.u[2] = UUelPu;
  else
    sum1.u[2] = 0;
    max1.u[2] = max1.u[1];
  end if;
  max1.u[3] = max1.u[1];

  connect(UsPu, firstOrder.u) annotation(
    Line(points = {{-200, -40}, {-162, -40}}, color = {0, 0, 127}));
  connect(UPssPu, add3.u1) annotation(
    Line(points = {{-200, 40}, {-140, 40}, {-140, 8}, {-122, 8}}, color = {0, 0, 127}));
  connect(firstOrder.y, add3.u2) annotation(
    Line(points = {{-139, -40}, {-130, -40}, {-130, 0}, {-122, 0}}, color = {0, 0, 127}));
  connect(UsRefPu, add3.u3) annotation(
    Line(points = {{-200, 0}, {-150, 0}, {-150, -8}, {-122, -8}}, color = {0, 0, 127}));
  connect(add3.y, sum1.u[1]) annotation(
    Line(points = {{-99, 0}, {-82, 0}}, color = {0, 0, 127}));
  connect(sum1.y, limiter.u) annotation(
    Line(points = {{-59, 0}, {-42, 0}}, color = {0, 0, 127}));
  connect(limiter.y, max1.u[1]) annotation(
    Line(points = {{-19, 0}, {0, 0}}, color = {0, 0, 127}));
  connect(max1.y, transferFunction.u) annotation(
    Line(points = {{21, 0}, {47, 0}}, color = {0, 0, 127}));
  connect(transferFunction.y, limitedFirstOrder.u) annotation(
    Line(points = {{71, 0}, {107, 0}}, color = {0, 0, 127}));
  connect(limitedFirstOrder.y, EfdPu) annotation(
    Line(points = {{131, 0}, {190, 0}}, color = {0, 0, 127}));

  annotation(
    preferredView = "diagram",
    Diagram(coordinateSystem(extent = {{-180, -100}, {180, 100}})),
    Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(extent = {{-100, 60}, {100, -60}}, textString = "AC4A")}),
    Documentation(info = "<html><head></head><body>This model implements the IEEE type AC4A excitation system as defined in IEEE Std 421.5-2005.</body></html>"));
end Ac4a;
