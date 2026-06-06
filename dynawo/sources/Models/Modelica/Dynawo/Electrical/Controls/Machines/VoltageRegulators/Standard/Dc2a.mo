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

model Dc2a "IEEE excitation system type DC2A model (2005 standard)"
  /*
    IEEE type DC2A field-controlled dc commutator exciter. It differs from
    the type DC1A model only in the voltage regulator output limits, which
    are proportional to the stator voltage instead of being constant.
    The voltage regulator output limits are of the windup type.
  */

  //Regulation parameters
  parameter Types.PerUnit AEx "Gain of saturation function";
  parameter Types.PerUnit BEx "Exponential coefficient of saturation function";
  parameter Types.PerUnit Ka "Voltage regulator gain";
  parameter Types.PerUnit Ke "Exciter field proportional constant";
  parameter Types.PerUnit Kf "Exciter rate feedback gain";
  parameter Integer PositionUel "Input location : (0) none, (1) voltage error summation, (2) take-over at AVR output";
  parameter Types.Time tA "Voltage regulator time constant in s";
  parameter Types.Time tB "Voltage regulator lag time constant in s";
  parameter Types.Time tC "Voltage regulator lead time constant in s";
  parameter Types.Time tE "Exciter time constant in s";
  parameter Types.Time tF "Exciter rate feedback time constant in s";
  parameter Types.Time tR "Stator voltage filter time constant in s";
  parameter Types.PerUnit VrMaxPu "Maximum field voltage in pu (base UNom), as a ratio applied to the stator voltage";
  parameter Types.PerUnit VrMinPu "Minimum field voltage in pu (base UNom), as a ratio applied to the stator voltage";

  //Input variables
  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0) "Power system stabilizer output voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-320, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu) "Stator voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-320, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu) "Reference stator voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-320, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput UUelPu(start = 0) "Underexcitation limitation output voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-320, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  //Output variable
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu) "Excitation voltage in pu (user-selected base voltage)" annotation(
    Placement(visible = true, transformation(origin = {310, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Continuous.FirstOrder firstOrder(T = tR, y_start = Us0Pu) annotation(
    Placement(visible = true, transformation(origin = {-270, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add3 add3(k2 = -1) annotation(
    Placement(visible = true, transformation(origin = {-210, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sum sum1(nin = 3) annotation(
    Placement(visible = true, transformation(origin = {-170, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction transferFunction(a = {tB, 1}, b = {tC, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Va0Pu / Ka) annotation(
    Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.NonLinear.Max3 max1 annotation(
    Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder regulator(T = tA, k = Ka, y_start = Va0Pu) annotation(
    Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.NonLinear.VariableLimiter variableLimiter annotation(
    Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gainMax(k = VrMaxPu) annotation(
    Placement(visible = true, transformation(origin = {10, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gainMin(k = VrMinPu) annotation(
    Placement(visible = true, transformation(origin = {10, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator(k = 1 / tE, y_start = Efd0Pu) annotation(
    Placement(visible = true, transformation(origin = {190, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Derivative derivative(k = Kf, T = tF, x_start = Efd0Pu) annotation(
    Placement(visible = true, transformation(origin = {50, -100}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Power power(base = exp(BEx)) annotation(
    Placement(visible = true, transformation(origin = {250, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = Ke) annotation(
    Placement(visible = true, transformation(origin = {250, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add(k1 = AEx) annotation(
    Placement(visible = true, transformation(origin = {190, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
    Placement(visible = true, transformation(origin = {130, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));

  //Generator initial parameters
  parameter Types.VoltageModulePu Efd0Pu "Initial excitation voltage in pu (user-selected base voltage)";
  parameter Types.VoltageModulePu Us0Pu "Initial stator voltage in pu (base UNom)";

  final parameter Types.VoltageModulePu UsRef0Pu = Va0Pu / Ka + Us0Pu "Initial reference stator voltage in pu (base UNom)";
  final parameter Types.VoltageModulePu Va0Pu = Efd0Pu * (Ke + AEx * exp(BEx * Efd0Pu)) "Initial output voltage of voltage regulator in pu (user-selected base voltage)";

equation
  if PositionUel == 1 then
    sum1.u[3] = UUelPu;
    max1.u[2] = max1.u[1];
  elseif PositionUel == 2 then
    sum1.u[3] = 0;
    max1.u[2] = UUelPu;
  else
    sum1.u[3] = 0;
    max1.u[2] = max1.u[1];
  end if;
  sum1.u[2] = 0;
  max1.u[3] = max1.u[1];

  connect(UPssPu, add3.u1) annotation(
    Line(points = {{-320, 40}, {-240, 40}, {-240, 8}, {-222, 8}}, color = {0, 0, 127}));
  connect(UsPu, firstOrder.u) annotation(
    Line(points = {{-320, 0}, {-282, 0}}, color = {0, 0, 127}));
  connect(firstOrder.y, add3.u2) annotation(
    Line(points = {{-259, 0}, {-223, 0}}, color = {0, 0, 127}));
  connect(UsRefPu, add3.u3) annotation(
    Line(points = {{-320, -40}, {-240, -40}, {-240, -8}, {-222, -8}}, color = {0, 0, 127}));
  connect(add3.y, sum1.u[1]) annotation(
    Line(points = {{-199, 0}, {-183, 0}}, color = {0, 0, 127}));
  connect(sum1.y, feedback.u1) annotation(
    Line(points = {{-159, 0}, {-129, 0}}, color = {0, 0, 127}));
  connect(feedback.y, transferFunction.u) annotation(
    Line(points = {{-111, 0}, {-83, 0}}, color = {0, 0, 127}));
  connect(transferFunction.y, max1.u[1]) annotation(
    Line(points = {{-59, 0}, {-40, 0}}, color = {0, 0, 127}));
  connect(max1.y, regulator.u) annotation(
    Line(points = {{-19, 0}, {18, 0}}, color = {0, 0, 127}));
  connect(regulator.y, variableLimiter.u) annotation(
    Line(points = {{41, 0}, {58, 0}}, color = {0, 0, 127}));
  connect(UsPu, gainMax.u) annotation(
    Line(points = {{-320, 0}, {-290, 0}, {-290, 60}, {-2, 60}}, color = {0, 0, 127}));
  connect(UsPu, gainMin.u) annotation(
    Line(points = {{-320, 0}, {-290, 0}, {-290, -60}, {-2, -60}}, color = {0, 0, 127}));
  connect(gainMax.y, variableLimiter.limit1) annotation(
    Line(points = {{21, 60}, {50, 60}, {50, 8}, {58, 8}}, color = {0, 0, 127}));
  connect(gainMin.y, variableLimiter.limit2) annotation(
    Line(points = {{21, -60}, {50, -60}, {50, -8}, {58, -8}}, color = {0, 0, 127}));
  connect(variableLimiter.y, feedback1.u1) annotation(
    Line(points = {{81, 0}, {92, 0}}, color = {0, 0, 127}));
  connect(feedback1.y, integrator.u) annotation(
    Line(points = {{109, 0}, {178, 0}}, color = {0, 0, 127}));
  connect(integrator.y, EfdPu) annotation(
    Line(points = {{201, 0}, {310, 0}}, color = {0, 0, 127}));
  connect(integrator.y, derivative.u) annotation(
    Line(points = {{201, 0}, {280, 0}, {280, -100}, {62, -100}}, color = {0, 0, 127}));
  connect(derivative.y, feedback.u2) annotation(
    Line(points = {{39, -100}, {-120, -100}, {-120, -8}}, color = {0, 0, 127}));
  connect(integrator.y, power.u) annotation(
    Line(points = {{201, 0}, {280, 0}, {280, -40}, {262, -40}}, color = {0, 0, 127}));
  connect(integrator.y, product.u2) annotation(
    Line(points = {{201, 0}, {280, 0}, {280, -100}, {160, -100}, {160, -86}, {142, -86}}, color = {0, 0, 127}));
  connect(power.y, add.u1) annotation(
    Line(points = {{239, -40}, {220, -40}, {220, -54}, {202, -54}}, color = {0, 0, 127}));
  connect(const.y, add.u2) annotation(
    Line(points = {{239, -80}, {220, -80}, {220, -66}, {202, -66}}, color = {0, 0, 127}));
  connect(add.y, product.u1) annotation(
    Line(points = {{179, -60}, {160, -60}, {160, -74}, {142, -74}}, color = {0, 0, 127}));
  connect(product.y, feedback1.u2) annotation(
    Line(points = {{119, -80}, {100, -80}, {100, -8}}, color = {0, 0, 127}));

  annotation(
    preferredView = "diagram",
    Diagram(coordinateSystem(extent = {{-300, -140}, {300, 140}})),
    Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(extent = {{-100, 60}, {100, -60}}, textString = "DC2A")}),
    Documentation(info = "<html><head></head><body>This model implements the IEEE type DC2A excitation system as defined in IEEE Std 421.5-2005. It differs from the type DC1A model only in the voltage regulator output limits, which are proportional to the stator voltage.</body></html>"));
end Dc2a;
