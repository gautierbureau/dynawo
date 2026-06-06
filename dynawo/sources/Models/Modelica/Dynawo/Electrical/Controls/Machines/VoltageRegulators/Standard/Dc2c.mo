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

model Dc2c "IEEE excitation system type DC2C model (2016 standard)"
  /*
    IEEE type DC2C field-controlled dc commutator exciter. It reuses the
    type DC2A signal path (stator voltage transducer, transient gain
    reduction lead-lag, voltage regulator with stator-voltage-proportional
    output limits, exciter integrator with saturation and rate feedback)
    and extends it with the configurable overexcitation, underexcitation
    and stator current limiter inputs of the 2016 standard, routed through
    their PositionOel / PositionUel / PositionScl parameters, and with a
    minimum excitation voltage on the exciter integrator.
    The voltage regulator output limits are of the windup type.
  */

  //Regulation parameters
  parameter Types.PerUnit AEx "Gain of saturation function";
  parameter Types.PerUnit BEx "Exponential coefficient of saturation function";
  parameter Types.VoltageModulePu EfdMinPu "Minimum excitation voltage in pu (user-selected base voltage)";
  parameter Types.PerUnit Ka "Voltage regulator gain";
  parameter Types.PerUnit Ke "Exciter field proportional constant";
  parameter Types.PerUnit Kf "Exciter rate feedback gain";
  parameter Integer PositionOel "Input location : (0) none, (1) voltage error summation, (2) take-over at AVR output";
  parameter Integer PositionScl "Input location : (0) none, (1) voltage error summation, (2) take-over at AVR output";
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
  Modelica.Blocks.Interfaces.RealInput UOelPu(start = 0) "Overexcitation limitation output voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-320, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0) "Power system stabilizer output voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-320, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu) "Stator voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-320, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu) "Reference stator voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-320, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput USclOelPu(start = 0) "Stator current overexcitation limitation output voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-320, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput USclUelPu(start = 0) "Stator current underexcitation limitation output voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-320, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput UUelPu(start = 0) "Underexcitation limitation output voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-320, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  //Output variable
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu) "Excitation voltage in pu (user-selected base voltage)" annotation(
    Placement(visible = true, transformation(origin = {310, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Continuous.FirstOrder firstOrder(T = tR, y_start = Us0Pu) annotation(
    Placement(visible = true, transformation(origin = {-270, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add3 add3(k2 = -1) annotation(
    Placement(visible = true, transformation(origin = {-210, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sum sum1(nin = 4) annotation(
    Placement(visible = true, transformation(origin = {-170, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback annotation(
    Placement(visible = true, transformation(origin = {-130, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction transferFunction(a = {tB, 1}, b = {tC, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Va0Pu / Ka) annotation(
    Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.NonLinear.Max3 max1 annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.NonLinear.Min3 min1 annotation(
    Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder regulator(T = tA, k = Ka, y_start = Va0Pu) annotation(
    Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.NonLinear.VariableLimiter variableLimiter annotation(
    Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gainMax(k = VrMaxPu) annotation(
    Placement(visible = true, transformation(origin = {30, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gainMin(k = VrMinPu) annotation(
    Placement(visible = true, transformation(origin = {30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimIntegrator limIntegrator(k = 1 / tE, outMax = 999, outMin = EfdMinPu, y_start = Efd0Pu) annotation(
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
  if PositionOel == 1 then
    sum1.u[2] = UOelPu;
    min1.u[2] = min1.u[1];
  elseif PositionOel == 2 then
    sum1.u[2] = 0;
    min1.u[2] = UOelPu;
  else
    sum1.u[2] = 0;
    min1.u[2] = min1.u[1];
  end if;

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

  if PositionScl == 1 then
    sum1.u[4] = USclOelPu + USclUelPu;
    max1.u[3] = max1.u[1];
    min1.u[3] = min1.u[1];
  elseif PositionScl == 2 then
    sum1.u[4] = 0;
    max1.u[3] = USclUelPu;
    min1.u[3] = USclOelPu;
  else
    sum1.u[4] = 0;
    max1.u[3] = max1.u[1];
    min1.u[3] = min1.u[1];
  end if;

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
    Line(points = {{-159, 0}, {-139, 0}}, color = {0, 0, 127}));
  connect(feedback.y, transferFunction.u) annotation(
    Line(points = {{-121, 0}, {-103, 0}}, color = {0, 0, 127}));
  connect(transferFunction.y, max1.u[1]) annotation(
    Line(points = {{-79, 0}, {-60, 0}}, color = {0, 0, 127}));
  connect(max1.y, min1.u[1]) annotation(
    Line(points = {{-39, 0}, {-20, 0}}, color = {0, 0, 127}));
  connect(min1.y, regulator.u) annotation(
    Line(points = {{1, 0}, {18, 0}}, color = {0, 0, 127}));
  connect(regulator.y, variableLimiter.u) annotation(
    Line(points = {{41, 0}, {58, 0}}, color = {0, 0, 127}));
  connect(UsPu, gainMax.u) annotation(
    Line(points = {{-320, 0}, {-290, 0}, {-290, 60}, {18, 60}}, color = {0, 0, 127}));
  connect(UsPu, gainMin.u) annotation(
    Line(points = {{-320, 0}, {-290, 0}, {-290, -60}, {18, -60}}, color = {0, 0, 127}));
  connect(gainMax.y, variableLimiter.limit1) annotation(
    Line(points = {{41, 60}, {50, 60}, {50, 8}, {58, 8}}, color = {0, 0, 127}));
  connect(gainMin.y, variableLimiter.limit2) annotation(
    Line(points = {{41, -60}, {50, -60}, {50, -8}, {58, -8}}, color = {0, 0, 127}));
  connect(variableLimiter.y, feedback1.u1) annotation(
    Line(points = {{81, 0}, {102, 0}}, color = {0, 0, 127}));
  connect(feedback1.y, limIntegrator.u) annotation(
    Line(points = {{119, 0}, {178, 0}}, color = {0, 0, 127}));
  connect(limIntegrator.y, EfdPu) annotation(
    Line(points = {{201, 0}, {310, 0}}, color = {0, 0, 127}));
  connect(limIntegrator.y, derivative.u) annotation(
    Line(points = {{201, 0}, {280, 0}, {280, -100}, {62, -100}}, color = {0, 0, 127}));
  connect(derivative.y, feedback.u2) annotation(
    Line(points = {{39, -100}, {-130, -100}, {-130, -8}}, color = {0, 0, 127}));
  connect(limIntegrator.y, power.u) annotation(
    Line(points = {{201, 0}, {280, 0}, {280, -40}, {262, -40}}, color = {0, 0, 127}));
  connect(limIntegrator.y, product.u2) annotation(
    Line(points = {{201, 0}, {280, 0}, {280, -100}, {160, -100}, {160, -86}, {142, -86}}, color = {0, 0, 127}));
  connect(power.y, add.u1) annotation(
    Line(points = {{239, -40}, {220, -40}, {220, -54}, {202, -54}}, color = {0, 0, 127}));
  connect(const.y, add.u2) annotation(
    Line(points = {{239, -80}, {220, -80}, {220, -66}, {202, -66}}, color = {0, 0, 127}));
  connect(add.y, product.u1) annotation(
    Line(points = {{179, -60}, {160, -60}, {160, -74}, {142, -74}}, color = {0, 0, 127}));
  connect(product.y, feedback1.u2) annotation(
    Line(points = {{119, -80}, {110, -80}, {110, -8}}, color = {0, 0, 127}));

  annotation(
    preferredView = "diagram",
    Diagram(coordinateSystem(extent = {{-300, -140}, {300, 140}})),
    Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(extent = {{-100, 60}, {100, -60}}, textString = "DC2C")}),
    Documentation(info = "<html><head></head><body>This model implements the IEEE type DC2C excitation system as defined in IEEE Std 421.5-2016. It extends the type DC2A signal path with the configurable overexcitation, underexcitation and stator current limiter inputs of the 2016 standard.</body></html>"));
end Dc2c;
