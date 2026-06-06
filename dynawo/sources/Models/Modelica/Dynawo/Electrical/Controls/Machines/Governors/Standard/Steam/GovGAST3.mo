within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovGAST3 "Simplified gas turbine governor (CGMES GovGAST3)"
  /*
    Simplified gas turbine governor (PSS/E GAST3). Speed error
    (omegaRef - omega) is scaled by 1 / R and added to PmRef minus the
    permanent-droop position feedback Bp * gate; that error drives a T1
    first-order valve servo bounded VMin..VMax, followed by a
    (1 + sTsa) / (1 + sTsb) lead-lag compensator producing the raw gate
    position. The acceleration limit is the lower of two signals:
      - the compensator output (normal operation), and
      - an acceleration-limit ceiling Bca - Kca * s / (1 + sTsi) * (omega
        - omegaRef) which caps the gate when the rotor is accelerating.
    Pm = min(compensator, accel ceiling) - Dturb * (omega - omegaRef).
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit Bca "Acceleration limiter bias (steady-state ceiling)";
  parameter Types.PerUnit Bp "Gate permanent-droop bias";
  parameter Types.PerUnit DTurb;
  parameter Types.PerUnit Kca "Acceleration limiter gain";
  parameter Types.PerUnit R;
  parameter Types.Time t1;
  parameter Types.Time tSa "Compensator lead time constant in s";
  parameter Types.Time tSb "Compensator lag time constant in s";
  parameter Types.Time tSi "Acceleration limiter time constant in s";
  parameter Types.PerUnit VMax;
  parameter Types.PerUnit VMin;

  parameter Types.ActivePowerPu Pm0Pu;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PmRefPu(start = Pm0Pu * (1 + Bp));
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add dW(k1 = -1, k2 = 1) "Speed error omegaRef - omega";
  Modelica.Blocks.Math.Gain droop(k = 1 / R) "1 / R droop on speed error";
  Modelica.Blocks.Math.Add3 errSum(k1 = 1, k2 = 1, k3 = -1) "PmRef + droop * dW - Bp * gate";
  Modelica.Blocks.Math.Gain bpGain(k = Bp) "Gate position permanent-droop feedback";
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder valve(K = 1, tFilter = t1, Y0 = Pm0Pu, YMax = VMax, YMin = VMin);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction compensator(a = {tSb, 1}, b = {tSa, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu);
  Modelica.Blocks.Math.Add accelCeiling(k1 = 1, k2 = -1) "Bca - Kca / (1+sTsi) * dW";
  Modelica.Blocks.Sources.Constant bcaConst(k = Bca);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction accelLim(a = {tSi, 1}, b = {Kca, 0}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = 0);
  Dynawo.NonElectrical.Blocks.NonLinear.Min2 accelMin "min(compensator, accel ceiling)";
  Modelica.Blocks.Math.Gain damp(k = DTurb) "Turbine speed damping";
  Modelica.Blocks.Math.Feedback fbOut "min - Dturb * dW";

equation
  connect(omegaRefPu, dW.u1);
  connect(omegaPu, dW.u2);
  connect(dW.y, droop.u);
  connect(PmRefPu, errSum.u1);
  connect(droop.y, errSum.u2);
  connect(compensator.y, bpGain.u);
  connect(bpGain.y, errSum.u3);
  connect(errSum.y, valve.u);
  connect(valve.y, compensator.u);

  connect(dW.y, accelLim.u);
  connect(bcaConst.y, accelCeiling.u1);
  connect(accelLim.y, accelCeiling.u2);
  connect(compensator.y, accelMin.u1);
  connect(accelCeiling.y, accelMin.u2);

  connect(accelMin.y, fbOut.u1);
  connect(dW.y, damp.u);
  connect(damp.y, fbOut.u2);
  connect(fbOut.y, PmPu);

  annotation(preferredView = "text");
end GovGAST3;
