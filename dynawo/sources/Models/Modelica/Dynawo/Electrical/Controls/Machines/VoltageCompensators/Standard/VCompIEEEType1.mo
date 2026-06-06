within Dynawo.Electrical.Controls.Machines.VoltageCompensators.Standard;

/*
* Copyright (c) 2024, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.
*/

model VCompIEEEType1 "IEEE voltage compensator type 1 (CGMES VCompIEEEType1)"
  /*
    Computes the compensated voltage VcPu = |utPu + (Rc + j*Xc) * I_gen|
    that is fed to a voltage regulator in place of the raw terminal
    voltage magnitude. With Rc = Xc = 0 the model reduces to |utPu|.

    Convention: the current input itPu is in Dynawo receptor convention
    (terminal.i, flowing INTO the machine, base SnRef), and the model
    flips the sign internally to recover the IEEE generator convention.
    The compensation impedance (Rc, Xc) is in machine base (SNom), so
    the current is rebased to SNom inside the block.

    Corresponds to CGMES IEC 61970-302 VCompIEEEType1 / IEEE Std 421.5-2016
    Section 5.2 Figure 2 (compensation block, without the voltage
    transducer 1/(1+sTR) which lives in the AVR). Block-based: the
    equation section contains only connect statements.
  */
  import Modelica.Blocks;
  import Modelica.ComplexBlocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit RcPu "Resistive component of load compensation in pu (base SNom)";
  parameter Types.PerUnit XcPu "Reactive component of load compensation in pu (base SNom)";
  parameter Types.ApparentPowerModule SNom "Nominal apparent power of the machine in MVA";

  // Inputs / output
  ComplexBlocks.Interfaces.ComplexInput utPu "Complex terminal voltage in pu (base UNom)";
  ComplexBlocks.Interfaces.ComplexInput itPu "Complex terminal current in pu (base SnRef, UNom) (receptor convention)";
  Blocks.Interfaces.RealOutput VcPu "Compensated voltage magnitude in pu (base UNom)";

  // Combine V + Z * I_gen, with sign flip and base change on the current:
  //   Zc_effective = -(Rc + j*Xc) * SnRef/SNom
  //     -> "minus" cancels the receptor->generator sign on itPu
  //     -> "SnRef/SNom" rebases the current from network to machine base
  final parameter Real currBaseChange = SystemBase.SnRef / SNom "SnRef/SNom current base factor";

  ComplexBlocks.ComplexMath.Add adder(k1 = Complex(1, 0), k2 = Complex(-RcPu * currBaseChange, -XcPu * currBaseChange)) "VcPhasor = utPu - (Rc + j*Xc) * (SnRef/SNom) * itPu";
  Dynawo.NonElectrical.Blocks.Complex.ComplexToPolar toPolar "Take the magnitude of the compensated phasor";

equation
  connect(utPu, adder.u1);
  connect(itPu, adder.u2);
  connect(adder.y, toPolar.u);
  connect(toPolar.len, VcPu);

  annotation(preferredView = "text");
end VCompIEEEType1;
