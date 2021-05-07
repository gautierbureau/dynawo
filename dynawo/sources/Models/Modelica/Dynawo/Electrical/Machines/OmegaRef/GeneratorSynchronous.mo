within Dynawo.Electrical.Machines.OmegaRef;

/*
* Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.
*/

model GeneratorSynchronous "Synchronous machine"

  extends BaseClasses.BaseGeneratorSynchronous;
  extends AdditionalIcons.Machine;

  /*
   This model represents a sychronous machine.
   The synchronous machine can include a generator transformer.
   The model respects the receptor convention (i > 0 when entering the device)

       Stator side                                                 Terminal side (network side)

                 iStator  rTfo                                i

        uStator ----<------oo------ RTfoPu + jXTfoPu ---------<---- U
        sStator                                                     P
        QStator                                                     Q

  */

  public

    // Voltage, active and reactive power at terminal
    Types.VoltageModulePu UPu (start = U0Pu) "Voltage amplitude at terminal in p.u (base UNom)";
    Types.ActivePowerPu PGenPu(start = PGen0Pu) "Active power generated by the synchronous machine in p.u (base SnRef)";
    Types.ReactivePowerPu QGenPu(start = QGen0Pu) "Reactive power generated by the synchronous machine in p.u (base SnRef)";
    Types.ActivePower PGen(start = PGen0Pu*SystemBase.SnRef) "Active power generated by the synchronous machine in MW";
    Types.ReactivePower QGen(start = QGen0Pu*SystemBase.SnRef) "Reactive power generated by the synchronous machine in Mvar";

    // Output variables for external controllers
    Connectors.ImPin UStatorPu(value(start = UStator0Pu)) "Stator voltage amplitude in p.u (base UNom)";
    Connectors.ImPin IStatorPu(value(start = IStator0Pu)) "Stator current amplitude in p.u (base UNom, SnRef)";
    Connectors.ImPin QStatorPu(value(start = QStator0Pu)) "Stator reactive power generated in p.u (base SnRef)";
    Connectors.ImPin QStatorPuQNom(value(start = QStator0PuQNom)) "Stator reactive power generated in p.u (base QNomAlt)";
    Connectors.ImPin IRotorPu(value(start = IRotor0Pu)) "Rotor current in p.u (base SNom, user-selected base voltage)";
    Connectors.ImPin thetaInternal(value(start = ThetaInternal0)) "Internal angle in rad";

  protected

    // Start values calculated by the initialization model
    parameter Types.ActivePowerPu PGen0Pu "Start value of active power at terminal in p.u (base SnRef) (generator convention)";
    parameter Types.ReactivePowerPu QGen0Pu "Start value of reactive power at terminal in p.u (base SnRef) (generator convention)";

    parameter Types.ComplexApparentPowerPu sStator0Pu "Start value of complex apparent power at stator side in p.u (base Snref)";
    parameter Types.ComplexVoltagePu uStator0Pu "Start value of complex voltage at stator side in p.u (base UNom)";
    parameter Types.ComplexCurrentPu iStator0Pu "Start value of complex current at stator side in p.u (base UNom, Snref)";

    parameter Types.VoltageModulePu UStator0Pu "Start value of stator voltage amplitude in p.u (base UNom)";
    parameter Types.CurrentModulePu IStator0Pu "Start value of stator current amplitude in p.u (base UNom, SnRef)";
    parameter Types.ReactivePowerPu QStator0Pu "Start value of stator reactive power generated in p.u (base SnRef)";
    parameter Types.ReactivePowerPu QStator0PuQNom "Start value of stator reactive power generated in p.u (base QNomAlt)";
    parameter Types.CurrentModulePu IRotor0Pu "Start value of rotor current in p.u (base SNom, user-selected base voltage)";
    parameter Types.Angle ThetaInternal0 "Start value of internal angle in rad";

    // Stator variables
    Types.ComplexApparentPowerPu sStatorPu(re(start = sStator0Pu.re), im(start = sStator0Pu.im)) "Complex apparent power at stator side in p.u (base Snref)";
    Types.ComplexVoltagePu uStatorPu(re(start = uStator0Pu.re), im(start = uStator0Pu.im)) "Complex voltage at stator side in p.u (base UNom)";
    Types.ComplexCurrentPu iStatorPu(re(start = iStator0Pu.re), im(start = iStator0Pu.im)) "Complex current at stator side in p.u (base UNom, Snref)";

equation

  if running.value then

    UPu = ComplexMath.'abs' (terminal.V);

    // Active and reactive power at terminal
    PGenPu = - ComplexMath.real(terminal.V * ComplexMath.conj(terminal.i));
    QGenPu = - ComplexMath.imag(terminal.V * ComplexMath.conj(terminal.i));
    PGen = PGenPu*SystemBase.SnRef;
    QGen = QGenPu*SystemBase.SnRef;

    // Stator complex variables
    uStatorPu = 1 / rTfoPu * (terminal.V - terminal.i * Complex(RTfoPu, XTfoPu) * SystemBase.SnRef / SNom);
    iStatorPu = rTfoPu * terminal.i;
    sStatorPu = uStatorPu * ComplexMath.conj(iStatorPu);

    // Output variables for external controllers
    UStatorPu.value = ComplexMath.'abs' (uStatorPu);
    IStatorPu.value = ComplexMath.'abs' (iStatorPu);
    QStatorPu.value = - ComplexMath.imag(sStatorPu);
    QStatorPuQNom.value = QStatorPu.value * SystemBase.SnRef / QNomAlt;
    IRotorPu.value = RfPPu / (Kuf * rTfoPu) * ifPu;
    thetaInternal.value = ComplexMath.arg(Complex(uqPu, udPu));

  else
    UPu = 0;
    PGenPu = 0;
    QGenPu = 0;
    PGen = 0;
    QGen = 0;
    uStatorPu = Complex(0);
    iStatorPu = Complex(0);
    sStatorPu = Complex(0);
    UStatorPu.value = 0;
    IStatorPu.value = 0;
    QStatorPu.value = 0;
    QStatorPuQNom.value = 0;
    IRotorPu.value = 0;
    thetaInternal.value = 0;
  end if;

annotation(preferredView = "text");
end GeneratorSynchronous;
