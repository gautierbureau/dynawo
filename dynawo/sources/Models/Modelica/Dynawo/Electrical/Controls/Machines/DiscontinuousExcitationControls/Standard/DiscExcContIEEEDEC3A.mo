within Dynawo.Electrical.Controls.Machines.DiscontinuousExcitationControls.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model DiscExcContIEEEDEC3A "IEEE Std 421.5-2016 Clause 14.4 Type DEC3A discontinuous excitation controller (CGMES DiscExcContIEEEDEC3A)"
  /*
    IEEE Std 421.5-2016 Clause 14.4 Type DEC3A. The simplest of the three
    DEC blocks: a temporary interruption of the stabilizing signal. The
    block passes the PSS signal VstPu through to its output Vs1Pu while
    the terminal voltage VtPu stays above VtMinPu. As soon as VtPu falls
    below VtMinPu the output is forced to zero and held there for tDr
    seconds (the "PSS switch open" interval), then re-enabled.

    This prevents the PSS from competing with the AVR during the first
    swing after a severe fault.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.VoltageModulePu VtMinPu "Terminal voltage threshold that triggers the PSS interruption";
  parameter Types.Time tDr "Duration of the PSS interruption in s";

  Modelica.Blocks.Interfaces.RealInput VtPu(start = 1) "Terminal voltage magnitude in pu";
  Modelica.Blocks.Interfaces.RealInput VstPu(start = 0) "PSS output signal in pu";
  Modelica.Blocks.Interfaces.RealOutput Vs1Pu(start = 0) "DEC3A output (PSS gated by the interruption logic) in pu";

  discrete Real tTrip(start = -1e9) "Last time the trigger fired (-inf at init)";
  Boolean open "PSS path is interrupted";

equation
  when VtPu < VtMinPu then
    tTrip = time;
  end when;

  open = (time - tTrip) < tDr;
  Vs1Pu = if open then 0 else VstPu;

  annotation(preferredView = "text");
end DiscExcContIEEEDEC3A;
