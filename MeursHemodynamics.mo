within ;
package MeursHemodynamics
  package Components
    model HeartInterval
     parameter Real AVDelay=0.01 "atrio-ventricular delay";
      Modelica.Blocks.Interfaces.RealInput HR "Heart rate - beats per minute"
        annotation (Placement(transformation(extent={{-124,-20},{-84,20}})));
       Modelica.Blocks.Interfaces.RealOutput Tas "length of
atrial systole in sec"     annotation (Placement(transformation(extent={{100,66},{120,
                86}}), iconTransformation(extent={{100,20},{120,40}})));
      Modelica.Blocks.Interfaces.RealOutput Tav "atrioventricular delay in sec"
        annotation (Placement(transformation(extent={{100,82},{120,102}}),
            iconTransformation(extent={{100,-40},{120,-20}})));
      Modelica.Blocks.Interfaces.RealOutput Tvs "legth of
ventricular systole in sec"     annotation (Placement(transformation(extent={{100,48},
                {120,68}}), iconTransformation(extent={{100,-90},{120,-70}})));
       Modelica.Blocks.Interfaces.RealOutput T0 "start time of
  current cardiac cycle in sec"     annotation (Placement(transformation(extent={{100,
                30},{120,50}}), iconTransformation(extent={{100,70},{120,90}})));
      Real HP(start=0) "heart period - duration of cardiac cycle in sec";
      Boolean b;
    equation
      b=time - pre(T0) >= pre(HP);
     when {initial(),b} then
        T0 = time;
        HP = 60/HR;
        Tas = 0.03 + 0.09*HP;
        Tav = AVDelay;
        Tvs = 0.16 + 0.2*HP;
     end when;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-92,12},{-42,-12}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="HR"),
            Text(
              extent={{28,44},{96,16}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Tas"),
            Text(
              extent={{28,-16},{96,-44}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Tav"),
            Text(
              extent={{30,-62},{98,-90}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Tvs"),
            Text(
              extent={{24,90},{92,62}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="T0"),
            Text(
              extent={{-100,-106},{108,-132}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="%name")}),                                 Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end HeartInterval;

    model AtrialElastance
      parameter Real EMAX_mmHg_per_ml = 1 "maximal systolic elastance in mmHg/ml";
      parameter Real EMIN_mmHg_per_ml = 0 "diastolic elastance in mmHg/ml";
      Real EMAX=EMAX_mmHg_per_ml*133.322387415/1e-6;
      Real EMIN=EMIN_mmHg_per_ml*133.322387415/1e-6;
      Modelica.Blocks.Interfaces.RealInput Tas annotation (Placement(transformation(
              extent={{-122,46},{-82,86}}), iconTransformation(extent={{-122,46},{-82,
                86}})));
      Modelica.Blocks.Interfaces.RealInput T0 annotation (Placement(transformation(
              extent={{-122,46},{-82,86}}), iconTransformation(extent={{-122,-54},{-82,
                -14}})));
      Types.HydraulicElastanceOutput Et annotation (Placement(transformation(extent={
                {-340,96},{-320,116}}), iconTransformation(extent={{100,-8},{120,12}})));
    equation
      if (time-T0<=Tas) then
        //Et = EMAX;
        Et = EMIN + (EMAX - EMIN) * sin(Modelica.Constants.pi * (time - T0) / Tas);
      else
        Et = EMIN; //it will be specified later
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-74,88},{-30,48}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Tas"),
            Text(
              extent={{-70,-14},{-26,-54}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="T0"),
            Text(
              extent={{50,20},{94,-20}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Et"),
            Text(
              extent={{-100,-106},{100,-128}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="%name")}),            Diagram(coordinateSystem(
              preserveAspectRatio=false)));
    end AtrialElastance;

    model VentricularElastance
      parameter Real EMAX_mmHg_per_ml = 1 "maximal systolic elastance in mmHg/ml";
      parameter Real EMIN_mmHg_per_ml = 0 "diastolic elastance in mmHg/ml";
      Real EMAX=EMAX_mmHg_per_ml*133.322387415/1e-6;
      Real EMIN=EMIN_mmHg_per_ml*133.322387415/1e-6;
      Modelica.Blocks.Interfaces.RealInput Tas "in sec" annotation (Placement(transformation(
              extent={{-120,56},{-80,96}}), iconTransformation(extent={{-120,10},{-80,
                50}})));
      Modelica.Blocks.Interfaces.RealInput T0 "in sec" annotation (Placement(transformation(
              extent={{-120,8},{-80,48}}),  iconTransformation(extent={{-120,60},{-80,
                100}})));
      Modelica.Blocks.Interfaces.RealInput Tav "in sec" annotation (Placement(transformation(
              extent={{-120,-80},{-80,-40}}),
                                            iconTransformation(extent={{-120,-50},{-80,
                -10}})));
      Modelica.Blocks.Interfaces.RealInput Tvs "in sec" annotation (Placement(transformation(
              extent={{-122,-30},{-82,10}}),iconTransformation(extent={{-120,-100},{
                -80,-60}})));
      Types.HydraulicElastanceOutput Et "in Pa/m3" annotation (Placement(
            transformation(extent={{-214,88},{-194,108}}), iconTransformation(
              extent={{100,-8},{120,12}})));
      Real heartInterval "in sec";
      constant Real Kn = 0.57923032735652;
    equation
      heartInterval = time-T0;
      if heartInterval>=(Tas+Tav) and  heartInterval<=(Tas+Tav+Tvs) then
        //Et = EMAX;
       Et = EMIN+(EMAX-EMIN)*(heartInterval - (Tas + Tav)) / Tvs * sin(Modelica.Constants.pi * (heartInterval - (Tas + Tav)) / Tvs) / Kn;
      else
        Et = EMIN; //it will be specified later
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-86,40},{-18,12}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Tas"),
            Text(
              extent={{-90,-14},{-22,-42}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Tav"),
            Text(
              extent={{-84,-64},{-16,-92}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Tvs"),
            Text(
              extent={{-94,98},{-26,70}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="T0"),
            Text(
              extent={{38,16},{106,-12}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Et"),
            Text(
              extent={{-100,-108},{102,-132}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="%name")}),            Diagram(coordinateSystem(
              preserveAspectRatio=false)));
    end VentricularElastance;

    model CardiacElastance

      parameter Real atrialElmin = 0 "minimal atrial elastance in mmHg/ml";
      parameter Real atrialElmax = 1 "maximal atrial elastance in mmHg/ml";
      parameter Real ventricularElmin = 0 "minimal ventricular elastance in mmHg/ml";
      parameter Real ventricularElmax = 1 "maximal ventricular elastance in mmHg/ml";

      Modelica.Blocks.Interfaces.RealInput HR "Heart rate - beats per minute"
        annotation (Placement(transformation(extent={{-114,-40},{-74,0}}),
            iconTransformation(extent={{-130,-20},{-90,20}})));

      Types.HydraulicElastanceOutput Etv annotation (Placement(transformation(
              extent={{74,-30},{94,-10}}),iconTransformation(extent={{100,52},{120,72}})));
      Types.HydraulicElastanceOutput Eta annotation (Placement(transformation(
              extent={{68,50},{88,70}}),   iconTransformation(extent={{100,-46},{120,
                -26}})));
      AtrialElastance atrialElastance( EMAX_mmHg_per_ml=atrialElmax,EMIN_mmHg_per_ml=atrialElmin)
        annotation (Placement(transformation(extent={{0,42},{38,78}})));
      VentricularElastance ventricularElastance( EMAX_mmHg_per_ml=ventricularElmax,EMIN_mmHg_per_ml=ventricularElmin)
        annotation (Placement(transformation(extent={{2,-40},{40,0}})));
      HeartInterval heartInterval
        annotation (Placement(transformation(extent={{-66,-40},{-28,0}})));
    equation
      connect(heartInterval.T0, ventricularElastance.T0)
        annotation (Line(points={{-26.1,-4},{2,-4}}, color={0,0,127}));
      connect(HR, heartInterval.HR) annotation (Line(points={{-94,-20},{-66.76,-20}},
                                  color={0,0,127}));
      connect(heartInterval.Tas, ventricularElastance.Tas)
        annotation (Line(points={{-26.1,-14},{2,-14}}, color={0,0,127}));
      connect(heartInterval.Tav, ventricularElastance.Tav)
        annotation (Line(points={{-26.1,-26},{2,-26}}, color={0,0,127}));
      connect(heartInterval.Tvs, ventricularElastance.Tvs)
        annotation (Line(points={{-26.1,-36},{2,-36}}, color={0,0,127}));
      connect(atrialElastance.T0, ventricularElastance.T0) annotation (Line(points={
              {-0.38,53.88},{-10,53.88},{-10,-4},{2,-4}}, color={0,0,127}));
      connect(atrialElastance.Tas, ventricularElastance.Tas) annotation (Line(
            points={{-0.38,71.88},{-16,71.88},{-16,-14},{2,-14}}, color={0,0,127}));
      connect(atrialElastance.Et, Eta) annotation (Line(points={{39.9,60.36},{55.95,
              60.36},{55.95,60},{78,60}}, color={0,0,127}));
      connect(ventricularElastance.Et, Etv) annotation (Line(points={{41.9,-19.6},{61.95,
              -19.6},{61.95,-20},{84,-20}}, color={0,0,127}));
        annotation (Placement(transformation(extent={{20,-20},{60,20}})),
                    Placement(transformation(extent={{18,28},{56,60}})),
                  Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-92,16},{-16,-20}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="HR"),
            Text(
              extent={{26,80},{102,44}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Etv"),
            Text(
              extent={{28,-16},{104,-52}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Eta"),
            Text(
              extent={{-184,-112},{172,-140}},
              lineColor={28,108,200},
              textString="%name")}),
                                   Diagram(coordinateSystem(preserveAspectRatio=false)));
    end CardiacElastance;

    connector VolumeFlowConnector "Connector for blood flow"
      flow Types.VolumeFlowRate q "blood flow in m3/sec";
      Types.Pressure pressure "Pressure in Pa";
    end VolumeFlowConnector;

    connector VolumeFlowInflow "Blood flow inflow"
      extends VolumeFlowConnector;
      annotation (
        Icon(graphics={  Rectangle(visible = true, origin = {2.04082, -1.0101}, fillColor = {255, 0, 0},
                fillPattern =                                                                                          FillPattern.Solid, extent = {{-102.041, -98.9899}, {97.9592, 101.01}})}, coordinateSystem(extent = {{-100.0, -100.0}, {100.0, 100.0}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})));
    end VolumeFlowInflow;

    connector VolumeFlowOutflow "Blood flow inflow"
      extends VolumeFlowConnector;
      annotation (
        Icon(graphics={  Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255},
                fillPattern =                                                                                                    FillPattern.Solid)}));
    end VolumeFlowOutflow;

    partial model VolumeFlowOnePort
      Types.Pressure pressureDrop;
      Types.VolumeFlowRate bloodFlow;
      VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{-374,20},{-354,40}}), iconTransformation(extent={{-114,-10},
                {-94,10}})));
      VolumeFlowOutflow bloodFlowOutflow annotation (Placement(transformation(
              extent={{-374,44},{-354,64}}), iconTransformation(extent={{86,-10},
                {106,10}})));
    equation
      pressureDrop = bloodFlowInflow.pressure - bloodFlowOutflow.pressure;
      bloodFlowInflow.q + bloodFlowOutflow.q = 0;
      bloodFlow = bloodFlowInflow.q;
      annotation (
        Icon(graphics));
    end VolumeFlowOnePort;

    model Resistor
      extends VolumeFlowOnePort;
      parameter Real bloodResistance_NonSI  "resistance in mmHg s/ml";
      Real bloodResistance = bloodResistance_NonSI * 133.322387415/1e-6 "resistance in Pa s/m3";
    equation
      pressureDrop = bloodFlow * bloodResistance;
      annotation (
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics={  Line(visible = true, origin = {-70.0, 0.0}, points = {{-20.0, 0.0}, {20.0, 0.0}}), Line(visible = true, origin = {70.0, 0.0}, points = {{-20.0, 0.0}, {20.0, 0.0}}), Text(visible = true, extent={{
                  -240,-140},{260,-110}},                                                                                                                                                                                                        fontName = "Arial", textString = "%name", lineColor = {0, 0, 0}),                                                                           Polygon(points = {{-100, 100}, {0, 0}, {-100, -100}, {-100, 100}}, lineColor = {0, 0, 255}, smooth = Smooth.None, fillColor = {215, 215, 215},
                fillPattern =                                                                                                    FillPattern.Solid), Polygon(points = {{100, 100}, {100, -100}, {0, 0}, {100, 100}}, lineColor = {0, 0, 255}, smooth = Smooth.None, fillColor = {215, 215, 215},
                fillPattern =                                                                                                    FillPattern.Solid), Text(extent = {{-50, -40}, {50, -100}}, lineColor = {0, 0, 255}, textString = "R")}),
        Diagram(coordinateSystem(extent = {{-148.5, -105.0}, {148.5, 105.0}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
    end Resistor;

    model Conductance
      extends VolumeFlowOnePort;
      parameter Real bloodConductance_NonSI  "conductance in ml/(mmHg s)";
      Real bloodConductance = bloodConductance_NonSI * 1e-6/133.322387415 "conductance in m3/(Pa s)";
    equation
      pressureDrop * bloodConductance = bloodFlow;
      annotation (
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics={  Line(visible = true, origin = {-70.0, 0.0}, points = {{-20.0, 0.0}, {20.0, 0.0}}), Line(visible = true, origin = {70.0, 0.0}, points = {{-20.0, 0.0}, {20.0, 0.0}}), Text(visible = true, extent = {{-180, -140}, {190, -110}}, fontName = "Arial", textString = "%name", lineColor = {0, 0, 0}),                                                                                   Polygon(points = {{-100, 100}, {0, 0}, {-100, -100}, {-100, 100}}, lineColor = {0, 0, 255}, smooth = Smooth.None, fillColor = {215, 215, 215},
                fillPattern =                                                                                                    FillPattern.Solid), Polygon(points = {{100, 100}, {100, -100}, {0, 0}, {100, 100}}, lineColor = {0, 0, 255}, smooth = Smooth.None, fillColor = {215, 215, 215},
                fillPattern =                                                                                                    FillPattern.Solid), Text(extent = {{-30, -40}, {40, -90}}, lineColor = {0, 0, 255}, textString = "G")}),
        Diagram(coordinateSystem(extent = {{-148.5, -105.0}, {148.5, 105.0}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5}), graphics={  Rectangle(visible = true, fillColor = {255, 255, 255}, extent = {{-60.0, -20.0}, {60.0, 20.0}}), Rectangle(visible = true, origin = {-80.0, 0.0}, fillColor = {255, 255, 255}, extent = {{-20.0, 0.0}, {20.0, 0.0}}), Rectangle(visible = true, origin = {80.0, 0.0}, fillColor = {255, 255, 255}, extent = {{-20.0, 0.0}, {20.0, 0.0}})}));
    end Conductance;

    model Inductor
      extends VolumeFlowOnePort;
      parameter Real inertance_NonSI "inertance in mmHg s2/ml";
      Real inertance = inertance_NonSI*133.322387415/1e-6 "Inertance in Pa s2/m3";
    equation
      pressureDrop = der(bloodFlow) * inertance;
      annotation (
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics={  Line(visible = true, origin={
                  -80,0},                                                                                                                                                                    points = {{-20.0, 0.0}, {20.0, 0.0}}), Line(visible = true, origin = {70.0, 0.0}, points = {{-20.0, 0.0}, {20.0, 0.0}}), Text(visible = true, extent={{
                  -260,-100},{270,-70}},                                                                                                                                                                                                        textString = "%name", fontName = "Arial"),                                                                                   Rectangle(visible = true,
                lineThickness =                                                                                                    1, extent = {{-100.0, -55.0}, {100.0, 75.0}}), Rectangle(visible = true, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, pattern = LinePattern.None,
                fillPattern =                                                                                                    FillPattern.Solid,
                lineThickness =                                                                                                    1, extent={{
                  -65,-50},{55,70}}),                                                                                                                                           Text(visible = true, origin={
                  -11.8817,52.9448},
                fillPattern =                                                                                                    FillPattern.Solid, extent = {{-78.8574, -9.8385}, {78.8574, 9.8385}}, textString = "Inertance", fontName = "Arial")}),
        Diagram(coordinateSystem(extent = {{-148.5, -105.0}, {148.5, 105.0}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
    end Inductor;

    model Valve
      Real bloodFlow;
      Real pressureDrop;
      Real S "parametric independent variable";
      Boolean open;
      VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{-10,-10},{10,10}}), iconTransformation(extent={{-100,-10},
                {-80,10}})));
      VolumeFlowOutflow bloodFlowOutflow annotation (Placement(transformation(
              extent={{-80,-30},{-60,-10}}), iconTransformation(extent={{80,-10},
                {100,10}})));
    equation
      bloodFlowInflow.q + bloodFlowOutflow.q = 0;
      bloodFlow = bloodFlowInflow.q;
      pressureDrop = bloodFlowInflow.pressure - bloodFlowOutflow.pressure;
      open = S > 0;
      if open then
        pressureDrop = 0;
        bloodFlow = S;
      else
        pressureDrop = S;
        bloodFlow = 0;
      end if;
      annotation (
        Icon(graphics={  Rectangle(visible = true, lineColor = {0, 0, 255}, fillColor = {255, 170, 170},
                fillPattern =                                                                                          FillPattern.Solid, extent = {{42, -100}, {70, 100}}), Polygon(visible = true, lineColor = {0, 0, 255}, fillColor = {255, 170, 170},
                fillPattern =                                                                                                    FillPattern.Solid, points = {{-70, 100}, {-70, -100}, {36, -12}, {36, 12}, {-70, 100}}), Text(visible = true, extent = {{-166, -142}, {174, -116}}, textString = "%name", fontName = "Arial")},
                                                                                                                                                                                                        coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})),
        Diagram(                     coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})));
    end Valve;

  model CardiacValve
      parameter Real outflowResistance "outflow resistance of valve in mmHg/ml";
      parameter Real backflowConductance = 0 "backflow conductance of valve in mmHg/ml";
      Valve outflowValve
        annotation (Placement(transformation(extent={{-56,20},{-16,60}})));
      Valve backflowValve annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=180,
            origin={-34,-40})));
      VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{-98,-10},{-78,10}}), iconTransformation(extent={{-110,-10},
                {-90,10}})));
      VolumeFlowOutflow bloodFlowOutflow annotation (Placement(transformation(
              extent={{78,-10},{98,10}}), iconTransformation(extent={{90,-10},{
                110,10}})));
      Conductance backflowBloodConductance(bloodConductance_NonSI=
            backflowConductance) annotation (Placement(transformation(
            extent={{-18,-18},{18,18}},
            rotation=180,
            origin={22,-40})));
      Resistor outflowBloodResistor(bloodResistance_NonSI=outflowResistance)
        annotation (Placement(transformation(extent={{4,22},{40,58}})));
  equation
      connect(outflowValve.bloodFlowInflow, bloodFlowInflow) annotation (Line(
            points={{-54,40},{-66,40},{-66,0},{-88,0}},  color={0,0,0},
          thickness=1));
    connect(bloodFlowInflow, bloodFlowInflow)
      annotation (Line(points={{-88,0},{-88,0}}, color={0,0,0}));
    connect(backflowValve.bloodFlowOutflow, bloodFlowInflow) annotation (Line(
        points={{-52,-40},{-66,-40},{-66,0},{-88,0}},
        color={0,0,0},
        thickness=1));
    connect(backflowBloodConductance.bloodFlowInflow, bloodFlowOutflow)
      annotation (Line(
        points={{40.72,-40},{72,-40},{72,0},{88,0}},
        color={0,0,0},
        thickness=1));
    connect(backflowBloodConductance.bloodFlowOutflow, backflowValve.bloodFlowInflow)
      annotation (Line(
        points={{4.72,-40},{-16,-40}},
        color={0,0,0},
        thickness=1));
    connect(outflowValve.bloodFlowOutflow, outflowBloodResistor.bloodFlowInflow)
      annotation (Line(
        points={{-18,40},{3.28,40}},
        color={0,0,0},
        thickness=1));
    connect(outflowBloodResistor.bloodFlowOutflow, bloodFlowOutflow) annotation (
        Line(
        points={{39.28,40},{72,40},{72,0},{88,0}},
        color={0,0,0},
        thickness=1));
      annotation (Placement(transformation(extent={{8,24},{40,56}})),
      Icon(graphics={  Polygon(visible = true, fillColor = {255, 170, 170},
              fillPattern =                                                               FillPattern.Solid, points={{
                  -100,100},{80,0},{-100,-100},{-100,100}}),                                                                                                                              Rectangle(visible = true, fillColor = {255, 170, 170},
              fillPattern =                                                                                                    FillPattern.Solid, extent = {{80.0, -100.0}, {100.0, 100.0}}), Text(visible = true, origin={
                7.02304,-15.4544},                                                                                                                                                                                                        fillColor = {0, 0, 255},
              fillPattern =                                                                                                    FillPattern.Solid, extent={{
                -267.023,-114.545},{252.977,-94.5456}},                                                                                                                                         textString = "%name", fontName = "Arial")},       coordinateSystem(extent = {{-100.0, -100.0}, {100.0, 100.0}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})));
  end CardiacValve;

  model ElasticCompartment
      VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{-50,-35},{-30,-15}}), iconTransformation(extent={{-10,-10},
                {10,10}})));
    parameter Real elastance_NonSI "elastance in mmHg/ml";
    Types.HydraulicElastance elastance = elastance_NonSI *133.322387415/1e-6 "elastance in Pa/m3";

    parameter Real unstressedVolume_NonSI "volume in ml";
    Types.Volume unstressedVolume=unstressedVolume_NonSI*1e-6 "unstressed volume from ml to m3";

    parameter Real externalPressure_NonSI "external pressure in mmHg";
    Types.Pressure externalPressure=externalPressure_NonSI*133.322387415 "external pressure from mmHg to Pa";

    parameter Real V0_NonSI "initial volume in ml";
    Types.Volume volume(start = V0_NonSI*1e-6);

    Types.Volume stressedVolume "volume in m3";
    Types.Pressure transmuralPressure "pressure in Pa";
    Types.Pressure pressure "pressure in Pa";
      Types.VolumeOutput y annotation (Placement(transformation(extent={{-200,
                -40},{-180,-20}}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={90,-60})));
  equation
    bloodFlowInflow.pressure = pressure;
    transmuralPressure = pressure - externalPressure;
    der(volume) = bloodFlowInflow.q;
    stressedVolume = volume - unstressedVolume;
    if stressedVolume > 0 then
      transmuralPressure = elastance * stressedVolume;
    else
      transmuralPressure = 0;
    end if;
    annotation (
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics={  Ellipse(                origin={
                2.9379,0},                                                                                                                                                                     fillColor=
                {215,215,215},
              fillPattern=FillPattern.Solid,                                                                                                      extent = {{-102.9379, -100.0}, {102.9379, 100.0}},
            lineColor={0,0,0}),                                                                                                                                                                       Text(                origin={
                8.837,-90.936},                                                                                                                                                                                                        extent = {{-147.185, -57.4195}, {151.163, -29.0642}}, fontName=
                "Arial",                                                                                                                                                                                                        lineColor=
                {0,0,0},
            textString="%name")}),
      Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end ElasticCompartment;

  model VariableElasticCompartment
      VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{-50,-35},{-30,-15}}), iconTransformation(extent={{-10,-10},
                {10,10}})));
    Types.HydraulicElastanceInput inputElastance annotation (Placement(
          transformation(extent={{-120,60},{-80,100}}), iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=270,
          origin={0,110})));

    Types.HydraulicElastance elastance = inputElastance "input elastance in Pa/m3";

    parameter Real unstressedVolume_NonSI "volume in ml";
    Types.Volume unstressedVolume=unstressedVolume_NonSI*1e-6 "unstressed volume from ml to m3";

    parameter Real externalPressure_NonSI "external pressure in mmHg";
    Types.Pressure externalPressure=externalPressure_NonSI*133.322387415 "external pressure from mmHg to Pa";

    parameter Real V0_NonSI "initial volume in ml";
    Types.Volume volume(start = V0_NonSI*1e-6);

    Types.Volume stressedVolume "volume in m3";
    Types.Pressure transmuralPressure "pressure in Pa";
    Types.Pressure pressure "pressure in Pa";
  equation
    bloodFlowInflow.pressure = pressure;
    transmuralPressure = pressure - externalPressure;
    der(volume) = bloodFlowInflow.q;
    stressedVolume = volume - unstressedVolume;
    if stressedVolume > 0 then
      transmuralPressure = elastance * stressedVolume;
    else
      transmuralPressure = 0;
    end if;
    annotation (
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics={  Ellipse(                origin={0,
                0},                                                                                                                                                                            fillColor=
                {215,215,215},
              fillPattern=FillPattern.Solid,                                                                                                      extent={{
                -100,-100},{100,100}},
            lineColor={0,0,0}),                                                                                                                                                                       Text(                origin={
                8.837,-90.936},                                                                                                                                                                                                        extent = {{-147.185, -57.4195}, {151.163, -29.0642}}, fontName=
                "Arial",                                                                                                                                                                                                        lineColor=
                {0,0,0},
            textString="%name"),
          Ellipse(extent={{-100,50},{100,-50}}, lineColor={0,0,0})}),
      Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end VariableElasticCompartment;

  model ElasticCompartmentWithOptionalInputs
      VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{-50,-35},{-30,-15}}), iconTransformation(extent={{-10,-10},
                {10,10}})));
    Types.PressureInput externalPressureInput=externalPressure if useExternalPressureInput annotation (Placement(
          transformation(extent={{-95,20},{-55,60}}),   iconTransformation(extent={{-20,-20},
                {20,20}},
            rotation=270,
            origin={40,100})));
    Types.VolumeInput UnstressedVolumeInput=unstressedVolume if useUnstressedVolumeInput annotation (Placement(transformation(extent={{-140,45},
              {-100,85}}),
                     iconTransformation(extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,100})));
      Types.HydraulicElastanceInput hydraulicElastanceInput=elastance if     useHydraulicElastanceInput
        annotation (Placement(transformation(extent={{-400,30},{-360,70}}),
            iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
    parameter Real elastance_NonSI "elastance in mmHg/ml";
    parameter Real unstressedVolume_NonSI=0 "volume in ml";
    parameter Real externalPressure_NonSI=0 "external pressure in mmHg";
    parameter Boolean useExternalPressureInput = false;
    parameter Boolean useUnstressedVolumeInput = false;
    parameter Boolean useHydraulicElastanceInput = false;
    Types.HydraulicElastance elastance;
    Types.Pressure externalPressure;
    Types.Volume unstressedVolume;
    parameter Real V0_NonSI=0 "initial volume in ml";
    Types.Volume volume(start = V0_NonSI*1e-6);
    Types.Volume stressedVolume "volume in m3";
    Types.Pressure transmuralPressure "pressure in Pa";
    Types.Pressure pressure "pressure in Pa";
  equation
    if not useExternalPressureInput then
      externalPressure = externalPressure_NonSI*133.322387415 "external pressure from mmHg to Pa";
    end if;
    if not useUnstressedVolumeInput then
      unstressedVolume = unstressedVolume_NonSI*1e-6 "unstressed volume from ml to m3";
    end if;
    if not useHydraulicElastanceInput then
      elastance = elastance_NonSI *133.322387415/1e-6 "elastance in Pa/m3";
    end if;
    bloodFlowInflow.pressure = pressure;
    transmuralPressure = pressure - externalPressure;
    der(volume) = bloodFlowInflow.q;
    stressedVolume = volume - unstressedVolume;
    if stressedVolume > 0 then
      transmuralPressure = elastance * stressedVolume;
    else
      transmuralPressure = 0;
    end if;
    annotation (
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics={  Ellipse(                origin={0,
                  0},                                                                                                                                                                          fillColor=
                  {215,215,215},
              fillPattern=FillPattern.Solid,                                                                                                      extent={{
                  -100,-100},{100,100}},
            lineColor={0,0,0}),                                                                                                                                                                       Text(                origin={
                8.837,-90.936},                                                                                                                                                                                                        extent = {{-147.185, -57.4195}, {151.163, -29.0642}}, fontName=
                "Arial",                                                                                                                                                                                                        lineColor=
                {0,0,0},
            textString="%name"),
          Ellipse(
            extent={{-100,40},{100,-40}},
            lineColor={0,0,0},
            startAngle=0,
            endAngle=360)}),
      Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end ElasticCompartmentWithOptionalInputs;

    model UnlimitedVolumeSource "unlimited blood source with given pressure"
      VolumeFlowOutflow bloodFlowOutflow annotation (Placement(transformation(
              extent={{-216,52},{-196,72}}), iconTransformation(extent={{78,-10},
                {98,10}})));
    parameter Real pressure_NonSI "in mmHg";
    Types.Pressure pressure = pressure_NonSI*133.322387415 "pressure in Pa";
    equation
       bloodFlowOutflow.pressure = pressure;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
              extent={{-82,54},{56,36}},
              lineColor={127,0,0},
              pattern=LinePattern.None,
              lineThickness=0.5),
            Ellipse(
              extent={{-100,66},{68,-66}},
              fillColor={244,125,35},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Rectangle(
              extent={{14,12},{100,-14}},
              fillColor={244,125,35},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Polygon(
              points={{2,0},{42,0},{42,8},{64,0},{42,-12},{42,-4},{2,-4},{2,0}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid)}),
                                    Diagram(coordinateSystem(
              preserveAspectRatio=false)));
    end UnlimitedVolumeSource;

    model UnlimitedVolumeSink "unlimited blood outflow with given pressure"
      VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{-216,44},{-196,64}}), iconTransformation(extent={{-98,-10},
                {-78,10}})));
    parameter Real pressure_NonSI "in mmHg";
    Types.Pressure pressure = pressure_NonSI*133.322387415 "pressure in Pa";

    equation
      bloodFlowInflow.pressure=pressure;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
              extent={{-82,54},{56,36}},
              lineColor={127,0,0},
              pattern=LinePattern.None,
              lineThickness=0.5),
            Ellipse(
              extent={{-74,64},{100,-62}},
              fillColor={244,125,35},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Rectangle(
              extent={{-100,12},{-14,-14}},
              fillColor={244,125,35},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Polygon(
              points={{-31,2},{9,2},{9,10},{31,2},{9,-10},{9,-2},{-31,-2},{-31,2}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              origin={-27,-2},
              rotation=360)}),      Diagram(coordinateSystem(
              preserveAspectRatio=false)));
    end UnlimitedVolumeSink;

    model Heart
      parameter Real currentHeartRate=72 "heart rate in beats per min";
      VolumeFlowInflow rightAtriumFlowInflow annotation (Placement(
            transformation(extent={{-98,-4},{-90,4}}), iconTransformation(
              extent={{-82,0},{-62,20}})));
      VolumeFlowOutflow pulmonaryArteryOutflow annotation (Placement(
            transformation(extent={{-10,-4},{-2,4}}), iconTransformation(extent=
               {{-42,36},{-22,56}})));
      VolumeFlowInflow leftAtriumFlowInflow annotation (Placement(
            transformation(extent={{2,-4},{10,4}}), iconTransformation(extent={
                {24,36},{44,56}})));
      VolumeFlowOutflow aortaOutflow annotation (Placement(transformation(
              extent={{92,-4},{100,4}}), iconTransformation(extent={{68,0},{88,
                20}})));
      VariableElasticCompartment rightAtrium(
        unstressedVolume_NonSI=30,
        externalPressure_NonSI=-4,
        V0_NonSI=135)
        annotation (Placement(transformation(extent={{-86,-10},{-66,10}})));
      VariableElasticCompartment leftAtrium(
        unstressedVolume_NonSI=30,
        externalPressure_NonSI=-4,
        V0_NonSI=93.1)
        annotation (Placement(transformation(extent={{12,-10},{32,10}})));
      VariableElasticCompartment rightVentricle(
        unstressedVolume_NonSI=40,
        externalPressure_NonSI=-4,
        V0_NonSI=131)
        annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
      VariableElasticCompartment leftVentricle(
        unstressedVolume_NonSI=60,
        externalPressure_NonSI=-4,
        V0_NonSI=144)
        annotation (Placement(transformation(extent={{52,-10},{72,10}})));
      CardiacValve tricuspidalValve(outflowResistance=0.003, backflowConductance=0)
        annotation (Placement(transformation(extent={{-64,-6},{-52,6}})));
      CardiacValve pulmonicValve(outflowResistance=0.003)
        annotation (Placement(transformation(extent={{-26,-6},{-14,6}})));
      CardiacValve mitralValve(outflowResistance=0.003, backflowConductance=0)
        annotation (Placement(transformation(extent={{36,-6},{48,6}})));
      CardiacValve aorticValve(outflowResistance=0.003, backflowConductance=0)
        annotation (Placement(transformation(extent={{76,-6},{88,6}})));
      CardiacElastance rightCardiacElastance(
        atrialElmin=0.05,
        atrialElmax=0.15,
        ventricularElmin=0.057,
        ventricularElmax=0.49) annotation (Placement(transformation(
            extent={{-7,-7},{7,7}},
            rotation=0,
            origin={-87,33})));
      CardiacElastance leftCardiacElastance(
        atrialElmin=0.12,
        atrialElmax=0.28,
        ventricularElmin=0.09,
        ventricularElmax=4) annotation (Placement(transformation(
            extent={{-7,-7},{7,7}},
            rotation=0,
            origin={7,33})));
      Modelica.Blocks.Sources.Constant heartRate(k=currentHeartRate)
        annotation (Placement(transformation(extent={{-78,54},{-64,68}})));
    equation
      connect(rightAtrium.bloodFlowInflow, rightAtriumFlowInflow) annotation (Line(
          points={{-76,0},{-94,0}},
          color={28,108,200},
          smooth=Smooth.Bezier,
          thickness=1));
      connect(rightAtrium.bloodFlowInflow, tricuspidalValve.bloodFlowInflow)
        annotation (Line(
          points={{-76,0},{-64,0}},
          color={28,108,200},
          smooth=Smooth.Bezier,
          thickness=1));
      connect(tricuspidalValve.bloodFlowOutflow, rightVentricle.bloodFlowInflow)
        annotation (Line(
          points={{-52,0},{-52,0},{-40,0}},
          color={28,108,200},
          smooth=Smooth.Bezier,
          thickness=1));
      connect(rightVentricle.bloodFlowInflow, pulmonicValve.bloodFlowInflow)
        annotation (Line(
          points={{-40,0},{-26,0}},
          color={28,108,200},
          thickness=1));
      connect(pulmonaryArteryOutflow, pulmonicValve.bloodFlowOutflow) annotation (
          Line(
          points={{-6,0},{-14,0}},
          color={28,108,200},
          thickness=1));
      connect(leftAtrium.bloodFlowInflow, leftAtriumFlowInflow) annotation (Line(
          points={{22,0},{6,0}},
          color={238,46,47},
          thickness=1));
      connect(leftAtrium.bloodFlowInflow, mitralValve.bloodFlowInflow) annotation (
          Line(
          points={{22,0},{36,0}},
          color={238,46,47},
          thickness=1));
      connect(mitralValve.bloodFlowOutflow, leftVentricle.bloodFlowInflow)
        annotation (Line(
          points={{48,0},{62,0}},
          color={238,46,47},
          thickness=1));
      connect(aorticValve.bloodFlowInflow, leftVentricle.bloodFlowInflow)
        annotation (Line(
          points={{76,0},{62,0}},
          color={238,46,47},
          thickness=1));
      connect(aorticValve.bloodFlowOutflow, aortaOutflow) annotation (Line(
          points={{88,0},{96,0}},
          color={238,46,47},
          thickness=1));
      connect(rightCardiacElastance.Eta, rightAtrium.inputElastance) annotation (
          Line(points={{-79.3,30.48},{-76,30.48},{-76,11}}, color={0,0,127}));
      connect(rightCardiacElastance.Etv, rightVentricle.inputElastance) annotation (
         Line(points={{-79.3,37.34},{-40,37.34},{-40,11}}, color={0,0,127}));
      connect(leftCardiacElastance.Eta, leftAtrium.inputElastance) annotation (Line(
            points={{14.7,30.48},{22,30.48},{22,11}}, color={0,0,127}));
      connect(leftCardiacElastance.Etv, leftVentricle.inputElastance) annotation (
          Line(points={{14.7,37.34},{62,37.34},{62,11}}, color={0,0,127}));
      connect(rightCardiacElastance.HR, heartRate.y) annotation (Line(points={{-94.7,
              33},{-98,33},{-98,42},{-52,42},{-52,61},{-63.3,61}}, color={0,0,127}));
      connect(leftCardiacElastance.HR, heartRate.y) annotation (Line(points={{-0.7,33},
              {-10,33},{-10,42},{-52,42},{-52,61},{-63.3,61}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
              points={{0,56},{-20,62},{-56,80},{-90,28},{-88,-28},{-30,-90},{26,-88},
                  {56,-70},{88,-30},{94,30},{64,66},{44,80},{22,68},{0,56}},
              lineColor={255,0,0},
              smooth=Smooth.Bezier,
              fillColor={244,125,35},
              fillPattern=FillPattern.Solid), Text(
              extent={{-154,-106},{158,-132}},
              lineColor={0,0,0},
              lineThickness=1,
              fillColor={255,0,0},
              fillPattern=FillPattern.Solid,
              textString="%name")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Heart;

    model SystemicCirculation
      VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{96,-2},{100,2}}), iconTransformation(extent={{86,-10},{
                106,10}})));
      VolumeFlowOutflow bloodFlowOutflow annotation (Placement(transformation(
              extent={{-100,-2},{-96,2}}), iconTransformation(extent={{-112,-10},
                {-92,10}})));
      ElasticCompartment intrathoracicArteries(
        elastance_NonSI=1.43,
        unstressedVolume_NonSI=140,
        externalPressure_NonSI=-4,
        V0_NonSI=204)
        annotation (Placement(transformation(extent={{82,-32},{102,-12}})));
      Resistor extrathoracicArterialResistance(bloodResistance_NonSI=0.06)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={48,0})));
      Inductor aorticFlowInertia(inertance_NonSI=0.0017) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={78,0})));
      Resistor systemicArteriolarResistance(bloodResistance_NonSI=0.8)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={10,0})));
      ElasticCompartment extrathoracicArteries(
        elastance_NonSI=0.556,
        unstressedVolume_NonSI=370,
        externalPressure_NonSI=0,
        V0_NonSI=526)
        annotation (Placement(transformation(extent={{20,-32},{40,-12}})));
      ElasticCompartment SystemicTissues(
        elastance_NonSI=0.262,
        unstressedVolume_NonSI=185,
        externalPressure_NonSI=0,
        V0_NonSI=283)
        annotation (Placement(transformation(extent={{-16,-32},{4,-12}})));
      Resistor smallVenuleResistance(bloodResistance_NonSI=0.2) annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-24,0})));
      Resistor venousResistance(bloodResistance_NonSI=0.09) annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-52,0})));
      ElasticCompartment intrathoracicVeins(
        elastance_NonSI=0.0182,
        unstressedVolume_NonSI=1190,
        externalPressure_NonSI=-4,
        V0_NonSI=1480)
        annotation (Placement(transformation(extent={{-78,-32},{-58,-12}})));
      ElasticCompartment extrathoracicVeins(
        elastance_NonSI=0.0169,
        unstressedVolume_NonSI=1000,
        externalPressure_NonSI=0,
        V0_NonSI=1530)
        annotation (Placement(transformation(extent={{-48,-32},{-28,-12}})));
      Resistor centralVenousResistance(bloodResistance_NonSI=0.003) annotation (
         Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-82,0})));




    equation
      connect(aorticFlowInertia.bloodFlowInflow, intrathoracicArteries.bloodFlowInflow)
        annotation (Line(
          points={{88.4,-1.77636e-15},{92,-1.77636e-15},{92,-22}},
          color={238,46,47},
          thickness=1));
      connect(intrathoracicArteries.bloodFlowInflow, bloodFlowInflow)
        annotation (Line(
          points={{92,-22},{92,0},{98,0}},
          color={238,46,47},
          thickness=1));
      connect(aorticFlowInertia.bloodFlowOutflow,
        extrathoracicArterialResistance.bloodFlowInflow) annotation (Line(
          points={{68.4,6.66134e-16},{62,6.66134e-16},{62,-1.77636e-15},{58.4,
              -1.77636e-15}},
          color={238,46,47},
          thickness=1));
      connect(extrathoracicArterialResistance.bloodFlowOutflow,
        extrathoracicArteries.bloodFlowInflow) annotation (Line(
          points={{38.4,4.44089e-16},{30,4.44089e-16},{30,-22}},
          color={238,46,47},
          thickness=1));
      connect(extrathoracicArteries.bloodFlowInflow,
        systemicArteriolarResistance.bloodFlowInflow) annotation (Line(
          points={{30,-22},{30,-1.77636e-15},{20.4,-1.77636e-15}},
          color={255,0,0},
          thickness=1));
      connect(SystemicTissues.bloodFlowInflow, systemicArteriolarResistance.bloodFlowOutflow)
        annotation (Line(
          points={{-6,-22},{-6,6.66134e-16},{0.4,6.66134e-16}},
          color={238,46,47},
          thickness=1));
      connect(smallVenuleResistance.bloodFlowInflow, SystemicTissues.bloodFlowInflow)
        annotation (Line(
          points={{-13.6,-1.77636e-15},{-6,-1.77636e-15},{-6,-22}},
          color={28,108,200},
          thickness=1));
      connect(smallVenuleResistance.bloodFlowOutflow, extrathoracicVeins.bloodFlowInflow)
        annotation (Line(
          points={{-33.6,6.66134e-16},{-38,6.66134e-16},{-38,-22}},
          color={28,108,200},
          thickness=1));
      connect(bloodFlowOutflow, centralVenousResistance.bloodFlowOutflow)
        annotation (Line(
          points={{-98,0},{-91.6,0}},
          color={28,108,200},
          thickness=1));
      connect(smallVenuleResistance.bloodFlowOutflow, venousResistance.bloodFlowInflow)
        annotation (Line(
          points={{-33.6,6.66134e-16},{-40,6.66134e-16},{-40,-1.77636e-15},{
              -41.6,-1.77636e-15}},
          color={28,108,200},
          thickness=1));
      connect(centralVenousResistance.bloodFlowInflow, venousResistance.bloodFlowOutflow)
        annotation (Line(
          points={{-71.6,0},{-60,0},{-60,4.44089e-16},{-61.6,4.44089e-16}},
          color={28,108,200},
          thickness=1));
      connect(centralVenousResistance.bloodFlowInflow, intrathoracicVeins.bloodFlowInflow)
        annotation (Line(
          points={{-71.6,0},{-68,0},{-68,-22}},
          color={28,108,200},
          thickness=1));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
              extent={{-100,20},{100,-20}},
              lineColor={255,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={244,125,35}), Text(
              extent={{-128,-28},{122,-46}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={244,125,35},
              textString="%name")}),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=5,
          __Dymola_NumberOfIntervals=50000,
          __Dymola_Algorithm="Dassl"));
    end SystemicCirculation;

    model PulmonaryCirculation



      VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{-62,-2},{-58,2}}), iconTransformation(extent={{-112,-10},
                {-92,10}})));
      VolumeFlowOutflow bloodFlowOutflow annotation (Placement(transformation(
              extent={{78,-2},{82,2}}), iconTransformation(extent={{90,-10},{
                110,10}})));
      ElasticCompartment pulmonaryArteries(
        elastance_NonSI=0.233,
        unstressedVolume_NonSI=50,
        externalPressure_NonSI=-4,
        V0_NonSI=99)
        annotation (Placement(transformation(extent={{-50,-30},{-30,-10}})));
      Resistor pulmonaryResistance(bloodResistance_NonSI=0.11)
        annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
      ElasticCompartment pulmonaryVeins(
        elastance_NonSI=0.0455,
        unstressedVolume_NonSI=350,
        externalPressure_NonSI=-4,
        V0_NonSI=516)
        annotation (Placement(transformation(extent={{10,-30},{30,-10}})));
      Resistor pulmonaryVenousResistance(bloodResistance_NonSI=0.003)
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    equation
      connect(bloodFlowInflow, pulmonaryArteries.bloodFlowInflow) annotation (
          Line(
          points={{-60,0},{-40,0},{-40,-20}},
          color={28,108,200},
          thickness=1));
      connect(pulmonaryResistance.bloodFlowInflow, pulmonaryArteries.bloodFlowInflow)
        annotation (Line(
          points={{-20.4,0},{-40,0},{-40,-20}},
          color={28,108,200},
          thickness=1));
      connect(pulmonaryResistance.bloodFlowOutflow, pulmonaryVeins.bloodFlowInflow)
        annotation (Line(
          points={{-0.4,0},{20,0},{20,-20}},
          color={238,46,47},
          thickness=1));
      connect(pulmonaryVenousResistance.bloodFlowInflow, pulmonaryVeins.bloodFlowInflow)
        annotation (Line(
          points={{39.6,0},{20,0},{20,-20}},
          color={238,46,47},
          thickness=1));
      connect(pulmonaryVenousResistance.bloodFlowOutflow, bloodFlowOutflow)
        annotation (Line(
          points={{59.6,0},{80,0}},
          color={238,46,47},
          thickness=1));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
              extent={{-100,20},{100,-20}},
              lineColor={255,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={244,125,35}), Text(
              extent={{-128,-28},{122,-46}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={244,125,35},
              textString="%name")}), Diagram(coordinateSystem(
              preserveAspectRatio=false)));
    end PulmonaryCirculation;

    model vanMeursHemodynamicsModel
      Heart heart(currentHeartRate=72)
        annotation (Placement(transformation(extent={{-22,-22},{20,24}})));
      SystemicCirculation systemicCirculation
        annotation (Placement(transformation(extent={{-30,-84},{30,-24}})));
      PulmonaryCirculation pulmonaryCirculation
        annotation (Placement(transformation(extent={{-30,14},{30,74}})));
    equation
      connect(systemicCirculation.bloodFlowOutflow, heart.rightAtriumFlowInflow)
        annotation (Line(
          points={{-30.6,-54},{-40,-54},{-40,3.3},{-16.12,3.3}},
          color={28,108,200},
          thickness=1));
      connect(heart.pulmonaryArteryOutflow, pulmonaryCirculation.bloodFlowInflow)
        annotation (Line(
          points={{-7.72,11.58},{-40,11.58},{-40,44},{-30.6,44}},
          color={28,108,200},
          thickness=1));
      connect(systemicCirculation.bloodFlowInflow, heart.aortaOutflow)
        annotation (Line(
          points={{28.8,-54},{40,-54},{40,3.3},{15.38,3.3}},
          color={255,0,0},
          thickness=1));
      connect(heart.leftAtriumFlowInflow, pulmonaryCirculation.bloodFlowOutflow)
        annotation (Line(
          points={{6.14,11.58},{40,11.58},{40,44},{30,44}},
          color={238,46,47},
          thickness=1));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={Ellipse(
              extent={{-62,62},{64,-64}},
              lineColor={255,0,0},
              pattern=LinePattern.None,
              lineThickness=1,
              fillPattern=FillPattern.Sphere,
              fillColor={244,125,35})}),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=10,
          __Dymola_NumberOfIntervals=50000,
          __Dymola_Algorithm="Dassl"));
    end vanMeursHemodynamicsModel;

    model BreathInterval

      Modelica.Blocks.Interfaces.RealInput RR "Respiration rate - breaths per minute"
        annotation (Placement(transformation(extent={{-124,-20},{-84,20}})));
       Modelica.Blocks.Interfaces.RealOutput Pm "realtive pressure of breath muscle (from 0 to 1)" annotation (Placement(transformation(extent={{100,-10},
                {120,10}}),     iconTransformation(extent={{100,-10},{120,10}})));
      Real HB(start=0) "breath period - duration of breath cycle in sec";
      Boolean b;

      Real Ti "length of inspiration";
      Real Te "length of expiration";
      Real T0 "start time of current breath in sec";
    equation
      b=time - pre(T0) >= pre(HB);
     when {initial(),b} then
        T0 = time;
        HB = 60/RR;
        Ti = HB/3;
        Te = Ti*2;
     end when;

     if (time-T0) < Ti then
       Pm=(time-T0)/Ti;
     else
       Pm=(exp(-(time-(T0+Ti))/Te/0.4)-exp(-1/0.4))/(1-exp(-1/0.4));
     end if;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-92,12},{-42,-12}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="RR"),
            Text(
              extent={{34,16},{102,-12}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="Pm"),
            Text(
              extent={{-100,-106},{108,-132}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.None,
              textString="%name")}),                                 Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end BreathInterval;

  model ComplianceCompartment
      VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{-50,-35},{-30,-15}}), iconTransformation(extent={{-10,-10},
                {10,10}})));
    parameter Real compliance_NonSI "compliance in ml/mmHg";
    Types.HydraulicCompliance compliance = compliance_NonSI *1E-6/133.322387415 "compliance in m3/Pa";

    parameter Real unstressedVolume_NonSI "volume in ml";
    Types.Volume unstressedVolume=unstressedVolume_NonSI*1e-6 "unstressed volume from ml to m3";

    parameter Real externalPressure_NonSI "external pressure in mmHg";
    Types.Pressure externalPressure=externalPressure_NonSI*133.322387415 "external pressure from mmHg to Pa";

    parameter Real V0_NonSI "initial volume in ml";
    parameter Boolean hardElastance=false;

    //Types.Volume volume(start = V0_NonSI*1e-6);

    Types.VolumeOutput volume(start = V0_NonSI*1e-6) annotation (Placement(transformation(extent={{-200,0},{-180,
              20}}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={70,-70})));

    Types.Volume stressedVolume "volume in m3";
    Types.Pressure transmuralPressure "pressure in Pa";
    Types.Pressure pressure "pressure in Pa";

  equation
    bloodFlowInflow.pressure = pressure;
    transmuralPressure = pressure - externalPressure;
    der(volume) = bloodFlowInflow.q;
    stressedVolume = volume - unstressedVolume;
    if hardElastance then
      transmuralPressure = stressedVolume/compliance;
    else
      if stressedVolume > 0 then
        transmuralPressure = stressedVolume/compliance;
      else
        transmuralPressure = 0;
      end if;
    end if;

    annotation (Icon(coordinateSystem(
          extent={{-100,-100},{100,100}},
          preserveAspectRatio=true,
          initialScale=0.1,
          grid={10,10}), graphics={Ellipse(
            origin={0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            extent={{-100,-100},{100,100}},
            lineColor={0,0,0}), Text(
            origin={8.837,-90.936},
            extent={{-147.185,-57.4195},{151.163,-29.0642}},
            fontName="Arial",
            lineColor={0,0,0},
            textString="%name")}), Diagram(coordinateSystem(
          extent={{-148.5,-105},{148.5,105}},
          preserveAspectRatio=true,
          initialScale=0.1,
          grid={5,5})));
  end ComplianceCompartment;

  model ComplianceCompartmentWithOptionalInputs
    VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(extent={
              {-50,-35},{-30,-15}}), iconTransformation(extent={{-10,-10},{10,10}})));
    Types.PressureInput externalPressureInput=externalPressure if useExternalPressureInput annotation (Placement(
          transformation(extent={{-95,20},{-55,60}}),   iconTransformation(extent={{-20,-20},
              {20,20}},
          rotation=270,
          origin={40,100})));
    Types.VolumeInput UnstressedVolumeInput=unstressedVolume if useUnstressedVolumeInput annotation (Placement(transformation(extent={{-140,45},
              {-100,85}}),
                     iconTransformation(extent={{-20,-20},{20,20}},
          rotation=270,
          origin={-40,100})));
      Types.HydraulicComplianceInput hydraulicComplianceInput=compliance if     useHydraulicComplianceInput
        annotation (Placement(transformation(extent={{-400,30},{-360,70}}),
            iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
    parameter Real compliance_NonSI = 1 "compliance in ml/mmHg";
    parameter Real unstressedVolume_NonSI= 0 "volume in ml";
    parameter Real externalPressure_NonSI= 0 "external pressure in mmHg";
    parameter Boolean useExternalPressureInput = false;
    parameter Boolean useUnstressedVolumeInput = false;
    parameter Boolean useHydraulicComplianceInput = false;
    parameter Boolean hardElastance=false;
    Types.HydraulicCompliance compliance;
    Types.Pressure externalPressure;
    Types.Volume unstressedVolume;
    parameter Real V0_NonSI= 0 "initial volume in ml";
    //Types.Volume volume(start = V0_NonSI*1e-6);
    Types.VolumeOutput volume(start = V0_NonSI*1e-6) annotation (Placement(transformation(extent={{-145,-5},{-125,
              15}}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={70,-70})));
    Types.Volume stressedVolume "volume in m3";
    Types.Pressure transmuralPressure "pressure in Pa";
    Types.Pressure pressure "pressure in Pa";
  equation
    if not useExternalPressureInput then
      externalPressure = externalPressure_NonSI*133.322387415 "external pressure from mmHg to Pa";
    end if;
    if not useUnstressedVolumeInput then
      unstressedVolume = unstressedVolume_NonSI*1e-6 "unstressed volume from ml to m3";
    end if;
    if not useHydraulicComplianceInput then
      compliance = 1E-6*compliance_NonSI /133.322387415 "compliance in m3/Pa";
    end if;
    bloodFlowInflow.pressure = pressure;
    transmuralPressure = pressure - externalPressure;
    der(volume) = bloodFlowInflow.q;
    stressedVolume = volume - unstressedVolume;
    if hardElastance then
      transmuralPressure = stressedVolume/compliance;
    else
      if stressedVolume > 0 then
        transmuralPressure = stressedVolume/compliance;
      else
        transmuralPressure = 0;
      end if;
    end if;

    annotation (
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics={  Ellipse(                origin={0,
                  0},                                                                                                                                                                          fillColor=
                  {215,215,215},
              fillPattern=FillPattern.Solid,                                                                                                      extent={{
                  -100,-100},{100,100}},
            lineColor={0,0,0}),                                                                                                                                                                       Text(                origin={
                8.837,-90.936},                                                                                                                                                                                                        extent = {{-147.185, -57.4195}, {151.163, -29.0642}}, fontName=
                "Arial",                                                                                                                                                                                                        lineColor=
                {0,0,0},
            textString="%name"),
          Ellipse(
            extent={{-100,40},{100,-40}},
            lineColor={0,0,0},
            startAngle=0,
            endAngle=360)}),
      Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end ComplianceCompartmentWithOptionalInputs;

  model OuterElasticVessel
    VolumeFlowInflow bloodFlowInflow annotation (Placement(transformation(extent={
              {-50,-35},{-30,-15}}), iconTransformation(extent={{-10,-90},{10,-70}})));
    Types.PressureInput externalPressureInput=externalPressure if useExternalPressureInput annotation (Placement(
          transformation(extent={{-95,20},{-55,60}}),   iconTransformation(extent={{-20,-20},
              {20,20}},
          rotation=270,
          origin={-20,10})));
    Types.VolumeInput UnstressedVolumeInput=unstressedVolume if useUnstressedVolumeInput annotation (Placement(transformation(extent={{-140,45},
              {-100,85}}),
                     iconTransformation(extent={{-20,-20},{20,20}},
          rotation=270,
          origin={-100,10})));
      Types.HydraulicComplianceInput hydraulicComplianceInput=compliance if     useHydraulicComplianceInput
        annotation (Placement(transformation(extent={{-400,30},{-360,70}}),
            iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-60,10})));
    parameter Real compliance_NonSI "compliance in mmHg/ml";
    parameter Real unstressedVolume_NonSI "volume in ml";
    parameter Real externalPressure_NonSI "external pressure in mmHg";
    parameter Boolean useExternalPressureInput = false;
    parameter Boolean useUnstressedVolumeInput = false;
    parameter Boolean useHydraulicComplianceInput = false;
    parameter Boolean hardElastance=false;
    Types.HydraulicCompliance compliance;
    Types.Pressure externalPressure;
    Types.Volume unstressedVolume;
    parameter Real V0_NonSI "initial inner volume in ml";
    //Types.Volume volume(start = V0_NonSI*1e-6);
    Types.VolumeOutput volume(start=OuterVolume+InnerVolume) annotation (Placement(transformation(extent={{-145,-5},{-125,
              15}}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={70,-70})));
    Types.Volume OuterVolume( start = V0_NonSI*1e-6, fixed=true);
    Types.Volume stressedVolume "volume in m3";
    Types.Pressure transmuralPressure "pressure in Pa";
    //Types.Pressure pressure "pressure in Pa";
    Types.VolumeInput InnerVolume
      annotation (Placement(transformation(extent={{-140,45},{-100,85}}),
          iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=270,
          origin={70,-30})));
    Types.PressureOutput pressure "pressure in Pa" annotation (Placement(transformation(extent={{-200,
              -40},{-180,-20}}), iconTransformation(extent={{100,-30},{120,-10}})));
  equation
    if not useExternalPressureInput then
      externalPressure = externalPressure_NonSI*133.322387415 "external pressure from mmHg to Pa";
    end if;
    if not useUnstressedVolumeInput then
      unstressedVolume = unstressedVolume_NonSI*1e-6 "unstressed volume from ml to m3";
    end if;
    if not useHydraulicComplianceInput then
      compliance = 1E-6*compliance_NonSI /133.322387415 "compliance in m3/Pa";
    end if;
    bloodFlowInflow.pressure = pressure;
    transmuralPressure = pressure - externalPressure;

    der(OuterVolume) = bloodFlowInflow.q;
    volume=OuterVolume+InnerVolume;

    stressedVolume = volume - unstressedVolume;
    if hardElastance then
      transmuralPressure = stressedVolume/compliance;
    else
      if stressedVolume > 0 then
        transmuralPressure = stressedVolume/compliance;
      else
        transmuralPressure = 0;
      end if;
    end if;

    annotation (
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics={                                                         Text(                origin={
                8.837,-90.936},                                                                                                                                                                                                        extent = {{-147.185, -57.4195}, {151.163, -29.0642}}, fontName=
                "Arial",                                                                                                                                                                                                        lineColor=
                {0,0,0},
            textString="%name"),
          Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            closure=EllipseClosure.Chord,
            startAngle=180,
            endAngle=360,
            origin={0,0},
            rotation=180),
          Ellipse(
            extent={{-100,60},{100,-60}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            closure=EllipseClosure.Chord,
            startAngle=180,
            endAngle=360,
            origin={0,7.10543e-15},
            rotation=180)}),
      Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end OuterElasticVessel;

    model WattMeter
      extends Modelica.Icons.RoundSensor;
      MeursHemodynamics.Components.VolumeFlowInflow volumeFlowInflow annotation (
          Placement(transformation(extent={{-108,-10},{-88,10}}),
            iconTransformation(extent={{-110,-8},{-90,12}})));
      MeursHemodynamics.Components.VolumeFlowOutflow volumeFlowOutflow annotation (
          Placement(transformation(extent={{92,-10},{112,10}}), iconTransformation(
              extent={{90,-6},{110,14}})));
      Types.PowerOutput power annotation (Placement(transformation(extent={{-192,-54},
                {-172,-34}}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-94})));
    equation
      volumeFlowInflow.q+volumeFlowOutflow.q=0;
      volumeFlowInflow.pressure=volumeFlowOutflow.pressure;
      power=volumeFlowInflow.q*volumeFlowInflow.pressure;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Text(
              extent={{-58,-14},{70,-70}},
              textColor={28,108,200},
              textString="W")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end WattMeter;

    model FlowMeter
      extends Modelica.Icons.RoundSensor;
      MeursHemodynamics.Components.VolumeFlowInflow volumeFlowInflow annotation (
          Placement(transformation(extent={{-108,-10},{-88,10}}),
            iconTransformation(extent={{-110,-8},{-90,12}})));
      MeursHemodynamics.Components.VolumeFlowOutflow volumeFlowOutflow annotation (
          Placement(transformation(extent={{92,-10},{112,10}}), iconTransformation(
              extent={{90,-6},{110,14}})));
      Types.VolumeFlowRateOutput volumeFlow annotation (Placement(transformation(extent={{-248,
                -36},{-228,-16}}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-102})));
    equation
      volumeFlowInflow.q+volumeFlowOutflow.q=0;
      volumeFlowInflow.pressure=volumeFlowOutflow.pressure;
      volumeFlow=volumeFlowInflow.q;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Text(
              extent={{-68,-12},{70,-76}},
              textColor={28,108,200},
              textString="Q"), Line(points={{-90,82},{80,82},{60,90},{60,74},{
                  80,82}}, color={0,0,0})}),
                                 Diagram(coordinateSystem(preserveAspectRatio=false)));
    end FlowMeter;

    model PressureMeter
      extends Modelica.Icons.RoundSensor;
      MeursHemodynamics.Components.VolumeFlowInflow volumeFlowInflow annotation (
          Placement(transformation(extent={{-108,-10},{-88,10}}),
            iconTransformation(extent={{-10,-96},{10,-76}})));
      Types.PressureOutput pressure annotation (Placement(transformation(extent={{-240,-16},
                {-220,4}}), iconTransformation(extent={{76,-14},{104,14}})));
    equation
      pressure=volumeFlowInflow.pressure
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Text(
              extent={{-68,-12},{70,-76}},
              textColor={28,108,200},
              textString="P")}), Diagram(coordinateSystem(preserveAspectRatio=false)));

    end PressureMeter;
  end Components;

  package Tests
    model TestCardiacElastances
      Modelica.Blocks.Sources.Constant HeartRate(k=72)
        annotation (Placement(transformation(extent={{-96,10},{-76,30}})));
      Modelica.Blocks.Math.Gain REtv(k=1)
        annotation (Placement(transformation(extent={{46,64},{66,84}})));
      Modelica.Blocks.Math.Gain REta(k=1)
        annotation (Placement(transformation(extent={{46,32},{66,52}})));
      Components.CardiacElastance rightHeart(
        atrialElmin=0.05,
        atrialElmax=0.15,
        ventricularElmin=0.057,
        ventricularElmax=0.49)
        annotation (Placement(transformation(extent={{-34,42},{0,80}})));
      Components.CardiacElastance leftHeart(
        atrialElmin=0.12,
        atrialElmax=0.28,
        ventricularElmin=0.09,
        ventricularElmax=4)
        annotation (Placement(transformation(extent={{-38,-30},{-4,8}})));
      Modelica.Blocks.Math.Gain LEtv(k=1)
        annotation (Placement(transformation(extent={{42,-10},{62,10}})));
      Modelica.Blocks.Math.Gain LEta(k=1)
        annotation (Placement(transformation(extent={{42,-42},{62,-22}})));
    equation
      connect(rightHeart.HR, HeartRate.y) annotation (Line(points={{-35.7,61},{
              -62,61},{-62,20},{-75,20}}, color={0,0,127}));
      connect(leftHeart.HR, HeartRate.y) annotation (Line(points={{-39.7,-11},{
              -62,-11},{-62,20},{-75,20}}, color={0,0,127}));
      connect(rightHeart.Etv, REtv.u) annotation (Line(points={{1.7,72.78},{
              22.85,72.78},{22.85,74},{44,74}}, color={0,0,127}));
      connect(rightHeart.Eta, REta.u) annotation (Line(points={{1.7,54.16},{28,
              54.16},{28,42},{44,42}}, color={0,0,127}));
      connect(leftHeart.Etv, LEtv.u) annotation (Line(points={{-2.3,0.78},{
              17.85,0.78},{17.85,0},{40,0}}, color={0,0,127}));
      connect(leftHeart.Eta, LEta.u) annotation (Line(points={{-2.3,-17.84},{26,
              -17.84},{26,-32},{40,-32}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end TestCardiacElastances;

    model TestOptionalElasticCompartment
      Components.ElasticCompartmentWithOptionalInputs OneInput(
          useExternalPressureInput=false, useHydraulicElastanceInput=true)
        annotation (Placement(transformation(extent={{-36,34},{-10,60}})));
      Components.ElasticCompartmentWithOptionalInputs TwoInputs(
          useUnstressedVolumeInput=true, useHydraulicElastanceInput=true)
        annotation (Placement(transformation(extent={{2,34},{28,60}})));
      Components.ElasticCompartmentWithOptionalInputs ThreeInputs(
        elastance_NonSI=ThreeInputs,
        useExternalPressureInput=true,
        useUnstressedVolumeInput=true,
        useHydraulicElastanceInput=true)
        annotation (Placement(transformation(extent={{42,34},{68,60}})));
      Components.ElasticCompartmentWithOptionalInputs NoInputs
        annotation (Placement(transformation(extent={{-76,34},{-50,60}})));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end TestOptionalElasticCompartment;

    model TestVanMeursHemodynamicsModel
      Components.Heart heart
        annotation (Placement(transformation(extent={{-22,-22},{20,24}})));
      Components.SystemicCirculation systemicCirculation
        annotation (Placement(transformation(extent={{-30,-84},{30,-24}})));
      Components.PulmonaryCirculation pulmonaryCirculation
        annotation (Placement(transformation(extent={{-30,14},{30,74}})));
    equation
      connect(systemicCirculation.bloodFlowOutflow, heart.rightAtriumFlowInflow)
        annotation (Line(
          points={{-30.6,-54},{-40,-54},{-40,3.3},{-16.12,3.3}},
          color={28,108,200},
          thickness=1));
      connect(heart.pulmonaryArteryOutflow, pulmonaryCirculation.bloodFlowInflow)
        annotation (Line(
          points={{-7.72,11.58},{-40,11.58},{-40,44},{-30.6,44}},
          color={28,108,200},
          thickness=1));
      connect(systemicCirculation.bloodFlowInflow, heart.aortaOutflow)
        annotation (Line(
          points={{28.8,-54},{40,-54},{40,3.3},{15.38,3.3}},
          color={255,0,0},
          thickness=1));
      connect(heart.leftAtriumFlowInflow, pulmonaryCirculation.bloodFlowOutflow)
        annotation (Line(
          points={{6.14,11.58},{40,11.58},{40,44},{30,44}},
          color={238,46,47},
          thickness=1));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={Ellipse(
              extent={{-62,62},{64,-64}},
              lineColor={255,0,0},
              pattern=LinePattern.None,
              lineThickness=1,
              fillPattern=FillPattern.Sphere,
              fillColor={244,125,35})}),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=10,
          __Dymola_NumberOfIntervals=50000,
          __Dymola_Algorithm="Dassl"));
    end TestVanMeursHemodynamicsModel;

    model TestBreathInterval
      Components.BreathInterval breathInterval
        annotation (Placement(transformation(extent={{-26,-6},{-6,14}})));
      Modelica.Blocks.Sources.Constant BreathRate(k=12)
        annotation (Placement(transformation(extent={{-70,-6},{-50,14}})));
    equation
      connect(BreathRate.y, breathInterval.RR)
        annotation (Line(points={{-49,4},{-26.4,4}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end TestBreathInterval;

    model testElasticCompartments
      MeursHemodynamicsPhysiolibrary.PhysiolibraryExpantion.ElasticVessel CL(
        volume_start=0.0023,
        ZeroPressureVolume=0.0013,
        Compliance(displayUnit="ml/mmHg") = 2.0394324259559e-06,
        useExternalPressureInput=true,
        hardElastance=true)
        annotation (Placement(transformation(extent={{56,-42},{76,-22}})));
      Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume2(
          usePressureInput=false)
        annotation (Placement(transformation(extent={{-38,-42},{-18,-22}})));
      MeursHemodynamicsPhysiolibrary.PhysiolibraryExpantion.OuterElasticVessel
                                                Cw(
        outer_volume_start=0,
        useV0Input=false,
        ZeroPressureVolume=0.00352,
        useComplianceInput=false,
        Compliance(displayUnit="ml/mmHg") = 2.4881075596661e-06,
        useExternalPressureInput=false,
        hardElastance=false)
        annotation (Placement(transformation(extent={{60,-66},{80,-46}})));
      Physiolibrary.Hydraulic.Components.Resistor resistor(Resistance(
            displayUnit="(mmHg.s)/ml") = 362846.05)
        annotation (Placement(transformation(extent={{26,-42},{46,-22}})));
      Components.BreathInterval                   breathInterval1
        annotation (Placement(transformation(extent={{-30,-90},{-10,-70}})));
      Modelica.Blocks.Sources.Constant BreathRate(k=6)
        annotation (Placement(transformation(extent={{-76,-90},{-56,-70}})));
      Physiolibrary.Types.Constants.PressureConst RespiratoryMuscleAmplitude(k(
            displayUnit="cmH2O") = -441.29925)
        annotation (Placement(transformation(extent={{-20,-58},{-12,-50}})));
      Modelica.Blocks.Math.Product product1
        annotation (Placement(transformation(extent={{10,-82},{30,-62}})));
      Components.UnlimitedVolumeSource unlimitedVolumeSource(pressure_NonSI=0)
        annotation (Placement(transformation(extent={{-60,34},{-40,54}})));
      Components.Resistor resistor1(bloodResistance_NonSI=0.0027215688005237)
        annotation (Placement(transformation(extent={{-26,34},{-6,54}})));
      Components.ComplianceCompartmentWithOptionalInputs CL1(
        compliance_NonSI=271.902,
        unstressedVolume_NonSI=1300,
        externalPressure_NonSI=-3.6777956,
        useExternalPressureInput=false,
        useUnstressedVolumeInput=false,
        useHydraulicComplianceInput=false,
        hardElastance=true,
        V0_NonSI=2300)
        annotation (Placement(transformation(extent={{22,34},{42,54}})));
      Components.UnlimitedVolumeSource unlimitedVolumeSource1(pressure_NonSI=0)
        annotation (Placement(transformation(extent={{-62,62},{-42,82}})));
      Components.Resistor resistor2(bloodResistance_NonSI=0.0027215688005237)
        annotation (Placement(transformation(extent={{-32,62},{-12,82}})));
      Components.ComplianceCompartment CL0(
        compliance_NonSI=271.902,
        unstressedVolume_NonSI=1300,
        externalPressure_NonSI=-3.6777956,
        V0_NonSI=2300,
        hardElastance=true)
        annotation (Placement(transformation(extent={{18,62},{38,82}})));
      Components.UnlimitedVolumeSource unlimitedVolumeSource2(pressure_NonSI=0)
        annotation (Placement(transformation(extent={{-52,-4},{-32,16}})));
      Components.Resistor resistor3(bloodResistance_NonSI=0.0027215688005237)
        annotation (Placement(transformation(extent={{-18,-4},{2,16}})));
      Components.ComplianceCompartmentWithOptionalInputs CL2(
        compliance_NonSI=271.902,
        unstressedVolume_NonSI=1300,
        externalPressure_NonSI=-3.6777956,
        useExternalPressureInput=true,
        hardElastance=true,
        V0_NonSI=2300)
        annotation (Placement(transformation(extent={{30,-4},{50,16}})));
    equation
      connect(Cw.InnerVolume,CL. volume)
        annotation (Line(points={{76.2,-58},{72,-58},{72,-42}},
                                                           color={0,0,127}));
      connect(Cw.pressure,CL. externalPressure) annotation (Line(points={{82.1,
              -57.1},{82.1,-20},{74,-20},{74,-24}},
                                          color={0,0,127}));
      connect(resistor.q_out,CL. q_in) annotation (Line(
          points={{46,-32},{66,-32}},
          color={0,0,0},
          thickness=1));
      connect(BreathRate.y,breathInterval1. RR)
        annotation (Line(points={{-55,-80},{-30.4,-80}}, color={0,0,127}));
      connect(breathInterval1.Pm,product1. u2) annotation (Line(points={{-9,-80},
              {2,-80},{2,-78},{8,-78}},color={0,0,127}));
      connect(RespiratoryMuscleAmplitude.y,product1. u1) annotation (Line(points={{-11,-54},
              {2,-54},{2,-66},{8,-66}},   color={0,0,127}));
      connect(unlimitedVolume2.y, resistor.q_in) annotation (Line(
          points={{-18,-32},{26,-32}},
          color={0,0,0},
          thickness=1));
      connect(unlimitedVolumeSource.bloodFlowOutflow, resistor1.bloodFlowInflow)
        annotation (Line(points={{-41.2,44},{-26.4,44}}, color={0,0,0}));
      connect(resistor1.bloodFlowOutflow, CL1.bloodFlowInflow)
        annotation (Line(points={{-6.4,44},{32,44}}, color={0,0,0}));
      connect(unlimitedVolumeSource1.bloodFlowOutflow, resistor2.bloodFlowInflow)
        annotation (Line(points={{-43.2,72},{-32.4,72}}, color={0,0,0}));
      connect(resistor2.bloodFlowOutflow, CL0.bloodFlowInflow)
        annotation (Line(points={{-12.4,72},{28,72}}, color={0,0,0}));
      connect(unlimitedVolumeSource2.bloodFlowOutflow, resistor3.bloodFlowInflow)
        annotation (Line(points={{-33.2,6},{-18.4,6}}, color={0,0,0}));
      connect(resistor3.bloodFlowOutflow,CL2. bloodFlowInflow)
        annotation (Line(points={{1.6,6},{40,6}},    color={0,0,0}));
      connect(CL2.externalPressureInput, CL.externalPressure) annotation (Line(
            points={{44,16},{44,24},{80,24},{80,-20},{74,-20},{74,-24}}, color=
              {0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end testElasticCompartments;

    model testRespiration
      MeursHemodynamicsPhysiolibrary.PhysiolibraryExpantion.ElasticVessel CL(
        volume_start=0.0023,
        ZeroPressureVolume=0.0013,
        Compliance(displayUnit="ml/mmHg") = 2.0394324259559e-06,
        useExternalPressureInput=true,
        hardElastance=true)
        annotation (Placement(transformation(extent={{56,-42},{76,-22}})));
      Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume2(
          usePressureInput=false)
        annotation (Placement(transformation(extent={{-38,-42},{-18,-22}})));
      MeursHemodynamicsPhysiolibrary.PhysiolibraryExpantion.OuterElasticVessel
                                                Cw(
        outer_volume_start=0,
        useV0Input=false,
        ZeroPressureVolume=0.00352,
        useComplianceInput=false,
        Compliance(displayUnit="ml/mmHg") = 2.4881075596661e-06,
        useExternalPressureInput=true,
        hardElastance=false)
        annotation (Placement(transformation(extent={{60,-66},{80,-46}})));
      Physiolibrary.Hydraulic.Components.Resistor resistor(Resistance(
            displayUnit="(mmHg.s)/ml") = 362846.05)
        annotation (Placement(transformation(extent={{26,-42},{46,-22}})));
      Components.BreathInterval                   breathInterval1
        annotation (Placement(transformation(extent={{-30,-90},{-10,-70}})));
      Modelica.Blocks.Sources.Constant BreathRate(k=12)
        annotation (Placement(transformation(extent={{-76,-90},{-56,-70}})));
      Physiolibrary.Types.Constants.PressureConst RespiratoryMuscleAmplitude(k(
            displayUnit="cmH2O") = -441.29925)
        annotation (Placement(transformation(extent={{-20,-58},{-12,-50}})));
      Modelica.Blocks.Math.Product product1
        annotation (Placement(transformation(extent={{10,-82},{30,-62}})));
      Components.UnlimitedVolumeSource unlimitedVolumeSource(pressure_NonSI=0)
        annotation (Placement(transformation(extent={{-92,34},{-72,54}})));
      Components.Resistor resistor1(bloodResistance_NonSI=0.0027215688005237)
        annotation (Placement(transformation(extent={{-40,34},{-20,54}})));
      Components.ComplianceCompartmentWithOptionalInputs CL1(
        compliance_NonSI=271.902,
        unstressedVolume_NonSI=1300,
        externalPressure_NonSI=0,
        useExternalPressureInput=true,
        hardElastance=true,
        V0_NonSI=2300)
        annotation (Placement(transformation(extent={{22,34},{42,54}})));
      Components.OuterElasticVessel Cw1(
        compliance_NonSI=331.72044,
        unstressedVolume_NonSI=3520,
        externalPressure_NonSI=0,
        useExternalPressureInput=true,
        hardElastance=true,
        V0_NonSI=0)
        annotation (Placement(transformation(extent={{22,6},{42,26}})));
      Components.FlowMeter flowMeter
        annotation (Placement(transformation(extent={{-10,34},{10,54}})));
      Physiolibrary.Hydraulic.Sensors.FlowMeasure flowMeasure
        annotation (Placement(transformation(extent={{-6,-42},{14,-22}})));
    equation
      connect(Cw.InnerVolume,CL. volume)
        annotation (Line(points={{76.2,-58},{72,-58},{72,-42}},
                                                           color={0,0,127}));
      connect(Cw.pressure,CL. externalPressure) annotation (Line(points={{82.1,
              -57.1},{82.1,-20},{74,-20},{74,-24}},
                                          color={0,0,127}));
      connect(resistor.q_out,CL. q_in) annotation (Line(
          points={{46,-32},{66,-32}},
          color={0,0,0},
          thickness=1));
      connect(BreathRate.y,breathInterval1. RR)
        annotation (Line(points={{-55,-80},{-30.4,-80}}, color={0,0,127}));
      connect(breathInterval1.Pm,product1. u2) annotation (Line(points={{-9,-80},
              {2,-80},{2,-78},{8,-78}},color={0,0,127}));
      connect(RespiratoryMuscleAmplitude.y,product1. u1) annotation (Line(points={{-11,-54},
              {2,-54},{2,-66},{8,-66}},   color={0,0,127}));
      connect(unlimitedVolumeSource.bloodFlowOutflow, resistor1.bloodFlowInflow)
        annotation (Line(points={{-73.2,44},{-40.4,44}}, color={0,0,0}));
      connect(product1.y, Cw.externalPressure) annotation (Line(points={{31,-72},
              {42,-72},{42,-48},{68.4,-48},{68.4,-52}}, color={0,0,127}));
      connect(Cw1.pressure, CL1.externalPressureInput) annotation (Line(points=
              {{43,14},{64,14},{64,60},{36,60},{36,54}}, color={0,0,127}));
      connect(CL1.volume, Cw1.InnerVolume)
        annotation (Line(points={{39,37},{39,13}}, color={0,0,127}));
      connect(Cw1.externalPressureInput, Cw.externalPressure) annotation (Line(
            points={{30,17},{30,22},{12,22},{12,-48},{68,-48},{68,-52},{68.4,-52}},
            color={0,0,127}));
      connect(resistor1.bloodFlowOutflow, flowMeter.volumeFlowInflow)
        annotation (Line(points={{-20.4,44},{-16,44},{-16,44.2},{-10,44.2}},
            color={0,0,0}));
      connect(flowMeter.volumeFlowOutflow, CL1.bloodFlowInflow)
        annotation (Line(points={{10,44.4},{32,44}}, color={0,0,0}));
      connect(unlimitedVolume2.y, flowMeasure.q_in) annotation (Line(
          points={{-18,-32},{-6,-32}},
          color={0,0,0},
          thickness=1));
      connect(flowMeasure.q_out, resistor.q_in) annotation (Line(
          points={{14,-32},{26,-32}},
          color={0,0,0},
          thickness=1));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end testRespiration;

    model TestVanMeursHemodynamicsModelWattMeasure
      Components.Heart heart
        annotation (Placement(transformation(extent={{-22,-22},{20,24}})));
      Components.SystemicCirculation systemicCirculation
        annotation (Placement(transformation(extent={{-30,-84},{30,-24}})));
      Components.PulmonaryCirculation pulmonaryCirculation
        annotation (Placement(transformation(extent={{-30,14},{30,74}})));
      Components.WattMeter wattMeter
        annotation (Placement(transformation(extent={{56,-8},{76,12}})));
    equation
      connect(systemicCirculation.bloodFlowOutflow, heart.rightAtriumFlowInflow)
        annotation (Line(
          points={{-30.6,-54},{-40,-54},{-40,3.3},{-16.12,3.3}},
          color={28,108,200},
          thickness=1));
      connect(heart.pulmonaryArteryOutflow, pulmonaryCirculation.bloodFlowInflow)
        annotation (Line(
          points={{-7.72,11.58},{-40,11.58},{-40,44},{-30.6,44}},
          color={28,108,200},
          thickness=1));
      connect(heart.leftAtriumFlowInflow, pulmonaryCirculation.bloodFlowOutflow)
        annotation (Line(
          points={{6.14,11.58},{40,11.58},{40,44},{30,44}},
          color={238,46,47},
          thickness=1));
      connect(wattMeter.volumeFlowInflow, heart.aortaOutflow) annotation (Line(
            points={{56,2.2},{56,3.3},{15.38,3.3}}, color={0,0,0}));
      connect(wattMeter.volumeFlowOutflow, systemicCirculation.bloodFlowInflow)
        annotation (Line(points={{76,2.4},{80,2.4},{80,-54},{28.8,-54}}, color=
              {0,0,0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={Ellipse(
              extent={{-62,62},{64,-64}},
              lineColor={255,0,0},
              pattern=LinePattern.None,
              lineThickness=1,
              fillPattern=FillPattern.Sphere,
              fillColor={244,125,35})}),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=10,
          __Dymola_NumberOfIntervals=50000,
          __Dymola_Algorithm="Dassl"));
    end TestVanMeursHemodynamicsModelWattMeasure;
  end Tests;

  package Types
    type Pressure =  Modelica.Units.SI.Pressure(displayUnit="mmHg", nominal=133.322387415);
    type Power = Modelica.Units.SI.Power (
                                        displayUnit="watt", nominal=1);
    type Volume =  Modelica.Units.SI.Volume (
           displayUnit="ml", nominal=1e-6, min=0);
    type HydraulicElastance = Real(final quantity="HydraulicElastance",final unit="Pa/m3", displayUnit="mmHg/ml", nominal=(133.322387415)/(1e-6));
    type HydraulicCompliance =  Real(final quantity="HydraulicCompliance",final unit="m3/Pa", displayUnit="ml/mmHg", nominal=(1e-6)/(133.322387415));
    type VolumeFlowRate =
        Modelica.Units.SI.VolumeFlowRate (displayUnit="ml/min", nominal=(1e-6)/60);
    connector PressureInput =
                          input Pressure "'input Pressure' as connector"
                                                                 annotation (
      defaultComponentName="u",
      Icon(graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid,
          points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})},
        coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}},
          preserveAspectRatio=true,
          initialScale=0.2)),
      Diagram(
        coordinateSystem(preserveAspectRatio=true,
          initialScale=0.2,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid,
          points={{0.0,50.0},{100.0,0.0},{0.0,-50.0},{0.0,50.0}}),
        Text(
          lineColor={0,0,127},
          extent={{-10.0,60.0},{-10.0,85.0}},
          textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one input signal of type Real.
</p>
</html>"));
    connector VolumeInput=input Volume "'input Volume' as connector"
                                                                 annotation (
      defaultComponentName="u",
      Icon(graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid,
          points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})},
        coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}},
          preserveAspectRatio=true,
          initialScale=0.2)),
      Diagram(
        coordinateSystem(preserveAspectRatio=true,
          initialScale=0.2,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid,
          points={{0.0,50.0},{100.0,0.0},{0.0,-50.0},{0.0,50.0}}),
        Text(
          lineColor={0,0,127},
          extent={{-10.0,60.0},{-10.0,85.0}},
          textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one input signal of type Real.
</p>
</html>"));
    connector VolumeFlowRateInput =
                          input VolumeFlowRate "'input VolumeFlowRate' as connector"
                                                                 annotation (
      defaultComponentName="u",
      Icon(graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid,
          points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})},
        coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}},
          preserveAspectRatio=true,
          initialScale=0.2)),
      Diagram(
        coordinateSystem(preserveAspectRatio=true,
          initialScale=0.2,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid,
          points={{0.0,50.0},{100.0,0.0},{0.0,-50.0},{0.0,50.0}}),
        Text(
          lineColor={0,0,127},
          extent={{-10.0,60.0},{-10.0,85.0}},
          textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one input signal of type Real.
</p>
</html>"));
    connector HydraulicElastanceInput =
                          input HydraulicElastance "'input HydraulicElastance' as connector"
                                                                 annotation (
      defaultComponentName="u",
      Icon(graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid,
          points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})},
        coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}},
          preserveAspectRatio=true,
          initialScale=0.2)),
      Diagram(
        coordinateSystem(preserveAspectRatio=true,
          initialScale=0.2,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid,
          points={{0.0,50.0},{100.0,0.0},{0.0,-50.0},{0.0,50.0}}),
        Text(
          lineColor={0,0,127},
          extent={{-10.0,60.0},{-10.0,85.0}},
          textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one input signal of type Real.
</p>
</html>"));
    connector HydraulicComplianceInput = input
        Physiolibrary.Types.HydraulicCompliance
      "input HydraulicCompliance as connector"
      annotation (defaultComponentName="hydrauliccompliance",
      Icon(graphics={Polygon(
              points={{-100,100},{100,0},{-100,-100},{-100,100}},
              lineColor={0,0,127},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid)},
           coordinateSystem(extent={{-100,-100},{100,100}}, preserveAspectRatio=true, initialScale=0.2)),
      Diagram(coordinateSystem(
            preserveAspectRatio=true, initialScale=0.2,
            extent={{-100,-100},{100,100}},
            grid={1,1}), graphics={Polygon(
              points={{0,50},{100,0},{0,-50},{0,50}},
              lineColor={0,0,127},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid), Text(
              extent={{-10,85},{-10,60}},
              lineColor={0,0,127},
              textString="%name")}),
        Documentation(info="<html>
    <p>
    Connector with one input signal of type HydraulicCompliance.
    </p>
    </html>"));
    connector PressureOutput =
                           output Pressure "'output Pressure' as connector"
                                                                    annotation (
      defaultComponentName="y",
      Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})}),
      Diagram(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          points={{-100.0,50.0},{0.0,0.0},{-100.0,-50.0}}),
        Text(
          lineColor={0,0,127},
          extent={{30.0,60.0},{30.0,110.0}},
          textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one output signal of type Real.
</p>
</html>"));
    connector VolumeOutput=output Volume "'output Volume' as connector"
                                                                    annotation (
      defaultComponentName="y",
      Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})}),
      Diagram(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          points={{-100.0,50.0},{0.0,0.0},{-100.0,-50.0}}),
        Text(
          lineColor={0,0,127},
          extent={{30.0,60.0},{30.0,110.0}},
          textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one output signal of type Real.
</p>
</html>"));
    connector VolumeFlowRateOutput =
                           output VolumeFlowRate "'output VolumeFlowRate' as connector"
                                                                    annotation (
      defaultComponentName="y",
      Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})}),
      Diagram(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          points={{-100.0,50.0},{0.0,0.0},{-100.0,-50.0}}),
        Text(
          lineColor={0,0,127},
          extent={{30.0,60.0},{30.0,110.0}},
          textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one output signal of type Real.
</p>
</html>"));
    connector HydraulicElastanceOutput =
                           output HydraulicElastance "'output Hydraulic relastance' as connector"
                                                                    annotation (
      defaultComponentName="y",
      Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})}),
      Diagram(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          points={{-100.0,50.0},{0.0,0.0},{-100.0,-50.0}}),
        Text(
          lineColor={0,0,127},
          extent={{30.0,60.0},{30.0,110.0}},
          textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one output signal of type Real.
</p>
</html>"));
    connector HydraulicComplianceOutput = output
        Physiolibrary.Types.HydraulicCompliance
      "output HydraulicCompliance as connector"
      annotation (defaultComponentName="hydrauliccompliance",
      Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}},
            grid={1,1}), graphics={Polygon(
              points={{-100,100},{100,0},{-100,-100},{-100,100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
      Diagram(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}},
            grid={1,1}), graphics={Polygon(
              points={{-100,50},{0,0},{-100,-50},{-100,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{30,110},{30,60}},
              lineColor={0,0,127},
              textString="%name")}),
        Documentation(info="<html>
  <p>
  Connector with one output signal of type Real.
  </p>
  </html>"));
    connector PowerOutput = output Power "output Power as connector"
      annotation (defaultComponentName="power",
      Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}},
            grid={1,1}), graphics={Polygon(
              points={{-100,100},{100,0},{-100,-100},{-100,100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
      Diagram(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}},
            grid={1,1}), graphics={Polygon(
              points={{-100,50},{0,0},{-100,-50},{-100,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{30,110},{30,60}},
              lineColor={0,0,127},
              textString="%name")}),
        Documentation(info="<html>
  <p>
  Connector with one output signal of type Power.
  </p>
  </html>"));
  end Types;

  package MeursHemodynamicsPhysiolibrary
    package Components
      model HeartPhysiolibrary
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a RightAtriumInflow
          annotation (Placement(transformation(extent={{-58,22},{-38,42}}),
              iconTransformation(extent={{-68,-30},{-48,-10}})));
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_b
          pulmonaryArteryOutflow annotation (Placement(transformation(extent={{84,22},
                  {104,42}}),       iconTransformation(extent={{-68,20},{-48,40}})));
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a leftAtriumInflow
          annotation (Placement(transformation(extent={{-62,-86},{-42,-66}}),
              iconTransformation(extent={{56,14},{76,34}})));
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_b AortaOutflow
          annotation (Placement(transformation(extent={{82,-86},{104,-64}}),
              iconTransformation(extent={{56,-34},{76,-14}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance TricuspidValve(
          _Goff=0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245)
                              annotation (Placement(transformation(
              origin={5,32},
              extent={{-13,12},{13,-12}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance PulmonaryValve(
          _Goff(displayUnit="ml/(mmHg.s)") = 0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{54,44},{80,20}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance LeftAtriumPhysiolibrary(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=9.31e-05,
          ZeroPressureVolume=3e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{-40,-90},{-12,-62}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance RightVentricle(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.000131,
          ZeroPressureVolume=4e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{22,16},{52,48}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance RightAtriumPhysiolibrary(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.000135,
          ZeroPressureVolume=3e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{-38,18},{-10,46}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance LeftVentricle(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.000144,
          ZeroPressureVolume=6e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{24,-90},{54,-60}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance mitralValve(
          _Goff=0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245) annotation (
            Placement(transformation(origin={3,-75},extent={{-13,11},{13,-11}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance AorticValve(
          _Goff(displayUnit="ml/(mmHg.s)") = 0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{56,-62},{82,-88}})));
        replaceable Physiolibrary.Types.Constants.FrequencyConst HeartRate(k(
              displayUnit="1/min") = 1.2)                                                                  annotation(Placement(transformation(origin={-89,90.5},      extent = {{-11, -6.5}, {11, 6.5}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.AtrialElastance RAtrialElastance(EMIN=
              6666119.37075, EMAX=19998358.11225)
          annotation (Placement(transformation(extent={{-70,52},{-32,84}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.VentricularElastance
          RVentricularElastance(EMIN=7599376.082655, EMAX=65327969.83335)
          annotation (Placement(transformation(extent={{-6,54},{24,88}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.AtrialElastance LAtrialElastance(
          Tav(displayUnit="s"),
          EMIN=15998686.4898,
          EMAX=37330268.4762)
          annotation (Placement(transformation(extent={{-74,-42},{-36,-10}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.VentricularElastance
          LVentricularElastance(EMIN=11999014.86735, EMAX=533289549.66)
          annotation (Placement(transformation(extent={{-12,-44},{24,-12}})));
      equation
        connect(AortaOutflow, AorticValve.q_out) annotation (Line(
            points={{93,-75},{82,-75}},
            color={0,0,0},
            thickness=1));
        connect(LeftAtriumPhysiolibrary.q_in, leftAtriumInflow) annotation (
            Line(
            points={{-26,-76},{-52,-76}},
            color={0,0,0},
            thickness=1));
        connect(LeftAtriumPhysiolibrary.q_in, mitralValve.q_in) annotation (
            Line(
            points={{-26,-76},{-26,-75},{-10,-75}},
            color={0,0,0},
            thickness=1));
        connect(mitralValve.q_out, LeftVentricle.q_in) annotation (Line(
            points={{16,-75},{39,-75}},
            color={0,0,0},
            thickness=1));
        connect(LeftVentricle.q_in, AorticValve.q_in) annotation (Line(
            points={{39,-75},{56,-75}},
            color={0,0,0},
            thickness=1));
        connect(RightAtriumPhysiolibrary.q_in, TricuspidValve.q_in) annotation (
           Line(
            points={{-24,32},{-8,32}},
            color={0,0,0},
            thickness=1));
        connect(RightAtriumInflow, RightAtriumPhysiolibrary.q_in) annotation (
            Line(
            points={{-48,32},{-24,32}},
            color={0,0,0},
            thickness=1));
        connect(TricuspidValve.q_out, RightVentricle.q_in) annotation (Line(
            points={{18,32},{37,32}},
            color={0,0,0},
            thickness=1));
        connect(RightVentricle.q_in, PulmonaryValve.q_in) annotation (Line(
            points={{37,32},{54,32}},
            color={0,0,0},
            thickness=1));
        connect(PulmonaryValve.q_out, pulmonaryArteryOutflow) annotation (Line(
            points={{80,32},{94,32}},
            color={0,0,0},
            thickness=1));
        connect(HeartRate.y, RAtrialElastance.HR) annotation (Line(points={{
                -75.25,90.5},{-51,90.5},{-51,80.8}}, color={0,0,127}));
        connect(RVentricularElastance.Ct, RightVentricle.compliance)
          annotation (Line(points={{26.85,74.91},{37,74.91},{37,44.8}}, color={
                0,0,127}));
        connect(RVentricularElastance.HR, RAtrialElastance.HR) annotation (Line(
              points={{9,84.6},{9,90},{-52,90},{-52,90.5},{-51,90.5},{-51,80.8}},
              color={0,0,127}));
        connect(RAtrialElastance.Ct, RightAtriumPhysiolibrary.compliance)
          annotation (Line(points={{-28.39,67.84},{-24,67.84},{-24,43.2}},
              color={0,0,127}));
        connect(LVentricularElastance.Ct, LeftVentricle.compliance) annotation (
           Line(points={{27.42,-24.32},{39,-24.32},{39,-63}}, color={0,0,127}));
        connect(LAtrialElastance.Ct, LeftAtriumPhysiolibrary.compliance)
          annotation (Line(points={{-32.39,-26.16},{-26,-26.16},{-26,-64.8}},
              color={0,0,127}));
        connect(LAtrialElastance.HR, HeartRate.y) annotation (Line(points={{-55,
                -13.2},{-55,0},{-82,0},{-82,74},{-75.25,74},{-75.25,90.5}},
              color={0,0,127}));
        connect(LVentricularElastance.HR, HeartRate.y) annotation (Line(points=
                {{6,-15.2},{6,0},{-82,0},{-82,74},{-72,74},{-72,90.5},{-75.25,
                90.5}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Polygon(
                points={{2,60},{-18,66},{-54,84},{-88,32},{-86,-24},{-28,-86},{
                    28,-84},{58,-66},{90,-26},{96,34},{66,70},{46,84},{24,72},{
                    2,60}},
                lineColor={255,0,0},
                smooth=Smooth.Bezier,
                fillColor={244,125,35},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
                preserveAspectRatio=false)));
      end HeartPhysiolibrary;

      model PulmonaryCirculation
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a pulmonaryBloodInflow
          annotation (Placement(transformation(extent={{-104,-10},{-84,10}}),
              iconTransformation(extent={{-110,-10},{-90,10}})));
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_b
          pulmonaryBloodOutflow annotation (Placement(transformation(extent={{
                  82,-10},{102,10}}), iconTransformation(extent={{90,-8},{110,
                  12}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance pulmonaryArteries(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000106,
          ZeroPressureVolume=5e-05,
          ExternalPressure=-533.28954966,
          Elastance=31064116.267695)
          annotation (Placement(transformation(extent={{-82,-38},{-54,-10}})));
        Physiolibrary.Hydraulic.Components.Resistor pulmonaryResistance(
            useConductanceInput=false, Resistance(displayUnit="(mmHg.s)/ml")=
            14665462.61565)
          annotation (Placement(transformation(extent={{-54,-13},{-20,13}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance pulmonaryVeins(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000518,
          ZeroPressureVolume=0.00035,
          ExternalPressure=-533.28954966,
          Elastance=6066168.6273825)
          annotation (Placement(transformation(extent={{-6,-40},{28,-12}})));
        Physiolibrary.Hydraulic.Components.Resistor PulmonaryVenousResistance(
            useConductanceInput=false, Resistance(displayUnit="(mmHg.s)/ml")=
            399967.162245)
          annotation (Placement(transformation(extent={{36,-12},{66,12}})));
      equation
        connect(pulmonaryBloodInflow, pulmonaryResistance.q_in) annotation (
            Line(
            points={{-94,0},{-54,0}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryResistance.q_out, PulmonaryVenousResistance.q_in)
          annotation (Line(
            points={{-20,0},{36,0}},
            color={0,0,0},
            thickness=1));
        connect(PulmonaryVenousResistance.q_out, pulmonaryBloodOutflow)
          annotation (Line(
            points={{66,0},{92,0}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryVeins.q_in, PulmonaryVenousResistance.q_in)
          annotation (Line(
            points={{11,-26},{8,-26},{8,0},{36,0}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryArteries.q_in, pulmonaryResistance.q_in) annotation (
            Line(
            points={{-68,-24},{-68,0},{-54,0}},
            color={0,0,0},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Rectangle(
                extent={{-100,20},{100,-20}},
                lineColor={255,0,0},
                fillPattern=FillPattern.HorizontalCylinder,
                fillColor={244,125,35}), Text(
                extent={{-128,-28},{122,-46}},
                lineColor={0,0,0},
                fillPattern=FillPattern.HorizontalCylinder,
                fillColor={244,125,35},
                textString="%name")}), Diagram(coordinateSystem(
                preserveAspectRatio=false)));
      end PulmonaryCirculation;

      model SystemicCirculation
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a systemicBloodInflow
          annotation (Placement(transformation(extent={{90,-10},{110,10}}),
              iconTransformation(extent={{90,-10},{110,10}})));
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_b systemicBloodOutflow
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
              iconTransformation(extent={{-110,-10},{-90,10}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance intrathoracicArteries(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000204,
          ZeroPressureVolume=0.00014,
          ExternalPressure=-533.28954966,
          Elastance=190651014.00345)
          annotation (Placement(transformation(extent={{80,-36},{102,-14}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance extrathoracicArteries(
          volume_start(displayUnit="ml") = 0.000526,
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          ZeroPressureVolume=0.00037,
          Elastance=74127247.40274)
          annotation (Placement(transformation(extent={{26,-36},{50,-14}})));
        Physiolibrary.Hydraulic.Components.Resistor extrathoracicArterialResistance(
            useConductanceInput=false, Resistance(displayUnit="(mmHg.s)/ml")=
            7999343.2449)
          annotation (Placement(transformation(extent={{42,-10},{62,10}})));
        Physiolibrary.Hydraulic.Components.Resistor venousResistance(
            useConductanceInput=false, Resistance(displayUnit="(mmHg.s)/ml")=
            11999014.86735)
          annotation (Placement(transformation(extent={{-32,-10},{-56,10}})));
        Physiolibrary.Hydraulic.Components.Inertia aorticFlowInertia(I(
              displayUnit="mmHg.s2/ml") = 226648.0586055, volumeFlow_start(
              displayUnit="ml/min") = 0) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={76,0})));
        Physiolibrary.Hydraulic.Components.Resistor systemicArteriolarResistance(
            useConductanceInput=false, Resistance(displayUnit="(mmHg.s)/ml")=
            106657909.932) annotation (Placement(transformation(extent={{12,-10},
                  {-12,10}}, origin={22,0})));
        Physiolibrary.Hydraulic.Components.Resistor smallVenuleResistance(
            useConductanceInput=false, Resistance(displayUnit="(mmHg.s)/ml")=
            26664477.483) annotation (Placement(transformation(extent={{13,-10},
                  {-13,10}}, origin={-9,0})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance SystemicTissues(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000283,
          ZeroPressureVolume=0.000185,
          Elastance=34930465.50273)
          annotation (Placement(transformation(extent={{-6,18},{18,40}})));
        Physiolibrary.Hydraulic.Components.Resistor centralVenousResistance(
            useConductanceInput=false, Resistance(displayUnit="(mmHg.s)/ml")=
            399967.162245)
          annotation (Placement(transformation(extent={{-66,-10},{-92,10}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance intrathoracicVeins(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.00148,
          ZeroPressureVolume=0.00119,
          ExternalPressure=-533.28954966,
          Elastance=2426467.450953)
          annotation (Placement(transformation(extent={{-72,14},{-50,36}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance extrathoracicVeins(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.00153,
          ZeroPressureVolume=0.001,
          Elastance=2253148.3473135)
          annotation (Placement(transformation(extent={{-38,-40},{-16,-18}})));
      equation
        connect(systemicBloodInflow, aorticFlowInertia.q_in) annotation (Line(
            points={{100,0},{86,-1.9984e-15}},
            color={0,0,0},
            thickness=1));
        connect(intrathoracicArteries.q_in, aorticFlowInertia.q_in) annotation (
           Line(
            points={{91,-25},{91,0},{86,0},{86,-1.9984e-15}},
            color={0,0,0},
            thickness=1));
        connect(systemicBloodOutflow, centralVenousResistance.q_out)
          annotation (Line(
            points={{-100,0},{-92,0}},
            color={0,0,0},
            thickness=1));
        connect(centralVenousResistance.q_in, venousResistance.q_out)
          annotation (Line(
            points={{-66,0},{-56,0}},
            color={0,0,0},
            thickness=1));
        connect(venousResistance.q_in, smallVenuleResistance.q_out) annotation (
           Line(
            points={{-32,0},{-22,0}},
            color={0,0,0},
            thickness=1));
        connect(smallVenuleResistance.q_in, systemicArteriolarResistance.q_out)
          annotation (Line(
            points={{4,0},{10,0}},
            color={0,0,0},
            thickness=1));
        connect(systemicArteriolarResistance.q_in,
          extrathoracicArterialResistance.q_in) annotation (Line(
            points={{34,0},{42,0}},
            color={0,0,0},
            thickness=1));
        connect(extrathoracicArterialResistance.q_out, aorticFlowInertia.q_out)
          annotation (Line(
            points={{62,0},{66,7.77156e-16}},
            color={0,0,0},
            thickness=1));
        connect(intrathoracicVeins.q_in, venousResistance.q_out) annotation (
            Line(
            points={{-61,25},{-62,0},{-56,0}},
            color={0,0,0},
            thickness=1));
        connect(extrathoracicVeins.q_in, smallVenuleResistance.q_out)
          annotation (Line(
            points={{-27,-29},{-28,0},{-22,0}},
            color={0,0,0},
            thickness=1));
        connect(SystemicTissues.q_in, systemicArteriolarResistance.q_out)
          annotation (Line(
            points={{6,29},{6,0},{10,0}},
            color={0,0,0},
            thickness=1));
        connect(extrathoracicArteries.q_in, extrathoracicArterialResistance.q_in)
          annotation (Line(
            points={{38,-25},{38,0},{42,0}},
            color={0,0,0},
            thickness=1));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
                extent={{-100,20},{100,-20}},
                lineColor={255,0,0},
                fillPattern=FillPattern.HorizontalCylinder,
                fillColor={244,125,35}), Text(
                extent={{-128,-28},{122,-46}},
                lineColor={0,0,0},
                fillPattern=FillPattern.HorizontalCylinder,
                fillColor={244,125,35},
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=5,
            __Dymola_NumberOfIntervals=50000,
            __Dymola_Algorithm="Dassl"));
      end SystemicCirculation;

      model AtrialElastance
        extends
          Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.HeartIntervals;
        Physiolibrary.Types.RealIO.HydraulicComplianceOutput Ct "compliance" annotation(Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -20}, {138, 18}})));
        Physiolibrary.Types.HydraulicElastance Et "elasticity";
        parameter Physiolibrary.Types.HydraulicElastance EMIN
          "Diastolic elastance";
      parameter Boolean useEs_extInput = false
          "=true, if external elastance/compliance value is used"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true),Dialog(group="External inputs/outputs"));
        parameter Physiolibrary.Types.HydraulicElastance EMAX
          "Maximum systolic elastance"         annotation (Dialog(enable=not useEs_extInput));
      Physiolibrary.Types.RealIO.HydraulicComplianceInput Es_ext(start=1/EMAX)=1/es_int if useEs_extInput
         annotation (
            Placement(transformation(extent={{60,60},{100,100}}), iconTransformation(
              extent={{-20,-20},{20,20}},
              origin={-80,80})));
      protected
         Physiolibrary.Types.HydraulicElastance es_int;
      equation
        if not useEs_extInput then
          es_int=EMAX;
        end if;
        if time - T0 < Tas then
          Et = EMIN + (es_int - EMIN) * sin(Modelica.Constants.pi * (time - T0) / Tas);
        else
          Et = EMIN;
        end if;
        Ct = 1 / Et "reciprocal value of elastance";
        annotation(Icon(coordinateSystem(preserveAspectRatio=false,   extent={{-100,
                  -100},{100,100}}),                                                                        graphics={  Rectangle(extent = {{-100, 82}, {100, -100}}, pattern = LinePattern.None,
                  lineThickness =                                                                                                   1, fillColor = {255, 255, 170},
                  fillPattern =                                                                                                   FillPattern.Solid, lineColor = {0, 0, 255}), Text(extent={{
                    -98,82},{98,24}},                                                                                                    lineColor = {0, 0, 255},
                  lineThickness =                                                                                                   1, fillColor = {255, 255, 170},
                  fillPattern =                                                                                                   FillPattern.Solid, textString = "Atrial elastance"), Line(points = {{-78, -34}, {-76, -26}, {-70, -14}, {-58, 6}, {-36, 36}, {-14, 14}, {-6, -10}, {0, -32}, {6, -34}, {88, -34}, {94, -34}}, color = {0, 0, 255}, smooth = Smooth.Bezier), Text(extent = {{-220, -102}, {200, -120}}, lineColor = {0, 0, 255},
                  lineThickness =                                                                                                   1, fillColor = {255, 255, 170},
                  fillPattern =                                                                                                   FillPattern.Solid, textString = "%name"), Text(extent = {{72, 4}, {102, -8}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 170},
                  fillPattern =                                                                                                   FillPattern.Solid, textString = "Ct")}));
      end AtrialElastance;

      model VentricularElastance
        extends
          Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.HeartIntervals;
        Physiolibrary.Types.RealIO.HydraulicComplianceOutput Ct
          "ventricular elasticity"                                                       annotation(Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, 4}, {138, 42}})));
        Modelica.Blocks.Interfaces.RealOutput Et0
          "normalized ventricular elasticity (0..1)"                                         annotation(Placement(transformation(extent = {{100, -24}, {120, -4}}), iconTransformation(extent = {{100, -40}, {138, -2}})));
        Physiolibrary.Types.RealIO.TimeOutput HeartInterval "eapsed time" annotation(Placement(transformation(extent = {{102, -42}, {122, -22}}), iconTransformation(extent = {{100, -98}, {138, -60}})));
        Physiolibrary.Types.HydraulicElastance Et;
        parameter Physiolibrary.Types.HydraulicElastance EMIN
          "Diastolic elastance ";
        constant Real Kn = 0.57923032735652;
        //Kn is always = 0.5792303273565197
        //... the t * sin(pi*t) has its maximum at t = 0.645773676543406 and = 0.5792303273565197
        //Equation to calculate normalized elastance ET0 was:
        //Et0=EMIN+(EMAX-EMIN)*((time-T0)-(Tas+Tav))/Tvs)*sin(Modelica.Constants.pi*(((time-T0)-(Tas+Tav))/Tvs));
      parameter Boolean useEs_extInput = false
          "=true, if external elastance/compliance value is used"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true),Dialog(group="External inputs/outputs"));
        parameter Physiolibrary.Types.HydraulicElastance EMAX
          "Maximum systolic elastance"         annotation (Dialog(enable=not useEs_extInput));
      Physiolibrary.Types.RealIO.HydraulicComplianceInput Es_ext(start=1/EMAX)=1/es_int if useEs_extInput
         annotation (
            Placement(transformation(extent={{60,60},{100,100}}), iconTransformation(
              extent={{-20,-20},{20,20}},
              origin={-80,80})));
      protected
         Physiolibrary.Types.HydraulicElastance es_int;
      equation
        if not useEs_extInput then
          es_int=EMAX;
        end if;
        HeartInterval = time - T0;
        Et = EMIN + (es_int - EMIN) * Et0;
        if HeartInterval >= Tas + Tav and HeartInterval < Tas + Tav + Tvs then
          Et0 = (HeartInterval - (Tas + Tav)) / Tvs * sin(Modelica.Constants.pi * (HeartInterval - (Tas + Tav)) / Tvs) / Kn;
        else
          Et0 = 0;
        end if;
        Ct = 1 / Et "reciprocal value of elastance";
        annotation(Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-98, 82}, {100, -100}}, pattern = LinePattern.None,
                  lineThickness =                                                                                                   1, fillColor = {255, 255, 170},
                  fillPattern =                                                                                                   FillPattern.Solid, lineColor = {0, 0, 255}), Text(extent = {{-82, 82}, {80, 24}}, lineColor = {0, 0, 255},
                  lineThickness =                                                                                                   1, fillColor = {255, 255, 170},
                  fillPattern =                                                                                                   FillPattern.Solid, textString = "Ventricular elastance"), Line(points = {{-72, -34}, {-62, -34}, {-52, -34}, {-44, 8}, {-18, 38}, {-12, 14}, {-6, -10}, {0, -32}, {6, -34}, {88, -34}, {94, -34}}, color = {0, 0, 255}, smooth = Smooth.Bezier), Text(extent = {{-220, -102}, {200, -120}}, lineColor = {0, 0, 255},
                  lineThickness =                                                                                                   1, fillColor = {255, 255, 170},
                  fillPattern =                                                                                                   FillPattern.Solid, textString = "%name"), Text(extent = {{96, -32}, {68, -8}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 170},
                  fillPattern =                                                                                                   FillPattern.Solid, textString = "Et0"), Text(extent = {{42, -72}, {88, -84}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 170},
                  fillPattern =                                                                                                   FillPattern.Solid, textString = "Heart interval"), Text(extent = {{62, 30}, {96, 8}}, lineColor = {0, 0, 255},
                  lineThickness =                                                                                                   1, fillColor = {255, 255, 170},
                  fillPattern =                                                                                                   FillPattern.Solid, textString = "Ct")}));
      end VentricularElastance;

      model HeartIntervals
        discrete Physiolibrary.Types.Time Tas, T0, Tvs;
        parameter Physiolibrary.Types.Time Tav(displayUnit = "s") = 0.01
          "atrioventricular delay";
        discrete Modelica.Units.SI.Time HP(start=0) "heart period";
        Boolean b(start = false);
        Physiolibrary.Types.RealIO.FrequencyInput HR "heart rate" annotation(Placement(transformation(extent = {{-12, 68}, {28, 108}}), iconTransformation(extent = {{-20, -20}, {20, 20}}, rotation = 270, origin = {0, 80})));
      equation
        b = time - pre(T0) >= pre(HP) "true if new pulse occurs";
        when {initial(), b} then
          T0 = time "start time of cardiac cycle";
          HP = 1 / HR "update heart period per heart rate";
          Tas = 0.03 + 0.09 * HP "duration of atrial systole";
          Tvs = 0.16 + 0.2 * HP "duration of ventricular systole";
        end when;
        annotation(Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent={{
                    -64,102},{-6,78}},                                                                                                    lineColor = {0, 0, 255}, textString = "HR")}));
      end HeartIntervals;

      model MeursModel
        extends Physiolibrary.Icons.CardioVascular;
        Components.HeartPhysiolibrary heartPhysiolibrary
          annotation (Placement(transformation(extent={{-44,-30},{38,42}})));
        Components.PulmonaryCirculation pulmonaryCirculation
          annotation (Placement(transformation(extent={{-62,24},{60,116}})));
        Components.SystemicCirculation systemicCirculation
          annotation (Placement(transformation(extent={{-60,-104},{60,2}})));
      equation
        connect(heartPhysiolibrary.pulmonaryArteryOutflow, pulmonaryCirculation.pulmonaryBloodInflow)
          annotation (Line(
            points={{-26.78,16.8},{-86,16.8},{-86,70},{-62,70}},
            color={0,0,0},
            thickness=1));
        connect(systemicCirculation.systemicBloodOutflow, heartPhysiolibrary.RightAtriumInflow)
          annotation (Line(
            points={{-60,-51},{-60,-52},{-88,-52},{-88,-1.2},{-26.78,-1.2}},
            color={0,0,0},
            thickness=1));
        connect(systemicCirculation.systemicBloodInflow, heartPhysiolibrary.AortaOutflow)
          annotation (Line(
            points={{60,-51},{84,-51},{84,-2.64},{24.06,-2.64}},
            color={0,0,0},
            thickness=1));
        connect(heartPhysiolibrary.leftAtriumInflow, pulmonaryCirculation.pulmonaryBloodOutflow)
          annotation (Line(
            points={{24.06,14.64},{86,14.64},{86,70.92},{60,70.92}},
            color={0,0,0},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=20,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end MeursModel;

      model UnlimitedGasStorage "Constant ideal gas source"
        extends Physiolibrary.Chemical.Interfaces.ConditionalHeatPort;
        Physiolibrary.Chemical.Interfaces.ChemicalPort_b q_out
          "constant gas concentration with any possible flow"
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));

        parameter Boolean usePartialPressureInput = false
          "=true, if fixed partial pressure is from input instead of parameter"
        annotation(Evaluate=true, HideResult=true, choices(checkBox=true),Dialog(group="External inputs/outputs"));

         parameter Physiolibrary.Types.Pressure PartialPressure=0
          "Fixed partial pressure if usePartialPressureInput=false"
          annotation (Dialog(enable=not usePartialPressureInput));

        Physiolibrary.Types.RealIO.PressureInput partialPressure(start=
              PartialPressure)=p if usePartialPressureInput
          "Partial pressure of Gas = air pressure * gas fraction"
          annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));

       parameter Boolean isIsolatedInSteadyState = true
          "=true, if there is no flow at port in steady state"
          annotation (Evaluate=true, HideResult=true, Dialog(group="Simulation",tab="Equilibrium"));

        parameter Physiolibrary.Types.SimulationType Simulation=Physiolibrary.Types.SimulationType.NormalInit
          "If in equilibrium, then zero-flow equation is added." annotation (
          Evaluate=true,
          HideResult=true,
          Dialog(group="Simulation", tab="Equilibrium"));

        DeathVolume deathVolume
          annotation (Placement(transformation(extent={{-244,20},{-224,40}})));
      protected
        Physiolibrary.Types.Pressure p "Current partial pressure";

      initial equation
        if isIsolatedInSteadyState and (Simulation == Physiolibrary.Types.SimulationType.InitSteadyState) then
          q_out.q = 0;
        end if;

      equation
        if not usePartialPressureInput then
          p=PartialPressure;
        end if;

        q_out.conc = p / (Modelica.Constants.R * T_heatPort);  //ideal gas equation

        if isIsolatedInSteadyState and (Simulation == Physiolibrary.Types.SimulationType.SteadyState) then
           q_out.q = 0;
        end if;

        lossHeat=0; //only read temperature from heat port

        annotation ( Icon(coordinateSystem(
                preserveAspectRatio=false,extent={{-100,-100},{100,100}}),
              graphics={
              Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              pattern=LinePattern.None,
              fillColor={170,255,255},
              fillPattern=FillPattern.Backward),
              Polygon(
                points={{-100,100},{100,-100},{100,100},{-100,100}},
                fillColor={159,159,223},
                fillPattern=FillPattern.Backward,
                pattern=LinePattern.None,
                lineColor={0,0,0}),
              Text(
                extent={{0,0},{-100,-100}},
                lineColor={0,0,0},
                textString="P,T"),
              Line(
                points={{-62,0},{56,0}},
                color={191,0,0},
                thickness=0.5),
              Polygon(
                points={{38,-20},{38,20},{78,0},{38,-20}},
                lineColor={191,0,0},
                fillColor={191,0,0},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-150,150},{150,110}},
                textString="%name",
                lineColor={0,0,255}),
              Text(
                extent={{-150,-110},{150,-140}},
                lineColor={0,0,0},
                textString="T=%T")}),
          Documentation(revisions="<html>
<p><i>2009-2010</i></p>
<p>Marek Matejak, Charles University, Prague, Czech Republic </p>
</html>"));
      end UnlimitedGasStorage;

      model GasEquation

        Physiolibrary.Types.RealIO.VolumeInput
                                           V1(
                                         displayUnit="ml") annotation (Placement(transformation(extent={
                  {-118,42},{-78,82}}), iconTransformation(extent={{-100,66},{-72,94}})));
        Physiolibrary.Types.RealIO.PressureInput
                                           P1(
                                         displayUnit="mmHg") annotation (Placement(transformation(extent={
                  {-118,42},{-78,82}}), iconTransformation(extent={{-100,6},{
                -72,34}})));
        Physiolibrary.Types.RealIO.TemperatureInput
                                           T1(
                                         displayUnit="degC") annotation (Placement(transformation(extent={
                  {-118,42},{-78,82}}), iconTransformation(extent={{-100,-54},{-72,
                  -26}})));
        Physiolibrary.Types.RealIO.PressureInput
                                           P2(
                                        displayUnit="mmHg") annotation (Placement(transformation(extent={
                  {-118,42},{-78,82}}), iconTransformation(
              extent={{-14,-14},{14,14}},
              rotation=180,
              origin={86,20})));
        Physiolibrary.Types.RealIO.TemperatureInput
                                           T2(
                                         displayUnit="degC") annotation (Placement(transformation(extent={
                  {-118,42},{-78,82}}), iconTransformation(
              extent={{-14,-14},{14,14}},
              rotation=180,
              origin={86,-40})));
        Physiolibrary.Types.RealIO.VolumeOutput
                                            V2(
                                          displayUnit="ml") annotation (Placement(transformation(extent=
                  {{56,54},{96,94}}), iconTransformation(extent={{72,66},{100,94}})));
      equation
        (P1*V1)/(T1)=(P2*V2)/(T2);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={170,213,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{0,100},{0,-100}},
                lineColor={0,0,0},
                fillColor={170,213,255},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-98,136},{100,100}},
                lineColor={0,0,0},
                fillColor={170,213,255},
                fillPattern=FillPattern.Solid,
                textString="%name"),
            Text(
              extent={{-66,34},{132,8}},
              textColor={28,108,200},
              textString="P1",
              horizontalAlignment=TextAlignment.Left),
            Text(
              extent={{-68,92},{132,66}},
              textColor={28,108,200},
              textString="V1",
              horizontalAlignment=TextAlignment.Left),
            Text(
              extent={{-68,-26},{130,-54}},
              textColor={28,108,200},
              textString="T1",
              horizontalAlignment=TextAlignment.Left),
            Text(
              extent={{-130,92},{70,66}},
              textColor={28,108,200},
              horizontalAlignment=TextAlignment.Right,
              textString="V2"),
            Text(
              extent={{-128,32},{70,6}},
              textColor={28,108,200},
              horizontalAlignment=TextAlignment.Right,
              textString="P2"),
            Text(
              extent={{-128,-26},{70,-54}},
              textColor={28,108,200},
              horizontalAlignment=TextAlignment.Right,
              textString="T2")}));
      end GasEquation;

      model UnlimitedGasSource
        Physiolibrary.Types.RealIO.PressureInput partialPressure annotation (
            Placement(transformation(extent={{-92,16},{-52,56}}),
              iconTransformation(extent={{-124,60},{-84,100}})));
        Physiolibrary.Types.RealIO.TemperatureInput temperature annotation (
            Placement(transformation(extent={{-122,-64},{-82,-24}}),
              iconTransformation(extent={{-118,-80},{-78,-40}})));
        Physiolibrary.Chemical.Interfaces.ChemicalPort_a port_a annotation (
            Placement(transformation(extent={{80,4},{100,24}}),
              iconTransformation(extent={{90,-16},{110,4}})));
        Physiolibrary.Chemical.Sources.UnlimitedGasStorage unlimitedGasStorage(
            useHeatPort=true, usePartialPressureInput=true)
          annotation (Placement(transformation(extent={{14,12},{34,32}})));
        Physiolibrary.Thermal.Sources.UnlimitedHeat unlimitedHeat(
            useTemperatureInput=true)
          annotation (Placement(transformation(extent={{-48,-52},{-28,-32}})));
      equation
        connect(port_a, unlimitedGasStorage.q_out) annotation (Line(
            points={{90,14},{40,14},{40,22},{34,22}},
            color={107,45,134},
            thickness=1));
        connect(unlimitedHeat.temperature, temperature) annotation (Line(points
              ={{-48,-42},{-78,-42},{-78,-44},{-102,-44}}, color={0,0,127}));
        connect(unlimitedHeat.port, unlimitedGasStorage.heatPort) annotation (
            Line(
            points={{-28,-42},{2,-42},{2,-48},{24,-48},{24,22}},
            color={191,0,0},
            thickness=1));
        connect(unlimitedGasStorage.partialPressure, partialPressure)
          annotation (Line(points={{14,22},{-46,22},{-46,36},{-72,36}}, color={
                0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Polygon(
                points={{-100,100},{-100,-100},{100,-40},{100,20},{-100,100}},
                lineColor={28,108,200},
                fillColor={255,255,0},
                fillPattern=FillPattern.Solid), Text(
                extent={{-198,-96},{188,-136}},
                textColor={28,108,200},
                textString="%name")}), Diagram(coordinateSystem(
                preserveAspectRatio=false)));
      end UnlimitedGasSource;

      model PartialPressureMeasure
          extends Modelica.Icons.RoundSensor;
        Physiolibrary.Types.RealIO.TemperatureInput temperature annotation (Placement(
              transformation(extent={{-148,34},{-108,74}}), iconTransformation(extent=
                 {{-104,26},{-64,66}})));
        Physiolibrary.Types.RealIO.ConcentrationInput concentration annotation (
            Placement(transformation(extent={{-150,-20},{-110,20}}),
              iconTransformation(extent={{-108,-56},{-68,-16}})));
        Physiolibrary.Types.RealIO.PressureOutput pressure annotation (Placement(
              transformation(extent={{-134,-42},{-114,-22}}), iconTransformation(
                extent={{92,-8},{112,12}})));
      equation
       concentration = pressure / (Modelica.Constants.R * temperature);  //ideal gas equation
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{12,28},{80,-60}},
                textColor={28,108,200},
                textString="P")}),                                     Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end PartialPressureMeasure;

      model VaporPressure

        Physiolibrary.Types.RealIO.TemperatureInput
                                           T(
                                        displayUnit="degC")   annotation (Placement(transformation(extent={
                  {-118,42},{-78,82}}), iconTransformation(extent={{-100,-14},{-72,14}})));
        Physiolibrary.Types.RealIO.PressureOutput
                                            VaporPressure_(
                                                    displayUnit="mmHg")
                                                          annotation (Placement(transformation(extent=
                  {{56,54},{96,94}}), iconTransformation(extent={{100,-12},{128,
                16}})));
      equation
       VaporPressure_ =  if T<273.15 then 0 else if T>373.15 then 101325 else
                          (101325/760)*exp(18.6686-(4030.183/(T-273.15+235)));
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Rectangle(
                extent={{-100,46},{100,-42}},
                lineColor={0,0,0},
                fillColor={170,213,255},
                fillPattern=FillPattern.Solid), Text(
              extent={{-186,28},{192,-16}},
              textColor={28,108,200},
              textString="Vapor
pressure")}));
      end VaporPressure;

      model IdealValveChemical
        extends Physiolibrary.Chemical.Interfaces.OnePort;
        parameter Boolean useChatteringProtection = false annotation(Evaluate = true);
        parameter Physiolibrary.Types.Time chatteringProtectionTime(displayUnit="ms") = 0 "Minimal period of time, in which a closed valve stays closed";
        Physiolibrary.Types.Time lastChange(start = 0);
         Boolean open(start=true) "Switching state";
         Real passableVariable(start=0, final unit="1")
          "Auxiliary variable for actual position on the ideal diode characteristic";
        /*  = 0: knee point
      < 0: below knee point, diode locking
      > 0: above knee point, diode conducting */
        parameter Physiolibrary.Types.HydraulicConductance _Gon(
          final min=0,
          displayUnit="l/(mmHg.min)")=1.2501026264094e-02
          "Forward state-on conductance (open valve conductance)"
          annotation (Dialog(enable=not useLimitationInputs)); //= the same as resistance 1e-5 mmHg/(l/min)
        parameter Physiolibrary.Types.HydraulicConductance _Goff(
          final min=0,
          displayUnit="l/(mmHg.min)")=1.2501026264094e-12
          "Backward state-off conductance (closed valve conductance)"
          annotation (Dialog(enable=not useLimitationInputs)); //= 1e-5 (l/min)/mmHg
        parameter Physiolibrary.Types.Pressure Pknee(final min=0)=0
          "Forward threshold pressure";
        parameter Boolean useLimitationInputs = false
          "=true, if Gon and Goff are from inputs"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true),Dialog(group="External inputs/outputs"));
        Physiolibrary.Types.RealIO.HydraulicConductanceInput Gon(start=_Gon)=gon if
          useLimitationInputs "open valve conductance = infinity for ideal case"
          annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={-60,100})));
        Physiolibrary.Types.RealIO.HydraulicConductanceInput Goff(start=_Goff)=goff if
             useLimitationInputs "closed valve conductance = zero for ideal case"
          annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={60,100})));
      protected
        Physiolibrary.Types.HydraulicConductance gon;
        Physiolibrary.Types.HydraulicConductance goff;
        constant Physiolibrary.Types.Pressure unitPressure=1;
        constant Physiolibrary.Types.VolumeFlowRate unitFlow=1;
      equation
        if not useLimitationInputs then
          gon = _Gon;
          goff = _Goff;
        end if;
        when useChatteringProtection and change(open) then
          lastChange = time;
        end when;

        if not useChatteringProtection then
          open = passableVariable > 0;
        else
          if pre(lastChange) + chatteringProtectionTime > time then
            // under protection
            open = pre(open);
          else
            // protection expired
            open = passableVariable > 0;
          end if;
        end if;

        dp = (passableVariable*unitFlow)*(if open then 1/gon else 1) + Pknee;
        volumeFlowRate = (passableVariable*unitPressure)*(if open then 1 else goff) + goff*Pknee;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                               graphics={Polygon(
                points={{-76,66},{-76,-82},{34,-10},{34,12},{-66,68},{-76,74},{
                    -76,66}},
                lineColor={0,0,127},
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid), Rectangle(
                extent={{40,96},{68,-94}},
                lineColor={0,0,127},
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-188,-96},{192,-118}},
                lineColor={255,0,0},
                fillPattern=FillPattern.Sphere,
                fillColor={255,85,85},
                textString="%name")}),
          Documentation(info="<html>
<p>Ideal Valve allows a volumetric flow in one direction in case of pressure gradient is greater. </p>
</html>",   revisions="<html>
<p><i>2014</i></p>
<p>Tomas Kulhanek, Charles University, Prague, Czech Republic </p>
</html>"));
      end IdealValveChemical;

      model DeathVolume
        Physiolibrary.Chemical.Interfaces.ChemicalPort_a environmentalFlow annotation (Placement(
              transformation(extent={{-8,78},{12,98}}),    iconTransformation(extent={
                  {-16,66},{4,86}})));
        Physiolibrary.Chemical.Interfaces.ChemicalPort_b alveolarFlow annotation (Placement(
              transformation(extent={{-10,-94},{10,-74}}),    iconTransformation(
                extent={{-12,-100},{8,-80}})));
        Physiolibrary.Types.RealIO.VolumeInput volume annotation (Placement(
              transformation(extent={{-112,26},{-72,66}}), iconTransformation(extent={{-118,0},
                  {-78,40}})));
        Physiolibrary.Types.RealIO.VolumeFlowRateInput volumeFlow annotation (
            Placement(transformation(extent={{-110,-44},{-70,-4}}),
              iconTransformation(extent={{-118,-60},{-78,-20}})));
        Physiolibrary.Types.RealIO.ConcentrationOutput concentration annotation (
            Placement(transformation(extent={{84,6},{104,26}}),
              iconTransformation(extent={{100,-24},{134,10}})));
        Physiolibrary.Types.AmountOfSubstance solute(start=0, fixed=true);
      equation
        /*
   der( solute) = environmentalFlow.q+alveolarFlow.q;
   concentration=solute/volume;
  if volumeFlow>0 then
    environmentalFlow.q=volumeFlow*environmentalFlow.conc;
    alveolarFlow.conc=concentration;
    alveolarFlow.q=-concentration*volumeFlow; //záporné vytéká
  else
    alveolarFlow.q=-volumeFlow*alveolarFlow.conc; //kladné vtéká
    environmentalFlow.conc=concentration;
    environmentalFlow.q=concentration*volumeFlow;
    end if;//záporné vytéká
    */

          der(solute)=environmentalFlow.q+alveolarFlow.q;
          concentration=solute/volume;
          if volumeFlow>0 then
             alveolarFlow.conc=concentration;
             environmentalFlow.q=environmentalFlow.conc*volumeFlow; //vtéká kladné
          else
             environmentalFlow.conc=concentration;
             alveolarFlow.q=-alveolarFlow.conc*volumeFlow; //vtékaá kladné
          end if;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(
                extent={{-100,-40},{100,-96}},
                lineColor={28,108,200},
                fillColor={255,255,170},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-100,70},{100,-70}},
                fillColor={255,255,170},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-100,100},{100,44}},
                lineColor={28,108,200},
                fillColor={196,196,196},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
                preserveAspectRatio=false)));
      end DeathVolume;
    end Components;

    package Tests

      model Flat_Meurs_Physiolibrary
      extends Physiolibrary.Icons.CardioVascular;
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Epa(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000106,
          ZeroPressureVolume=5e-05,
          ExternalPressure=-533.28954966,
          Elastance=31064116.267695)
          annotation (Placement(transformation(extent={{-94,122},{-66,150}})));
        Physiolibrary.Hydraulic.Components.Resistor Rpp(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 14665462.61565)
          annotation (Placement(transformation(extent={{-56,123},{-22,149}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Epv(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000518,
          ZeroPressureVolume=0.00035,
          ExternalPressure=-533.28954966,
          Elastance=6066168.6273825)
          annotation (Placement(transformation(extent={{-10,122},{24,150}})));
        Physiolibrary.Hydraulic.Components.Resistor Rlain(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{26,124},{56,148}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance LeftAtrium(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=9.31e-05,
          ZeroPressureVolume=3e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{74,88},{102,116}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance LeftVentricle(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.000144,
          ZeroPressureVolume=6e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{150,88},{178,116}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance AorticValve(
          _Goff(displayUnit="ml/(mmHg.s)") = 0,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245,
          chatteringProtectionTime=0.01,                                                       lastChange(displayUnit = "s"), useChatteringProtection = true,
          useLimitationInputs=false)
          annotation (Placement(transformation(extent={{182,114},{206,90}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.AtrialElastance LAtrialElastance(
          Tav(displayUnit="s"),
          EMIN=15998686.4898,
          EMAX=37330268.4762)
          annotation (Placement(transformation(extent={{80,130},{118,162}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.VentricularElastance
          LVentricularElastance(EMIN=11999014.86735, EMAX=533289549.66)
          annotation (Placement(transformation(extent={{164,126},{200,158}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance MitralValve(
          _Goff(displayUnit="ml/(mmHg.s)") = 0,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245,
          chatteringProtectionTime=0.01,                                                       lastChange(displayUnit = "s"), useChatteringProtection = true,useLimitationInputs=false)
                              annotation (Placement(transformation(
              origin={127,102},
              extent={{-13,12},{13,-12}})));

        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eitha(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000204,
          ZeroPressureVolume=0.00014,
          ExternalPressure=-533.28954966,
          Elastance=190651014.00345)
          annotation (Placement(transformation(extent={{168,44},{190,66}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eetha(
          volume_start(displayUnit="ml") = 0.000526,
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          ZeroPressureVolume=0.00037,
          Elastance=74127247.40274)
          annotation (Placement(transformation(extent={{56,42},{82,68}})));
        Physiolibrary.Hydraulic.Components.Inertia inertia(I(displayUnit=
                "mmHg.s2/ml") = 226648.0586055, volumeFlow_start(displayUnit=
                "ml/min") = 0)                                                                                                                                             annotation(Placement(transformation(extent={{-11,-11},
                  {11,11}},                                                                                                    rotation = 180, origin={141,55})));
        Physiolibrary.Hydraulic.Components.Resistor Retha(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 7999343.2449)
          annotation (Placement(transformation(extent={{90,44},{112,66}})));
        Physiolibrary.Hydraulic.Components.Resistor Rsart(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 106657909.932) annotation (
            Placement(transformation(
              extent={{14,-13},{-14,13}},
              origin={24,55})));
        Physiolibrary.Hydraulic.Components.Resistor Rsven(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 26664477.483) annotation (
            Placement(transformation(
              extent={{14,-13},{-14,13}},
              origin={-60,55})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Est(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000283,
          ZeroPressureVolume=0.000185,
          Elastance=34930465.50273)
          annotation (Placement(transformation(extent={{-28,44},{-4,66}})));
        Physiolibrary.Hydraulic.Components.Resistor Rethv(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 11999014.86735)
          annotation (Placement(transformation(extent={{-120,42},{-146,68}})));
        Physiolibrary.Hydraulic.Components.Resistor Rrain(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{-208,42},{-236,68}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eithv(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.00148,
          ZeroPressureVolume=0.00119,
          ExternalPressure=-533.28954966,
          Elastance=2426467.450953)
          annotation (Placement(transformation(extent={{-194,42},{-166,68}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eethv(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.00153,
          ZeroPressureVolume=0.001,
          Elastance=2253148.3473135)
          annotation (Placement(transformation(extent={{-108,42},{-82,68}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance RightAtrium(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.000135,
          ZeroPressureVolume=3e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{-242,82},{-214,110}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance RightVentricle(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.000131,
          ZeroPressureVolume=4e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{-170,80},{-140,110}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance PulmonaryValve(
          Pknee = Modelica.Constants.eps,_Goff(displayUnit="ml/(mmHg.s)") = 0,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245,
          chatteringProtectionTime=0.01,                                                       lastChange(displayUnit = "s"), useChatteringProtection = true,
          useLimitationInputs=false)
          annotation (Placement(transformation(extent={{-132,108},{-106,82}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.AtrialElastance RAtrialElastance(EMIN=
              6666119.37075, EMAX=19998358.11225)
          annotation (Placement(transformation(extent={{-246,124},{-208,156}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.VentricularElastance
          RVentricularElastance(EMIN=7599376.082655, EMAX=65327969.83335)
          annotation (Placement(transformation(extent={{-182,126},{-152,160}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance TricuspidValve(
          _Goff=0,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245,
          chatteringProtectionTime=0.01,                                                       lastChange(displayUnit = "s"), open(fixed = true, start = true), useChatteringProtection = true,
          useLimitationInputs=false)
                              annotation (Placement(transformation(
              origin={-189,96},
              extent={{-13,12},{13,-12}})));
        replaceable Physiolibrary.Types.Constants.FrequencyConst HeartRate(k(displayUnit = "1/min") = 1.2) annotation(Placement(transformation(origin={-259,
                  166.5},                                                                                                                                              extent = {{-11, -6.5}, {11, 6.5}})));
      equation
        connect(Epa.q_in, Rpp.q_in) annotation (Line(
            points={{-80,136},{-56,136}},
            thickness=1));
        connect(Rpp.q_out, Epv.q_in) annotation (Line(
            points={{-22,136},{7,136}},
            thickness=1));
        connect(Epv.q_in, Rlain.q_in) annotation (Line(
            points={{7,136},{26,136}},
            thickness=1));
        connect(LeftAtrium.q_in, MitralValve.q_in) annotation (Line(
            points={{88,102},{114,102}},
            thickness=1));
        connect(LeftVentricle.q_in, MitralValve.q_out) annotation (Line(
            points={{164,102},{140,102}},
            thickness=1));
        connect(LeftVentricle.q_in, AorticValve.q_in) annotation (Line(
            points={{164,102},{182,102}},
            thickness=1));
        connect(LeftVentricle.compliance, LVentricularElastance.Ct) annotation (
           Line(
            points={{164,113.2},{164,112},{212,112},{212,145.68},{203.42,145.68}},
            color={0,0,127}));
        connect(Rlain.q_out, LeftAtrium.q_in) annotation (Line(
            points={{56,136},{74,136},{74,102},{88,102}},
            thickness=1));
        connect(Retha.q_in, Eetha.q_in) annotation (Line(
            points={{90,55},{69,55}},
            thickness=1));
        connect(Retha.q_out, inertia.q_out) annotation (Line(
            points={{112,55},{130,55}},
            thickness=1));
        connect(inertia.q_in, Eitha.q_in) annotation (Line(
            points={{152,55},{179,55}},
            thickness=1));
        connect(Eitha.q_in, AorticValve.q_out) annotation (Line(
            points={{179,55},{216,55},{216,102},{206,102}},
            thickness=1));
        connect(Rrain.q_in, Eithv.q_in) annotation (Line(
            points={{-208,55},{-180,55}},
            thickness=1));
        connect(Eithv.q_in, Rethv.q_out) annotation (Line(
            points={{-180,55},{-146,55}},
            thickness=1));
        connect(Rethv.q_in, Eethv.q_in) annotation (Line(
            points={{-120,55},{-95,55}},
            thickness=1));
        connect(RightAtrium.q_in, TricuspidValve.q_in) annotation (Line(
            points={{-228,96},{-202,96}},
            thickness=1));
        connect(RightVentricle.q_in, TricuspidValve.q_out) annotation (Line(
            points={{-155,95},{-164.5,95},{-164.5,96},{-176,96}},
            thickness=1));
        connect(RightVentricle.q_in, PulmonaryValve.q_in) annotation (Line(
            points={{-155,95},{-132,95}},
            thickness=1));
        connect(Rrain.q_out, RightAtrium.q_in) annotation (Line(
            points={{-236,55},{-250,55},{-250,96},{-228,96}},
            thickness=1));
        connect(RightAtrium.compliance,RAtrialElastance. Ct) annotation(Line(points={{-228,
                107.2},{-228,130},{-204.39,130},{-204.39,139.84}},                                                                                  color = {0, 0, 127}));
        connect(PulmonaryValve.q_out, Epa.q_in) annotation (Line(
            points={{-106,95},{-92,95},{-92,136},{-80,136}},
            thickness=1));
        connect(RightVentricle.compliance,RVentricularElastance. Ct) annotation(Line(points={{-155,
                107},{-155,118},{-126,118},{-126,146.91},{-149.15,146.91}},                                                                                            color = {0, 0, 127}));
        connect(LeftAtrium.compliance, LAtrialElastance.Ct) annotation (Line(
            points={{88,113.2},{88,112},{121.61,112},{121.61,145.84}},
            color={0,0,127}));
        connect(HeartRate.y,RAtrialElastance. HR) annotation(Line(points={{-245.25,
                166.5},{-227,166.5},{-227,152.8}},                                                                           color = {0, 0, 127}));
        connect(RVentricularElastance.HR, HeartRate.y) annotation(Line(points={{-167,
                156.6},{-167,166.5},{-245.25,166.5}},                                                                             color = {0, 0, 127}));
        connect(LAtrialElastance.HR, HeartRate.y) annotation (Line(
            points={{99,158.8},{99,166.5},{-245.25,166.5}},
            color={0,0,127}));
        connect(LVentricularElastance.HR, HeartRate.y) annotation (Line(
            points={{182,154.8},{182,166.5},{-245.25,166.5}},
            color={0,0,127}));
        connect(Est.q_in, Rsart.q_out) annotation (Line(
            points={{-16,55},{10,55}},
            thickness=1));
        connect(Rsart.q_in, Eetha.q_in) annotation (Line(
            points={{38,55},{69,55}},
            thickness=1));
        connect(Eethv.q_in, Rsven.q_out) annotation (Line(
            points={{-95,55},{-74,55}},
            thickness=1));
        connect(Rsven.q_in, Est.q_in) annotation (Line(
            points={{-46,55},{-16,55}},
            thickness=1));
        annotation(Diagram(coordinateSystem(extent={{-280,-140},{280,180}},      preserveAspectRatio=false)),             Icon(coordinateSystem(extent = {{-280, -140}, {280, 180}}, preserveAspectRatio = false), graphics),
          Documentation(info="<html>
<p>Model of cardiovascular system using to demonstrate elastic and resistance features of veins and arteries in pulmonary and systemic circulation and influence of cardiac output on it.</p>
<ul>
<li>J. A. Goodwin, W. L. van Meurs, C. D. Sa Couto, J. E. W.Beneken, S. A. Graves, A model for educational simulation of infant cardiovascular physiology., Anesthesia and analgesia 99 (6)(2004) 1655&ndash;1664. doi:10.1213/01.ANE.0000134797.52793.AF.</li>
<li>C. D. Sa Couto, W. L. van Meurs, J. A. Goodwin, P. Andriessen,A Model for Educational Simulation of Neonatal Cardiovascular Pathophysiology, Simulation in Healthcare 1 (Inaugural) (2006) 4&ndash;12.</li>
<li>W. van Meurs, Modeling and Simulation in Biomedical Engineering: Applications in Cardiorespiratory Physiology, McGraw-Hill Professional, 2011.</li>
</ul>
</html>",       revisions="<html>
<ul>
<li><i>Jul 2015 </i>by Tomas Kulhanek: Created. </li>
</ul>
</html>"),experiment(
            StopTime=20,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end Flat_Meurs_Physiolibrary;

      model Flat_Meurs_no_physiolibrary
      extends Physiolibrary.Icons.CardioVascular;

        MeursHemodynamics.Components.ElasticCompartment
                           intrathoracicArteries(
          elastance_NonSI=1.43,
          unstressedVolume_NonSI=140,
          externalPressure_NonSI=-4,
          V0_NonSI=204)
          annotation (Placement(transformation(extent={{232,-56},
                  {252,-36}})));
        MeursHemodynamics.Components.Resistor extrathoracicArterialResistance(
            bloodResistance_NonSI=0.06) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={150,-26})));
        MeursHemodynamics.Components.Inductor
                 aorticFlowInertia(inertance_NonSI=0.0017) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={198,-26})));
        MeursHemodynamics.Components.Resistor systemicArteriolarResistance(
            bloodResistance_NonSI=0.8) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={62,-26})));
        MeursHemodynamics.Components.ElasticCompartment
                           extrathoracicArteries(
          elastance_NonSI=0.556,
          unstressedVolume_NonSI=370,
          externalPressure_NonSI=0,
          V0_NonSI=526)
          annotation (Placement(transformation(extent={{102,-56},{122,-36}})));
        MeursHemodynamics.Components.ElasticCompartment
                           SystemicTissues(
          elastance_NonSI=0.262,
          unstressedVolume_NonSI=185,
          externalPressure_NonSI=0,
          V0_NonSI=283)
          annotation (Placement(transformation(extent={{10,-54},{30,-34}})));
        MeursHemodynamics.Components.Resistor smallVenuleResistance(
            bloodResistance_NonSI=0.2) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-40,-26})));
        MeursHemodynamics.Components.Resistor venousResistance(
            bloodResistance_NonSI=0.09) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-116,-26})));
        MeursHemodynamics.Components.ElasticCompartment
                           intrathoracicVeins(
          elastance_NonSI=0.0182,
          unstressedVolume_NonSI=1190,
          externalPressure_NonSI=-4,
          V0_NonSI=1480)
          annotation (Placement(transformation(extent={{-182,-50},{-162,-30}})));
        MeursHemodynamics.Components.ElasticCompartment
                           extrathoracicVeins(
          elastance_NonSI=0.0169,
          unstressedVolume_NonSI=1000,
          externalPressure_NonSI=0,
          V0_NonSI=1530)
          annotation (Placement(transformation(extent={{-92,-56},{-72,-36}})));
        MeursHemodynamics.Components.Resistor centralVenousResistance(
            bloodResistance_NonSI=0.003) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-208,-26})));
        MeursHemodynamics.Components.VariableElasticCompartment rightAtrium(
          unstressedVolume_NonSI=30,
          externalPressure_NonSI=-4,
          V0_NonSI=135) annotation (Placement(transformation(extent={{-222,8},{
                  -198,32}})));
        MeursHemodynamics.Components.VariableElasticCompartment rightVentricle(
          unstressedVolume_NonSI=40,
          externalPressure_NonSI=-4,
          V0_NonSI=131) annotation (Placement(transformation(extent={{-134,6},{
                  -108,32}})));
        MeursHemodynamics.Components.VariableElasticCompartment leftAtrium(
          unstressedVolume_NonSI=30,
          externalPressure_NonSI=-4,
          V0_NonSI=93.1)
          annotation (Placement(transformation(extent={{108,12},{134,38}})));
        MeursHemodynamics.Components.VariableElasticCompartment leftVentricle(
          unstressedVolume_NonSI=60,
          externalPressure_NonSI=-4,
          V0_NonSI=144)
          annotation (Placement(transformation(extent={{186,10},{212,36}})));
        MeursHemodynamics.Components.ElasticCompartment
                           pulmonaryArteries(
          elastance_NonSI=0.233,
          unstressedVolume_NonSI=50,
          externalPressure_NonSI=-4,
          V0_NonSI=106)
          annotation (Placement(transformation(extent={{-60,32},{-40,52}})));
        MeursHemodynamics.Components.Resistor pulmonaryResistance(
            bloodResistance_NonSI=0.11)
          annotation (Placement(transformation(extent={{-20,54},{0,74}})));
        MeursHemodynamics.Components.ElasticCompartment
                           pulmonaryVeins(
          elastance_NonSI=0.0455,
          unstressedVolume_NonSI=350,
          externalPressure_NonSI=-4,
          V0_NonSI=518)
          annotation (Placement(transformation(extent={{10,34},{30,54}})));
        MeursHemodynamics.Components.Resistor pulmonaryVenousResistance(
            bloodResistance_NonSI=0.003)
          annotation (Placement(transformation(extent={{50,54},{70,74}})));
        Modelica.Blocks.Sources.Constant HeartRate1(k=72)
          annotation (Placement(transformation(extent={{-250,74},{-230,94}})));
        MeursHemodynamics.Components.CardiacElastance
                         rightCardiacElastance(
          atrialElmin=0.05,
          atrialElmax=0.15,
          ventricularElmin=0.057,
          ventricularElmax=0.49) annotation (Placement(transformation(
              extent={{-17,-10},{17,10}},
              rotation=0,
              origin={-173,68})));
        MeursHemodynamics.Components.CardiacElastance
                         leftCardiacElastance(
          atrialElmin=0.12,
          atrialElmax=0.28,
          ventricularElmin=0.09,
          ventricularElmax=4) annotation (Placement(transformation(
              extent={{-17,-12},{17,12}},
              rotation=0,
              origin={121,72})));
        MeursHemodynamics.Components.CardiacValve
                     tricuspidalValve(outflowResistance=0.003,
            backflowConductance=0)
          annotation (Placement(transformation(extent={{-178,10},{-158,30}})));
        MeursHemodynamics.Components.CardiacValve
                     pulmonicValve(outflowResistance=0.003)
          annotation (Placement(transformation(extent={{-98,8},{-78,28}})));
        MeursHemodynamics.Components.CardiacValve
                     mitralValve(outflowResistance=0.003, backflowConductance=0)
          annotation (Placement(transformation(extent={{146,14},{166,34}})));
        MeursHemodynamics.Components.CardiacValve
                     aorticValve(outflowResistance=0.003, backflowConductance=0)
          annotation (Placement(transformation(extent={{222,10},{246,34}})));
      equation
        connect(aorticFlowInertia.bloodFlowInflow,intrathoracicArteries. bloodFlowInflow)
          annotation (Line(
            points={{208.4,-26},{242,-26},{242,
                -46}},
            color={0,0,0},
            thickness=1));
        connect(aorticFlowInertia.bloodFlowOutflow,
          extrathoracicArterialResistance.bloodFlowInflow) annotation (Line(
            points={{188.4,-26},{160.4,-26}},
            color={0,0,0},
            thickness=1));
        connect(extrathoracicArterialResistance.bloodFlowOutflow,
          extrathoracicArteries.bloodFlowInflow) annotation (Line(
            points={{140.4,-26},{112,-26},{112,-46}},
            color={0,0,0},
            thickness=1));
        connect(extrathoracicArteries.bloodFlowInflow,
          systemicArteriolarResistance.bloodFlowInflow) annotation (Line(
            points={{112,-46},{112,-26},{72.4,-26}},
            color={0,0,0},
            thickness=1));
        connect(SystemicTissues.bloodFlowInflow,systemicArteriolarResistance. bloodFlowOutflow)
          annotation (Line(
            points={{20,-44},{20,-26},{52.4,-26}},
            color={0,0,0},
            thickness=1));
        connect(smallVenuleResistance.bloodFlowInflow,SystemicTissues. bloodFlowInflow)
          annotation (Line(
            points={{-29.6,-26},{20,-26},{20,-44}},
            color={28,108,200},
            thickness=1));
        connect(smallVenuleResistance.bloodFlowOutflow,extrathoracicVeins. bloodFlowInflow)
          annotation (Line(
            points={{-49.6,-26},{-82,-26},{-82,-46}},
            color={28,108,200},
            thickness=1));
        connect(smallVenuleResistance.bloodFlowOutflow,venousResistance. bloodFlowInflow)
          annotation (Line(
            points={{-49.6,-26},{-105.6,-26}},
            color={28,108,200},
            thickness=1));
        connect(centralVenousResistance.bloodFlowInflow,venousResistance. bloodFlowOutflow)
          annotation (Line(
            points={{-197.6,-26},{-125.6,-26}},
            color={28,108,200},
            thickness=1));
        connect(centralVenousResistance.bloodFlowInflow,intrathoracicVeins. bloodFlowInflow)
          annotation (Line(
            points={{-197.6,-26},{-172,-26},{-172,-40}},
            color={28,108,200},
            thickness=1));
        connect(pulmonaryResistance.bloodFlowInflow,pulmonaryArteries. bloodFlowInflow)
          annotation (Line(
            points={{-20.4,64},{-50,64},{-50,42}},
            color={28,108,200},
            thickness=1));
        connect(pulmonaryResistance.bloodFlowOutflow,pulmonaryVeins. bloodFlowInflow)
          annotation (Line(
            points={{-0.4,64},{20,64},{20,44}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryVenousResistance.bloodFlowInflow,pulmonaryVeins. bloodFlowInflow)
          annotation (Line(
            points={{49.6,64},{20,64},{20,44}},
            color={0,0,0},
            thickness=1));
        connect(centralVenousResistance.bloodFlowOutflow, rightAtrium.bloodFlowInflow)
          annotation (Line(
            points={{-217.6,-26},{-240,-26},{-240,20},{-210,20}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryVenousResistance.bloodFlowOutflow, leftAtrium.bloodFlowInflow)
          annotation (Line(
            points={{69.6,64},{90,64},{90,25},{121,25}},
            color={0,0,0},
            thickness=1));
        connect(HeartRate1.y, rightCardiacElastance.HR) annotation (Line(
            points={{-229,84},{-200,84},{-200,68},{-191.7,68}},
            color={0,0,127},
            thickness=1));
        connect(rightAtrium.inputElastance, rightCardiacElastance.Eta)
          annotation (Line(
            points={{-210,33.2},{-210,48},{-146,48},{-146,64.4},{-154.3,64.4}},
            color={0,0,127},
            thickness=1));

        connect(rightCardiacElastance.Etv, rightVentricle.inputElastance)
          annotation (Line(
            points={{-154.3,74.2},{-121,74.2},{-121,33.3}},
            color={0,0,127},
            thickness=1));
        connect(leftCardiacElastance.HR, rightCardiacElastance.HR) annotation (
            Line(
            points={{102.3,72},{90,72},{90,84},{-200,84},{-200,68},{-191.7,68}},
            color={0,0,127},
            thickness=1));

        connect(leftCardiacElastance.Eta, leftAtrium.inputElastance)
          annotation (Line(
            points={{139.7,67.68},{160,67.68},{160,52},{120,52},{120,39.3},{121,
                39.3}},
            color={0,0,127},
            thickness=1));
        connect(leftVentricle.inputElastance, leftCardiacElastance.Etv)
          annotation (Line(
            points={{199,37.3},{200,37.3},{200,79.44},{139.7,79.44}},
            color={0,0,127},
            thickness=1));
        connect(rightAtrium.bloodFlowInflow, tricuspidalValve.bloodFlowInflow)
          annotation (Line(
            points={{-210,20},{-178,20}},
            color={0,0,0},
            thickness=1));
        connect(tricuspidalValve.bloodFlowOutflow, rightVentricle.bloodFlowInflow)
          annotation (Line(
            points={{-158,20},{-121,19}},
            color={0,0,0},
            thickness=1));
        connect(rightVentricle.bloodFlowInflow, pulmonicValve.bloodFlowInflow)
          annotation (Line(
            points={{-121,19},{-98,19},{-98,18}},
            color={0,0,0},
            thickness=1));
        connect(pulmonicValve.bloodFlowOutflow, pulmonaryArteries.bloodFlowInflow)
          annotation (Line(
            points={{-78,18},{-68,18},{-68,64},{-50,64},{-50,42}},
            color={0,0,0},
            thickness=1));
        connect(leftAtrium.bloodFlowInflow, mitralValve.bloodFlowInflow)
          annotation (Line(
            points={{121,25},{122,25},{122,24},{146,24}},
            color={0,0,0},
            thickness=1));
        connect(mitralValve.bloodFlowOutflow, leftVentricle.bloodFlowInflow)
          annotation (Line(
            points={{166,24},{199,23}},
            color={0,0,0},
            thickness=1));
        connect(leftVentricle.bloodFlowInflow, aorticValve.bloodFlowInflow)
          annotation (Line(
            points={{199,23},{200,23},{200,22},{222,22}},
            color={0,0,0},
            thickness=1));
        connect(aorticValve.bloodFlowOutflow, intrathoracicArteries.bloodFlowInflow)
          annotation (Line(
            points={{246,22},{258,22},{258,-26},{
                242,-26},{242,-46}},
            color={0,0,0},
            thickness=1));
        annotation(Diagram(coordinateSystem(extent={{-280,-140},{280,180}},      preserveAspectRatio=false)),             Icon(coordinateSystem(extent = {{-280, -140}, {280, 180}}, preserveAspectRatio = false), graphics),
          Documentation(info="<html>
<p>Model of cardiovascular system using to demonstrate elastic and resistance features of veins and arteries in pulmonary and systemic circulation and influence of cardiac output on it.</p>
<ul>
<li>J. A. Goodwin, W. L. van Meurs, C. D. Sa Couto, J. E. W.Beneken, S. A. Graves, A model for educational simulation of infant cardiovascular physiology., Anesthesia and analgesia 99 (6)(2004) 1655&ndash;1664. doi:10.1213/01.ANE.0000134797.52793.AF.</li>
<li>C. D. Sa Couto, W. L. van Meurs, J. A. Goodwin, P. Andriessen,A Model for Educational Simulation of Neonatal Cardiovascular Pathophysiology, Simulation in Healthcare 1 (Inaugural) (2006) 4&ndash;12.</li>
<li>W. van Meurs, Modeling and Simulation in Biomedical Engineering: Applications in Cardiorespiratory Physiology, McGraw-Hill Professional, 2011.</li>
</ul>
</html>",       revisions="<html>
<ul>
<li><i>Jul 2015 </i>by Tomas Kulhanek: Created. </li>
</ul>
</html>"),experiment(
            StopTime=20,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end Flat_Meurs_no_physiolibrary;

      model Flat_Meurs_Physiolibrary_withModelSettings

      extends Physiolibrary.Icons.CardioVascular;
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Epa(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=modelSettings.pulmonaryArteries_initialVolume,
          ZeroPressureVolume=5e-05,
          ExternalPressure=-533.28954966,
          Elastance=31064116.267695)
          annotation (Placement(transformation(extent={{-94,122},{-66,150}})));

        Physiolibrary.Hydraulic.Components.Resistor Rpp(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 14665462.61565)
          annotation (Placement(transformation(extent={{-56,123},{-22,149}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Epv(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=modelSettings.pulmonaryVeins_initialVolume,
          ZeroPressureVolume=0.00035,
          ExternalPressure=-533.28954966,
          Elastance=6066168.6273825)
          annotation (Placement(transformation(extent={{-10,120},
                  {24,148}})));

        Physiolibrary.Hydraulic.Components.Resistor Rlain(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{26,124},{56,148}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance LeftAtrium(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=modelSettings.leftAtrium_initialVolume,
          ZeroPressureVolume=3e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{74,88},{102,116}})));

        Physiolibrary.Hydraulic.Components.ElasticVesselElastance LeftVentricle(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=modelSettings.leftVentricle_initialVolume,
          ZeroPressureVolume=6e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{150,88},{178,116}})));

        Physiolibrary.Hydraulic.Components.IdealValveResistance AorticValve(
          _Goff(displayUnit="ml/(mmHg.s)") = 0,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245,
          chatteringProtectionTime=0.01,                                                       lastChange(displayUnit = "s"), useChatteringProtection = true,
          useLimitationInputs=false)
          annotation (Placement(transformation(extent={{182,114},{206,90}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.AtrialElastance LAtrialElastance(
          Tav(displayUnit="s"),
          EMIN=15998686.4898,
          EMAX=37330268.4762)
          annotation (Placement(transformation(extent={{80,130},{118,162}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.VentricularElastance
          LVentricularElastance(EMIN=11999014.86735, EMAX=533289549.66)
          annotation (Placement(transformation(extent={{164,126},{200,158}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance MitralValve(
          _Goff(displayUnit="ml/(mmHg.s)") = 0,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245,
          chatteringProtectionTime=0.01,                                                       lastChange(displayUnit = "s"), useChatteringProtection = true,useLimitationInputs=false)
                              annotation (Placement(transformation(
              origin={127,102},
              extent={{-13,12},{13,-12}})));

        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eitha(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=modelSettings.intrathoracicArteries_initialVolume,
          ZeroPressureVolume=0.00014,
          ExternalPressure=-533.28954966,
          Elastance=190651014.00345)
          annotation (Placement(transformation(extent={{168,44},{190,66}})));

        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eetha(
          volume_start(displayUnit="ml")=
            modelSettings.extrathoracicArteries_initialVolume,
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          ZeroPressureVolume=0.00037,
          Elastance=74127247.40274)
          annotation (Placement(transformation(extent={{56,42},{82,68}})));
        Physiolibrary.Hydraulic.Components.Inertia inertia(I(displayUnit=
                "mmHg.s2/ml") = 226648.0586055, volumeFlow_start(displayUnit=
                "ml/min") = 0)                                                                                                                                             annotation(Placement(transformation(extent={{-11,-11},
                  {11,11}},                                                                                                    rotation = 180, origin={141,55})));
        Physiolibrary.Hydraulic.Components.Resistor Retha(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 7999343.2449)
          annotation (Placement(transformation(extent={{90,44},{112,66}})));
        Physiolibrary.Hydraulic.Components.Resistor Rsart(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 106657909.932) annotation (
            Placement(transformation(
              extent={{14,-13},{-14,13}},
              origin={24,55})));
        Physiolibrary.Hydraulic.Components.Resistor Rsven(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 26664477.483) annotation (
            Placement(transformation(
              extent={{14,-13},{-14,13}},
              origin={-60,55})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Est(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=modelSettings.systemicTissues_initialVolume,
          ZeroPressureVolume=0.000185,
          Elastance=34930465.50273)
          annotation (Placement(transformation(extent={{-28,44},{-4,66}})));

        Physiolibrary.Hydraulic.Components.Resistor Rethv(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 11999014.86735)
          annotation (Placement(transformation(extent={{-120,42},{-146,68}})));
        Physiolibrary.Hydraulic.Components.Resistor Rrain(useConductanceInput=false,
            Resistance(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{-208,42},{-236,68}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eithv(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=modelSettings.intrathoracicVein_initialVolume,
          ZeroPressureVolume=0.00119,
          ExternalPressure=-533.28954966,
          Elastance=2426467.450953)
          annotation (Placement(transformation(extent={{-194,42},{-166,68}})));

        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eethv(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=modelSettings.extrathoracicVein_initialVolume,
          ZeroPressureVolume=0.001,
          Elastance=2253148.3473135)
          annotation (Placement(transformation(extent={{-108,42},{-82,68}})));

        Physiolibrary.Hydraulic.Components.ElasticVesselElastance RightAtrium(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=modelSettings.rightAtrium_initialVolume,
          ZeroPressureVolume=3e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{-242,82},{-214,110}})));

        Physiolibrary.Hydraulic.Components.ElasticVesselElastance RightVentricle(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=modelSettings.rightVentricle_initialVolume,
          ZeroPressureVolume=4e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{-170,80},{-140,110}})));

        Physiolibrary.Hydraulic.Components.IdealValveResistance PulmonaryValve(
          Pknee = Modelica.Constants.eps,_Goff(displayUnit="ml/(mmHg.s)") = 0,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245,
          chatteringProtectionTime=0.01,                                                       lastChange(displayUnit = "s"), useChatteringProtection = true,
          useLimitationInputs=false)
          annotation (Placement(transformation(extent={{-132,108},{-106,82}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.AtrialElastance RAtrialElastance(EMIN=
              6666119.37075, EMAX=19998358.11225)
          annotation (Placement(transformation(extent={{-246,124},{-208,156}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.VentricularElastance
          RVentricularElastance(EMIN=7599376.082655, EMAX=65327969.83335)
          annotation (Placement(transformation(extent={{-182,126},{-152,160}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance TricuspidValve(
          _Goff=0,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245,
          chatteringProtectionTime=0.01,                                                       lastChange(displayUnit = "s"), open(fixed = true, start = true), useChatteringProtection = true,
          useLimitationInputs=false)
                              annotation (Placement(transformation(
              origin={-189,96},
              extent={{-13,12},{13,-12}})));
        replaceable Physiolibrary.Types.Constants.FrequencyConst HeartRate(k(displayUnit = "1/min") = 1.2) annotation(Placement(transformation(origin={-259,
                  166.5},                                                                                                                                              extent = {{-11, -6.5}, {11, 6.5}})));
        ModelSettings modelSettings(
          intrathoracicArteries_initialVolume=0.000207,
          extrathoracicArteries_initialVolume=0.000526,
          systemicTissues_initialVolume=0.000283,
          extrathoracicVein_initialVolume=0.00183,
          intrathoracicVein_initialVolume=0.00148,
          pulmonaryArteries_initialVolume=0.000106,
          pulmonaryVeins_initialVolume=0.000518,
          rightAtrium_initialVolume=0.000135,
          rightVentricle_initialVolume=0.000131,
          leftAtrium_initialVolume=9.3e-05,
          leftVentricle_initialVolume=0.000144,
          aorticFlowIntertia_inertance=815933010.9798)
          annotation (Placement(transformation(
                extent={{-150,-48},{-90,2}})));

      equation
        connect(Epa.q_in, Rpp.q_in) annotation (Line(
            points={{-80,136},{-56,136}},
            thickness=1));
        connect(Rpp.q_out, Epv.q_in) annotation (Line(
            points={{-22,136},{-8,136},{-8,134},{
                7,134}},
            thickness=1));
        connect(Epv.q_in, Rlain.q_in) annotation (Line(
            points={{7,134},{16,134},{16,136},{26,
                136}},
            thickness=1));
        connect(LeftAtrium.q_in, MitralValve.q_in) annotation (Line(
            points={{88,102},{114,102}},
            thickness=1));
        connect(LeftVentricle.q_in, MitralValve.q_out) annotation (Line(
            points={{164,102},{140,102}},
            thickness=1));
        connect(LeftVentricle.q_in, AorticValve.q_in) annotation (Line(
            points={{164,102},{182,102}},
            thickness=1));
        connect(LeftVentricle.compliance, LVentricularElastance.Ct) annotation (
           Line(
            points={{164,113.2},{164,112},{212,112},{212,145.68},{203.42,145.68}},
            color={0,0,127}));
        connect(Rlain.q_out, LeftAtrium.q_in) annotation (Line(
            points={{56,136},{74,136},{74,102},{88,102}},
            thickness=1));
        connect(Retha.q_in, Eetha.q_in) annotation (Line(
            points={{90,55},{69,55}},
            thickness=1));
        connect(Retha.q_out, inertia.q_out) annotation (Line(
            points={{112,55},{130,55}},
            thickness=1));
        connect(inertia.q_in, Eitha.q_in) annotation (Line(
            points={{152,55},{179,55}},
            thickness=1));
        connect(Eitha.q_in, AorticValve.q_out) annotation (Line(
            points={{179,55},{216,55},{216,102},{206,102}},
            thickness=1));
        connect(Rrain.q_in, Eithv.q_in) annotation (Line(
            points={{-208,55},{-180,55}},
            thickness=1));
        connect(Eithv.q_in, Rethv.q_out) annotation (Line(
            points={{-180,55},{-146,55}},
            thickness=1));
        connect(Rethv.q_in, Eethv.q_in) annotation (Line(
            points={{-120,55},{-95,55}},
            thickness=1));
        connect(RightAtrium.q_in, TricuspidValve.q_in) annotation (Line(
            points={{-228,96},{-202,96}},
            thickness=1));
        connect(RightVentricle.q_in, TricuspidValve.q_out) annotation (Line(
            points={{-155,95},{-164.5,95},{-164.5,96},{-176,96}},
            thickness=1));
        connect(RightVentricle.q_in, PulmonaryValve.q_in) annotation (Line(
            points={{-155,95},{-132,95}},
            thickness=1));
        connect(Rrain.q_out, RightAtrium.q_in) annotation (Line(
            points={{-236,55},{-250,55},{-250,96},{-228,96}},
            thickness=1));
        connect(RightAtrium.compliance,RAtrialElastance. Ct) annotation(Line(points={{-228,
                107.2},{-228,130},{-204.39,130},{-204.39,139.84}},                                                                                  color = {0, 0, 127}));
        connect(PulmonaryValve.q_out, Epa.q_in) annotation (Line(
            points={{-106,95},{-92,95},{-92,136},{-80,136}},
            thickness=1));
        connect(RightVentricle.compliance,RVentricularElastance. Ct) annotation(Line(points={{-155,
                107},{-155,118},{-126,118},{-126,146.91},{-149.15,146.91}},                                                                                            color = {0, 0, 127}));
        connect(LeftAtrium.compliance, LAtrialElastance.Ct) annotation (Line(
            points={{88,113.2},{88,112},{121.61,112},{121.61,145.84}},
            color={0,0,127}));
        connect(HeartRate.y,RAtrialElastance. HR) annotation(Line(points={{-245.25,
                166.5},{-227,166.5},{-227,152.8}},                                                                           color = {0, 0, 127}));
        connect(RVentricularElastance.HR, HeartRate.y) annotation(Line(points={{-167,
                156.6},{-167,166.5},{-245.25,166.5}},                                                                             color = {0, 0, 127}));
        connect(LAtrialElastance.HR, HeartRate.y) annotation (Line(
            points={{99,158.8},{99,166.5},{-245.25,166.5}},
            color={0,0,127}));
        connect(LVentricularElastance.HR, HeartRate.y) annotation (Line(
            points={{182,154.8},{182,166.5},{-245.25,166.5}},
            color={0,0,127}));
        connect(Est.q_in, Rsart.q_out) annotation (Line(
            points={{-16,55},{10,55}},
            thickness=1));
        connect(Rsart.q_in, Eetha.q_in) annotation (Line(
            points={{38,55},{69,55}},
            thickness=1));
        connect(Eethv.q_in, Rsven.q_out) annotation (Line(
            points={{-95,55},{-74,55}},
            thickness=1));
        connect(Rsven.q_in, Est.q_in) annotation (Line(
            points={{-46,55},{-16,55}},
            thickness=1));
        annotation(Diagram(coordinateSystem(extent={{-280,-140},{280,180}},      preserveAspectRatio=false)),             Icon(coordinateSystem(extent = {{-280, -140}, {280, 180}}, preserveAspectRatio = false), graphics),
          Documentation(info="<html>
<p>Model of cardiovascular system using to demonstrate elastic and resistance features of veins and arteries in pulmonary and systemic circulation and influence of cardiac output on it.</p>
<ul>
<li>J. A. Goodwin, W. L. van Meurs, C. D. Sa Couto, J. E. W.Beneken, S. A. Graves, A model for educational simulation of infant cardiovascular physiology., Anesthesia and analgesia 99 (6)(2004) 1655&ndash;1664. doi:10.1213/01.ANE.0000134797.52793.AF.</li>
<li>C. D. Sa Couto, W. L. van Meurs, J. A. Goodwin, P. Andriessen,A Model for Educational Simulation of Neonatal Cardiovascular Pathophysiology, Simulation in Healthcare 1 (Inaugural) (2006) 4&ndash;12.</li>
<li>W. van Meurs, Modeling and Simulation in Biomedical Engineering: Applications in Cardiorespiratory Physiology, McGraw-Hill Professional, 2011.</li>
</ul>
</html>",       revisions="<html>
<ul>
<li><i>Jul 2015 </i>by Tomas Kulhanek: Created. </li>
</ul>
</html>"),experiment(
            StopTime=20,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end Flat_Meurs_Physiolibrary_withModelSettings;

      model ModelSettings
        //initial volumes
        parameter Physiolibrary.Types.Volume intrathoracicArteries_initialVolume=0.000204;
        parameter Physiolibrary.Types.Volume extrathoracicArteries_initialVolume=0.000526;
        parameter Physiolibrary.Types.Volume systemicTissues_initialVolume=0.000283;
        parameter Physiolibrary.Types.Volume extrathoracicVein_initialVolume=0.00153;
        parameter Physiolibrary.Types.Volume intrathoracicVein_initialVolume=0.00148;

        parameter Physiolibrary.Types.Volume pulmonaryArteries_initialVolume=0.000106;
        parameter Physiolibrary.Types.Volume pulmonaryVeins_initialVolume=0.000518;

        parameter Physiolibrary.Types.Volume rightAtrium_initialVolume=0.000135;
        parameter Physiolibrary.Types.Volume rightVentricle_initialVolume=0.000131;
        parameter Physiolibrary.Types.Volume leftAtrium_initialVolume=9.3e-05;
        parameter Physiolibrary.Types.Volume leftVentricle_initialVolume=0.000144;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Polygon(
                points={{80,100},{40,100},{40,98},{40,60},{-60,-40},{-100,-40},{-100,-80},
                    {-80,-60},{-60,-80},{-80,-100},{-40,-100},{-40,-60},{60,40},{100,40},
                    {100,80},{80,60},{60,80},{80,100}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid), Polygon(
                points={{-80,100},{-40,100},{-40,98},{-40,60},{60,-40},{100,-40},{100,
                    -80},{80,-60},{60,-80},{80,-100},{40,-100},{40,-60},{-60,40},{-100,
                    40},{-100,80},{-80,60},{-60,80},{-80,100}},
                lineColor={0,0,0},
                fillColor={238,46,47},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-122,-110},{116,-134}},
                textColor={28,108,200},
                textString="%name")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end ModelSettings;

      model MeursModelPhysiolobrary
        extends Physiolibrary.Icons.CardioVascular;
        Components.HeartPhysiolibrary heartPhysiolibrary
          annotation (Placement(transformation(extent={{-44,-30},{38,42}})));
        Components.PulmonaryCirculation pulmonaryCirculation
          annotation (Placement(transformation(extent={{-62,24},{60,116}})));
        Components.SystemicCirculation systemicCirculation
          annotation (Placement(transformation(extent={{-60,-104},{60,2}})));
      equation
        connect(heartPhysiolibrary.pulmonaryArteryOutflow, pulmonaryCirculation.pulmonaryBloodInflow)
          annotation (Line(
            points={{-26.78,16.8},{-86,16.8},{-86,70},{-62,70}},
            color={0,0,0},
            thickness=1));
        connect(systemicCirculation.systemicBloodOutflow, heartPhysiolibrary.RightAtriumInflow)
          annotation (Line(
            points={{-60,-51},{-60,-52},{-88,-52},{-88,-1.2},{-26.78,-1.2}},
            color={0,0,0},
            thickness=1));
        connect(systemicCirculation.systemicBloodInflow, heartPhysiolibrary.AortaOutflow)
          annotation (Line(
            points={{60,-51},{84,-51},{84,-2.64},{24.06,-2.64}},
            color={0,0,0},
            thickness=1));
        connect(heartPhysiolibrary.leftAtriumInflow, pulmonaryCirculation.pulmonaryBloodOutflow)
          annotation (Line(
            points={{24.06,14.64},{86,14.64},{86,70.92},{60,70.92}},
            color={0,0,0},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=20,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end MeursModelPhysiolobrary;

      model MeursModelNoPhysiolibrary
        MeursHemodynamics.Components.Heart heart(
            currentHeartRate=72) annotation (
            Placement(transformation(extent={{-22,
                  -22},{20,24}})));
        MeursHemodynamics.Components.SystemicCirculation
          systemicCirculation annotation (
            Placement(transformation(extent={{-30,
                  -84},{30,-24}})));
        MeursHemodynamics.Components.PulmonaryCirculation
          pulmonaryCirculation annotation (
            Placement(transformation(extent={{-30,
                  14},{30,74}})));
      equation
        connect(systemicCirculation.bloodFlowOutflow, heart.rightAtriumFlowInflow)
          annotation (Line(
            points={{-30.6,-54},{-40,-54},{-40,3.3},{-16.12,3.3}},
            color={28,108,200},
            thickness=1));
        connect(heart.pulmonaryArteryOutflow, pulmonaryCirculation.bloodFlowInflow)
          annotation (Line(
            points={{-7.72,11.58},{-40,11.58},{-40,44},{-30.6,44}},
            color={28,108,200},
            thickness=1));
        connect(systemicCirculation.bloodFlowInflow, heart.aortaOutflow)
          annotation (Line(
            points={{28.8,-54},{40,-54},{40,3.3},{15.38,3.3}},
            color={255,0,0},
            thickness=1));
        connect(heart.leftAtriumFlowInflow, pulmonaryCirculation.bloodFlowOutflow)
          annotation (Line(
            points={{6.14,11.58},{40,11.58},{40,44},{30,44}},
            color={238,46,47},
            thickness=1));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false), graphics={Ellipse(
                extent={{-62,62},{64,-64}},
                lineColor={255,0,0},
                pattern=LinePattern.None,
                lineThickness=1,
                fillPattern=FillPattern.Sphere,
                fillColor={244,125,35})}),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=50000,
            __Dymola_Algorithm="Dassl"));
      end MeursModelNoPhysiolibrary;

      model testElsticVesselsWithInnerVolume
        PhysiolibraryExpantion.Obsolent.ElasticVesselWithInnerVolume
          doubleWallElastic(
          useV0Input=false,
          ZeroPressureVolume=0.0001,
          useComplianceInput=false,
          Compliance(displayUnit="ml/mmHg") = 7.5006157584566e-08,
          useExternalPressureInput=false,
          useInnerInput=true)
          annotation (Placement(transformation(extent={{20,-54},{40,-34}})));
        Physiolibrary.Hydraulic.Components.ElasticVessel
          elasticVessel1(
          volume_start=0.001,
          Compliance=7.5006157584566e-09,
            useExternalPressureInput=true)
                         annotation (Placement(
              transformation(extent={{18,-20},{38,
                  0}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedPump
          unlimitedPump(useSolutionFlowInput=true)
          annotation (Placement(transformation(
                extent={{-30,-24},{-10,-4}})));
        Physiolibrary.Types.Constants.VolumeConst
          volume(k=0.0001)
                          annotation (Placement(
              transformation(extent={{-90,30},{
                  -82,38}})));
        Modelica.Blocks.Math.Product product1
          annotation (Placement(transformation(
                extent={{-54,30},{-34,50}})));
        Modelica.Blocks.Sources.Sine sine(offset=0)
          annotation (Placement(transformation(
                extent={{-94,50},{-74,70}})));
        Physiolibrary.Hydraulic.Components.ElasticVessel
          elasticVessel2(volume_start=0.0001,
                         useExternalPressureInput=false)
                         annotation (Placement(
              transformation(extent={{18,36},{38,56}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedPump
          unlimitedPump1(useSolutionFlowInput=true)
          annotation (Placement(transformation(
                extent={{-12,36},{8,56}})));
      equation
        connect(doubleWallElastic.InnerVolume, elasticVessel1.volume)
          annotation (Line(points={{26,-42.6},{26,-44},{6,-44},{6,-28},{34,-28},
                {34,-20}}, color={0,0,127}));
        connect(doubleWallElastic.pressure, elasticVessel1.externalPressure)
          annotation (Line(points={{31.5,-53.9},{-2,-53.9},{-2,6},{36,6},{36,-2}},
              color={0,0,127}));
        connect(unlimitedPump.q_out,
          elasticVessel1.q_in) annotation (Line(
            points={{-10,-14},{14,-14},{14,-10},{
                28,-10}},
            color={0,0,0},
            thickness=1));
        connect(volume.y, product1.u2)
          annotation (Line(points={{-81,34},{-56,
                34}}, color={0,0,127}));
        connect(product1.y, unlimitedPump.solutionFlow)
          annotation (Line(points={{-33,40},{-20,
                40},{-20,-7}}, color={0,0,127}));
        connect(sine.y, product1.u1) annotation (
            Line(points={{-73,60},{-62,60},{-62,
                46},{-56,46}}, color={0,0,127}));
        connect(unlimitedPump1.q_out, elasticVessel2.q_in) annotation (Line(
            points={{8,46},{28,46}},
            color={0,0,0},
            thickness=1));
        connect(unlimitedPump1.solutionFlow, unlimitedPump.solutionFlow)
          annotation (Line(points={{-2,53},{-2,58},{-22,58},{-22,40},{-20,40},{
                -20,-7}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(
                preserveAspectRatio=false)),
            Diagram(coordinateSystem(
                preserveAspectRatio=false)));
      end testElsticVesselsWithInnerVolume;

      model TestSimpleRespirationMechnaics
        Physiolibrary.Hydraulic.Components.Resistor Rp(Resistance(displayUnit=
                "(cmH2O.s)/l") = 49033.25)
          annotation (Placement(transformation(extent={{0,26},{20,46}})));
        Physiolibrary.Hydraulic.Components.Resistor Rc(Resistance(displayUnit=
                "(cmH2O.s)/l") = 98066.5)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-34,36})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume(
            usePressureInput=true)
          annotation (Placement(transformation(extent={{-76,28},{-56,48}})));
        Physiolibrary.Types.Constants.PressureConst pressure(k(displayUnit=
                "cmH2O") = 245.16625)
          annotation (Placement(transformation(extent={{-84,86},{-76,94}})));
        Modelica.Blocks.Sources.Sine sine(f(displayUnit="1/min") = 0.25)
          annotation (Placement(transformation(extent={{-106,50},{-86,70}})));
        Modelica.Blocks.Math.Product product1
          annotation (Placement(transformation(extent={{-46,62},{-26,82}})));
        PhysiolibraryExpantion.Obsolent.SerialElasticConnections
          serialElasticConnections
          annotation (Placement(transformation(extent={{-18,-70},{2,-50}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume1(
            usePressureInput=true)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-88,-38})));
        Physiolibrary.Hydraulic.Components.Resistor Rp1(Resistance(displayUnit=
                "(cmH2O.s)/l") = 49033.25)
          annotation (Placement(transformation(extent={{-38,-70},{-18,-50}})));
        Physiolibrary.Hydraulic.Components.Resistor Rc1(Resistance(displayUnit=
                "(cmH2O.s)/l") = 98066.5)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-78,-60})));
        PhysiolibraryExpantion.ElasticVessel Cs(Compliance(displayUnit=
                "l/cmH2O") = 5.0985810648896e-08)
          annotation (Placement(transformation(extent={{-26,-4},{-6,16}})));
        PhysiolibraryExpantion.ElasticVessel Cs1(Compliance(displayUnit=
                "l/cmH2O") = 5.0985810648896e-08)
          annotation (Placement(transformation(extent={{-64,-88},{-44,-68}})));
        PhysiolibraryExpantion.ElasticVessel CL(
          volume_start=1e-05,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{32,26},{52,46}})));
        PhysiolibraryExpantion.ElasticVessel CL1(
          volume_start=1e-05,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{-18,-38},{2,-18}})));
        PhysiolibraryExpantion.ElasticVessel Cw1(
          volume_start=1e-05,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=false,
          hardElastance=true)
          annotation (Placement(transformation(extent={{-16,-100},{4,-80}})));
        PhysiolibraryExpantion.Obsolent.ElasticVesselWithInnerVolume Cw(
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useInnerInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{38,-8},{58,12}})));
      equation
        connect(product1.u1, pressure.y) annotation (Line(points={{-48,78},{-68,
                78},{-68,90},{-75,90}}, color={0,0,127}));
        connect(sine.y, product1.u2) annotation (Line(points={{-85,60},{-56,60},
                {-56,66},{-48,66}}, color={0,0,127}));
        connect(product1.y, unlimitedVolume.pressure) annotation (Line(points={
                {-25,72},{-22,72},{-22,54},{-84,54},{-84,38},{-76,38}}, color={
                0,0,127}));
        connect(Rc.q_in, unlimitedVolume.y) annotation (Line(
            points={{-44,36},{-50,36},{-50,38},{-56,38}},
            color={0,0,0},
            thickness=1));
        connect(unlimitedVolume1.pressure, unlimitedVolume.pressure)
          annotation (Line(points={{-88,-28},{-86,-28},{-86,38},{-76,38}},
              color={0,0,127}));
        connect(Rc1.q_in, unlimitedVolume1.y) annotation (Line(
            points={{-88,-60},{-88,-48}},
            color={0,0,0},
            thickness=1));
        connect(Rp1.q_out, serialElasticConnections.Inner_a) annotation (Line(
            points={{-18,-60},{-8,-60},{-8,-60.2},{-7.4,-60.2}},
            color={0,0,0},
            thickness=1));
        connect(Cs.q_in, Rp.q_in) annotation (Line(
            points={{-16,6},{-16,36},{0,36}},
            color={0,0,0},
            thickness=1));
        connect(Rc.q_out, Rp.q_in) annotation (Line(
            points={{-24,36},{0,36}},
            color={0,0,0},
            thickness=1));
        connect(Cs1.q_in, Rp1.q_in) annotation (Line(
            points={{-54,-78},{-54,-60},{-38,-60}},
            color={0,0,0},
            thickness=1));
        connect(Rc1.q_out, Rp1.q_in) annotation (Line(
            points={{-68,-60},{-38,-60}},
            color={0,0,0},
            thickness=1));
        connect(Rp.q_out, CL.q_in) annotation (Line(
            points={{20,36},{42,36}},
            color={0,0,0},
            thickness=1));
        connect(serialElasticConnections.Inner_b, CL1.q_in) annotation (Line(
            points={{-7.4,-57.4},{-7.4,-43.7},{-8,-43.7},{-8,-28}},
            color={0,0,0},
            thickness=1));
        connect(serialElasticConnections.pressure, CL1.externalPressure)
          annotation (Line(points={{-1.4,-59},{12,-59},{12,-12},{0,-12},{0,-20}},
              color={0,0,127}));
        connect(serialElasticConnections.Outer, Cw1.q_in) annotation (Line(
            points={{-7.2,-63.2},{-7.2,-73.6},{-6,-73.6},{-6,-90}},
            color={0,0,0},
            thickness=1));
        connect(Cw.InnerVolume, CL.volume) annotation (Line(points={{44,3.4},{
                30,3.4},{30,16},{48,16},{48,26}}, color={0,0,127}));
        connect(Cw.pressure, CL.externalPressure) annotation (Line(points={{
                49.5,-7.9},{49.5,-18},{70,-18},{70,52},{50,52},{50,44}}, color=
                {0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=4,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end TestSimpleRespirationMechnaics;

      model TestSimpleRespirationConnectors
        Physiolibrary.Types.Constants.PressureConst pressure(k(displayUnit=
                "cmH2O") = 245.16625)
          annotation (Placement(transformation(extent={{-84,86},{-76,94}})));
        Modelica.Blocks.Sources.Sine sine(f(displayUnit="1/min") = 0.25)
          annotation (Placement(transformation(extent={{-106,50},{-86,70}})));
        Modelica.Blocks.Math.Product product1
          annotation (Placement(transformation(extent={{-46,62},{-26,82}})));
        PhysiolibraryExpantion.Obsolent.SerialElasticConnections
          serialElasticConnections
          annotation (Placement(transformation(extent={{-18,-70},{2,-50}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume1(
            usePressureInput=true)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-88,-38})));
        Physiolibrary.Hydraulic.Components.Resistor Rp1(Resistance(displayUnit=
                "(cmH2O.s)/l") = 49033.25)
          annotation (Placement(transformation(extent={{-38,-70},{-18,-50}})));
        Physiolibrary.Hydraulic.Components.Resistor Rc1(Resistance(displayUnit=
                "(cmH2O.s)/l") = 98066.5)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-78,-60})));
        PhysiolibraryExpantion.ElasticVessel Cs1(Compliance(displayUnit=
                "l/cmH2O") = 5.0985810648896e-08)
          annotation (Placement(transformation(extent={{-64,-86},{-44,-66}})));
        PhysiolibraryExpantion.ElasticVessel CL1(
          volume_start=0,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{-18,-38},{2,-18}})));
        PhysiolibraryExpantion.ElasticVessel Cw1(
          volume_start=0,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=false,
          hardElastance=true)
          annotation (Placement(transformation(extent={{-16,-94},{4,-74}})));
        PhysiolibraryExpantion.Obsolent.SerialElasticConnections
          serialElasticConnections1
          annotation (Placement(transformation(extent={{62,8},{82,28}})));
        Physiolibrary.Hydraulic.Components.Resistor Rp2(Resistance(displayUnit=
                "(cmH2O.s)/l") = 49033.25)
          annotation (Placement(transformation(extent={{42,8},{62,28}})));
        Physiolibrary.Hydraulic.Components.Resistor Rc2(Resistance(displayUnit=
                "(cmH2O.s)/l") = 98066.5)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=0,
              origin={2,18})));
        PhysiolibraryExpantion.ElasticVessel Cs2(Compliance(displayUnit=
                "l/cmH2O") = 5.0985810648896e-08)
          annotation (Placement(transformation(extent={{16,-8},{36,12}})));
        PhysiolibraryExpantion.ElasticVessel CL2(
          volume_start=0.002,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{62,40},{82,60}})));
        PhysiolibraryExpantion.ElasticVessel Cw2(
          volume_start=0.002,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=false,
          hardElastance=true)
          annotation (Placement(transformation(extent={{64,-16},{84,4}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume2(
            usePressureInput=false, P=333.3059685375)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-36,18})));
      equation
        connect(product1.u1, pressure.y) annotation (Line(points={{-48,78},{-68,
                78},{-68,90},{-75,90}}, color={0,0,127}));
        connect(sine.y, product1.u2) annotation (Line(points={{-85,60},{-56,60},
                {-56,66},{-48,66}}, color={0,0,127}));
        connect(Rc1.q_in, unlimitedVolume1.y) annotation (Line(
            points={{-88,-60},{-88,-48}},
            color={0,0,0},
            thickness=1));
        connect(Rp1.q_out, serialElasticConnections.Inner_a) annotation (Line(
            points={{-18,-60},{-8,-60},{-8,-60.2},{-7.4,-60.2}},
            color={0,0,0},
            thickness=1));
        connect(Cs1.q_in, Rp1.q_in) annotation (Line(
            points={{-54,-76},{-54,-60},{-38,-60}},
            color={0,0,0},
            thickness=1));
        connect(Rc1.q_out, Rp1.q_in) annotation (Line(
            points={{-68,-60},{-38,-60}},
            color={0,0,0},
            thickness=1));
        connect(serialElasticConnections.Inner_b, CL1.q_in) annotation (Line(
            points={{-7.4,-57.4},{-7.4,-43.7},{-8,-43.7},{-8,-28}},
            color={0,0,0},
            thickness=1));
        connect(serialElasticConnections.pressure, CL1.externalPressure)
          annotation (Line(points={{-1.4,-59},{12,-59},{12,-12},{0,-12},{0,-20}},
              color={0,0,127}));
        connect(serialElasticConnections.Outer, Cw1.q_in) annotation (Line(
            points={{-7.2,-63.2},{-7.2,-73.6},{-6,-73.6},{-6,-84}},
            color={0,0,0},
            thickness=1));
        connect(unlimitedVolume1.pressure, product1.y) annotation (Line(points=
                {{-88,-28},{-88,38},{-10,38},{-10,72},{-25,72}}, color={0,0,127}));
        connect(Rp2.q_out, serialElasticConnections1.Inner_a) annotation (Line(
            points={{62,18},{72,18},{72,17.8},{72.6,17.8}},
            color={0,0,0},
            thickness=1));
        connect(Cs2.q_in, Rp2.q_in) annotation (Line(
            points={{26,2},{26,18},{42,18}},
            color={0,0,0},
            thickness=1));
        connect(Rc2.q_out, Rp2.q_in) annotation (Line(
            points={{12,18},{42,18}},
            color={0,0,0},
            thickness=1));
        connect(serialElasticConnections1.Inner_b, CL2.q_in) annotation (Line(
            points={{72.6,20.6},{72.6,34.3},{72,34.3},{72,50}},
            color={0,0,0},
            thickness=1));
        connect(serialElasticConnections1.pressure, CL2.externalPressure)
          annotation (Line(points={{78.6,19},{92,19},{92,66},{80,66},{80,58}},
              color={0,0,127}));
        connect(serialElasticConnections1.Outer, Cw2.q_in) annotation (Line(
            points={{72.8,14.8},{72.8,4.4},{74,4.4},{74,-6}},
            color={0,0,0},
            thickness=1));
        connect(unlimitedVolume2.y, Rc2.q_in) annotation (Line(
            points={{-26,18},{-8,18}},
            color={0,0,0},
            thickness=1));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=4,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end TestSimpleRespirationConnectors;

      model TestSimpleRespirationDoubleElastance
        Physiolibrary.Hydraulic.Components.Resistor Rp(Resistance(displayUnit=
                "(cmH2O.s)/l") = 49033.25)
          annotation (Placement(transformation(extent={{0,26},{20,46}})));
        Physiolibrary.Hydraulic.Components.Resistor Rc(Resistance(displayUnit=
                "(cmH2O.s)/l") = 98066.5)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-34,36})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume(
            usePressureInput=true)
          annotation (Placement(transformation(extent={{-76,28},{-56,48}})));
        Physiolibrary.Types.Constants.PressureConst pressure(k(displayUnit=
                "cmH2O") = 245.16625)
          annotation (Placement(transformation(extent={{-84,86},{-76,94}})));
        Modelica.Blocks.Sources.Sine sine(f(displayUnit="1/min") = 0.25)
          annotation (Placement(transformation(extent={{-104,48},{-84,68}})));
        Modelica.Blocks.Math.Product product1
          annotation (Placement(transformation(extent={{-46,62},{-26,82}})));
        PhysiolibraryExpantion.ElasticVessel Cs(Compliance(displayUnit=
                "l/cmH2O") = 5.0985810648896e-08)
          annotation (Placement(transformation(extent={{-26,-2},{-6,18}})));
        PhysiolibraryExpantion.ElasticVessel CL(
          volume_start=0.002,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{32,26},{52,46}})));
        PhysiolibraryExpantion.Obsolent.ElasticVesselWithInnerVolume
          elasticVesselWithInnerVolume(useInnerInput=true)
          annotation (Placement(transformation(extent={{34,-12},{54,8}})));
      equation
        connect(product1.u1, pressure.y) annotation (Line(points={{-48,78},{-68,
                78},{-68,90},{-75,90}}, color={0,0,127}));
        connect(sine.y, product1.u2) annotation (Line(points={{-83,58},{-56,58},
                {-56,66},{-48,66}}, color={0,0,127}));
        connect(product1.y, unlimitedVolume.pressure) annotation (Line(points={
                {-25,72},{-22,72},{-22,54},{-84,54},{-84,38},{-76,38}}, color={
                0,0,127}));
        connect(Rc.q_in, unlimitedVolume.y) annotation (Line(
            points={{-44,36},{-50,36},{-50,38},{-56,38}},
            color={0,0,0},
            thickness=1));
        connect(Cs.q_in, Rp.q_in) annotation (Line(
            points={{-16,8},{-16,36},{0,36}},
            color={0,0,0},
            thickness=1));
        connect(Rc.q_out, Rp.q_in) annotation (Line(
            points={{-24,36},{0,36}},
            color={0,0,0},
            thickness=1));
        connect(Rp.q_out, CL.q_in) annotation (Line(
            points={{20,36},{42,36}},
            color={0,0,0},
            thickness=1));
        connect(elasticVesselWithInnerVolume.pressure, CL.externalPressure)
          annotation (Line(points={{45.5,-11.9},{45.5,-30},{76,-30},{76,56},{50,
                56},{50,44}}, color={0,0,127}));
        connect(elasticVesselWithInnerVolume.InnerVolume, CL.volume)
          annotation (Line(points={{40,-0.6},{20,-0.6},{20,14},{48,14},{48,26}},
              color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=4,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end TestSimpleRespirationDoubleElastance;

      model testHard
        Physiolibrary.Hydraulic.Sources.UnlimitedPump unlimitedPump(
            useSolutionFlowInput=true)
          annotation (Placement(transformation(extent={{-70,-38},{-50,-18}})));
        PhysiolibraryExpantion.ElasticVessel elasticVessel1(
          volume_start=0.0001,
          ZeroPressureVolume=2e-05,
          Compliance(displayUnit="l/mmHg") = 7.5006157584566e-10,
          hardElastance=false)
          annotation (Placement(transformation(extent={{-22,-38},{-2,-18}})));
        Physiolibrary.Types.Constants.VolumeFlowRateConst volumeFlowRate(k=-1.6666666666667e-05)
          annotation (Placement(transformation(extent={{-92,-10},{-84,-2}})));
      equation
        connect(volumeFlowRate.y, unlimitedPump.solutionFlow) annotation (Line(
              points={{-83,-6},{-60,-6},{-60,-21}}, color={0,0,127}));
        connect(unlimitedPump.q_out, elasticVessel1.q_in) annotation (Line(
            points={{-50,-28},{-12,-28}},
            color={0,0,0},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end testHard;

      model TestSimpleRespirationMechnaicsExtens
        Physiolibrary.Hydraulic.Components.Resistor Rp(Resistance(displayUnit=
                "(cmH2O.s)/l") = 49033.25)
          annotation (Placement(transformation(extent={{0,16},{20,36}})));
        Physiolibrary.Hydraulic.Components.Resistor Rc(Resistance(displayUnit=
                "(cmH2O.s)/l") = 98066.5)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-34,26})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume(
            usePressureInput=true)
          annotation (Placement(transformation(extent={{-78,18},{-58,38}})));
        Physiolibrary.Types.Constants.PressureConst pressure(k(displayUnit=
                "cmH2O") = 245.16625)
          annotation (Placement(transformation(extent={{-76,82},{-68,90}})));
        Modelica.Blocks.Sources.Sine sine(f(displayUnit="1/min") = 0.25)
          annotation (Placement(transformation(extent={{-98,46},{-78,66}})));
        Modelica.Blocks.Math.Product product1
          annotation (Placement(transformation(extent={{-46,56},{-26,76}})));
        PhysiolibraryExpantion.Obsolent.SerialElasticConnections
          serialElasticConnections
          annotation (Placement(transformation(extent={{-10,-74},{10,-54}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume1(
            usePressureInput=true)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-88,-48})));
        Physiolibrary.Hydraulic.Components.Resistor Rp1(Resistance(displayUnit=
                "(cmH2O.s)/l") = 49033.25)
          annotation (Placement(transformation(extent={{-30,-74},{-10,-54}})));
        Physiolibrary.Hydraulic.Components.Resistor Rc1(Resistance(displayUnit=
                "(cmH2O.s)/l") = 98066.5)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-70,-64})));
        PhysiolibraryExpantion.ElasticVessel Cs(Compliance(displayUnit=
                "l/cmH2O") = 5.0985810648896e-08)
          annotation (Placement(transformation(extent={{-26,-14},{-6,6}})));
        PhysiolibraryExpantion.ElasticVessel Cs1(Compliance(displayUnit=
                "l/cmH2O") = 5.0985810648896e-08)
          annotation (Placement(transformation(extent={{-56,-92},{-36,-72}})));
        PhysiolibraryExpantion.ElasticVessel CL(
          volume_start=0.0023,
          ZeroPressureVolume=0.0023,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{32,16},{52,36}})));
        PhysiolibraryExpantion.ElasticVessel CL1(
          volume_start=0.0023,
          ZeroPressureVolume=0.0023,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{-18,-48},{2,-28}})));
        PhysiolibraryExpantion.ElasticVessel Cw1(
          volume_start=0.0023,
          ZeroPressureVolume=0.0023,
          Compliance(displayUnit="l/cmH2O") = 2.4881075596661e-06,
          useExternalPressureInput=false,
          hardElastance=true)
          annotation (Placement(transformation(extent={{-8,-104},{12,-84}})));
        PhysiolibraryExpantion.Obsolent.ElasticVesselWithInnerVolume Cw(
          ZeroPressureVolume=0.0023,
          Compliance(displayUnit="l/cmH2O") = 2.4881075596661e-06,
          useInnerInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{38,-18},{58,2}})));
        Physiolibrary.Hydraulic.Components.Resistor Rc2(Resistance(displayUnit=
                "(cmH2O.s)/l") = 98066.5)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=0,
              origin={8,78})));
        Physiolibrary.Hydraulic.Components.Resistor Rp2(Resistance(displayUnit=
                "(cmH2O.s)/l") = 49033.25)
          annotation (Placement(transformation(extent={{26,68},{46,88}})));
        PhysiolibraryExpantion.ElasticVessel CL2(
          volume_start=0.0023,
          ZeroPressureVolume=0.0023,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{54,70},{74,90}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume2(
            usePressureInput=true)
          annotation (Placement(transformation(extent={{-28,80},{-8,100}})));
        PhysiolibraryExpantion.OuterElasticVessel outerElasticVessel(
          outer_volume_start=0,
          useV0Input=false,
          ZeroPressureVolume=0.0023,
          useComplianceInput=false,
          Compliance(displayUnit="l/cmH2O") = 2.4881075596661e-06,
          useExternalPressureInput=false,                   hardElastance=true)
          annotation (Placement(transformation(extent={{58,46},{78,66}})));
        Physiolibrary.Hydraulic.Sensors.PressureMeasure pressureMeasure
          annotation (Placement(transformation(extent={{80,74},{100,94}})));
        Modelica.Blocks.Math.Feedback feedback
          annotation (Placement(transformation(extent={{102,70},{122,90}})));
      equation
        connect(product1.u1,pressure. y) annotation (Line(points={{-48,72},{-60,
                72},{-60,86},{-67,86}}, color={0,0,127}));
        connect(sine.y,product1. u2) annotation (Line(points={{-77,56},{-48,56},
                {-48,60}},          color={0,0,127}));
        connect(product1.y,unlimitedVolume. pressure) annotation (Line(points={{-25,66},
                {-22,66},{-22,44},{-84,44},{-84,28},{-78,28}},          color={
                0,0,127}));
        connect(Rc.q_in,unlimitedVolume. y) annotation (Line(
            points={{-44,26},{-50,26},{-50,28},{-58,28}},
            color={0,0,0},
            thickness=1));
        connect(unlimitedVolume1.pressure,unlimitedVolume. pressure)
          annotation (Line(points={{-88,-38},{-86,-38},{-86,28},{-78,28}},
              color={0,0,127}));
        connect(Rc1.q_in,unlimitedVolume1. y) annotation (Line(
            points={{-80,-64},{-80,-58},{-88,-58}},
            color={0,0,0},
            thickness=1));
        connect(Rp1.q_out,serialElasticConnections. Inner_a) annotation (Line(
            points={{-10,-64},{0,-64},{0,-64.2},{0.6,-64.2}},
            color={0,0,0},
            thickness=1));
        connect(Cs.q_in,Rp. q_in) annotation (Line(
            points={{-16,-4},{-16,26},{0,26}},
            color={0,0,0},
            thickness=1));
        connect(Rc.q_out,Rp. q_in) annotation (Line(
            points={{-24,26},{0,26}},
            color={0,0,0},
            thickness=1));
        connect(Cs1.q_in,Rp1. q_in) annotation (Line(
            points={{-46,-82},{-46,-64},{-30,-64}},
            color={0,0,0},
            thickness=1));
        connect(Rc1.q_out,Rp1. q_in) annotation (Line(
            points={{-60,-64},{-30,-64}},
            color={0,0,0},
            thickness=1));
        connect(Rp.q_out,CL. q_in) annotation (Line(
            points={{20,26},{42,26}},
            color={0,0,0},
            thickness=1));
        connect(serialElasticConnections.Inner_b,CL1. q_in) annotation (Line(
            points={{0.6,-61.4},{0.6,-47.7},{-8,-47.7},{-8,-38}},
            color={0,0,0},
            thickness=1));
        connect(serialElasticConnections.pressure,CL1. externalPressure)
          annotation (Line(points={{6.6,-63},{12,-63},{12,-22},{0,-22},{0,-30}},
              color={0,0,127}));
        connect(serialElasticConnections.Outer,Cw1. q_in) annotation (Line(
            points={{0.8,-67.2},{0.8,-77.6},{2,-77.6},{2,-94}},
            color={0,0,0},
            thickness=1));
        connect(Cw.InnerVolume, CL.volume) annotation (Line(points={{44,-6.6},{
                30,-6.6},{30,6},{48,6},{48,16}}, color={0,0,127}));
        connect(Cw.pressure, CL.externalPressure) annotation (Line(points={{
                49.5,-17.9},{49.5,-28},{70,-28},{70,42},{50,42},{50,34}}, color=
               {0,0,127}));
        connect(Rc2.q_out, Rp2.q_in) annotation (Line(
            points={{18,78},{26,78}},
            color={0,0,0},
            thickness=1));
        connect(unlimitedVolume2.y, Rc2.q_in) annotation (Line(
            points={{-8,90},{-4,90},{-4,78},{-2,78}},
            color={0,0,0},
            thickness=1));
        connect(unlimitedVolume2.pressure, unlimitedVolume.pressure)
          annotation (Line(points={{-28,90},{-22,66},{-22,44},{-84,44},{-84,28},
                {-78,28}}, color={0,0,127}));
        connect(Rp2.q_out, CL2.q_in) annotation (Line(
            points={{46,78},{54,78},{54,80},{64,80}},
            color={0,0,0},
            thickness=1));
        connect(outerElasticVessel.InnerVolume, CL2.volume) annotation (Line(
              points={{74.2,54},{70,54},{70,70}}, color={0,0,127}));
        connect(outerElasticVessel.pressure, CL2.externalPressure) annotation (
            Line(points={{80.1,54.9},{80.1,92},{72,92},{72,88}}, color={0,0,127}));
        connect(CL2.q_in, pressureMeasure.q_in) annotation (Line(
            points={{64,80},{50,80},{50,98},{86,98},{86,78}},
            color={0,0,0},
            thickness=1));
        connect(pressureMeasure.pressure, feedback.u1)
          annotation (Line(points={{96,80},{104,80}}, color={0,0,127}));
        connect(feedback.u2, CL2.externalPressure) annotation (Line(points={{
                112,72},{112,58},{80.1,58},{80.1,92},{72,92},{72,88}}, color={0,
                0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=4,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end TestSimpleRespirationMechnaicsExtens;

      model TestSimpleRespirationMechachanics
        PhysiolibraryExpantion.ElasticVessel CL(
          volume_start=0.0023,
          ZeroPressureVolume=0.0013,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{46,20},{66,40}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume2(
            usePressureInput=false)
          annotation (Placement(transformation(extent={{-48,20},{-28,40}})));
        PhysiolibraryExpantion.OuterElasticVessel Cw(
          outer_volume_start=0,
          useV0Input=false,
          ZeroPressureVolume=0.00352,
          useComplianceInput=false,
          Compliance(displayUnit="l/cmH2O") = 2.4881075596661e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{50,-4},{70,16}})));
        Physiolibrary.Hydraulic.Components.Resistor resistor(Resistance(displayUnit="(cmH2O.s)/l")=
               362846.05)
          annotation (Placement(transformation(extent={{16,20},{36,40}})));
        MeursHemodynamics.Components.BreathInterval breathInterval1
          annotation (Placement(transformation(extent={{-40,-28},{-20,-8}})));
        Modelica.Blocks.Sources.Constant BreathRate(k=12)
          annotation (Placement(transformation(extent={{-86,-28},{-66,-8}})));
        Physiolibrary.Types.Constants.PressureConst RespiratoryMuscleAmplitude(k(
              displayUnit="cmH2O") = -441.29925)
          annotation (Placement(transformation(extent={{-30,4},{-22,12}})));
        Modelica.Blocks.Math.Product product1
          annotation (Placement(transformation(extent={{0,-20},{20,0}})));

           Physiolibrary.Types.Pressure AlveolarPressure;

        Physiolibrary.Blocks.Math.Integrator int
          annotation (Placement(transformation(extent={{24,50},{44,70}})));
        Physiolibrary.Hydraulic.Sensors.FlowMeasure flowMeasure
          annotation (Placement(transformation(extent={{-16,20},{4,40}})));
      equation
        AlveolarPressure=Cw.pressure+CL.q_in.pressure;
        connect(Cw.InnerVolume, CL.volume)
          annotation (Line(points={{66.2,4},{62,4},{62,20}}, color={0,0,127}));
        connect(Cw.pressure, CL.externalPressure) annotation (Line(points={{72.1,4.9},
                {72.1,42},{64,42},{64,38}}, color={0,0,127}));
        connect(resistor.q_out, CL.q_in) annotation (Line(
            points={{36,30},{56,30}},
            color={0,0,0},
            thickness=1));
        connect(BreathRate.y, breathInterval1.RR)
          annotation (Line(points={{-65,-18},{-40.4,-18}}, color={0,0,127}));
        connect(breathInterval1.Pm, product1.u2) annotation (Line(points={{-19,-18},{-8,
                -18},{-8,-16},{-2,-16}}, color={0,0,127}));
        connect(RespiratoryMuscleAmplitude.y, product1.u1) annotation (Line(points={{-21,
                8},{-8,8},{-8,-4},{-2,-4}}, color={0,0,127}));
        connect(product1.y, Cw.externalPressure) annotation (Line(points={{21,-10},{36,
                -10},{36,14},{58,14},{58,10},{58.4,10}}, color={0,0,127}));
        connect(flowMeasure.q_out, resistor.q_in) annotation (Line(
            points={{4,30},{16,30}},
            color={0,0,0},
            thickness=1));
        connect(flowMeasure.q_in, unlimitedVolume2.y) annotation (Line(
            points={{-16,30},{-28,30}},
            color={0,0,0},
            thickness=1));
        connect(flowMeasure.volumeFlow, int.u)
          annotation (Line(points={{-6,42},{-6,60},{22,60}}, color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=4,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end TestSimpleRespirationMechachanics;

      model TestVentilationPhysiomodel
        PhysiolibraryExpantion.AlveolarVentilation_ATPD
          alveolarVentilation_ATPD
          annotation (Placement(transformation(extent={{-26,-22},{22,16}})));
        Physiolibrary.Types.Constants.TemperatureConst AmbientTemperature(k=
              296.15)
          annotation (Placement(transformation(extent={{-72,32},{-64,40}})));
        Physiolibrary.Types.Constants.PressureConst AmbientPressure(k=
              101325.0144354)
          annotation (Placement(transformation(extent={{-88,8},{-80,16}})));
        Physiolibrary.Types.Constants.FractionConst fraction(k=0.8)
          annotation (Placement(transformation(extent={{-76,-28},{-68,-20}})));
        Physiolibrary.Types.Constants.VolumeConst tidalVolume(k=0.0005)
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={48,46})));
        Physiolibrary.Types.Constants.VolumeConst deadSpace(k=0.00015)
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={54,10})));
        Physiolibrary.Types.Constants.FrequencyConst respirationRate(k=0.2)
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={62,-4})));
        Physiolibrary.Types.Constants.TemperatureConst BodyTemperature(k=310.15)
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={64,-22})));
      equation
        connect(AmbientTemperature.y, alveolarVentilation_ATPD.AmbientTemperature)
          annotation (Line(points={{-63,36},{-8.85714,36},{-8.85714,16}}, color=
               {0,0,127}));
        connect(AmbientPressure.y, alveolarVentilation_ATPD.EnvironmentPressure)
          annotation (Line(points={{-79,12},{-32,12},{-32,5.63636},{-8.85714,
                5.63636}}, color={0,0,127}));
        connect(fraction.y, alveolarVentilation_ATPD.EnvironmentRelativeHumidity)
          annotation (Line(points={{-67,-24},{-32,-24},{-32,-4.72727},{-8.85714,
                -4.72727}}, color={0,0,127}));
        connect(alveolarVentilation_ATPD.TidalVolume, tidalVolume.y)
          annotation (Line(points={{18.5714,16},{32,16},{32,46},{43,46}}, color=
               {0,0,127}));
        connect(deadSpace.y, alveolarVentilation_ATPD.DeadSpace) annotation (
            Line(points={{49,10},{49,9.09091},{18.5714,9.09091}}, color={0,0,
                127}));
        connect(respirationRate.y, alveolarVentilation_ATPD.RespRate)
          annotation (Line(points={{57,-4},{38,-4},{38,2.18182},{18.5714,
                2.18182}}, color={0,0,127}));
        connect(BodyTemperature.y, alveolarVentilation_ATPD.core_T) annotation (
           Line(points={{59,-22},{30,-22},{30,-4.72727},{18.5714,-4.72727}},
              color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end TestVentilationPhysiomodel;

      model TestSimpleDeathVolume
        PhysiolibraryExpantion.ElasticVessel CL(
          volume_start=0.0023,
          ZeroPressureVolume=0.0013,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{40,48},{60,68}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume2(
            usePressureInput=false)
          annotation (Placement(transformation(extent={{-54,48},{-34,68}})));
        PhysiolibraryExpantion.OuterElasticVessel Cw(
          outer_volume_start=0,
          useV0Input=false,
          ZeroPressureVolume=0.00352,
          useComplianceInput=false,
          Compliance(displayUnit="l/cmH2O") = 2.4881075596661e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{44,24},{64,44}})));
        Physiolibrary.Hydraulic.Components.Resistor resistor(Resistance(displayUnit="(cmH2O.s)/l")=
               362846.05)
          annotation (Placement(transformation(extent={{10,48},{30,68}})));
        MeursHemodynamics.Components.BreathInterval breathInterval1
          annotation (Placement(transformation(extent={{-46,0},{-26,20}})));
        Modelica.Blocks.Sources.Constant BreathRate(k=12)
          annotation (Placement(transformation(extent={{-82,0},{-62,20}})));
        Physiolibrary.Types.Constants.PressureConst RespiratoryMuscleAmplitude(k(
              displayUnit="cmH2O") = 441.29925)
          annotation (Placement(transformation(extent={{-54,32},{-46,40}})));
        Modelica.Blocks.Math.Product product1
          annotation (Placement(transformation(extent={{-6,8},{14,28}})));

           Physiolibrary.Types.Pressure AlveolarPressure;

        Physiolibrary.Blocks.Math.Integrator int
          annotation (Placement(transformation(extent={{8,76},{28,96}})));
        Physiolibrary.Hydraulic.Sensors.FlowMeasure flowMeasure
          annotation (Placement(transformation(extent={{-22,48},{-2,68}})));
        Physiolibrary.Types.Constants.TemperatureConst BodyTemperature(k=310.15)
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=0,
              origin={-84,-84})));
        Physiolibrary.Types.Constants.PressureConst Pressure(k=101325.0144354)
          annotation (Placement(transformation(extent={{-88,-42},{-80,-34}})));
        Modelica.Blocks.Math.Feedback minus_pH2O annotation (Placement(
              transformation(
              extent={{-7,-7},{7,7}},
              rotation=0,
              origin={-49,-39})));
        Physiolibrary.Types.Constants.FractionConst FiO2(k=0.21)
          annotation (Placement(transformation(extent={{-90,-24},{-82,-16}})));
        Modelica.Blocks.Math.Product PiO2
          annotation (Placement(transformation(extent={{-28,-36},{-18,-26}})));
        Physiolibrary.Types.Constants.PressureConst PAO2(k=13332.2387415)
          annotation (Placement(transformation(extent={{26,-22},{34,-14}})));
        Components.UnlimitedGasSource unlimitedGasSource
          annotation (Placement(transformation(extent={{0,-58},{20,-38}})));
        Components.UnlimitedGasSource unlimitedGasSource1
          annotation (Placement(transformation(extent={{54,-36},{74,-16}})));
        Components.VaporPressure vaporPressure
          annotation (Placement(transformation(extent={{-64,-78},{-44,-58}})));
        Physiolibrary.Chemical.Sensors.ConcentrationMeasure
          concentrationMeasure
          annotation (Placement(transformation(extent={{24,-78},{44,-58}})));
        Physiolibrary.Chemical.Sensors.ConcentrationMeasure
          concentrationMeasure1
          annotation (Placement(transformation(extent={{72,-76},{92,-56}})));
        Components.PartialPressureMeasure partialPressureMeasure
          annotation (Placement(transformation(extent={{22,-112},{42,-92}})));
        Components.PartialPressureMeasure partialPressureMeasure1
          annotation (Placement(transformation(extent={{74,-96},{94,-76}})));
      equation
        AlveolarPressure=Cw.pressure+CL.q_in.pressure;
        connect(Cw.InnerVolume, CL.volume)
          annotation (Line(points={{60.2,32},{56,32},{56,48}},
                                                             color={0,0,127}));
        connect(Cw.pressure, CL.externalPressure) annotation (Line(points={{66.1,
                32.9},{66.1,70},{58,70},{58,66}},
                                            color={0,0,127}));
        connect(resistor.q_out, CL.q_in) annotation (Line(
            points={{30,58},{50,58}},
            color={0,0,0},
            thickness=1));
        connect(BreathRate.y, breathInterval1.RR)
          annotation (Line(points={{-61,10},{-46.4,10}},   color={0,0,127}));
        connect(breathInterval1.Pm, product1.u2) annotation (Line(points={{-25,10},
                {-14,10},{-14,12},{-8,12}},
                                         color={0,0,127}));
        connect(RespiratoryMuscleAmplitude.y, product1.u1) annotation (Line(points={{-45,36},
                {-14,36},{-14,24},{-8,24}}, color={0,0,127}));
        connect(product1.y, Cw.externalPressure) annotation (Line(points={{15,18},
                {30,18},{30,42},{52,42},{52,38},{52.4,38}},
                                                         color={0,0,127}));
        connect(flowMeasure.q_out, resistor.q_in) annotation (Line(
            points={{-2,58},{10,58}},
            color={0,0,0},
            thickness=1));
        connect(flowMeasure.q_in, unlimitedVolume2.y) annotation (Line(
            points={{-22,58},{-34,58}},
            color={0,0,0},
            thickness=1));
        connect(flowMeasure.volumeFlow, int.u)
          annotation (Line(points={{-12,70},{-12,86},{6,86}},color={0,0,127}));
        connect(Pressure.y, minus_pH2O.u1)
          annotation (Line(points={{-79,-38},{-54.6,-39}}, color={0,0,127}));
        connect(minus_pH2O.y, PiO2.u2) annotation (Line(points={{-42.7,-39},{
                -29,-39},{-29,-34}}, color={0,0,127}));
        connect(PiO2.u1, FiO2.y) annotation (Line(points={{-29,-28},{-64,-28},{
                -64,-20},{-81,-20}}, color={0,0,127}));
        connect(unlimitedGasSource.partialPressure, PiO2.y) annotation (Line(
              points={{0.4,-42.8},{-12,-42.8},{-12,-31},{-17.5,-31}},  color={0,
                0,127}));
        connect(BodyTemperature.y, unlimitedGasSource.temperature) annotation (
            Line(points={{-79,-84},{-6,-84},{-6,-52},{0.2,-52}},  color={0,0,
                127}));
        connect(PAO2.y, unlimitedGasSource1.partialPressure) annotation (Line(
              points={{35,-18},{48,-18},{48,-20.8},{54.4,-20.8}}, color={0,0,
                127}));
        connect(unlimitedGasSource1.temperature, unlimitedGasSource.temperature)
          annotation (Line(points={{54.2,-30},{52,-30},{52,-88},{48,-88},{48,
                -87.8},{-20,-87.8},{-20,-84},{-6,-84},{-6,-52},{0.2,-52}},
              color={0,0,127}));
        connect(vaporPressure.T, unlimitedGasSource.temperature) annotation (
            Line(points={{-62.6,-68},{-72,-68},{-72,-84},{-6,-84},{-6,-52},{0.2,
                -52}}, color={0,0,127}));
        connect(vaporPressure.VaporPressure_, minus_pH2O.u2) annotation (Line(
              points={{-42.6,-67.8},{-38,-67.8},{-38,-54},{-49,-54},{-49,-44.6}},
              color={0,0,127}));
        connect(concentrationMeasure.q_in, unlimitedGasSource.port_a)
          annotation (Line(
            points={{34,-68},{34,-48},{20,-48},{20,-48.8}},
            color={107,45,134},
            thickness=1));
        connect(concentrationMeasure1.q_in, unlimitedGasSource1.port_a)
          annotation (Line(
            points={{82,-66},{84,-66},{84,-26.8},{74,-26.8}},
            color={107,45,134},
            thickness=1));
        connect(partialPressureMeasure.temperature, unlimitedGasSource.temperature)
          annotation (Line(points={{23.6,-97.4},{-20,-97.4},{-20,-84},{-6,-84},
                {-6,-52},{0.2,-52}}, color={0,0,127}));
        connect(partialPressureMeasure1.temperature, unlimitedGasSource.temperature)
          annotation (Line(points={{75.6,-81.4},{52,-81.4},{52,-88},{48,-88},{
                48,-87.8},{-20,-87.8},{-20,-84},{-6,-84},{-6,-52},{0.2,-52}},
              color={0,0,127}));
        connect(partialPressureMeasure.concentration, concentrationMeasure.concentration)
          annotation (Line(points={{23.2,-105.6},{10,-105.6},{10,-80},{34,-80},
                {34,-76}}, color={0,0,127}));
        connect(partialPressureMeasure1.concentration, concentrationMeasure1.concentration)
          annotation (Line(points={{75.2,-89.6},{60,-89.6},{60,-74},{82,-74}},
              color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=4,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end TestSimpleDeathVolume;

      model TwoConcentrationCompartments
        extends Modelica.Icons.Example;

        Physiolibrary.Chemical.Components.Substance substance(
            useNormalizedVolume=false, solute_start=0.001)
          annotation (Placement(transformation(extent={{-100,-20},{-80,0}})));
        Physiolibrary.Chemical.Components.Substance substance1(solute_start=0,
            useNormalizedVolume=false)
          annotation (Placement(transformation(extent={{60,-20},{80,0}})));
        Physiolibrary.Chemical.Components.AdvectionStream advectionStream(
          useSolutionFlowInput=true,
          length=1,
          diffusion_coefficient=1)
          annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
        Physiolibrary.Hydraulic.Components.ElasticVessel elasticVessel(
            volume_start=0.001)
          annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
        Physiolibrary.Hydraulic.Components.ElasticVessel elasticVessel1(
            volume_start=0.0005)
          annotation (Placement(transformation(extent={{60,40},{80,60}})));
        Physiolibrary.Hydraulic.Components.Resistor resistor(Resistance=1.0)
          annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
        Physiolibrary.Hydraulic.Components.Inertia inertia(volumeFlow_start=0.0,
            I=10.0)
          annotation (Placement(transformation(extent={{-20,40},{0,60}})));
        Physiolibrary.Hydraulic.Sensors.FlowMeasure flowMeasure
          annotation (Placement(transformation(extent={{20,60},{40,40}})));
      equation
        connect(substance.q_out, advectionStream.q_in) annotation (Line(
            points={{-90,-10},{-20,-10}},
            color={107,45,134},
            thickness=1));
        connect(advectionStream.q_out, substance1.q_out) annotation (Line(
            points={{0,-10},{70,-10}},
            color={107,45,134},
            thickness=1));
        connect(elasticVessel.q_in, resistor.q_in) annotation (Line(
            points={{-90,50},{-76,50},{-76,50},{-60,50}},
            color={0,0,0},
            thickness=1));
        connect(resistor.q_out, inertia.q_in) annotation (Line(
            points={{-40,50},{-20,50}},
            color={0,0,0},
            thickness=1));
        connect(inertia.q_out, flowMeasure.q_in) annotation (Line(
            points={{0,50},{20,50}},
            color={0,0,0},
            thickness=1));
        connect(flowMeasure.q_out, elasticVessel1.q_in) annotation (Line(
            points={{40,50},{70,50}},
            color={0,0,0},
            thickness=1));
        connect(elasticVessel.volume, substance.solutionVolume) annotation (Line(
              points={{-84,40},{-90,40},{-90,-6},{-94,-6}}, color={0,0,127}));
        connect(elasticVessel1.volume, substance1.solutionVolume) annotation (
            Line(points={{76,40},{72,40},{72,-6},{66,-6}}, color={0,0,127}));
        connect(flowMeasure.volumeFlow, advectionStream.solutionFlow) annotation (
           Line(points={{30,38},{10,38},{10,-3},{-10,-3}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          Documentation(info="<html>
<p>This simple model demonstrates combination of hydraulics and chemical domain. It is model of two elastic vessels, with back and forth oscillating volume. In addition, each vessel does have some initial solute concentration, represented by the chemical domain. Both vessels are connected using resistive tube with large inertance, which introduces advection dead-space with diffusion capabilities.</p>
<p>The units are demonstrative and thus not to any scale of any real-world physical system.</p>
</html>"),   Documentation(revisions="<html>
<p><i>2018</i></p>
<p>Filip Jezek, Charles University, Prague, Czech Republic </p>
</html>"),experiment(StopTime=20));
      end TwoConcentrationCompartments;

      model test_valves
        Physiolibrary.Hydraulic.Components.Conductor conductor(Conductance=
              1.2501026264094e-10)
          annotation (Placement(transformation(extent={{-38,82},{-18,102}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume(P=0)
          annotation (Placement(transformation(extent={{30,68},{50,88}})));
        Physiolibrary.Hydraulic.Components.Conductor conductor1(Conductance=
              1.2501026264094e-10)
          annotation (Placement(transformation(extent={{-32,46},{-12,66}})));
        Physiolibrary.Hydraulic.Components.IdealValve idealValve
          annotation (Placement(transformation(extent={{-10,82},{10,102}})));
        Physiolibrary.Hydraulic.Components.IdealValve idealValve1(_Gon=
              1.2501026264094e+92, _Goff=0) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={4,56})));
        Physiolibrary.Hydraulic.Sources.UnlimitedPump unlimitedPump(
            SolutionFlow=1.6666666666667e-06)
          annotation (Placement(transformation(extent={{-84,64},{-64,84}})));
        Physiolibrary.Hydraulic.Components.Conductor conductor2(Conductance=
              1.2501026264094e-10)
          annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume1(P=0)
          annotation (Placement(transformation(extent={{30,4},{50,24}})));
        Physiolibrary.Hydraulic.Components.Conductor conductor3(Conductance=
              1.2501026264094e-10)
          annotation (Placement(transformation(extent={{-36,-16},{-16,4}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedPump unlimitedPump1(
            SolutionFlow=1.6666666666667e-06)
          annotation (Placement(transformation(extent={{-84,0},{-64,20}})));
        Components.UnlimitedGasSource unlimitedGasSource
          annotation (Placement(transformation(extent={{-44,-50},{-24,-30}})));
        Modelica.Blocks.Math.Product PiO2
          annotation (Placement(transformation(extent={{-62,-38},{-52,-28}})));
        Components.VaporPressure vaporPressure
          annotation (Placement(transformation(extent={{-88,-74},{-68,-54}})));
        Physiolibrary.Types.Constants.PressureConst Pressure(k=101325.0144354)
          annotation (Placement(transformation(extent={{-94,-46},{-86,-38}})));
        Physiolibrary.Types.Constants.FractionConst FiO2(k=0.21)
          annotation (Placement(transformation(extent={{-88,-30},{-80,-22}})));
        Physiolibrary.Types.Constants.TemperatureConst BodyTemperature(k=310.15)
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=0,
              origin={-88,-84})));
        Modelica.Blocks.Math.Feedback minus_pH2O annotation (Placement(
              transformation(
              extent={{-7,-7},{7,7}},
              rotation=0,
              origin={-73,-43})));
        Physiolibrary.Chemical.Components.Substance substance(
            useNormalizedVolume=false)
          annotation (Placement(transformation(extent={{26,-52},{46,-32}})));
        Physiolibrary.Chemical.Components.Stream Stream(SolutionFlow(
              displayUnit="l/s") = 0.00015)
          annotation (Placement(transformation(extent={{-10,-50},{10,-30}})));
        Physiolibrary.Chemical.Components.Stream Stream1(SolutionFlow=
              0.00016666666666667)
          annotation (Placement(transformation(extent={{56,-52},{76,-32}})));
        Physiolibrary.Chemical.Components.Substance substance1
          annotation (Placement(transformation(extent={{86,-52},{106,-32}})));
        Physiolibrary.Types.Constants.VolumeConst volume(k=0.00015)
          annotation (Placement(transformation(extent={{-8,-24},{0,-16}})));
      equation
        connect(conductor1.q_in, conductor.q_in) annotation (Line(
            points={{-32,56},{-44,56},{-44,92},{-38,92}},
            color={0,0,0},
            thickness=1));
        connect(conductor.q_out, idealValve.q_in) annotation (Line(
            points={{-18,92},{-10,92}},
            color={0,0,0},
            thickness=1));
        connect(conductor1.q_out, idealValve1.q_out) annotation (Line(
            points={{-12,56},{-6,56}},
            color={0,0,0},
            thickness=1));
        connect(unlimitedVolume.y, idealValve.q_out) annotation (Line(
            points={{50,78},{54,78},{54,92},{10,92}},
            color={0,0,0},
            thickness=1));
        connect(unlimitedVolume.y, idealValve1.q_in) annotation (Line(
            points={{50,78},{54,78},{54,92},{20,92},{20,56},{14,56}},
            color={0,0,0},
            thickness=1));
        connect(unlimitedPump.q_out, conductor.q_in) annotation (Line(
            points={{-64,74},{-44,74},{-44,92},{-38,92}},
            color={0,0,0},
            thickness=1));
        connect(conductor3.q_in, conductor2.q_in) annotation (Line(
            points={{-36,-6},{-44,-6},{-44,40},{-40,40}},
            color={0,0,0},
            thickness=1));
        connect(unlimitedPump1.q_out, conductor2.q_in) annotation (Line(
            points={{-64,10},{-44,10},{-44,40},{-40,40}},
            color={0,0,0},
            thickness=1));
        connect(conductor3.q_out, unlimitedVolume1.y) annotation (Line(
            points={{-16,-6},{56,-6},{56,14},{50,14}},
            color={0,0,0},
            thickness=1));
        connect(conductor2.q_out, unlimitedVolume1.y) annotation (Line(
            points={{-20,40},{54,40},{54,14},{50,14}},
            color={0,0,0},
            thickness=1));
        connect(PiO2.u1, FiO2.y) annotation (Line(points={{-63,-30},{-79,-30},{
                -79,-26}}, color={0,0,127}));
        connect(PiO2.y, unlimitedGasSource.partialPressure) annotation (Line(
              points={{-51.5,-33},{-48,-33},{-48,-34.8},{-43.6,-34.8}}, color={
                0,0,127}));
        connect(Pressure.y, minus_pH2O.u1)
          annotation (Line(points={{-85,-42},{-78.6,-43}}, color={0,0,127}));
        connect(minus_pH2O.y, PiO2.u2) annotation (Line(points={{-66.7,-43},{
                -63,-43},{-63,-36}}, color={0,0,127}));
        connect(vaporPressure.VaporPressure_, minus_pH2O.u2) annotation (Line(
              points={{-66.6,-63.8},{-66.6,-52},{-73,-52},{-73,-48.6}}, color={
                0,0,127}));
        connect(BodyTemperature.y, vaporPressure.T) annotation (Line(points={{
                -83,-84},{-80,-84},{-80,-74},{-94,-74},{-94,-64},{-86.6,-64}},
              color={0,0,127}));
        connect(unlimitedGasSource.temperature, vaporPressure.T) annotation (
            Line(points={{-43.8,-44},{-62,-44},{-62,-84},{-80,-84},{-80,-74},{
                -94,-74},{-94,-64},{-86.6,-64}}, color={0,0,127}));
        connect(unlimitedGasSource.port_a, Stream.q_in) annotation (Line(
            points={{-24,-40.8},{-18,-40.8},{-18,-40},{-10,-40}},
            color={107,45,134},
            thickness=1));
        connect(Stream.q_out, substance.q_out) annotation (Line(
            points={{10,-40},{24,-40},{24,-42},{36,-42}},
            color={107,45,134},
            thickness=1));
        connect(substance.q_out, Stream1.q_in) annotation (Line(
            points={{36,-42},{22,-42},{22,-28},{56,-28},{56,-42}},
            color={107,45,134},
            thickness=1));
        connect(Stream1.q_out, substance1.q_out) annotation (Line(
            points={{76,-42},{96,-42}},
            color={107,45,134},
            thickness=1));
        connect(volume.y, substance.solutionVolume) annotation (Line(points={{1,
                -20},{32,-20},{32,-38}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end test_valves;

      model test_respiration_valves
        Components.UnlimitedGasSource unlimitedGasSource
          annotation (Placement(transformation(extent={{-44,-50},{-24,-30}})));
        Modelica.Blocks.Math.Product PiO2
          annotation (Placement(transformation(extent={{-62,-38},{-52,-28}})));
        Components.VaporPressure vaporPressure
          annotation (Placement(transformation(extent={{-88,-74},{-68,-54}})));
        Physiolibrary.Types.Constants.PressureConst Pressure(k=101325.0144354)
          annotation (Placement(transformation(extent={{-94,-46},{-86,-38}})));
        Physiolibrary.Types.Constants.FractionConst FiO2(k=0.21)
          annotation (Placement(transformation(extent={{-88,-30},{-80,-22}})));
        Physiolibrary.Types.Constants.TemperatureConst BodyTemperature(k=310.15)
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=0,
              origin={-88,-84})));
        Modelica.Blocks.Math.Feedback minus_pH2O annotation (Placement(
              transformation(
              extent={{-7,-7},{7,7}},
              rotation=0,
              origin={-73,-43})));
        Physiolibrary.Chemical.Components.Substance substance(
            useNormalizedVolume=false)
          annotation (Placement(transformation(extent={{26,-52},{46,-32}})));
        Physiolibrary.Chemical.Components.Stream Stream(useSolutionFlowInput=
              true, SolutionFlow=1.6666666666667e-06)
          annotation (Placement(transformation(extent={{-4,-52},{16,-32}})));
        PhysiolibraryExpantion.ElasticVessel CL(
          volume_start=0.0023,
          ZeroPressureVolume=0.0013,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{38,50},{58,70}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume2(
            usePressureInput=false)
          annotation (Placement(transformation(extent={{-56,50},{-36,70}})));
        PhysiolibraryExpantion.OuterElasticVessel Cw(
          outer_volume_start=0,
          useV0Input=false,
          ZeroPressureVolume=0.00352,
          useComplianceInput=false,
          Compliance(displayUnit="l/cmH2O") = 2.4881075596661e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{42,26},{62,46}})));
        Physiolibrary.Hydraulic.Components.Resistor resistor(Resistance(
              displayUnit="(cmH2O.s)/l") = 362846.05)
          annotation (Placement(transformation(extent={{8,50},{28,70}})));
        MeursHemodynamics.Components.BreathInterval breathInterval1
          annotation (Placement(transformation(extent={{-48,2},{-28,22}})));
        Modelica.Blocks.Sources.Constant BreathRate(k=12)
          annotation (Placement(transformation(extent={{-84,2},{-64,22}})));
        Physiolibrary.Types.Constants.PressureConst RespiratoryMuscleAmplitude(k(
              displayUnit="cmH2O") = 441.29925)
          annotation (Placement(transformation(extent={{-56,34},{-48,42}})));
        Modelica.Blocks.Math.Product product1
          annotation (Placement(transformation(extent={{-8,10},{12,30}})));
        Physiolibrary.Blocks.Math.Integrator int
          annotation (Placement(transformation(extent={{6,78},{26,98}})));
        Physiolibrary.Hydraulic.Sensors.FlowMeasure flowMeasure
          annotation (Placement(transformation(extent={{-24,50},{-4,70}})));
        Physiolibrary.Types.Constants.VolumeConst volume(k=0.00015)
          annotation (Placement(transformation(extent={{18,-14},{26,-6}})));
        Physiolibrary.Chemical.Components.Stream Stream1(useSolutionFlowInput=
              true, SolutionFlow=1.6666666666667e-06)
          annotation (Placement(transformation(extent={{58,-52},{78,-32}})));
        Physiolibrary.Chemical.Components.Substance substance1(
            useNormalizedVolume=false, solute_start=0.001*(5.17*2300))
          annotation (Placement(transformation(extent={{86,-52},{106,-32}})));
        Components.UnlimitedGasSource unlimitedGasSource1
          annotation (Placement(transformation(extent={{56,-92},{76,-72}})));
        Physiolibrary.Types.Constants.PressureConst Pressure1(k=13332.2387415)
          annotation (Placement(transformation(extent={{24,-78},{32,-70}})));
      equation
        connect(PiO2.u1, FiO2.y) annotation (Line(points={{-63,-30},{-79,-30},{
                -79,-26}}, color={0,0,127}));
        connect(PiO2.y, unlimitedGasSource.partialPressure) annotation (Line(
              points={{-51.5,-33},{-48,-33},{-48,-32},{-44.4,-32}}, color={0,0,
                127}));
        connect(Pressure.y, minus_pH2O.u1)
          annotation (Line(points={{-85,-42},{-78.6,-43}}, color={0,0,127}));
        connect(minus_pH2O.y, PiO2.u2) annotation (Line(points={{-66.7,-43},{
                -63,-43},{-63,-36}}, color={0,0,127}));
        connect(vaporPressure.VaporPressure_, minus_pH2O.u2) annotation (Line(
              points={{-66.6,-63.8},{-66.6,-52},{-73,-52},{-73,-48.6}}, color={
                0,0,127}));
        connect(BodyTemperature.y, vaporPressure.T) annotation (Line(points={{
                -83,-84},{-80,-84},{-80,-74},{-94,-74},{-94,-64},{-86.6,-64}},
              color={0,0,127}));
        connect(unlimitedGasSource.temperature, vaporPressure.T) annotation (
            Line(points={{-43.8,-46},{-62,-46},{-62,-84},{-80,-84},{-80,-74},{
                -94,-74},{-94,-64},{-86.6,-64}}, color={0,0,127}));
        connect(unlimitedGasSource.port_a, Stream.q_in) annotation (Line(
            points={{-24,-40.6},{-18,-40.6},{-18,-42},{-4,-42}},
            color={107,45,134},
            thickness=1));
        connect(Stream.q_out, substance.q_out) annotation (Line(
            points={{16,-42},{36,-42}},
            color={107,45,134},
            thickness=1));
        connect(Cw.InnerVolume,CL. volume)
          annotation (Line(points={{58.2,34},{54,34},{54,50}},
                                                             color={0,0,127}));
        connect(Cw.pressure,CL. externalPressure) annotation (Line(points={{64.1,
                34.9},{64.1,72},{56,72},{56,68}},
                                            color={0,0,127}));
        connect(resistor.q_out,CL. q_in) annotation (Line(
            points={{28,60},{48,60}},
            color={0,0,0},
            thickness=1));
        connect(BreathRate.y,breathInterval1. RR)
          annotation (Line(points={{-63,12},{-48.4,12}},   color={0,0,127}));
        connect(breathInterval1.Pm,product1. u2) annotation (Line(points={{-27,12},
                {-16,12},{-16,14},{-10,14}},
                                         color={0,0,127}));
        connect(RespiratoryMuscleAmplitude.y,product1. u1) annotation (Line(points={{-47,38},
                {-16,38},{-16,26},{-10,26}},color={0,0,127}));
        connect(product1.y,Cw. externalPressure) annotation (Line(points={{13,20},
                {28,20},{28,44},{50,44},{50,40},{50.4,40}},
                                                         color={0,0,127}));
        connect(flowMeasure.q_out,resistor. q_in) annotation (Line(
            points={{-4,60},{8,60}},
            color={0,0,0},
            thickness=1));
        connect(flowMeasure.q_in,unlimitedVolume2. y) annotation (Line(
            points={{-24,60},{-36,60}},
            color={0,0,0},
            thickness=1));
        connect(flowMeasure.volumeFlow,int. u)
          annotation (Line(points={{-14,72},{-14,88},{4,88}},color={0,0,127}));
        connect(flowMeasure.volumeFlow, Stream.solutionFlow) annotation (Line(
              points={{-14,72},{28,72},{28,74},{86,74},{86,0},{6,0},{6,-35}},
              color={0,0,127}));
        connect(substance.solutionVolume, volume.y) annotation (Line(points={{
                32,-38},{32,-10},{27,-10}}, color={0,0,127}));
        connect(substance.q_out, Stream1.q_in) annotation (Line(
            points={{36,-42},{22,-42},{22,-60},{52,-60},{52,-42},{58,-42}},
            color={107,45,134},
            thickness=1));
        connect(Stream1.solutionFlow, Stream.solutionFlow) annotation (Line(
              points={{68,-35},{68,0},{6,0},{6,-35}}, color={0,0,127}));
        connect(Stream1.q_out, substance1.q_out) annotation (Line(
            points={{78,-42},{96,-42}},
            color={107,45,134},
            thickness=1));
        connect(substance1.solutionVolume, CL.volume) annotation (Line(points={
                {92,-38},{92,46},{54,46},{54,50}}, color={0,0,127}));
        connect(Pressure1.y, unlimitedGasSource1.partialPressure) annotation (
            Line(points={{33,-74},{50,-74},{50,-74},{55.6,-74}}, color={0,0,127}));
        connect(unlimitedGasSource1.temperature, vaporPressure.T) annotation (
            Line(points={{56.2,-88},{-2,-88},{-2,-84},{-80,-84},{-80,-74},{-94,
                -74},{-94,-64},{-86.6,-64}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end test_respiration_valves;

      model DeathVolumeTesting
        Components.UnlimitedGasSource unlimitedGasSource
          annotation (Placement(transformation(extent={{-20,30},{0,50}})));
        Physiolibrary.Types.Constants.FractionConst FiO2(k=0.21)
          annotation (Placement(transformation(extent={{-94,76},{-86,84}})));
        Modelica.Blocks.Math.Product PiO2
          annotation (Placement(transformation(extent={{-46,62},{-36,72}})));
        Physiolibrary.Types.Constants.PressureConst Pressure(k=101325.0144354)
          annotation (Placement(transformation(extent={{-98,56},{-90,64}})));
        Modelica.Blocks.Math.Feedback minus_pH2O annotation (Placement(
              transformation(
              extent={{-7,-7},{7,7}},
              rotation=0,
              origin={-71,59})));
        Components.VaporPressure vaporPressure
          annotation (Placement(transformation(extent={{-96,24},{-76,44}})));
        Physiolibrary.Types.Constants.TemperatureConst BodyTemperature(k=310.15)
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=0,
              origin={-78,-40})));
        Physiolibrary.Types.Constants.VolumeConst volume(k=0.00015)
          annotation (Placement(transformation(extent={{-48,-12},{-40,-4}})));
        Physiolibrary.Types.Constants.VolumeFlowRateConst volumeFlowRate(k(
              displayUnit="l/min") = 0.0001)
          annotation (Placement(transformation(extent={{-52,-28},{-44,-20}})));
        Physiolibrary.Chemical.Components.Substance substance1(
            useNormalizedVolume=false, solute_start=0.001*(5.17*2300))
          annotation (Placement(transformation(extent={{2,-96},{22,-76}})));
        Components.DeathVolume deathVolume
          annotation (Placement(transformation(extent={{0,-26},{26,2}})));
        Physiolibrary.Types.Constants.VolumeConst volume1(k=0.0023)
          annotation (Placement(transformation(extent={{-68,-68},{-60,-60}})));
        Physiolibrary.Chemical.Components.Stream Stream(useSolutionFlowInput=
              true) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={10,-48})));
      equation
        connect(unlimitedGasSource.temperature, vaporPressure.T) annotation (
            Line(points={{-19.8,34},{-52,34},{-52,18},{-86,18},{-86,34},{-94.6,
                34}}, color={0,0,127}));
        connect(vaporPressure.VaporPressure_, minus_pH2O.u2) annotation (Line(
              points={{-74.6,34.2},{-68,34.2},{-68,48},{-71,48},{-71,53.4}},
              color={0,0,127}));
        connect(Pressure.y, minus_pH2O.u1)
          annotation (Line(points={{-89,60},{-76.6,59}}, color={0,0,127}));
        connect(minus_pH2O.y, PiO2.u2) annotation (Line(points={{-64.7,59},{-47,
                59},{-47,64}}, color={0,0,127}));
        connect(FiO2.y, PiO2.u1) annotation (Line(points={{-85,80},{-52,80},{
                -52,70},{-47,70}}, color={0,0,127}));
        connect(PiO2.y, unlimitedGasSource.partialPressure) annotation (Line(
              points={{-35.5,67},{-24,67},{-24,48},{-20.4,48}}, color={0,0,127}));
        connect(BodyTemperature.y, vaporPressure.T) annotation (Line(points={{
                -73,-40},{-68,-40},{-68,18},{-86,18},{-86,34},{-94.6,34}},
              color={0,0,127}));
        connect(deathVolume.environmentalFlow, unlimitedGasSource.port_a)
          annotation (Line(
            points={{12.22,-1.36},{12.22,39.4},{0,39.4}},
            color={107,45,134},
            thickness=1));
        connect(volume.y, deathVolume.volume) annotation (Line(points={{-39,-8},
                {-39,-9.2},{0.26,-9.2}}, color={0,0,127}));
        connect(volumeFlowRate.y, deathVolume.volumeFlow) annotation (Line(
              points={{-43,-24},{-6,-24},{-6,-17.6},{0.26,-17.6}}, color={0,0,
                127}));
        connect(volume1.y, substance1.solutionVolume) annotation (Line(points={
                {-59,-64},{-2,-64},{-2,-82},{8,-82}}, color={0,0,127}));
        connect(Stream.q_in, deathVolume.alveolarFlow) annotation (Line(
            points={{10,-38},{10,-30},{12.74,-30},{12.74,-24.6}},
            color={107,45,134},
            thickness=1));
        connect(Stream.q_out, substance1.q_out) annotation (Line(
            points={{10,-58},{10,-72},{12,-72},{12,-86}},
            color={107,45,134},
            thickness=1));
        connect(Stream.solutionFlow, deathVolume.volumeFlow) annotation (Line(
              points={{17,-48},{30,-48},{30,-50},{46,-50},{46,-42},{-20,-42},{
                -20,-24},{-6,-24},{-6,-17.6},{0.26,-17.6}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DeathVolumeTesting;

      model TestDeathVolume
        Components.UnlimitedGasSource unlimitedGasSource
          annotation (Placement(transformation(extent={{-34,-42},{-14,-22}})));
        Physiolibrary.Types.Constants.PressureConst Pressure(k=101325.0144354)
          annotation (Placement(transformation(extent={{-94,-40},{-86,-32}})));
        Modelica.Blocks.Math.Feedback minus_pH2O annotation (Placement(
              transformation(
              extent={{-7,-7},{7,7}},
              rotation=0,
              origin={-67,-27})));
        Physiolibrary.Types.Constants.FractionConst FiO2(k=0.21)
          annotation (Placement(transformation(extent={{-90,-18},{-82,-10}})));
        Modelica.Blocks.Math.Product PiO2
          annotation (Placement(transformation(extent={{-54,-30},{-44,-20}})));
        Physiolibrary.Types.Constants.TemperatureConst BodyTemperature(k=310.15)
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=0,
              origin={-86,-64})));
        Physiolibrary.Types.Constants.VolumeConst volume(k=0.00015)
          annotation (Placement(transformation(extent={{-32,-68},{-24,-60}})));
        Physiolibrary.Types.Constants.VolumeFlowRateConst volumeFlowRate(k(
              displayUnit="l/min") = 0.0001)
          annotation (Placement(transformation(extent={{-36,-84},{-28,-76}})));
        Components.DeathVolume deathVolume
          annotation (Placement(transformation(extent={{16,-82},{42,-54}})));
      equation
        connect(minus_pH2O.y, PiO2.u2) annotation (Line(points={{-60.7,-27},{
                -60.7,-28},{-55,-28}}, color={0,0,127}));
        connect(Pressure.y, minus_pH2O.u1)
          annotation (Line(points={{-85,-36},{-72.6,-27}}, color={0,0,127}));
        connect(BodyTemperature.y, minus_pH2O.u2) annotation (Line(points={{-81,
                -64},{-67,-64},{-67,-32.6}}, color={0,0,127}));
        connect(PiO2.u1, FiO2.y) annotation (Line(points={{-55,-22},{-55,-14},{
                -81,-14}}, color={0,0,127}));
        connect(PiO2.y, unlimitedGasSource.partialPressure) annotation (Line(
              points={{-43.5,-25},{-38,-25},{-38,-24},{-34.4,-24}}, color={0,0,
                127}));
        connect(unlimitedGasSource.temperature, minus_pH2O.u2) annotation (Line(
              points={{-33.8,-38},{-50,-38},{-50,-40},{-67,-40},{-67,-32.6}},
              color={0,0,127}));
        connect(volume.y, deathVolume.volume) annotation (Line(points={{-23,-64},
                {-23,-65.2},{16.26,-65.2}}, color={0,0,127}));
        connect(volumeFlowRate.y, deathVolume.volumeFlow) annotation (Line(
              points={{-27,-80},{10,-80},{10,-73.6},{16.26,-73.6}}, color={0,0,
                127}));
        connect(unlimitedGasSource.port_a, deathVolume.environmentalFlow)
          annotation (Line(
            points={{-14,-32.6},{-2,-32.6},{-2,-34},{28.22,-34},{28.22,-57.36}},

            color={107,45,134},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end TestDeathVolume;

      model test_respiration
        PhysiolibraryExpantion.ElasticVessel CL(
          volume_start=0.0023,
          ZeroPressureVolume=0.0013,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{38,50},{58,70}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume2(
            usePressureInput=false)
          annotation (Placement(transformation(extent={{-56,50},{-36,70}})));
        PhysiolibraryExpantion.OuterElasticVessel Cw(
          outer_volume_start=0,
          useV0Input=false,
          ZeroPressureVolume=0.00352,
          useComplianceInput=false,
          Compliance(displayUnit="l/cmH2O") = 2.4881075596661e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{42,26},{62,46}})));
        Physiolibrary.Hydraulic.Components.Resistor resistor(Resistance(
              displayUnit="(cmH2O.s)/l") = 362846.05)
          annotation (Placement(transformation(extent={{8,50},{28,70}})));
        MeursHemodynamics.Components.BreathInterval breathInterval1
          annotation (Placement(transformation(extent={{-48,2},{-28,22}})));
        Modelica.Blocks.Sources.Constant BreathRate(k=12)
          annotation (Placement(transformation(extent={{-84,2},{-64,22}})));
        Physiolibrary.Types.Constants.PressureConst RespiratoryMuscleAmplitude(k(
              displayUnit="cmH2O") = 441.29925)
          annotation (Placement(transformation(extent={{-56,34},{-48,42}})));
        Modelica.Blocks.Math.Product product1
          annotation (Placement(transformation(extent={{-8,10},{12,30}})));
        Physiolibrary.Blocks.Math.Integrator int
          annotation (Placement(transformation(extent={{6,78},{26,98}})));
        Physiolibrary.Hydraulic.Sensors.FlowMeasure flowMeasure
          annotation (Placement(transformation(extent={{-24,50},{-4,70}})));
      equation
        connect(Cw.InnerVolume,CL. volume)
          annotation (Line(points={{58.2,34},{54,34},{54,50}},
                                                             color={0,0,127}));
        connect(Cw.pressure,CL. externalPressure) annotation (Line(points={{64.1,
                34.9},{64.1,72},{56,72},{56,68}},
                                            color={0,0,127}));
        connect(resistor.q_out,CL. q_in) annotation (Line(
            points={{28,60},{48,60}},
            color={0,0,0},
            thickness=1));
        connect(BreathRate.y,breathInterval1. RR)
          annotation (Line(points={{-63,12},{-48.4,12}},   color={0,0,127}));
        connect(breathInterval1.Pm,product1. u2) annotation (Line(points={{-27,12},
                {-16,12},{-16,14},{-10,14}},
                                         color={0,0,127}));
        connect(RespiratoryMuscleAmplitude.y,product1. u1) annotation (Line(points={{-47,38},
                {-16,38},{-16,26},{-10,26}},color={0,0,127}));
        connect(product1.y,Cw. externalPressure) annotation (Line(points={{13,20},
                {28,20},{28,44},{50,44},{50,40},{50.4,40}},
                                                         color={0,0,127}));
        connect(flowMeasure.q_out,resistor. q_in) annotation (Line(
            points={{-4,60},{8,60}},
            color={0,0,0},
            thickness=1));
        connect(flowMeasure.q_in,unlimitedVolume2. y) annotation (Line(
            points={{-24,60},{-36,60}},
            color={0,0,0},
            thickness=1));
        connect(flowMeasure.volumeFlow,int. u)
          annotation (Line(points={{-14,72},{-14,88},{4,88}},color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end test_respiration;
    end Tests;

    package PhysiolibraryExpantion

      model OuterElasticVessel
        "Elastic container for blood vessels, bladder, lumens"
       //extends Physiolibrary.Icons.ElasticBalloon;
      // extends SteadyStates.Interfaces.SteadyState(state_start=volume_start, storeUnit="ml");
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a
          q_in annotation (Placement(
              transformation(extent={{-14,-14},{
                  14,14}}), iconTransformation(extent={{-24,-78},{4,-50}})));

        parameter Physiolibrary.Types.Volume outer_volume_start=1e-11
          "Outer volume start value" annotation (Dialog(
              group="Initialization"));                                                //default = 1e-5 ml
        Physiolibrary.Types.Volume excessVolume
          "Additional volume, that generate pressure";

         parameter Boolean useV0Input=false
          "=true, if zero-pressure-volume input is used"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true),Dialog(group="External inputs/outputs"));

         parameter Physiolibrary.Types.Volume ZeroPressureVolume=1e-11
          "Maximal volume, that does not generate pressure if useV0Input=false"
          annotation (Dialog(enable=not
                useV0Input));                         //default = 1e-5 ml

         Physiolibrary.Types.RealIO.VolumeInput zeroPressureVolume(start=
              ZeroPressureVolume)=zpv if
          useV0Input annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={-80,80}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={-100,40})));
        parameter Boolean useComplianceInput=false
          "=true, if compliance input is used" annotation (
          Evaluate=true,
          HideResult=true,
          choices(checkBox=true),
          Dialog(group="External inputs/outputs"));
        parameter Physiolibrary.Types.HydraulicCompliance Compliance=1
          "Compliance if useComplianceInput=false" annotation (Dialog(enable=not
                useComplianceInput), HideResult=useComplianceInput);

        Physiolibrary.Types.RealIO.HydraulicComplianceInput compliance(start=
              Compliance) = c if
          useComplianceInput annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={0,80}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={-58,40})));
        parameter Boolean useExternalPressureInput=false
          "=true, if external pressure input is used" annotation (
          Evaluate=true,
          HideResult=true,
          choices(checkBox=true),
          Dialog(group="External inputs/outputs"));
        parameter Physiolibrary.Types.Pressure ExternalPressure=0
          "External pressure. Set zero if internal pressure is relative to external. Valid only if useExternalPressureInput=false."
          annotation (Dialog(enable=not useExternalPressureInput), HideResult=
              useExternalPressureInput);

        Physiolibrary.Types.RealIO.PressureInput externalPressure(start=
              ExternalPressure) = ep if
          useExternalPressureInput annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={80,80}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={-16,40})));

        Physiolibrary.Types.RealIO.VolumeOutput volume(start=InnerVolume +
              OuterVolume, fixed=true) annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={0,-100}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={60,-90})));
        Physiolibrary.Types.Volume OuterVolume(start=outer_volume_start, fixed=true);
        parameter Physiolibrary.Types.Volume excessVolume_min(min=-Modelica.Constants.inf)=
             0 "Minimal pressure below zeroPressureVolume" annotation (Evaluate=true);
        parameter Physiolibrary.Types.Pressure MinimalCollapsingPressure=-101325;
        parameter Physiolibrary.Types.Volume CollapsingPressureVolume=1e-12
          "Maximal volume, which generate negative collapsing pressure";
        //default = 1e-6 ml

        //parameter Boolean useInnerInput=false "=true, if inner volume input is used"

         Physiolibrary.Types.RealIO.VolumeInput InnerVolume
         annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={-80,80}),
              iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={62,-20})));

            parameter Boolean hardElastance=false;

        Physiolibrary.Types.RealIO.PressureOutput
          pressure annotation (Placement(transformation(
                extent={{-252,-120},{-232,-100}}),
              iconTransformation(
              extent={{-19,-19},{19,19}},
              rotation=0,
              origin={121,-11})));
      protected
        Physiolibrary.Types.Volume zpv;
        Physiolibrary.Types.HydraulicCompliance c;
        Physiolibrary.Types.Pressure ep;
        parameter Physiolibrary.Types.Pressure a=
            MinimalCollapsingPressure/log(
            Modelica.Constants.eps);

      equation
        if not useV0Input then
          zpv = ZeroPressureVolume;
        end if;
        if not useComplianceInput then
          c = Compliance;
        end if;
        if not useExternalPressureInput then
          ep = ExternalPressure;
        end if;
        excessVolume = volume - zpv;
        //excessVolume = max(excessVolume_min, volume - zpv);
        if hardElastance then
          q_in.pressure = excessVolume/c + ep;
        else
          q_in.pressure = smooth(0, if noEvent(volume > CollapsingPressureVolume)
             then (excessVolume/c + ep) else (a*log(max(Modelica.Constants.eps,
            volume/CollapsingPressureVolume)) + ep));
        end if;
        pressure = q_in.pressure;
        //then: normal physiological state
        //else: abnormal collapsing state

        //Collapsing state: the max function prevents the zero or negative input to logarithm, the logarithm brings more negative pressure for smaller volume
        //However this collapsing is limited with numerical precission, which is reached relatively soon.

        der(OuterVolume) = q_in.q;
        volume = InnerVolume + OuterVolume;
        assert(
          volume >= -Modelica.Constants.eps,
          "Collapsing of vessels are not supported!",
          AssertionLevel.warning);
          annotation (
          Evaluate=true,
          HideResult=true,
          choices(checkBox=true),
          Dialog(group="External inputs/outputs"),
                    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics={
              Text(
                extent={{-316,-148},{162,-108}},
                textString="%name",
                lineColor={0,0,255}),
              Ellipse(
                extent={{-20,26},{14,-4}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-100,84},{102,-96}},
                lineColor={238,46,47},
                lineThickness=1,
                fillColor={170,255,255},
                fillPattern=FillPattern.Solid,
                startAngle=0,
                endAngle=360),
              Ellipse(
                extent={{-100,54},{102,-46}},
                lineColor={238,46,47},
                lineThickness=1,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                startAngle=0,
                endAngle=360),
              Rectangle(
                extent={{-100,86},{102,8}},
                lineThickness=1,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None)}), Documentation(revisions="<html>
<p><i>2009-2014 - </i>Marek Matejak, Charles University, Prague, Czech Republic</p>
<ul>
<li>initial implementation </li>
</ul>
<p>4.5.2015 - Tom&aacute;&scaron; Kulh&aacute;nek, Charles University, Prague, Czech Republic</p>
<ul>
<li>fix of external pressure</li>
</ul>
</html>",       info="<html>
<p>Pressure can be generated by an elastic tissue surrounding some accumulated volume. Typically there is a threshold volume, below which the relative pressure is equal to external pressure and the wall of the blood vessels is not stressed. But if the volume rises above this value, the pressure increases proportionally. The slope in this pressure-volume characteristic is called &ldquo;Compliance&rdquo;.</p>
<ul>
<li>Increassing volume above ZeroPressureVolume (V0) generate positive pressure (greater than external pressure) lineary dependent on excess volume.</li>
<li>Decreasing volume below CollapsingPressureVolume (V00) generate negative pressure (lower than external pressure) logarithmicaly dependent on volume.</li>
<li>Otherwise external pressure is presented as pressure inside ElasticVessel.</li>
</ul>
<p><br><img src=\"modelica://Physiolibrary/Resources/Images/UserGuide/ElasticVessel_PV.png\"/></p>
</html>"));
      end OuterElasticVessel;

      model ElasticVessel
        "Elastic container for blood vessels, bladder, lumens"
       extends Physiolibrary.Icons.ElasticBalloon;
      // extends SteadyStates.Interfaces.SteadyState(state_start=volume_start, storeUnit="ml");
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a q_in
          annotation (Placement(transformation(extent={{-14,-14},{14,14}})));
        parameter Physiolibrary.Types.Volume volume_start=1e-11 "Volume start value"
          annotation (Dialog(group="Initialization"));                                 //default = 1e-5 ml
        Physiolibrary.Types.Volume excessVolume
          "Additional volume, that generate pressure";

         parameter Boolean useV0Input = false
          "=true, if zero-pressure-volume input is used"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true),Dialog(group="External inputs/outputs"));

         parameter Physiolibrary.Types.Volume ZeroPressureVolume=1e-11
          "Maximal volume, that does not generate pressure if useV0Input=false"
          annotation (Dialog(enable=not useV0Input)); //default = 1e-5 ml

         Physiolibrary.Types.RealIO.VolumeInput zeroPressureVolume(start=
              ZeroPressureVolume)=zpv if useV0Input annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={-80,80})));
        parameter Boolean useComplianceInput = false
          "=true, if compliance input is used"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true),Dialog(group="External inputs/outputs"));
        parameter Physiolibrary.Types.HydraulicCompliance Compliance=1
          "Compliance if useComplianceInput=false" annotation (Dialog(enable=not
                useComplianceInput), HideResult=useComplianceInput);

        Physiolibrary.Types.RealIO.HydraulicComplianceInput compliance(start=
              Compliance)=c if useComplianceInput annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={0,80})));
        parameter Boolean useExternalPressureInput = false
          "=true, if external pressure input is used"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true),Dialog(group="External inputs/outputs"));
        parameter Physiolibrary.Types.Pressure ExternalPressure=0
          "External pressure. Set zero if internal pressure is relative to external. Valid only if useExternalPressureInput=false."
          annotation (Dialog(enable=not useExternalPressureInput), HideResult=
              useExternalPressureInput);

        Physiolibrary.Types.RealIO.PressureInput externalPressure(start=
              ExternalPressure)=ep if useExternalPressureInput annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={80,80})));

        Physiolibrary.Types.RealIO.VolumeOutput volume(start=volume_start, fixed=true)
          annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={0,-100}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={60,-100})));
         parameter Physiolibrary.Types.Volume excessVolume_min(min = -Modelica.Constants.inf) = 0 "Minimal pressure below zeroPressureVolume" annotation(Evaluate = true);
        parameter Physiolibrary.Types.Pressure MinimalCollapsingPressure=-101325;
          parameter Physiolibrary.Types.Volume CollapsingPressureVolume=1e-12
                 "Maximal volume, which generate negative collapsing pressure";
                                                                         //default = 1e-6 ml
         parameter Boolean hardElastance=false;

      protected
        Physiolibrary.Types.Volume zpv;
        Physiolibrary.Types.HydraulicCompliance c;
        Physiolibrary.Types.Pressure ep;
        parameter Physiolibrary.Types.Pressure a=MinimalCollapsingPressure/log(
            Modelica.Constants.eps);

      equation
        if not useV0Input then
          zpv = ZeroPressureVolume;
        end if;
        if not useComplianceInput then
          c = Compliance;
        end if;
        if not useExternalPressureInput then
          ep = ExternalPressure;
        end if;
        excessVolume = max(excessVolume_min, volume - zpv);
        if hardElastance then
          q_in.pressure = (volume-zpv)/c + ep;
        else
          q_in.pressure = smooth(0, if noEvent(volume > CollapsingPressureVolume)
             then (excessVolume/c + ep) else (a*log(max(Modelica.Constants.eps,
            volume/CollapsingPressureVolume)) + ep));
        end if;
        //then: normal physiological state
        //else: abnormal collapsing state

        //Collapsing state: the max function prevents the zero or negative input to logarithm, the logarithm brings more negative pressure for smaller volume
        //However this collapsing is limited with numerical precission, which is reached relatively soon.

        der(volume) = q_in.q;
        assert(
          volume >= -Modelica.Constants.eps,
          "Collapsing of vessels are not supported!",
          AssertionLevel.warning);
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics={Text(
                extent={{-318,-140},{160,-100}},
                textString="%name",
                lineColor={0,0,255})}), Documentation(revisions="<html>
<p><i>2009-2014 - </i>Marek Matejak, Charles University, Prague, Czech Republic</p>
<ul>
<li>initial implementation </li>
</ul>
<p>4.5.2015 - Tom&aacute;&scaron; Kulh&aacute;nek, Charles University, Prague, Czech Republic</p>
<ul>
<li>fix of external pressure</li>
</ul>
</html>",       info="<html>
<p>Pressure can be generated by an elastic tissue surrounding some accumulated volume. Typically there is a threshold volume, below which the relative pressure is equal to external pressure and the wall of the blood vessels is not stressed. But if the volume rises above this value, the pressure increases proportionally. The slope in this pressure-volume characteristic is called &ldquo;Compliance&rdquo;.</p>
<ul>
<li>Increassing volume above ZeroPressureVolume (V0) generate positive pressure (greater than external pressure) lineary dependent on excess volume.</li>
<li>Decreasing volume below CollapsingPressureVolume (V00) generate negative pressure (lower than external pressure) logarithmicaly dependent on volume.</li>
<li>Otherwise external pressure is presented as pressure inside ElasticVessel.</li>
</ul>
<p><br><img src=\"modelica://Physiolibrary/Resources/Images/UserGuide/ElasticVessel_PV.png\"/></p>
</html>"));
      end ElasticVessel;

      package Obsolent
        model SerialElasticConnections
          Physiolibrary.Types.RealIO.PressureOutput pressure annotation (
              Placement(transformation(extent={{84,2},{104,22}}),
                iconTransformation(extent={{56,0},{76,20}})));
          Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a Inner_a annotation (
              Placement(transformation(extent={{-96,0},{-76,20}}),
                iconTransformation(extent={{-4,-12},{16,8}})));
          Physiolibrary.Hydraulic.Interfaces.HydraulicPort_b Inner_b annotation (
              Placement(transformation(extent={{-24,0},{-4,20}}),
                iconTransformation(extent={{-4,16},{16,36}})));
          Physiolibrary.Hydraulic.Interfaces.HydraulicPort_b Outer annotation (
              Placement(transformation(extent={{28,0},{48,20}}),
                iconTransformation(extent={{-2,-42},{18,-22}})));
          Physiolibrary.Hydraulic.Sources.UnlimitedPump unlimitedPump(
              useSolutionFlowInput=true)
            annotation (Placement(transformation(extent={{-4,0},{16,20}})));
          Physiolibrary.Hydraulic.Sensors.FlowMeasure flowMeasure
            annotation (Placement(transformation(extent={{-66,0},{-46,20}})));
          Physiolibrary.Hydraulic.Sensors.PressureMeasure pressureMeasure
            annotation (Placement(transformation(extent={{40,6},{60,26}})));
        equation
          connect(Inner_a, flowMeasure.q_in) annotation (Line(
              points={{-86,10},{-66,10}},
              color={0,0,0},
              thickness=1));
          connect(flowMeasure.q_out, Inner_b) annotation (Line(
              points={{-46,10},{-14,10}},
              color={0,0,0},
              thickness=1));
          connect(Outer, unlimitedPump.q_out) annotation (Line(
              points={{38,10},{16,10}},
              color={0,0,0},
              thickness=1));
          connect(pressureMeasure.q_in, Outer) annotation (Line(
              points={{46,10},{38,10}},
              color={0,0,0},
              thickness=1));
          connect(pressureMeasure.pressure, pressure)
            annotation (Line(points={{56,12},{94,12}}, color={0,0,127}));
          connect(flowMeasure.volumeFlow, unlimitedPump.solutionFlow) annotation (
             Line(points={{-56,22},{-56,34},{6,34},{6,17}}, color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                  Ellipse(
                  extent={{-48,64},{62,-44}},
                  lineColor={28,108,200},
                  lineThickness=1,
                  fillColor={170,255,255},
                  fillPattern=FillPattern.Solid), Ellipse(
                  extent={{-28,44},{40,-20}},
                  lineColor={28,108,200},
                  lineThickness=1,
                  fillColor={28,108,200},
                  fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
                  preserveAspectRatio=false)));
        end SerialElasticConnections;

        model ElasticVesselWithInnerVolume
          "Elastic container for blood vessels, bladder, lumens"
         extends Physiolibrary.Icons.ElasticBalloon;
        // extends SteadyStates.Interfaces.SteadyState(state_start=volume_start, storeUnit="ml");
          Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a
            q_in annotation (Placement(
                transformation(extent={{-14,-14},{
                    14,14}}), iconTransformation(extent={{
                    -16,-32},{12,-4}})));

          parameter Physiolibrary.Types.Volume outer_volume_start=1e-11
            "Outer volume start value" annotation (Dialog(
                group="Initialization"));                                                //default = 1e-5 ml
          Physiolibrary.Types.Volume excessVolume
            "Additional volume, that generate pressure";

           parameter Boolean useV0Input=false
            "=true, if zero-pressure-volume input is used"
            annotation(Evaluate=true, HideResult=true, choices(checkBox=true),Dialog(group="External inputs/outputs"));

           parameter Physiolibrary.Types.Volume ZeroPressureVolume=1e-11
            "Maximal volume, that does not generate pressure if useV0Input=false"
            annotation (Dialog(enable=not
                  useV0Input));                         //default = 1e-5 ml

           Physiolibrary.Types.RealIO.VolumeInput zeroPressureVolume(start=
                ZeroPressureVolume)=zpv if
            useV0Input annotation (Placement(
                transformation(
                extent={{-20,-20},{20,20}},
                rotation=270,
                origin={-80,80})));
          parameter Boolean useComplianceInput=false
            "=true, if compliance input is used" annotation (
            Evaluate=true,
            HideResult=true,
            choices(checkBox=true),
            Dialog(group="External inputs/outputs"));
          parameter Physiolibrary.Types.HydraulicCompliance Compliance=1
            "Compliance if useComplianceInput=false" annotation (Dialog(enable=not
                  useComplianceInput), HideResult=useComplianceInput);

          Physiolibrary.Types.RealIO.HydraulicComplianceInput compliance(start=
                Compliance) = c if
            useComplianceInput annotation (Placement(transformation(
                extent={{-20,-20},{20,20}},
                rotation=270,
                origin={0,80})));
          parameter Boolean useExternalPressureInput=false
            "=true, if external pressure input is used" annotation (
            Evaluate=true,
            HideResult=true,
            choices(checkBox=true),
            Dialog(group="External inputs/outputs"));
          parameter Physiolibrary.Types.Pressure ExternalPressure=0
            "External pressure. Set zero if internal pressure is relative to external. Valid only if useExternalPressureInput=false."
            annotation (Dialog(enable=not useExternalPressureInput), HideResult=
                useExternalPressureInput);

          Physiolibrary.Types.RealIO.PressureInput externalPressure(start=
                ExternalPressure) = ep if
            useExternalPressureInput annotation (Placement(transformation(
                extent={{-20,-20},{20,20}},
                rotation=270,
                origin={80,80})));

          Physiolibrary.Types.RealIO.VolumeOutput volume(start=InnerVolume +
                OuterVolume, fixed=true) annotation (Placement(transformation(
                extent={{-20,-20},{20,20}},
                rotation=270,
                origin={0,-100}), iconTransformation(
                extent={{-20,-20},{20,20}},
                rotation=270,
                origin={60,-100})));
          Physiolibrary.Types.Volume OuterVolume(start=outer_volume_start, fixed=true);
          parameter Physiolibrary.Types.Volume excessVolume_min(min=-Modelica.Constants.inf)=
               0 "Minimal pressure below zeroPressureVolume" annotation (Evaluate=true);
          parameter Physiolibrary.Types.Pressure MinimalCollapsingPressure=-101325;
          parameter Physiolibrary.Types.Volume CollapsingPressureVolume=1e-12
            "Maximal volume, which generate negative collapsing pressure";
          //default = 1e-6 ml

          parameter Boolean useInnerInput=false "=true, if inner volume input is used"
            annotation (
            Evaluate=true,
            HideResult=true,
            choices(checkBox=true),
            Dialog(group="External inputs/outputs"));

            Physiolibrary.Types.RealIO.VolumeInput InnerVolume
              annotation (Placement(
                transformation(
                extent={{-20,-20},{20,20}},
                rotation=270,
                origin={-80,80}),
                iconTransformation(
                extent={{-20,-20},{20,20}},
                rotation=0,
                origin={-40,14})));

           //Physiolibrary.Types.RealIO.VolumeInput InnerVolume=iv
           // if useInnerInput

              parameter Boolean hardElastance=false;

          Physiolibrary.Types.RealIO.PressureOutput
            pressure annotation (Placement(transformation(
                  extent={{-252,-120},{-232,-100}}),
                iconTransformation(
                extent={{-19,-19},{19,19}},
                rotation=270,
                origin={15,-99})));
        protected
          Physiolibrary.Types.Volume zpv;
          Physiolibrary.Types.HydraulicCompliance c;
          Physiolibrary.Types.Pressure ep;
          //Physiolibrary.Types.Volume iv;
          parameter Physiolibrary.Types.Pressure a=
              MinimalCollapsingPressure/log(
              Modelica.Constants.eps);

        equation
          if not useV0Input then
            zpv=ZeroPressureVolume;
          end if;
          if not useComplianceInput then
            c=Compliance;
          end if;
          if not useExternalPressureInput then
            ep=ExternalPressure;
          end if;
          if useInnerInput then
            excessVolume = volume - zpv;
          else
            excessVolume = max(excessVolume_min, volume - zpv);
          end if;
            if hardElastance then
            q_in.pressure = excessVolume/c + ep;
          else
            q_in.pressure = smooth(0, if noEvent(volume > CollapsingPressureVolume)
               then (excessVolume/c + ep) else (a*log(max(Modelica.Constants.eps,
              volume/CollapsingPressureVolume)) + ep));
          end if;
          pressure=q_in.pressure;
          //then: normal physiological state
          //else: abnormal collapsing state

          //Collapsing state: the max function prevents the zero or negative input to logarithm, the logarithm brings more negative pressure for smaller volume
          //However this collapsing is limited with numerical precission, which is reached relatively soon.

          der(OuterVolume) =  q_in.q;
          volume=InnerVolume+OuterVolume;
          assert(volume>=-Modelica.Constants.eps,"Collapsing of vessels are not supported!", AssertionLevel.warning);
         annotation (
            Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{
                    100,100}}), graphics={Text(
                  extent={{-316,-148},{162,-108}},
                  textString="%name",
                  lineColor={0,0,255}), Ellipse(
                  extent={{-20,28},{14,-2}},
                  lineColor={28,108,200},
                  fillColor={28,108,200},
                  fillPattern=FillPattern.Solid)}),
                                                 Documentation(revisions="<html>
<p><i>2009-2014 - </i>Marek Matejak, Charles University, Prague, Czech Republic</p>
<ul>
<li>initial implementation </li>
</ul>
<p>4.5.2015 - Tom&aacute;&scaron; Kulh&aacute;nek, Charles University, Prague, Czech Republic</p>
<ul>
<li>fix of external pressure</li>
</ul>
</html>",     info="<html>
<p>Pressure can be generated by an elastic tissue surrounding some accumulated volume. Typically there is a threshold volume, below which the relative pressure is equal to external pressure and the wall of the blood vessels is not stressed. But if the volume rises above this value, the pressure increases proportionally. The slope in this pressure-volume characteristic is called &ldquo;Compliance&rdquo;.</p>
<ul>
<li>Increassing volume above ZeroPressureVolume (V0) generate positive pressure (greater than external pressure) lineary dependent on excess volume.</li>
<li>Decreasing volume below CollapsingPressureVolume (V00) generate negative pressure (lower than external pressure) logarithmicaly dependent on volume.</li>
<li>Otherwise external pressure is presented as pressure inside ElasticVessel.</li>
</ul>
<p><br><img src=\"modelica://Physiolibrary/Resources/Images/UserGuide/ElasticVessel_PV.png\"/></p>
</html>"));
        end ElasticVesselWithInnerVolume;
      end Obsolent;

      model GasEquation

        Physiolibrary.Types.RealIO.VolumeInput
                                           V1(
                                         displayUnit="ml") annotation (Placement(transformation(extent={
                  {-118,42},{-78,82}}), iconTransformation(extent={{-100,66},{-72,94}})));
        Physiolibrary.Types.RealIO.PressureInput
                                           P1(
                                         displayUnit="mmHg") annotation (Placement(transformation(extent={
                  {-118,42},{-78,82}}), iconTransformation(extent={{-100,6},{
                -72,34}})));
        Physiolibrary.Types.RealIO.TemperatureInput
                                           T1(
                                         displayUnit="degC") annotation (Placement(transformation(extent={
                  {-118,42},{-78,82}}), iconTransformation(extent={{-100,-54},{-72,
                  -26}})));
        Physiolibrary.Types.RealIO.PressureInput
                                           P2(
                                        displayUnit="mmHg") annotation (Placement(transformation(extent={
                  {-118,42},{-78,82}}), iconTransformation(
              extent={{-14,-14},{14,14}},
              rotation=180,
              origin={86,20})));
        Physiolibrary.Types.RealIO.TemperatureInput
                                           T2(
                                         displayUnit="degC") annotation (Placement(transformation(extent={
                  {-118,42},{-78,82}}), iconTransformation(
              extent={{-14,-14},{14,14}},
              rotation=180,
              origin={86,-40})));
        Physiolibrary.Types.RealIO.VolumeOutput
                                            V2(
                                          displayUnit="ml") annotation (Placement(transformation(extent=
                  {{56,54},{96,94}}), iconTransformation(extent={{72,66},{100,94}})));
      equation
        (P1*V1)/(T1)=(P2*V2)/(T2);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={170,213,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{0,100},{0,-100}},
                lineColor={0,0,0},
                fillColor={170,213,255},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-98,136},{100,100}},
                lineColor={0,0,0},
                fillColor={170,213,255},
                fillPattern=FillPattern.Solid,
                textString="%name"),
            Text(
              extent={{-66,34},{132,8}},
              textColor={28,108,200},
              textString="P1",
              horizontalAlignment=TextAlignment.Left),
            Text(
              extent={{-68,92},{132,66}},
              textColor={28,108,200},
              textString="V1",
              horizontalAlignment=TextAlignment.Left),
            Text(
              extent={{-68,-26},{130,-54}},
              textColor={28,108,200},
              textString="T1",
              horizontalAlignment=TextAlignment.Left),
            Text(
              extent={{-130,92},{70,66}},
              textColor={28,108,200},
              horizontalAlignment=TextAlignment.Right,
              textString="V2"),
            Text(
              extent={{-128,32},{70,6}},
              textColor={28,108,200},
              horizontalAlignment=TextAlignment.Right,
              textString="P2"),
            Text(
              extent={{-128,-26},{70,-54}},
              textColor={28,108,200},
              horizontalAlignment=TextAlignment.Right,
              textString="T2")}));
      end GasEquation;

      model AlveolarVentilation_ATPD
        extends Physiolibrary.Icons.Lungs;
      //  parameter Real EnvironmentPressure(final displayUnit="mmHg");
      //  parameter Real EnvironmentTemperature(final displayUnit="degC");

        Physiolibrary.Types.RealIO.FrequencyInput RespRate
                                               annotation (Placement(transformation(
                extent={{34,80},{48,94}}),    iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=180,
              origin={80,20})));
        Physiolibrary.Types.RealIO.VolumeInput TidalVolume
                                                  annotation (Placement(
              transformation(
              extent={{-18,-18},{10,10}},
              origin={-54,-34}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=180,
              origin={80,100})));
        Physiolibrary.Types.RealIO.VolumeInput DeadSpace
                                                annotation (Placement(transformation(
                extent={{-72,-78},{-44,-50}}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=180,
              origin={80,60})));
        Physiolibrary.Types.RealIO.TemperatureInput core_T
                                               annotation (Placement(
              transformation(
              extent={{-20,-20},{8,8}},
              origin={-74,-86},
            rotation=0),         iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=180,
              origin={80,-20})));
        Physiomodel.Gases.Ventilation.GasEquation tidalVolume(V2(start=400))
          annotation (Placement(transformation(extent={{-12,-56},{12,-32}})));
        Physiomodel.Gases.Ventilation.GasEquation deadVolume(V2(start=150))
          annotation (Placement(transformation(extent={{-12,-86},{12,-62}})));
        Modelica.Blocks.Math.Product alveolarVentilation annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={54,52})));
        Physiomodel.Gases.Ventilation.VaporPressure vaporPressure annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={6,88})));
        Modelica.Blocks.Math.Division vaporPart annotation (Placement(
              transformation(
              extent={{-6,-6},{6,6}},
              rotation=270,
              origin={-20,34})));
        Modelica.Blocks.Math.Feedback added_pH2O annotation (Placement(
              transformation(
              extent={{-7,-7},{7,7}},
              rotation=270,
              origin={-15,61})));
        Physiomodel.Gases.Ventilation.VaporPressure vaporPressure1 annotation (
            Placement(transformation(extent={{-10,-10},{10,10}}, origin={-56,64})));
        Modelica.Blocks.Math.Product air_pH2O
          annotation (Placement(transformation(extent={{-36,68},{-26,78}})));
        Physiolibrary.Types.RealIO.TemperatureInput AmbientTemperature
                                               annotation (Placement(
              transformation(
              extent={{-20,-20},{8,8}},
              origin={-124,70}), iconTransformation(
              extent={{-20,-20},{20,20}},
              origin={-80,100})));
        Physiolibrary.Types.RealIO.PressureInput EnvironmentPressure
                                               annotation (Placement(
              transformation(
              extent={{-20,-20},{8,8}},
              origin={-86,-4}),  iconTransformation(
              extent={{-20,-20},{20,20}},
              origin={-80,40})));
        Physiolibrary.Types.RealIO.FractionInput EnvironmentRelativeHumidity
                                               annotation (Placement(
              transformation(
              extent={{-20,-20},{8,8}},
              origin={-124,92}), iconTransformation(
              extent={{-20,-20},{20,20}},
              origin={-80,-20})));
        Modelica.Blocks.Math.Feedback alveolarVolume
        annotation (Placement(transformation(extent={{60,-44},{80,-24}})));
        Modelica.Blocks.Math.Feedback airPressureWitoutVapor
          annotation (Placement(transformation(extent={{-62,0},{-42,-20}})));
        Physiolibrary.Types.RealIO.VolumeFlowRateOutput AlveolarVentilation
        annotation (Placement(transformation(extent={{70,22},{84,36}}),
            iconTransformation(
            extent={{-20,-20},{20,20}},
            origin={100,-60})));
        Modelica.Blocks.Math.Feedback dilution
          annotation (Placement(transformation(extent={{-30,20},{-10,0}})));
      Physiolibrary.Types.Constants.FractionConst Constant(k=1)
        annotation (Placement(transformation(extent={{-48,6},{-40,14}})));
        Physiolibrary.Types.RealIO.FractionOutput BronchiDilution
                                               annotation (Placement(transformation(
                extent={{6,4},{18,16}}),      iconTransformation(
              extent={{-20,-20},{20,20}},
              origin={100,-100})));
      equation

        connect(TidalVolume,tidalVolume. V1) annotation (Line(
            points={{-58,-38},{-32,-38},{-32,-34.4},{-10.32,-34.4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(DeadSpace, deadVolume.V1)   annotation (Line(
            points={{-58,-64},{-10.32,-64},{-10.32,-64.4}},
            color={0,0,127},
            smooth=Smooth.None));
      connect(RespRate, alveolarVentilation.u2) annotation (Line(
          points={{41,87},{40,87},{40,70},{48,70},{48,64}},
          color={0,0,127},
          smooth=Smooth.None));
        connect(core_T, vaporPressure.T)   annotation (Line(
            points={{-80,-92},{32,-92},{32,88},{14.6,88}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(air_pH2O.y, added_pH2O.u2) annotation (Line(
            points={{-25.5,73},{-25.5,61},{-20.6,61}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(added_pH2O.y, vaporPart.u1) annotation (Line(
            points={{-15,54.7},{-50,54.7},{-50,52},{-16,52},{-16,46},{-16.4,46},
              {-16.4,41.2}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(AmbientTemperature, vaporPressure1.T) annotation (Line(
            points={{-130,64},{-94,64},{-94,64},{-64.6,64}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(EnvironmentRelativeHumidity, air_pH2O.u1) annotation (Line(
            points={{-130,86},{-90,86},{-90,76},{-37,76}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(core_T, deadVolume.T1) annotation (Line(
            points={{-80,-92},{-40,-92},{-40,-78.8},{-10.32,-78.8}},
            color={0,0,127},
            smooth=Smooth.None));
      connect(alveolarVolume.y, alveolarVentilation.u1) annotation (Line(
          points={{79,-34},{90,-34},{90,80},{60,80},{60,64}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(tidalVolume.V2, alveolarVolume.u1) annotation (Line(
          points={{10.32,-34.4},{62,-34.4},{62,-34}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(alveolarVolume.u2, deadVolume.V2) annotation (Line(
          points={{70,-42},{70,-64},{10.32,-64},{10.32,-64.4}},
          color={0,0,127},
          smooth=Smooth.None));
        connect(EnvironmentPressure, airPressureWitoutVapor.u1) annotation (Line(
            points={{-92,-10},{-60,-10}},
            color={0,0,127},
            smooth=Smooth.None));

        connect(airPressureWitoutVapor.y, tidalVolume.P1) annotation (Line(
            points={{-43,-10},{-28,-10},{-28,-41.6},{-10.32,-41.6}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(airPressureWitoutVapor.y, deadVolume.P1) annotation (Line(
            points={{-43,-10},{-28,-10},{-28,-71.6},{-10.32,-71.6}},
            color={0,0,127},
            smooth=Smooth.None));

      connect(Constant.y, dilution.u1) annotation (Line(
          points={{-39,10},{-28,10}},
          color={0,0,127},
          smooth=Smooth.None));
        connect(vaporPart.y, dilution.u2) annotation (Line(
            points={{-20,27.4},{-20,18}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(dilution.y, BronchiDilution) annotation (Line(
            points={{-11,10},{12,10}},
            color={0,0,127},
            smooth=Smooth.None));
      connect(vaporPressure1.VaporPressure_, air_pH2O.u2) annotation (Line(
          points={{-44.6,64.2},{-40,64.2},{-40,70},{-37,70}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(vaporPressure.VaporPressure_, added_pH2O.u1) annotation (Line(
          points={{-5.4,87.8},{-15,87.8},{-15,66.6}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(alveolarVentilation.y, AlveolarVentilation) annotation (Line(
          points={{54,41},{54,29},{77,29}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(added_pH2O.y, airPressureWitoutVapor.u2) annotation (Line(
          points={{-15,54.7},{-52,54.7},{-52,-2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(EnvironmentPressure, vaporPart.u2) annotation (Line(
          points={{-92,-10},{-70,-10},{-70,46},{-23.6,46},{-23.6,41.2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(EnvironmentPressure, tidalVolume.P2) annotation (Line(
          points={{-92,-10},{-70,-10},{-70,-22},{20,-22},{20,-41.6},{10.32,
              -41.6}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(EnvironmentPressure, deadVolume.P2) annotation (Line(
          points={{-92,-10},{-70,-10},{-70,-22},{20,-22},{20,-71.6},{10.32,
              -71.6}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(tidalVolume.T1, deadVolume.T1) annotation (Line(points={{-10.32,
              -48.8},{-40,-48.8},{-40,-78.8},{-10.32,-78.8}}, color={0,0,127}));
      connect(deadVolume.T2, vaporPressure1.T) annotation (Line(points={{10.32,
              -78.8},{24,-78.8},{24,-102},{-114,-102},{-114,64},{-64.6,64}},
            color={0,0,127}));
      connect(tidalVolume.T2, vaporPressure1.T) annotation (Line(points={{10.32,
              -48.8},{24,-48.8},{24,-102},{-114,-102},{-114,64},{-64.6,64}},
            color={0,0,127}));
       annotation (
          Icon(coordinateSystem(preserveAspectRatio=true, extent={{-180,-120},{
                100,100}}),   graphics={Text(
                extent={{-100,-100},{76,-70}},
                textString="%name",
                lineColor={0,0,255})}), Diagram(coordinateSystem(extent={{-180,
                -120},{100,100}}), graphics={
            Text(
              extent={{-50,18},{-40,16}},
              textColor={28,108,200},
              textString="100"),
            Text(
              extent={{-48,-34},{-32,-36}},
              textColor={28,108,200},
              textString="BTPS"),
            Text(
              extent={{-20,-24},{-4,-26}},
              textColor={28,108,200},
              textString="BTPS"),
            Text(
              extent={{2,-24},{18,-26}},
              textColor={28,108,200},
              textString="ATPD"),
            Text(
              extent={{66,22},{82,20}},
              textColor={28,108,200},
              textString="ATPD")}));
      end AlveolarVentilation_ATPD;
    end PhysiolibraryExpantion;
  end MeursHemodynamicsPhysiolibrary;

  package Model
    model VanMeursHemodynamicsModel
      Components.Heart heart
        annotation (Placement(transformation(extent={{-22,-22},{20,24}})));
      Components.SystemicCirculation systemicCirculation
        annotation (Placement(transformation(extent={{-30,-84},{30,-24}})));
      Components.PulmonaryCirculation pulmonaryCirculation
        annotation (Placement(transformation(extent={{-30,14},{30,74}})));
    equation
      connect(systemicCirculation.bloodFlowOutflow, heart.rightAtriumFlowInflow)
        annotation (Line(
          points={{-30.6,-54},{-40,-54},{-40,3.3},{-16.12,3.3}},
          color={28,108,200},
          thickness=1));
      connect(heart.pulmonaryArteryOutflow, pulmonaryCirculation.bloodFlowInflow)
        annotation (Line(
          points={{-7.72,11.58},{-40,11.58},{-40,44},{-30.6,44}},
          color={28,108,200},
          thickness=1));
      connect(systemicCirculation.bloodFlowInflow, heart.aortaOutflow)
        annotation (Line(
          points={{28.8,-54},{40,-54},{40,3.3},{15.38,3.3}},
          color={255,0,0},
          thickness=1));
      connect(heart.leftAtriumFlowInflow, pulmonaryCirculation.bloodFlowOutflow)
        annotation (Line(
          points={{6.14,11.58},{40,11.58},{40,44},{30,44}},
          color={238,46,47},
          thickness=1));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={Ellipse(
              extent={{-62,62},{64,-64}},
              lineColor={255,0,0},
              pattern=LinePattern.None,
              lineThickness=1,
              fillPattern=FillPattern.Sphere,
              fillColor={244,125,35})}),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=10,
          __Dymola_NumberOfIntervals=50000,
          __Dymola_Algorithm="Dassl"));
    end VanMeursHemodynamicsModel;

    model MeurtHemodynamicsWithPhysiolobrary
      extends Physiolibrary.Icons.CardioVascular;
      MeursHemodynamicsPhysiolibrary.Components.HeartPhysiolibrary heartPhysiolibrary
        annotation (Placement(transformation(extent={{-44,-30},{38,42}})));
      MeursHemodynamicsPhysiolibrary.Components.PulmonaryCirculation pulmonaryCirculation
        annotation (Placement(transformation(extent={{-62,24},{60,116}})));
      MeursHemodynamicsPhysiolibrary.Components.SystemicCirculation systemicCirculation
        annotation (Placement(transformation(extent={{-60,-104},{60,2}})));
    equation
      connect(heartPhysiolibrary.pulmonaryArteryOutflow, pulmonaryCirculation.pulmonaryBloodInflow)
        annotation (Line(
          points={{-26.78,16.8},{-86,16.8},{-86,70},{-62,70}},
          color={0,0,0},
          thickness=1));
      connect(systemicCirculation.systemicBloodOutflow, heartPhysiolibrary.RightAtriumInflow)
        annotation (Line(
          points={{-60,-51},{-60,-52},{-88,-52},{-88,-1.2},{-26.78,-1.2}},
          color={0,0,0},
          thickness=1));
      connect(systemicCirculation.systemicBloodInflow, heartPhysiolibrary.AortaOutflow)
        annotation (Line(
          points={{60,-51},{84,-51},{84,-2.64},{24.06,-2.64}},
          color={0,0,0},
          thickness=1));
      connect(heartPhysiolibrary.leftAtriumInflow, pulmonaryCirculation.pulmonaryBloodOutflow)
        annotation (Line(
          points={{24.06,14.64},{86,14.64},{86,70.92},{60,70.92}},
          color={0,0,0},
          thickness=1));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=20,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Dassl"));
    end MeurtHemodynamicsWithPhysiolobrary;
  end Model;
  annotation (uses(        Modelica(version="4.0.0"), Physiolibrary(version=
            "2.4.1"),
      Physiomodel(version="2")),
    version="3",
    conversion(noneFromVersion="", noneFromVersion="2"));
end MeursHemodynamics;
