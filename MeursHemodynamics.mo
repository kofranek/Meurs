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

    connector BloodFlowConnector "Connector for blood flow"
      flow Types.VolumeFlowRate q "blood flow in m3/sec";
      Types.Pressure pressure "Pressure in Pa";
    end BloodFlowConnector;

    connector BloodFlowInflow "Blood flow inflow"
      extends BloodFlowConnector;
      annotation (
        Icon(graphics={  Rectangle(visible = true, origin = {2.04082, -1.0101}, fillColor = {255, 0, 0},
                fillPattern =                                                                                          FillPattern.Solid, extent = {{-102.041, -98.9899}, {97.9592, 101.01}})}, coordinateSystem(extent = {{-100.0, -100.0}, {100.0, 100.0}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})));
    end BloodFlowInflow;

    connector BloodFlowOutflow "Blood flow inflow"
      extends BloodFlowConnector;
      annotation (
        Icon(graphics={  Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255},
                fillPattern =                                                                                                    FillPattern.Solid)}));
    end BloodFlowOutflow;

    partial model BloodFlowOnePort
      Types.Pressure pressureDrop;
      Types.VolumeFlowRate bloodFlow;
      BloodFlowInflow bloodFlowInflow annotation (
          Placement(transformation(extent={{-374,20},{
                -354,40}}), iconTransformation(extent=
               {{-114,-10},{-94,10}})));
      BloodFlowOutflow bloodFlowOutflow annotation (
          Placement(transformation(extent={{-374,44},{
                -354,64}}), iconTransformation(extent=
               {{86,-10},{106,10}})));
    equation
      pressureDrop = bloodFlowInflow.pressure - bloodFlowOutflow.pressure;
      bloodFlowInflow.q + bloodFlowOutflow.q = 0;
      bloodFlow = bloodFlowInflow.q;
      annotation (
        Icon(graphics));
    end BloodFlowOnePort;

    model BloodResistor
      extends BloodFlowOnePort;
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
    end BloodResistor;

    model BloodConductance
      extends BloodFlowOnePort;
      parameter Real bloodConductance_NonSI  "conductance in ml/(mmHg s)";
      Real bloodConductance = bloodConductance_NonSI * 1e-6/133.322387415 "conductance in m3/(Pa s)";
    equation
      pressureDrop * bloodConductance = bloodFlow;
      annotation (
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics={  Line(visible = true, origin = {-70.0, 0.0}, points = {{-20.0, 0.0}, {20.0, 0.0}}), Line(visible = true, origin = {70.0, 0.0}, points = {{-20.0, 0.0}, {20.0, 0.0}}), Text(visible = true, extent = {{-180, -140}, {190, -110}}, fontName = "Arial", textString = "%name", lineColor = {0, 0, 0}),                                                                                   Polygon(points = {{-100, 100}, {0, 0}, {-100, -100}, {-100, 100}}, lineColor = {0, 0, 255}, smooth = Smooth.None, fillColor = {215, 215, 215},
                fillPattern =                                                                                                    FillPattern.Solid), Polygon(points = {{100, 100}, {100, -100}, {0, 0}, {100, 100}}, lineColor = {0, 0, 255}, smooth = Smooth.None, fillColor = {215, 215, 215},
                fillPattern =                                                                                                    FillPattern.Solid), Text(extent = {{-30, -40}, {40, -90}}, lineColor = {0, 0, 255}, textString = "G")}),
        Diagram(coordinateSystem(extent = {{-148.5, -105.0}, {148.5, 105.0}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5}), graphics={  Rectangle(visible = true, fillColor = {255, 255, 255}, extent = {{-60.0, -20.0}, {60.0, 20.0}}), Rectangle(visible = true, origin = {-80.0, 0.0}, fillColor = {255, 255, 255}, extent = {{-20.0, 0.0}, {20.0, 0.0}}), Rectangle(visible = true, origin = {80.0, 0.0}, fillColor = {255, 255, 255}, extent = {{-20.0, 0.0}, {20.0, 0.0}})}));
    end BloodConductance;

    model Inductor
      extends BloodFlowOnePort;
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
      BloodFlowInflow bloodFlowInflow annotation (
          Placement(transformation(extent={{-10,-10},{10,10}}),
                            iconTransformation(extent=
               {{-100,-10},{-80,10}})));
      BloodFlowOutflow bloodFlowOutflow annotation (
          Placement(transformation(extent={{-80,-30},{-60,-10}}),
                            iconTransformation(extent=
               {{80,-10},{100,10}})));
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
      BloodFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{-98,-10},{-78,10}}),  iconTransformation(extent={{-110,
                -10},{-90,10}})));
      BloodFlowOutflow bloodFlowOutflow annotation (Placement(transformation(
              extent={{78,-10},{98,10}}),  iconTransformation(extent={{90,-10},
                {110,10}})));
      BloodConductance backflowBloodConductance( bloodConductance_NonSI=backflowConductance) annotation (Placement(
          transformation(
          extent={{-18,-18},{18,18}},
          rotation=180,
          origin={22,-40})));
      BloodResistor outflowBloodResistor(bloodResistance_NonSI=outflowResistance)
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
    BloodFlowInflow bloodFlowInflow annotation (
        Placement(transformation(extent={{-50,-35},{-30,-15}}),
                          iconTransformation(extent={{-10,-10},{10,10}})));
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
    BloodFlowInflow bloodFlowInflow annotation (
        Placement(transformation(extent={{-50,-35},{-30,-15}}),
                          iconTransformation(extent={{-10,-10},{10,10}})));
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
    Components.BloodFlowInflow bloodFlowInflow annotation (Placement(
            transformation(extent={{-50,-35},{-30,-15}}), iconTransformation(
              extent={{-10,-10},{10,10}})));
    Types.PressureInput externalPressureInput=externalPressure if useExternalPressureInput annotation (Placement(
          transformation(extent={{-95,20},{-55,60}}),   iconTransformation(extent={{-120,
                -80},{-80,-40}})));
    Types.VolumeInput UnstressedVolumeInput=unstressedVolume if useUnstressedVolumeInput annotation (Placement(transformation(extent={{-140,45},
              {-100,85}}),
                     iconTransformation(extent={{-120,40},{-80,80}})));
      Types.HydraulicElastanceInput hydraulicElastanceInput=elastance if     useHydraulicElastanceInput
        annotation (Placement(transformation(extent={{-400,30},{-360,70}}),
            iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
    parameter Real elastance_NonSI "elastance in mmHg/ml";
    parameter Real unstressedVolume_NonSI "volume in ml";
    parameter Real externalPressure_NonSI "external pressure in mmHg";
    parameter Boolean useExternalPressureInput = false;
    parameter Boolean useUnstressedVolumeInput = false;
    parameter Boolean useHydraulicElastanceInput = false;
    Types.HydraulicElastance elastance;
    Types.Pressure externalPressure;
    Types.Volume unstressedVolume;
    parameter Real V0_NonSI "initial volume in ml";
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

    model UnlimitedBloodSource "unlimited blood source with given pressure"
      BloodFlowOutflow bloodFlowOutflow annotation (Placement(transformation(extent=
               {{-216,52},{-196,72}}), iconTransformation(extent={{78,-10},{98,10}})));
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
    end UnlimitedBloodSource;

    model UnlimitedBloodSink
      "unlimited blood outflow with given pressure"
      BloodFlowInflow bloodFlowInflow annotation (Placement(transformation(extent={{
                -216,44},{-196,64}}), iconTransformation(extent={{-98,-10},{-78,10}})));
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
    end UnlimitedBloodSink;

    model Heart
      parameter Real currentHeartRate=60 "heart rate in beats per min";
      BloodFlowInflow rightAtriumFlowInflow annotation (Placement(transformation(
              extent={{-98,-4},{-90,4}}), iconTransformation(extent={{-82,0},{-62,20}})));
      BloodFlowOutflow pulmonaryArteryOutflow annotation (Placement(transformation(
              extent={{-10,-4},{-2,4}}), iconTransformation(extent={{-42,36},{-22,56}})));
      BloodFlowInflow leftAtriumFlowInflow annotation (Placement(transformation(
              extent={{2,-4},{10,4}}), iconTransformation(extent={{24,36},{44,56}})));
      BloodFlowOutflow aortaOutflow annotation (Placement(transformation(extent={{92,
                -4},{100,4}}), iconTransformation(extent={{68,0},{88,20}})));
      VariableElasticCompartment rightAtrium(
        unstressedVolume_NonSI=30,
        externalPressure_NonSI=-4,
        V0_NonSI=156)
        annotation (Placement(transformation(extent={{-86,-10},{-66,10}})));
      VariableElasticCompartment leftAtrium(
        unstressedVolume_NonSI=30,
        externalPressure_NonSI=-4,
        V0_NonSI=93)
        annotation (Placement(transformation(extent={{12,-10},{32,10}})));
      VariableElasticCompartment rightVentricle(
        unstressedVolume_NonSI=40,
        externalPressure_NonSI=-4,
        V0_NonSI=150)
        annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
      VariableElasticCompartment leftVentricle(
        unstressedVolume_NonSI=60,
        externalPressure_NonSI=-4,
        V0_NonSI=143)
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
      BloodFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{96,-2},{100,2}}), iconTransformation(extent={{86,-10},{
                106,10}})));
      BloodFlowOutflow bloodFlowOutflow annotation (Placement(transformation(
              extent={{-100,-2},{-96,2}}), iconTransformation(extent={{-112,-10},
                {-92,10}})));
      ElasticCompartment intrathoracicArteries(
        elastance_NonSI=1.43,
        unstressedVolume_NonSI=140,
        externalPressure_NonSI=-4,
        V0_NonSI=196)
        annotation (Placement(transformation(extent={{82,-32},{102,-12}})));
      BloodResistor extrathoracicArterialResistance(bloodResistance_NonSI=0.06)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={50,0})));
      Inductor aorticFlowInertia(inertance_NonSI=0.0017) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={78,0})));
      BloodResistor systemicArteriolarResistance(bloodResistance_NonSI=0.8)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={10,0})));
      ElasticCompartment extrathoracicArteries(
        elastance_NonSI=0.556,
        unstressedVolume_NonSI=370,
        externalPressure_NonSI=0,
        V0_NonSI=503)
        annotation (Placement(transformation(extent={{18,-30},{38,-10}})));
      ElasticCompartment SystemicTissues(
        elastance_NonSI=0.262,
        unstressedVolume_NonSI=185,
        externalPressure_NonSI=0,
        V0_NonSI=274)
        annotation (Placement(transformation(extent={{-16,-30},{4,-10}})));
      BloodResistor smallVenuleResistance(bloodResistance_NonSI=0.2)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-24,0})));
      BloodResistor venousResistance(bloodResistance_NonSI=0.09) annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-52,0})));
      ElasticCompartment intrathoracicVeins(
        elastance_NonSI=0.0182,
        unstressedVolume_NonSI=1190,
        externalPressure_NonSI=-4,
        V0_NonSI=1542)
        annotation (Placement(transformation(extent={{-78,-30},{-58,-10}})));
      ElasticCompartment extrathoracicVeins(
        elastance_NonSI=0.0169,
        unstressedVolume_NonSI=1000,
        externalPressure_NonSI=0,
        V0_NonSI=1528)
        annotation (Placement(transformation(extent={{-48,-30},{-28,-10}})));
      BloodResistor centralVenousResistance(bloodResistance_NonSI=0.003)
        annotation (Placement(transformation(
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
          points={{68.4,6.66134e-16},{62,6.66134e-16},{62,0},{60.4,0}},
          color={238,46,47},
          thickness=1));
      connect(extrathoracicArterialResistance.bloodFlowOutflow,
        extrathoracicArteries.bloodFlowInflow) annotation (Line(
          points={{40.4,0},{28,0},{28,-20}},
          color={238,46,47},
          thickness=1));
      connect(extrathoracicArteries.bloodFlowInflow,
        systemicArteriolarResistance.bloodFlowInflow) annotation (Line(
          points={{28,-20},{28,-1.77636e-15},{20.4,-1.77636e-15}},
          color={255,0,0},
          thickness=1));
      connect(SystemicTissues.bloodFlowInflow, systemicArteriolarResistance.bloodFlowOutflow)
        annotation (Line(
          points={{-6,-20},{-6,6.66134e-16},{0.4,6.66134e-16}},
          color={238,46,47},
          thickness=1));
      connect(smallVenuleResistance.bloodFlowInflow, SystemicTissues.bloodFlowInflow)
        annotation (Line(
          points={{-13.6,-1.77636e-15},{-6,-1.77636e-15},{-6,-20}},
          color={28,108,200},
          thickness=1));
      connect(smallVenuleResistance.bloodFlowOutflow, extrathoracicVeins.bloodFlowInflow)
        annotation (Line(
          points={{-33.6,6.66134e-16},{-38,6.66134e-16},{-38,-20}},
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
          points={{-71.6,0},{-68,0},{-68,-20}},
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
      BloodFlowInflow bloodFlowInflow annotation (Placement(transformation(
              extent={{-62,-2},{-58,2}}), iconTransformation(extent={{-112,-10},
                {-92,10}})));
      BloodFlowOutflow bloodFlowOutflow annotation (Placement(transformation(
              extent={{78,-2},{82,2}}), iconTransformation(extent={{90,-10},{
                110,10}})));
      ElasticCompartment pulmonaryArteries(
        elastance_NonSI=0.233,
        unstressedVolume_NonSI=50,
        externalPressure_NonSI=-4,
        V0_NonSI=99)
        annotation (Placement(transformation(extent={{-50,-30},{-30,-10}})));
      BloodResistor pulmonaryResistance(bloodResistance_NonSI=0.11)
        annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
      ElasticCompartment pulmonaryVeins(
        elastance_NonSI=0.0455,
        unstressedVolume_NonSI=350,
        externalPressure_NonSI=-4,
        V0_NonSI=516)
        annotation (Placement(transformation(extent={{10,-30},{30,-10}})));
      BloodResistor pulmonaryVenousResistance(bloodResistance_NonSI=0.003)
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
      Heart heart
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
              textString="HR"),
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
  end Components;

  package Model
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

    model HemodynamicsMeurs_flatNorm
    extends Physiolibrary.Icons.CardioVascular;
      Physiolibrary.Hydraulic.Components.ElasticVesselElastance Epa(
        useV0Input=false,
        useExternalPressureInput=false,
        useComplianceInput=false,
        volume_start=0.000106,
        ZeroPressureVolume=5e-05,
        ExternalPressure=-533.28954966,
        Elastance=31064116.267695)
        annotation (Placement(transformation(extent={{-94,84},{-66,112}})));
      Physiolibrary.Hydraulic.Components.Resistor Rpp(useConductanceInput=false,
          Resistance(displayUnit="(mmHg.s)/ml") = 14665462.61565)
        annotation (Placement(transformation(extent={{-56,85},{-22,111}})));
      Physiolibrary.Hydraulic.Components.ElasticVesselElastance Epv(
        useV0Input=false,
        useExternalPressureInput=false,
        useComplianceInput=false,
        volume_start=0.000518,
        ZeroPressureVolume=0.00035,
        ExternalPressure=-533.28954966,
        Elastance=6066168.6273825)
        annotation (Placement(transformation(extent={{-10,84},{24,112}})));
      Physiolibrary.Hydraulic.Components.Resistor Rlain(useConductanceInput=false,
          Resistance(displayUnit="(mmHg.s)/ml") = 399967.162245)
        annotation (Placement(transformation(extent={{26,86},{56,110}})));
      Physiolibrary.Hydraulic.Components.ElasticVesselElastance LeftAtrium(
        useComplianceInput=true,
        useV0Input=false,
        useExternalPressureInput=false,
        volume_start=9.31e-05,
        ZeroPressureVolume=3e-05,
        ExternalPressure=-533.28954966)
        annotation (Placement(transformation(extent={{74,50},{102,78}})));
      Physiolibrary.Hydraulic.Components.ElasticVesselElastance LeftVentricle(
        useComplianceInput=true,
        useV0Input=false,
        useExternalPressureInput=false,
        volume_start=0.000144,
        ZeroPressureVolume=6e-05,
        ExternalPressure=-533.28954966)
        annotation (Placement(transformation(extent={{150,50},{178,78}})));
      Physiolibrary.Hydraulic.Components.IdealValveResistance AorticValve(
        _Goff(displayUnit="ml/(mmHg.s)") = 0,
        _Ron(displayUnit= "(mmHg.min)/ml") = 1066579.09932, chatteringProtectionTime = 0.01, lastChange(displayUnit = "s"), useChatteringProtection = true,
        useLimitationInputs=false)
        annotation (Placement(transformation(extent={{184,76},{208,52}})));
      Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.AtrialElastance LAtrialElastance(
        Tav(displayUnit="s"),
        EMIN=15998686.4898,
        EMAX=37330268.4762)
        annotation (Placement(transformation(extent={{80,92},{118,124}})));
      Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.VentricularElastance
        LVentricularElastance(EMIN=11999014.86735, EMAX=533289549.66)
        annotation (Placement(transformation(extent={{164,88},{200,120}})));
      Physiolibrary.Hydraulic.Components.IdealValveResistance MitralValve(
        _Goff(displayUnit="ml/(mmHg.s)") = 0,
        _Ron(displayUnit= "(mmHg.min)/ml") = 399967.162245, chatteringProtectionTime = 0.01, lastChange(displayUnit = "s"), useChatteringProtection = true,useLimitationInputs=false)
                            annotation (Placement(transformation(
            origin={127,64},
            extent={{-13,12},{13,-12}})));

      Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eitha(
        useV0Input=false,
        useExternalPressureInput=false,
        useComplianceInput=false,
        volume_start=0.000204,
        ZeroPressureVolume=0.00014,
        ExternalPressure=-533.28954966,
        Elastance=190651014.00345)
        annotation (Placement(transformation(extent={{168,6},{190,28}})));
      Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eetha(
        volume_start(displayUnit="ml") = 0.000526,
        useV0Input=false,
        useExternalPressureInput=false,
        useComplianceInput=false,
        ZeroPressureVolume=0.00037,
        Elastance=74127247.40274)
        annotation (Placement(transformation(extent={{56,4},{82,30}})));
      Physiolibrary.Hydraulic.Components.Inertia inertia(I(displayUnit=
              "mmHg.s2/ml") = 226648.0586055,
          volumeFlow_start(displayUnit="ml/min") = 2.1666666666667e-05)                                                                                                  annotation(Placement(transformation(extent={{-11,-11},
                {11,11}},                                                                                                    rotation = 180, origin={141,17})));
      Physiolibrary.Hydraulic.Components.Resistor Retha(useConductanceInput=false,
          Resistance(displayUnit="(mmHg.s)/ml") = 7999343.2449)
        annotation (Placement(transformation(extent={{90,6},{112,28}})));
      Physiolibrary.Hydraulic.Components.Resistor Rsart(useConductanceInput=false,
          Resistance(displayUnit="(mmHg.s)/ml") = 106657909.932) annotation (
          Placement(transformation(
            extent={{14,-13},{-14,13}},
            origin={24,17})));
      Physiolibrary.Hydraulic.Components.Resistor Rsven(useConductanceInput=false,
          Resistance(displayUnit="(mmHg.s)/ml") = 26664477.483) annotation (
          Placement(transformation(
            extent={{14,-13},{-14,13}},
            origin={-60,17})));
      Physiolibrary.Hydraulic.Components.ElasticVesselElastance Est(
        useV0Input=false,
        useExternalPressureInput=false,
        useComplianceInput=false,
        volume_start=0.000283,
        ZeroPressureVolume=0.000185,
        Elastance=34930465.50273)
        annotation (Placement(transformation(extent={{-28,6},{-4,28}})));
      Physiolibrary.Hydraulic.Components.Resistor Rethv(useConductanceInput=false,
          Resistance(displayUnit="(mmHg.s)/ml") = 11999014.86735)
        annotation (Placement(transformation(extent={{-120,4},{-146,30}})));
      Physiolibrary.Hydraulic.Components.Resistor Rrain(useConductanceInput=false,
          Resistance(displayUnit="(mmHg.s)/ml") = 399967.162245)
        annotation (Placement(transformation(extent={{-208,4},{-236,30}})));
      Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eithv(
        useV0Input=false,
        useExternalPressureInput=false,
        useComplianceInput=false,
        volume_start=0.00148,
        ZeroPressureVolume=0.00119,
        ExternalPressure=-533.28954966,
        Elastance=2426467.450953)
        annotation (Placement(transformation(extent={{-194,4},{-166,30}})));
      Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eethv(
        useV0Input=false,
        useExternalPressureInput=false,
        useComplianceInput=false,
        volume_start=0.00153,
        ZeroPressureVolume=0.001,
        Elastance=2253148.3473135)
        annotation (Placement(transformation(extent={{-108,4},{-82,30}})));
      Physiolibrary.Hydraulic.Components.ElasticVesselElastance RightAtrium(
        useComplianceInput=true,
        useV0Input=false,
        useExternalPressureInput=false,
        volume_start=0.000135,
        ZeroPressureVolume=3e-05,
        ExternalPressure=-533.28954966)
        annotation (Placement(transformation(extent={{-242,44},{-214,72}})));
      Physiolibrary.Hydraulic.Components.ElasticVesselElastance RightVentricle(
        useComplianceInput=true,
        useV0Input=false,
        useExternalPressureInput=false,
        volume_start=0.000131,
        ZeroPressureVolume=4e-05,
        ExternalPressure=-533.28954966)
        annotation (Placement(transformation(extent={{-170,42},{-140,72}})));
      Physiolibrary.Hydraulic.Components.IdealValveResistance PulmonaryValve(
        Pknee = Modelica.Constants.eps,_Goff(displayUnit="ml/(mmHg.s)") = 0,
        _Ron(displayUnit= "(mmHg.min)/ml") = 399967.162245, chatteringProtectionTime = 0.01, lastChange(displayUnit = "s"), useChatteringProtection = true,
        useLimitationInputs=false)
        annotation (Placement(transformation(extent={{-132,70},{-106,44}})));
      Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.AtrialElastance RAtrialElastance(EMIN=
            6666119.37075, EMAX=19998358.11225)
        annotation (Placement(transformation(extent={{-244,86},{-206,118}})));
      Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.VentricularElastance
        RVentricularElastance(EMIN=7599376.082655, EMAX=65327969.83335)
        annotation (Placement(transformation(extent={{-180,88},{-150,122}})));
      Physiolibrary.Hydraulic.Components.IdealValveResistance TricuspidValve(
        _Goff=0,
        _Ron(displayUnit= "(mmHg.min)/ml") = 399967.162245, chatteringProtectionTime = 0.01, lastChange(displayUnit = "s"), open(fixed = true, start = true), useChatteringProtection = true,
        useLimitationInputs=false)
                            annotation (Placement(transformation(
            origin={-189,58},
            extent={{-13,12},{13,-12}})));
      replaceable Physiolibrary.Types.Constants.FrequencyConst HeartRate(k(displayUnit = "1/min") = 1.2) annotation(Placement(transformation(origin = {-243, 128.5}, extent = {{-11, -6.5}, {11, 6.5}})));
    equation
      connect(Epa.q_in, Rpp.q_in) annotation (Line(
          points={{-80,98},{-56,98}},
          thickness=1));
      connect(Rpp.q_out, Epv.q_in) annotation (Line(
          points={{-22,98},{7,98}},
          thickness=1));
      connect(Epv.q_in, Rlain.q_in) annotation (Line(
          points={{7,98},{26,98}},
          thickness=1));
      connect(LeftAtrium.q_in, MitralValve.q_in) annotation (Line(
          points={{88,64},{114,64}},
          thickness=1));
      connect(LeftVentricle.q_in, MitralValve.q_out) annotation (Line(
          points={{164,64},{140,64}},
          thickness=1));
      connect(LeftVentricle.q_in, AorticValve.q_in) annotation (Line(
          points={{164,64},{184,64}},
          thickness=1));
      connect(LeftVentricle.compliance, LVentricularElastance.Ct) annotation (
         Line(
          points={{164,75.2},{164,74},{212,74},{212,107.68},{203.42,107.68}},
          color={0,0,127}));
      connect(Rlain.q_out, LeftAtrium.q_in) annotation (Line(
          points={{56,98},{74,98},{74,64},{88,64}},
          thickness=1));
      connect(Retha.q_in, Eetha.q_in) annotation (Line(
          points={{90,17},{69,17}},
          thickness=1));
      connect(Retha.q_out, inertia.q_out) annotation (Line(
          points={{112,17},{130,17}},
          thickness=1));
      connect(inertia.q_in, Eitha.q_in) annotation (Line(
          points={{152,17},{179,17}},
          thickness=1));
      connect(Eitha.q_in, AorticValve.q_out) annotation (Line(
          points={{179,17},{216,17},{216,64},{208,64}},
          thickness=1));
      connect(Rrain.q_in, Eithv.q_in) annotation (Line(
          points={{-208,17},{-180,17}},
          thickness=1));
      connect(Eithv.q_in, Rethv.q_out) annotation (Line(
          points={{-180,17},{-146,17}},
          thickness=1));
      connect(Rethv.q_in, Eethv.q_in) annotation (Line(
          points={{-120,17},{-95,17}},
          thickness=1));
      connect(RightAtrium.q_in, TricuspidValve.q_in) annotation (Line(
          points={{-228,58},{-202,58}},
          thickness=1));
      connect(RightVentricle.q_in, TricuspidValve.q_out) annotation (Line(
          points={{-155,57},{-164.5,57},{-164.5,58},{-176,58}},
          thickness=1));
      connect(RightVentricle.q_in, PulmonaryValve.q_in) annotation (Line(
          points={{-155,57},{-132,57}},
          thickness=1));
      connect(Rrain.q_out, RightAtrium.q_in) annotation (Line(
          points={{-236,17},{-250,17},{-250,58},{-228,58}},
          thickness=1));
      connect(RightAtrium.compliance,RAtrialElastance. Ct) annotation(Line(points = {{-228, 69.2}, {-228, 92}, {-202.39, 92}, {-202.39, 101.84}}, color = {0, 0, 127}));
      connect(PulmonaryValve.q_out, Epa.q_in) annotation (Line(
          points={{-106,57},{-92,57},{-92,98},{-80,98}},
          thickness=1));
      connect(RightVentricle.compliance,RVentricularElastance. Ct) annotation(Line(points={{-155,69},
              {-155,80},{-126,80},{-126,108.91},{-147.15,108.91}},                                                                                                   color = {0, 0, 127}));
      connect(LeftAtrium.compliance, LAtrialElastance.Ct) annotation (Line(
          points={{88,75.2},{88,74},{121.61,74},{121.61,107.84}},
          color={0,0,127}));
      connect(HeartRate.y,RAtrialElastance. HR) annotation(Line(points = {{-229.25, 128.5}, {-225, 128.5}, {-225, 114.8}}, color = {0, 0, 127}));
      connect(RVentricularElastance.HR, HeartRate.y) annotation(Line(points = {{-165, 118.6}, {-165, 128.5}, {-229.25, 128.5}}, color = {0, 0, 127}));
      connect(LAtrialElastance.HR, HeartRate.y) annotation (Line(
          points={{99,120.8},{99,128.5},{-229.25,128.5}},
          color={0,0,127}));
      connect(LVentricularElastance.HR, HeartRate.y) annotation (Line(
          points={{182,116.8},{182,128.5},{-229.25,128.5}},
          color={0,0,127}));
      connect(Est.q_in, Rsart.q_out) annotation (Line(
          points={{-16,17},{10,17}},
          thickness=1));
      connect(Rsart.q_in, Eetha.q_in) annotation (Line(
          points={{38,17},{69,17}},
          thickness=1));
      connect(Eethv.q_in, Rsven.q_out) annotation (Line(
          points={{-95,17},{-74,17}},
          thickness=1));
      connect(Rsven.q_in, Est.q_in) annotation (Line(
          points={{-46,17},{-16,17}},
          thickness=1));
      annotation(Diagram(coordinateSystem(extent={{-280,-140},{280,180}},      preserveAspectRatio=false),   graphics), Icon(coordinateSystem(extent = {{-280, -140}, {280, 180}}, preserveAspectRatio = false), graphics),
        Documentation(info="<html>
<p>Model of cardiovascular system using to demonstrate elastic and resistance features of veins and arteries in pulmonary and systemic circulation and influence of cardiac output on it.</p>
<ul>
<li>J. A. Goodwin, W. L. van Meurs, C. D. Sa Couto, J. E. W.Beneken, S. A. Graves, A model for educational simulation of infant cardiovascular physiology., Anesthesia and analgesia 99 (6)(2004) 1655&ndash;1664. doi:10.1213/01.ANE.0000134797.52793.AF.</li>
<li>C. D. Sa Couto, W. L. van Meurs, J. A. Goodwin, P. Andriessen,A Model for Educational Simulation of Neonatal Cardiovascular Pathophysiology, Simulation in Healthcare 1 (Inaugural) (2006) 4&ndash;12.</li>
<li>W. van Meurs, Modeling and Simulation in Biomedical Engineering: Applications in Cardiorespiratory Physiology, McGraw-Hill Professional, 2011.</li>
</ul>
</html>",     revisions="<html>
<ul>
<li><i>Jul 2015 </i>by Tomas Kulhanek: Created. </li>
</ul>
</html>"),
        experiment(StopTime=5));
    end HemodynamicsMeurs_flatNorm;
  end Model;

  package Types
    type Pressure =  Modelica.Units.SI.Pressure(displayUnit="mmHg", nominal=133.322387415);
    type Volume =  Modelica.Units.SI.Volume (
           displayUnit="ml", nominal=1e-6, min=0);
    type VolumeFlowRate =
        Modelica.Units.SI.VolumeFlowRate (displayUnit="ml/min", nominal=(1e-6)/60);
    type HydraulicElastance = Real(final quantity="HydraulicElastance",final unit="Pa/m3", displayUnit="mmHg/ml", nominal=(133.322387415)/(1e-6));
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
  end Types;

  package MeursHemodynamicsPhysiolibrary
    package Components
      model HeartPhysiolibrary
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a RightAtriumInflow
          annotation (Placement(transformation(extent={{-94,-80},{-74,-60}}),
              iconTransformation(extent={{-68,-30},{-48,-10}})));
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_b
          pulmonaryArteryOutflow annotation (Placement(transformation(extent={{82,-86},
                  {102,-66}}),      iconTransformation(extent={{-68,20},{-48,40}})));
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a leftAtriumInflow
          annotation (Placement(transformation(extent={{-96,-6},{-76,14}}),
              iconTransformation(extent={{56,14},{76,34}})));
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_b AortaOutflow
          annotation (Placement(transformation(extent={{86,-4},{106,16}}),
              iconTransformation(extent={{56,-34},{76,-14}})));
        MeursHemodynamics.Components.CardiacElastance leftCardiacElastance(
          atrialElmin=0.12,
          atrialElmax=0.28,
          ventricularElmin=0.09,
          ventricularElmax=4)
          annotation (Placement(transformation(extent={{-58,62},{-38,82}})));
        MeursHemodynamics.Components.CardiacElastance RightCardiacElastance(
          atrialElmin=0.05,
          atrialElmax=0.15,
          ventricularElmin=0.057,
          ventricularElmax=0.49)
          annotation (Placement(transformation(extent={{-56,-42},{-36,-22}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance TricuspidValve(
          _Goff=0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245)
                              annotation (Placement(transformation(
              origin={-11,-76},
              extent={{-13,12},{13,-12}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance PulmonaryValve(
          _Goff(displayUnit="ml/(mmHg.s)") = 0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{50,-62},{76,-88}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance LeftAtriumPhysiolibrary(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=9.3e-05,
          ZeroPressureVolume=3e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{-50,-10},{-22,18}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance RightVentricle(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.00015,
          ZeroPressureVolume=4e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{14,-92},{44,-62}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance RightAtriumPhysiolibrary(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.000156,
          ZeroPressureVolume=3e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{-60,-88},{-32,-60}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance LeftVentricle(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.000143,
          ZeroPressureVolume=6e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{18,-10},{48,20}})));
        Physiolibrary.Blocks.Math.Reciprocal rec
          annotation (Placement(transformation(extent={{-54,20},{-46,28}})));
        Physiolibrary.Blocks.Math.Reciprocal rec1
          annotation (Placement(transformation(extent={{-60,-58},{-52,-50}})));
        Physiolibrary.Blocks.Math.Reciprocal rec2
          annotation (Placement(transformation(extent={{14,26},{22,34}})));
        Physiolibrary.Blocks.Math.Reciprocal rec3
          annotation (Placement(transformation(extent={{2,-58},{10,-50}})));
        Modelica.Blocks.Sources.Constant heartRate(k=72)
          annotation (Placement(transformation(extent={{-96,44},{-82,58}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance mitralValve(
          _Goff=0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245) annotation (
            Placement(transformation(origin={-5,4}, extent={{-13,12},{13,-12}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance AorticValve(
          _Goff(displayUnit="ml/(mmHg.s)") = 0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{56,18},{82,-8}})));
      equation
        connect(LeftAtriumPhysiolibrary.compliance, rec.y) annotation (Line(
              points={{-36,15.2},{-36,24},{-45.6,24}},
                                                     color={0,0,127}));
        connect(RightAtriumPhysiolibrary.compliance, rec1.y) annotation (Line(
              points={{-46,-62.8},{-46,-54},{-51.6,-54}}, color={0,0,127}));
        connect(AortaOutflow, AorticValve.q_out) annotation (Line(
            points={{96,6},{84,6},{84,5},{82,5}},
            color={0,0,0},
            thickness=1));
        connect(heartRate.y, leftCardiacElastance.HR) annotation (Line(points={
                {-81.3,51},{-77.65,51},{-77.65,72},{-59,72}}, color={0,0,127}));
        connect(LeftAtriumPhysiolibrary.q_in, leftAtriumInflow) annotation (
            Line(
            points={{-36,4},{-86,4}},
            color={0,0,0},
            thickness=1));
        connect(LeftAtriumPhysiolibrary.q_in, mitralValve.q_in) annotation (
            Line(
            points={{-36,4},{-18,4}},
            color={0,0,0},
            thickness=1));
        connect(mitralValve.q_out, LeftVentricle.q_in) annotation (Line(
            points={{8,4},{22,4},{22,5},{33,5}},
            color={0,0,0},
            thickness=1));
        connect(LeftVentricle.q_in, AorticValve.q_in) annotation (Line(
            points={{33,5},{44.5,5},{44.5,5},{56,5}},
            color={0,0,0},
            thickness=1));
        connect(rec2.y, LeftVentricle.compliance) annotation (Line(points={{22.4,30},
                {33,30},{33,17}},          color={0,0,127}));
        connect(leftCardiacElastance.Eta, rec.u) annotation (Line(points={{-37,
                68.4},{-36,68.4},{-36,40},{-64,40},{-64,24},{-54.8,24}}, color=
                {0,0,127}));
        connect(leftCardiacElastance.Etv, rec2.u) annotation (Line(points={{-37,
                78.2},{-2,78.2},{-2,30},{13.2,30}}, color={0,0,127}));
        connect(RightCardiacElastance.HR, leftCardiacElastance.HR) annotation (
            Line(points={{-57,-32},{-78,-32},{-78,51},{-77.65,51},{-77.65,72},{
                -59,72}}, color={0,0,127}));
        connect(rec1.u, RightCardiacElastance.Eta) annotation (Line(points={{-60.8,
                -54},{-68,-54},{-68,-50},{-26,-50},{-26,-35.6},{-35,-35.6}},
              color={0,0,127}));
        connect(RightCardiacElastance.Etv, rec3.u) annotation (Line(points={{-35,
                -25.8},{-8,-25.8},{-8,-54},{1.2,-54}},   color={0,0,127}));
        connect(rec3.y, RightVentricle.compliance) annotation (Line(points={{10.4,
                -54},{29,-54},{29,-65}},      color={0,0,127}));
        connect(RightAtriumPhysiolibrary.q_in, TricuspidValve.q_in) annotation (
           Line(
            points={{-46,-74},{-36,-74},{-36,-76},{-24,-76}},
            color={0,0,0},
            thickness=1));
        connect(RightAtriumInflow, RightAtriumPhysiolibrary.q_in) annotation (
            Line(
            points={{-84,-70},{-66,-70},{-66,-74},{-46,-74}},
            color={0,0,0},
            thickness=1));
        connect(TricuspidValve.q_out, RightVentricle.q_in) annotation (Line(
            points={{2,-76},{16,-76},{16,-77},{29,-77}},
            color={0,0,0},
            thickness=1));
        connect(RightVentricle.q_in, PulmonaryValve.q_in) annotation (Line(
            points={{29,-77},{39.5,-77},{39.5,-75},{50,-75}},
            color={0,0,0},
            thickness=1));
        connect(PulmonaryValve.q_out, pulmonaryArteryOutflow) annotation (Line(
            points={{76,-75},{84,-75},{84,-76},{92,-76}},
            color={0,0,0},
            thickness=1));
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
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance
          pulmonaryVeins(
          volume_start=0.000516,
          ZeroPressureVolume=0.00035,
          CollapsingPressureVolume=1e-12,
          ExternalPressure=-533.28954966,
          Elastance=6066168.6273825)
          annotation (Placement(transformation(extent={{-6,-34},{14,-14}})));
        Physiolibrary.Hydraulic.Components.Resistor pulmonaryResistance(
            Resistance(displayUnit="(mmHg.s)/ml") = 14665462.61565) annotation (
           Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-26,0})));
        Physiolibrary.Hydraulic.Components.Resistor pulmonaryVenousResistance(
            Resistance(displayUnit="(mmHg.s)/ml") = 399967.162245) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={50,0})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance
          pulmonaryArteries(
          volume_start=9.9e-05,
          ZeroPressureVolume=5e-05,
          CollapsingPressureVolume=1e-12,
          ExternalPressure=-533.28954966,
          Elastance=31064116.267695)
          annotation (Placement(transformation(extent={{-84,-34},{-64,-14}})));
      equation
        connect(pulmonaryBloodInflow, pulmonaryResistance.q_in) annotation (
            Line(
            points={{-94,0},{-36,0}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryResistance.q_out, pulmonaryVenousResistance.q_in)
          annotation (Line(
            points={{-16,2.22045e-16},{14,2.22045e-16},{14,0},{40,0},{40,
                2.22045e-16}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryVenousResistance.q_out, pulmonaryBloodOutflow)
          annotation (Line(
            points={{60,2.22045e-16},{76,2.22045e-16},{76,0},{92,0}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryVeins.q_in, pulmonaryVenousResistance.q_in)
          annotation (Line(
            points={{4,-24},{2,-24},{2,0},{0,0},{0,2.22045e-16},{14,2.22045e-16},
                {14,0},{40,0}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryArteries.q_in, pulmonaryResistance.q_in) annotation (
            Line(
            points={{-74,-24},{-74,0},{-36,0}},
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
          volume_start=0.000196,
          ZeroPressureVolume=0.00014,
          CollapsingPressureVolume=1e-12,
          ExternalPressure=-533.28954966,
          Elastance=190651014.00345)
          annotation (Placement(transformation(extent={{80,-40},{100,-20}})));
        Physiolibrary.Hydraulic.Components.Resistor
          extrathoracicArterialResistance(Resistance(displayUnit="(mmHg.s)/ml")=
               7999343.2449) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={48,0})));
        Physiolibrary.Hydraulic.Components.Inertia aorticFlowInertia(
            volumeFlow_start(displayUnit="m3/s"), I(displayUnit="mmHg.s2/ml")=
               226648.0586055)
                              annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={74,0})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance
          extrathoracicArteries(
          volume_start=0.000503,
          ZeroPressureVolume=0.00037,
          ExternalPressure=0,
          Elastance=74127247.40274)
          annotation (Placement(transformation(extent={{30,-34},{50,-14}})));
        Physiolibrary.Hydraulic.Components.Resistor
          systemicArteriolarResistance(Resistance(displayUnit="(mmHg.s)/ml")=
            106657909.932) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={16,0})));
        Physiolibrary.Hydraulic.Components.Resistor smallVenuleResistance(
            Resistance(displayUnit="(mmHg.s)/ml") = 26664477.483) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-14,0})));
        Physiolibrary.Hydraulic.Components.Resistor venousResistance(Resistance(
              displayUnit="(mmHg.s)/ml") = 11999014.86735) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-48,0})));
        Physiolibrary.Hydraulic.Components.Resistor centralVenousResistance(
            Resistance(displayUnit="(mmHg.s)/ml") = 399967.162245) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-78,0})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance
          SystemicTissues(
          volume_start=0.000274,
          ZeroPressureVolume=0.000185,
          CollapsingPressureVolume=1e-12,
          ExternalPressure=0,
          Elastance=34930465.50273)
          annotation (Placement(transformation(extent={{-8,-36},{12,-16}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance
          extrathoracicVeins(
          volume_start=0.001528,
          ZeroPressureVolume=0.001,
          CollapsingPressureVolume=1e-12,
          ExternalPressure=0,
          Elastance=2253148.3473135)
          annotation (Placement(transformation(extent={{-40,-34},{-20,-14}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance
          intrathoracicVeins(
          volume_start=0.001542,
          ZeroPressureVolume=0.00119,
          CollapsingPressureVolume=1e-12,
          ExternalPressure=-533.28954966,
          Elastance=2426467.450953)
          annotation (Placement(transformation(extent={{-74,-34},{-54,-14}})));
      equation
        connect(intrathoracicArteries.q_in, systemicBloodInflow) annotation (
            Line(
            points={{90,-30},{90,0},{100,0}},
            color={0,0,0},
            thickness=1));
        connect(aorticFlowInertia.q_in, systemicBloodInflow) annotation (Line(
            points={{84,-1.77636e-15},{96,-1.77636e-15},{96,0},{100,0}},
            color={0,0,0},
            thickness=1));
        connect(aorticFlowInertia.q_out, extrathoracicArterialResistance.q_in)
          annotation (Line(
            points={{64,4.44089e-16},{64,-1.77636e-15},{58,-1.77636e-15}},
            color={0,0,0},
            thickness=1));
        connect(extrathoracicArterialResistance.q_out,
          systemicArteriolarResistance.q_in) annotation (Line(
            points={{38,4.44089e-16},{34,4.44089e-16},{34,0},{26,0}},
            color={0,0,0},
            thickness=1));
        connect(systemicArteriolarResistance.q_out, smallVenuleResistance.q_in)
          annotation (Line(
            points={{6,0},{-4,0}},
            color={0,0,0},
            thickness=1));
        connect(smallVenuleResistance.q_out, venousResistance.q_in) annotation (
           Line(
            points={{-24,0},{-38,0}},
            color={0,0,0},
            thickness=1));
        connect(venousResistance.q_out, centralVenousResistance.q_in)
          annotation (Line(
            points={{-58,0},{-68,0}},
            color={0,0,0},
            thickness=1));
        connect(centralVenousResistance.q_out, systemicBloodOutflow)
          annotation (Line(
            points={{-88,4.44089e-16},{-92,4.44089e-16},{-92,0},{-100,0}},
            color={0,0,0},
            thickness=1));
        connect(extrathoracicArteries.q_in, systemicArteriolarResistance.q_in)
          annotation (Line(
            points={{40,-24},{40,0},{26,0},{26,-1.66533e-15}},
            color={0,0,0},
            thickness=1));
        connect(SystemicTissues.q_in, smallVenuleResistance.q_in) annotation (
            Line(
            points={{2,-26},{2,0},{-4,0},{-4,-1.66533e-15}},
            color={0,0,0},
            thickness=1));
        connect(extrathoracicVeins.q_in, venousResistance.q_in) annotation (
            Line(
            points={{-30,-24},{-30,0},{-38,0},{-38,-1.66533e-15}},
            color={0,0,0},
            thickness=1));
        connect(intrathoracicVeins.q_in, centralVenousResistance.q_in)
          annotation (Line(
            points={{-64,-24},{-64,0},{-68,0},{-68,-1.66533e-15}},
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

      model HeartPhysiolibraryElastance
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a RightAtriumInflow
          annotation (Placement(transformation(extent={{-94,-80},{-74,-60}}),
              iconTransformation(extent={{-68,-30},{-48,-10}})));
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_b
          pulmonaryArteryOutflow annotation (Placement(transformation(extent={{82,-86},
                  {102,-66}}),      iconTransformation(extent={{-68,20},{-48,40}})));
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_a leftAtriumInflow
          annotation (Placement(transformation(extent={{-96,-6},{-76,14}}),
              iconTransformation(extent={{56,14},{76,34}})));
        Physiolibrary.Hydraulic.Interfaces.HydraulicPort_b AortaOutflow
          annotation (Placement(transformation(extent={{86,-4},{106,16}}),
              iconTransformation(extent={{56,-34},{76,-14}})));
        MeursHemodynamics.Components.CardiacElastance leftCardiacElastance(
          atrialElmin=0.12,
          atrialElmax=0.28,
          ventricularElmin=0.09,
          ventricularElmax=4)
          annotation (Placement(transformation(extent={{-58,62},{-38,82}})));
        MeursHemodynamics.Components.CardiacElastance RightCardiacElastance(
          atrialElmin=0.05,
          atrialElmax=0.15,
          ventricularElmin=0.057,
          ventricularElmax=0.49)
          annotation (Placement(transformation(extent={{-56,-42},{-36,-22}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance TricuspidValve(
          _Goff=0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245)
                              annotation (Placement(transformation(
              origin={-11,-76},
              extent={{-13,12},{13,-12}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance PulmonaryValve(
          _Goff(displayUnit="ml/(mmHg.s)") = 0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{50,-62},{76,-88}})));
        Physiolibrary.Hydraulic.Components.ElasticVessel LeftAtriumPhysiolibrary(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=9.3e-05,
          ZeroPressureVolume=3e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{-50,-10},{-22,18}})));
        Physiolibrary.Hydraulic.Components.ElasticVessel RightVentricle(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.00015,
          ZeroPressureVolume=4e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{14,-92},{44,-62}})));
        Physiolibrary.Hydraulic.Components.ElasticVessel RightAtriumPhysiolibrary(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.000156,
          ZeroPressureVolume=3e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{-60,-88},{-32,-60}})));
        Physiolibrary.Hydraulic.Components.ElasticVessel LeftVentricle(
          useComplianceInput=true,
          useV0Input=false,
          useExternalPressureInput=false,
          volume_start=0.000143,
          ZeroPressureVolume=6e-05,
          ExternalPressure=-533.28954966)
          annotation (Placement(transformation(extent={{18,-10},{48,20}})));
        Physiolibrary.Blocks.Math.Reciprocal rec
          annotation (Placement(transformation(extent={{-54,20},{-46,28}})));
        Physiolibrary.Blocks.Math.Reciprocal rec1
          annotation (Placement(transformation(extent={{-60,-58},{-52,-50}})));
        Physiolibrary.Blocks.Math.Reciprocal rec2
          annotation (Placement(transformation(extent={{14,26},{22,34}})));
        Physiolibrary.Blocks.Math.Reciprocal rec3
          annotation (Placement(transformation(extent={{2,-58},{10,-50}})));
        Modelica.Blocks.Sources.Constant heartRate(k=72)
          annotation (Placement(transformation(extent={{-96,44},{-82,58}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance mitralValve(
          _Goff=0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245) annotation (
            Placement(transformation(origin={-5,4}, extent={{-13,12},{13,-12}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance AorticValve(
          _Goff(displayUnit="ml/(mmHg.s)") = 0,
          useLimitationInputs=false,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{56,18},{82,-8}})));
      equation
        connect(RightAtriumPhysiolibrary.compliance, rec1.y) annotation (Line(
              points={{-46,-62.8},{-46,-54},{-51.6,-54}}, color={0,0,127}));
        connect(AortaOutflow, AorticValve.q_out) annotation (Line(
            points={{96,6},{84,6},{84,5},{82,5}},
            color={0,0,0},
            thickness=1));
        connect(heartRate.y, leftCardiacElastance.HR) annotation (Line(points={
                {-81.3,51},{-77.65,51},{-77.65,72},{-59,72}}, color={0,0,127}));
        connect(LeftAtriumPhysiolibrary.q_in, leftAtriumInflow) annotation (
            Line(
            points={{-36,4},{-86,4}},
            color={0,0,0},
            thickness=1));
        connect(LeftAtriumPhysiolibrary.q_in, mitralValve.q_in) annotation (
            Line(
            points={{-36,4},{-18,4}},
            color={0,0,0},
            thickness=1));
        connect(mitralValve.q_out, LeftVentricle.q_in) annotation (Line(
            points={{8,4},{22,4},{22,5},{33,5}},
            color={0,0,0},
            thickness=1));
        connect(LeftVentricle.q_in, AorticValve.q_in) annotation (Line(
            points={{33,5},{44.5,5},{44.5,5},{56,5}},
            color={0,0,0},
            thickness=1));
        connect(rec2.y, LeftVentricle.compliance) annotation (Line(points={{22.4,30},
                {33,30},{33,17}},          color={0,0,127}));
        connect(leftCardiacElastance.Eta, rec.u) annotation (Line(points={{-37,
                68.4},{-26,68.4},{-26,40},{-64,40},{-64,24},{-54.8,24}}, color=
                {0,0,127}));
        connect(leftCardiacElastance.Etv, rec2.u) annotation (Line(points={{-37,
                78.2},{-2,78.2},{-2,30},{13.2,30}}, color={0,0,127}));
        connect(RightCardiacElastance.HR, leftCardiacElastance.HR) annotation (
            Line(points={{-57,-32},{-78,-32},{-78,51},{-77.65,51},{-77.65,72},{
                -59,72}}, color={0,0,127}));
        connect(rec1.u, RightCardiacElastance.Eta) annotation (Line(points={{-60.8,
                -54},{-68,-54},{-68,-50},{-26,-50},{-26,-35.6},{-35,-35.6}},
              color={0,0,127}));
        connect(RightCardiacElastance.Etv, rec3.u) annotation (Line(points={{-35,
                -25.8},{-8,-25.8},{-8,-54},{1.2,-54}},   color={0,0,127}));
        connect(rec3.y, RightVentricle.compliance) annotation (Line(points={{10.4,
                -54},{29,-54},{29,-65}},      color={0,0,127}));
        connect(RightAtriumPhysiolibrary.q_in, TricuspidValve.q_in) annotation (
           Line(
            points={{-46,-74},{-36,-74},{-36,-76},{-24,-76}},
            color={0,0,0},
            thickness=1));
        connect(RightAtriumInflow, RightAtriumPhysiolibrary.q_in) annotation (
            Line(
            points={{-84,-70},{-66,-70},{-66,-74},{-46,-74}},
            color={0,0,0},
            thickness=1));
        connect(TricuspidValve.q_out, RightVentricle.q_in) annotation (Line(
            points={{2,-76},{16,-76},{16,-77},{29,-77}},
            color={0,0,0},
            thickness=1));
        connect(RightVentricle.q_in, PulmonaryValve.q_in) annotation (Line(
            points={{29,-77},{39.5,-77},{39.5,-75},{50,-75}},
            color={0,0,0},
            thickness=1));
        connect(PulmonaryValve.q_out, pulmonaryArteryOutflow) annotation (Line(
            points={{76,-75},{84,-75},{84,-76},{92,-76}},
            color={0,0,0},
            thickness=1));
        connect(LeftAtriumPhysiolibrary.compliance, rec.y) annotation (Line(
              points={{-36,15.2},{-36,24},{-45.6,24}}, color={0,0,127}));
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
      end HeartPhysiolibraryElastance;
    end Components;

    package Model
      model MeursModel
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
            points={{-60,-51},{-76,-51},{-76,-52},{-88,-52},{-88,-1.2},{-26.78,
                -1.2}},
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
          experiment(StopTime=20, __Dymola_Algorithm="Dassl"));
      end MeursModel;

      model vanMeursHemodynamicsModel
        MeursHemodynamics.Components.Heart heart
          annotation (Placement(transformation(extent={{-22,-22},{20,24}})));
        MeursHemodynamics.Components.SystemicCirculation systemicCirculation
          annotation (Placement(transformation(extent={{-30,-84},{30,-24}})));
        MeursHemodynamics.Components.PulmonaryCirculation pulmonaryCirculation
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

      model MeursModel_elastanceHeart
        Components.PulmonaryCirculation pulmonaryCirculation
          annotation (Placement(transformation(extent={{-62,24},{60,116}})));
        Components.SystemicCirculation systemicCirculation
          annotation (Placement(transformation(extent={{-60,-104},{60,2}})));
        MeursHemodynamics.Components.Heart heart
          annotation (Placement(transformation(extent={{-16,2},{4,22}})));
      equation
        connect(heart.rightAtriumFlowInflow, systemicCirculation.systemicBloodOutflow)
          annotation (Line(points={{-13.2,13},{-76,13},{-76,-51},{-60,-51}},
              color={0,0,0}));
        connect(heart.aortaOutflow, systemicCirculation.systemicBloodInflow)
          annotation (Line(points={{1.8,13},{86,13},{86,-51},{60,-51}}, color={
                0,0,0}));
        connect(heart.pulmonaryArteryOutflow, pulmonaryCirculation.pulmonaryBloodInflow)
          annotation (Line(points={{-9.2,16.6},{-76,16.6},{-76,70},{-62,70}},
              color={0,0,0}));
        connect(pulmonaryCirculation.pulmonaryBloodOutflow, heart.leftAtriumFlowInflow)
          annotation (Line(
            points={{60,70.92},{82,70.92},{82,16},{-2.6,16},{-2.6,16.6}},
            color={0,0,0},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=20, __Dymola_Algorithm="Dassl"));
      end MeursModel_elastanceHeart;

      model MeursModel_nonPhysiolobraryHeart
        Components.PulmonaryCirculation pulmonaryCirculation
          annotation (Placement(transformation(extent={{-62,24},{60,116}})));
        Components.SystemicCirculation systemicCirculation
          annotation (Placement(transformation(extent={{-60,-104},{60,2}})));
        Components.HeartPhysiolibraryElastance heartPhysiolibraryElastance
          annotation (Placement(transformation(extent={{-14,-20},{16,10}})));
      equation
        connect(pulmonaryCirculation.pulmonaryBloodInflow,
          heartPhysiolibraryElastance.pulmonaryArteryOutflow) annotation (Line(
            points={{-62,70},{-80,70},{-80,-0.5},{-7.7,-0.5}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryCirculation.pulmonaryBloodOutflow,
          heartPhysiolibraryElastance.leftAtriumInflow) annotation (Line(
            points={{60,70.92},{86,70.92},{86,-1.4},{10.9,-1.4}},
            color={0,0,0},
            thickness=1));
        connect(heartPhysiolibraryElastance.RightAtriumInflow,
          systemicCirculation.systemicBloodOutflow) annotation (Line(
            points={{-7.7,-8},{-82,-8},{-82,-51},{-60,-51}},
            color={0,0,0},
            thickness=1));
        connect(heartPhysiolibraryElastance.AortaOutflow, systemicCirculation.systemicBloodInflow)
          annotation (Line(
            points={{10.9,-8.6},{86,-8.6},{86,-51},{60,-51}},
            color={0,0,0},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=20, __Dymola_Algorithm="Dassl"));
      end MeursModel_nonPhysiolobraryHeart;

      model HemodynamicsMeurs_flatNorm
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
                "mmHg.s2/ml") = 226648.0586055,
            volumeFlow_start(displayUnit="ml/min") = 2.1666666666667e-05)                                                                                                  annotation(Placement(transformation(extent={{-11,-11},
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
          annotation (Placement(transformation(extent={{-244,124},{-206,156}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.VentricularElastance
          RVentricularElastance(EMIN=7599376.082655, EMAX=65327969.83335)
          annotation (Placement(transformation(extent={{-180,126},{-150,160}})));
        Physiolibrary.Hydraulic.Components.IdealValveResistance TricuspidValve(
          _Goff=0,
          _Ron(displayUnit="(mmHg.s)/ml") = 399967.162245,
          chatteringProtectionTime=0.01,                                                       lastChange(displayUnit = "s"), open(fixed = true, start = true), useChatteringProtection = true,
          useLimitationInputs=false)
                              annotation (Placement(transformation(
              origin={-189,96},
              extent={{-13,12},{13,-12}})));
        replaceable Physiolibrary.Types.Constants.FrequencyConst HeartRate(k(displayUnit = "1/min") = 1.2) annotation(Placement(transformation(origin={-257,
                  166.5},                                                                                                                                              extent = {{-11, -6.5}, {11, 6.5}})));
        MeursHemodynamics.Components.ElasticCompartment
                           intrathoracicArteries(
          elastance_NonSI=1.43,
          unstressedVolume_NonSI=140,
          externalPressure_NonSI=-4,
          V0_NonSI=204)
          annotation (Placement(transformation(extent={{218,-126},{238,-106}})));
        MeursHemodynamics.Components.BloodResistor
                      extrathoracicArterialResistance(bloodResistance_NonSI=
              0.06)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={136,-98})));
        MeursHemodynamics.Components.Inductor
                 aorticFlowInertia(inertance_NonSI=0.0017) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={184,-98})));
        MeursHemodynamics.Components.BloodResistor
                      systemicArteriolarResistance(bloodResistance_NonSI=0.8)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={48,-98})));
        MeursHemodynamics.Components.ElasticCompartment
                           extrathoracicArteries(
          elastance_NonSI=0.556,
          unstressedVolume_NonSI=370,
          externalPressure_NonSI=0,
          V0_NonSI=526)
          annotation (Placement(transformation(extent={{88,-128},{108,-108}})));
        MeursHemodynamics.Components.ElasticCompartment
                           SystemicTissues(
          elastance_NonSI=0.262,
          unstressedVolume_NonSI=185,
          externalPressure_NonSI=0,
          V0_NonSI=283)
          annotation (Placement(transformation(extent={{-4,-126},{16,-106}})));
        MeursHemodynamics.Components.BloodResistor
                      smallVenuleResistance(bloodResistance_NonSI=0.2)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-54,-98})));
        MeursHemodynamics.Components.BloodResistor
                      venousResistance(bloodResistance_NonSI=0.09) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-130,-98})));
        MeursHemodynamics.Components.ElasticCompartment
                           intrathoracicVeins(
          elastance_NonSI=0.0182,
          unstressedVolume_NonSI=1190,
          externalPressure_NonSI=-4,
          V0_NonSI=1480)
          annotation (Placement(transformation(extent={{-196,-122},{-176,-102}})));
        MeursHemodynamics.Components.ElasticCompartment
                           extrathoracicVeins(
          elastance_NonSI=0.0169,
          unstressedVolume_NonSI=1000,
          externalPressure_NonSI=0,
          V0_NonSI=1530)
          annotation (Placement(transformation(extent={{-106,-128},{-86,-108}})));
        MeursHemodynamics.Components.BloodResistor
                      centralVenousResistance(bloodResistance_NonSI=0.003)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-222,-98})));
        MeursHemodynamics.Components.VariableElasticCompartment rightAtrium(
          unstressedVolume_NonSI=30,
          externalPressure_NonSI=-4,
          V0_NonSI=135) annotation (Placement(transformation(extent={{-236,-64},
                  {-212,-40}})));
        MeursHemodynamics.Components.VariableElasticCompartment rightVentricle(
          unstressedVolume_NonSI=40,
          externalPressure_NonSI=-4,
          V0_NonSI=131) annotation (Placement(transformation(extent={{-148,-66},
                  {-122,-40}})));
        MeursHemodynamics.Components.VariableElasticCompartment leftAtrium(
          unstressedVolume_NonSI=30,
          externalPressure_NonSI=-4,
          V0_NonSI=93.1)
          annotation (Placement(transformation(extent={{94,-60},{120,-34}})));
        MeursHemodynamics.Components.VariableElasticCompartment leftVentricle(
          unstressedVolume_NonSI=60,
          externalPressure_NonSI=-4,
          V0_NonSI=144)
          annotation (Placement(transformation(extent={{172,-62},{198,-36}})));
        MeursHemodynamics.Components.ElasticCompartment
                           pulmonaryArteries(
          elastance_NonSI=0.233,
          unstressedVolume_NonSI=50,
          externalPressure_NonSI=-4,
          V0_NonSI=106)
          annotation (Placement(transformation(extent={{-74,-40},{-54,-20}})));
        MeursHemodynamics.Components.BloodResistor
                      pulmonaryResistance(bloodResistance_NonSI=0.11)
          annotation (Placement(transformation(extent={{-34,-18},{-14,2}})));
        MeursHemodynamics.Components.ElasticCompartment
                           pulmonaryVeins(
          elastance_NonSI=0.0455,
          unstressedVolume_NonSI=350,
          externalPressure_NonSI=-4,
          V0_NonSI=518)
          annotation (Placement(transformation(extent={{-4,-38},{16,-18}})));
        MeursHemodynamics.Components.BloodResistor
                      pulmonaryVenousResistance(bloodResistance_NonSI=0.003)
          annotation (Placement(transformation(extent={{36,-18},{56,2}})));
        Modelica.Blocks.Sources.Constant HeartRate1(k=72)
          annotation (Placement(transformation(extent={{-264,2},{-244,22}})));
        MeursHemodynamics.Components.CardiacElastance
                         rightCardiacElastance(
          atrialElmin=0.05,
          atrialElmax=0.15,
          ventricularElmin=0.057,
          ventricularElmax=0.49) annotation (Placement(transformation(
              extent={{-17,-10},{17,10}},
              rotation=0,
              origin={-187,-4})));
        MeursHemodynamics.Components.CardiacElastance
                         leftCardiacElastance(
          atrialElmin=0.12,
          atrialElmax=0.28,
          ventricularElmin=0.09,
          ventricularElmax=4) annotation (Placement(transformation(
              extent={{-17,-12},{17,12}},
              rotation=0,
              origin={109,0})));
        MeursHemodynamics.Components.CardiacValve
                     tricuspidalValve(outflowResistance=0.003,
            backflowConductance=0)
          annotation (Placement(transformation(extent={{-192,-62},{-172,-42}})));
        MeursHemodynamics.Components.CardiacValve
                     pulmonicValve(outflowResistance=0.003)
          annotation (Placement(transformation(extent={{-112,-64},{-92,-44}})));
        MeursHemodynamics.Components.CardiacValve
                     mitralValve(outflowResistance=0.003, backflowConductance=0)
          annotation (Placement(transformation(extent={{132,-58},{152,-38}})));
        MeursHemodynamics.Components.CardiacValve
                     aorticValve(outflowResistance=0.003, backflowConductance=0)
          annotation (Placement(transformation(extent={{208,-62},{232,-38}})));
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
                107.2},{-228,130},{-202.39,130},{-202.39,139.84}},                                                                                  color = {0, 0, 127}));
        connect(PulmonaryValve.q_out, Epa.q_in) annotation (Line(
            points={{-106,95},{-92,95},{-92,136},{-80,136}},
            thickness=1));
        connect(RightVentricle.compliance,RVentricularElastance. Ct) annotation(Line(points={{-155,
                107},{-155,118},{-126,118},{-126,146.91},{-147.15,146.91}},                                                                                            color = {0, 0, 127}));
        connect(LeftAtrium.compliance, LAtrialElastance.Ct) annotation (Line(
            points={{88,113.2},{88,112},{121.61,112},{121.61,145.84}},
            color={0,0,127}));
        connect(HeartRate.y,RAtrialElastance. HR) annotation(Line(points={{-243.25,
                166.5},{-225,166.5},{-225,152.8}},                                                                           color = {0, 0, 127}));
        connect(RVentricularElastance.HR, HeartRate.y) annotation(Line(points={{-165,
                156.6},{-165,166.5},{-243.25,166.5}},                                                                             color = {0, 0, 127}));
        connect(LAtrialElastance.HR, HeartRate.y) annotation (Line(
            points={{99,158.8},{99,166.5},{-243.25,166.5}},
            color={0,0,127}));
        connect(LVentricularElastance.HR, HeartRate.y) annotation (Line(
            points={{182,154.8},{182,166.5},{-243.25,166.5}},
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
        connect(aorticFlowInertia.bloodFlowInflow,intrathoracicArteries. bloodFlowInflow)
          annotation (Line(
            points={{194.4,-98},{228,-98},{228,-116}},
            color={0,0,0},
            thickness=1));
        connect(aorticFlowInertia.bloodFlowOutflow,
          extrathoracicArterialResistance.bloodFlowInflow) annotation (Line(
            points={{174.4,-98},{146.4,-98}},
            color={0,0,0},
            thickness=1));
        connect(extrathoracicArterialResistance.bloodFlowOutflow,
          extrathoracicArteries.bloodFlowInflow) annotation (Line(
            points={{126.4,-98},{98,-98},{98,-118}},
            color={0,0,0},
            thickness=1));
        connect(extrathoracicArteries.bloodFlowInflow,
          systemicArteriolarResistance.bloodFlowInflow) annotation (Line(
            points={{98,-118},{98,-98},{58.4,-98}},
            color={0,0,0},
            thickness=1));
        connect(SystemicTissues.bloodFlowInflow,systemicArteriolarResistance. bloodFlowOutflow)
          annotation (Line(
            points={{6,-116},{6,-98},{38.4,-98}},
            color={0,0,0},
            thickness=1));
        connect(smallVenuleResistance.bloodFlowInflow,SystemicTissues. bloodFlowInflow)
          annotation (Line(
            points={{-43.6,-98},{6,-98},{6,-116}},
            color={28,108,200},
            thickness=1));
        connect(smallVenuleResistance.bloodFlowOutflow,extrathoracicVeins. bloodFlowInflow)
          annotation (Line(
            points={{-63.6,-98},{-96,-98},{-96,-118}},
            color={28,108,200},
            thickness=1));
        connect(smallVenuleResistance.bloodFlowOutflow,venousResistance. bloodFlowInflow)
          annotation (Line(
            points={{-63.6,-98},{-119.6,-98}},
            color={28,108,200},
            thickness=1));
        connect(centralVenousResistance.bloodFlowInflow,venousResistance. bloodFlowOutflow)
          annotation (Line(
            points={{-211.6,-98},{-139.6,-98}},
            color={28,108,200},
            thickness=1));
        connect(centralVenousResistance.bloodFlowInflow,intrathoracicVeins. bloodFlowInflow)
          annotation (Line(
            points={{-211.6,-98},{-186,-98},{-186,-112}},
            color={28,108,200},
            thickness=1));
        connect(pulmonaryResistance.bloodFlowInflow,pulmonaryArteries. bloodFlowInflow)
          annotation (Line(
            points={{-34.4,-8},{-64,-8},{-64,-30}},
            color={28,108,200},
            thickness=1));
        connect(pulmonaryResistance.bloodFlowOutflow,pulmonaryVeins. bloodFlowInflow)
          annotation (Line(
            points={{-14.4,-8},{6,-8},{6,-28}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryVenousResistance.bloodFlowInflow,pulmonaryVeins. bloodFlowInflow)
          annotation (Line(
            points={{35.6,-8},{6,-8},{6,-28}},
            color={0,0,0},
            thickness=1));
        connect(centralVenousResistance.bloodFlowOutflow, rightAtrium.bloodFlowInflow)
          annotation (Line(
            points={{-231.6,-98},{-254,-98},{-254,-52},{-224,-52}},
            color={0,0,0},
            thickness=1));
        connect(pulmonaryVenousResistance.bloodFlowOutflow, leftAtrium.bloodFlowInflow)
          annotation (Line(
            points={{55.6,-8},{76,-8},{76,-47},{107,-47}},
            color={0,0,0},
            thickness=1));
        connect(HeartRate1.y, rightCardiacElastance.HR) annotation (Line(
            points={{-243,12},{-214,12},{-214,-4},{-205.7,-4}},
            color={0,0,127},
            thickness=1));
        connect(rightAtrium.inputElastance, rightCardiacElastance.Eta)
          annotation (Line(
            points={{-224,-38.8},{-224,-24},{-160,-24},{-160,-7.6},{-168.3,-7.6}},

            color={0,0,127},
            thickness=1));
        connect(rightCardiacElastance.Etv, rightVentricle.inputElastance)
          annotation (Line(
            points={{-168.3,2.2},{-135,2.2},{-135,-38.7}},
            color={0,0,127},
            thickness=1));
        connect(leftCardiacElastance.HR, rightCardiacElastance.HR) annotation (
            Line(
            points={{90.3,0},{76,0},{76,12},{-214,12},{-214,-4},{-205.7,-4}},
            color={0,0,127},
            thickness=1));
        connect(leftCardiacElastance.Eta, leftAtrium.inputElastance)
          annotation (Line(
            points={{127.7,-4.32},{146,-4.32},{146,-20},{106,-20},{106,-32.7},{
                107,-32.7}},
            color={0,0,127},
            thickness=1));
        connect(leftVentricle.inputElastance, leftCardiacElastance.Etv)
          annotation (Line(
            points={{185,-34.7},{186,-34.7},{186,7.44},{127.7,7.44}},
            color={0,0,127},
            thickness=1));
        connect(rightAtrium.bloodFlowInflow, tricuspidalValve.bloodFlowInflow)
          annotation (Line(
            points={{-224,-52},{-192,-52}},
            color={0,0,0},
            thickness=1));
        connect(tricuspidalValve.bloodFlowOutflow, rightVentricle.bloodFlowInflow)
          annotation (Line(
            points={{-172,-52},{-135,-53}},
            color={0,0,0},
            thickness=1));
        connect(rightVentricle.bloodFlowInflow, pulmonicValve.bloodFlowInflow)
          annotation (Line(
            points={{-135,-53},{-112,-53},{-112,-54}},
            color={0,0,0},
            thickness=1));
        connect(pulmonicValve.bloodFlowOutflow, pulmonaryArteries.bloodFlowInflow)
          annotation (Line(
            points={{-92,-54},{-82,-54},{-82,-8},{-64,-8},{-64,-30}},
            color={0,0,0},
            thickness=1));
        connect(leftAtrium.bloodFlowInflow, mitralValve.bloodFlowInflow)
          annotation (Line(
            points={{107,-47},{108,-47},{108,-48},{132,-48}},
            color={0,0,0},
            thickness=1));
        connect(mitralValve.bloodFlowOutflow, leftVentricle.bloodFlowInflow)
          annotation (Line(
            points={{152,-48},{185,-49}},
            color={0,0,0},
            thickness=1));
        connect(leftVentricle.bloodFlowInflow, aorticValve.bloodFlowInflow)
          annotation (Line(
            points={{185,-49},{186,-49},{186,-50},{208,-50}},
            color={0,0,0},
            thickness=1));
        connect(aorticValve.bloodFlowOutflow, intrathoracicArteries.bloodFlowInflow)
          annotation (Line(
            points={{232,-50},{244,-50},{244,-98},{228,-98},{228,-116}},
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
</html>"),experiment(StopTime=5));
      end HemodynamicsMeurs_flatNorm;

      model TestCardiacElastances
        Modelica.Blocks.Sources.Constant HeartRate(k=72)
          annotation (Placement(transformation(extent={{-96,10},{-76,30}})));
        Modelica.Blocks.Math.Gain REtv(k=1)
          annotation (Placement(transformation(extent={{46,64},{66,84}})));
        Modelica.Blocks.Math.Gain REta(k=1)
          annotation (Placement(transformation(extent={{46,32},{66,52}})));
        MeursHemodynamics.Components.CardiacElastance rightHeart(
          atrialElmin=0.05,
          atrialElmax=0.15,
          ventricularElmin=0.057,
          ventricularElmax=0.49)
          annotation (Placement(transformation(extent={{-34,42},{0,80}})));
        MeursHemodynamics.Components.CardiacElastance leftHeart(
          atrialElmin=0.12,
          atrialElmax=0.28,
          ventricularElmin=0.09,
          ventricularElmax=4)
          annotation (Placement(transformation(extent={{-38,-30},{-4,8}})));
        Modelica.Blocks.Math.Gain LEtv(k=1)
          annotation (Placement(transformation(extent={{42,-10},{62,10}})));
        Modelica.Blocks.Math.Gain LEta(k=1)
          annotation (Placement(transformation(extent={{42,-42},{62,-22}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.AtrialElastance
          rignhtAtrialElastance(EMIN=6666119.37075, EMAX=19998358.11225)
          annotation (Placement(transformation(extent={{-66,-74},{-46,-54}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.VentricularElastance
          rightVentricularElastance(EMIN=7599376.082655, EMAX=65327969.83335)
          annotation (Placement(transformation(extent={{-6,-76},{14,-56}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.AtrialElastance
          leftAtrialElastance(EMIN=15998686.4898, EMAX=37330268.4762)
          annotation (Placement(transformation(extent={{-68,-108},{-48,-88}})));
        Physiolibrary.Hydraulic.Examples.MeursModel2011.Parts.VentricularElastance
          leftVentricularElastance(EMIN=11999014.86735, EMAX=533289549.66)
          annotation (Placement(transformation(extent={{-4,-106},{16,-86}})));
        Physiolibrary.Types.Constants.FrequencyConst frequency(k=1.2)
          annotation (Placement(transformation(extent={{-94,-34},{-86,-26}})));
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
        connect(leftVentricularElastance.HR, leftAtrialElastance.HR)
          annotation (Line(points={{6,-88},{6,-82},{-58,-82},{-58,-90}}, color=
                {0,0,127}));
        connect(frequency.y, rignhtAtrialElastance.HR) annotation (Line(points=
                {{-85,-30},{-56,-30},{-56,-56}}, color={0,0,127}));
        connect(rightVentricularElastance.HR, rignhtAtrialElastance.HR)
          annotation (Line(points={{4,-58},{4,-44},{-56,-44},{-56,-56}}, color=
                {0,0,127}));
        connect(frequency.y, leftAtrialElastance.HR) annotation (Line(points={{
                -85,-30},{-80,-30},{-80,-80},{-58,-80},{-58,-90}}, color={0,0,
                127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end TestCardiacElastances;

      model MinimalCirculation
        "Minimal circulation models driven by cardiac output"
         extends Modelica.Icons.Example;
        Physiolibrary.Hydraulic.Components.Pump heart(useSolutionFlowInput=true)
          annotation (Placement(transformation(extent={{-6,-50},{14,-30}})));
        Physiolibrary.Hydraulic.Components.ElasticVessel arteries(
          volume_start(displayUnit="l") = 0.001,
          ZeroPressureVolume(displayUnit="l") = 0.00085,
          Compliance(displayUnit="ml/mmHg") = 1.1625954425608e-08)
          annotation (Placement(transformation(extent={{36,-84},{56,-64}})));
        Physiolibrary.Hydraulic.Components.Conductor resistance(Conductance(
              displayUnit="ml/(mmHg.s)") = 6.2755151845753e-09)
          annotation (Placement(transformation(extent={{-4,-84},{16,-64}})));
        Physiolibrary.Hydraulic.Components.ElasticVessel veins(
          Compliance(displayUnit="ml/mmHg") = 6.1880080007267e-07,
          ZeroPressureVolume(displayUnit="l") = 0.00295,
          volume_start(displayUnit="l") = 0.0032)
          annotation (Placement(transformation(extent={{-42,-84},{-22,-64}})));
        Modelica.Blocks.Sources.Pulse pulse(
          width=25,
          amplitude=3.3e-4,
          period=60/75)
          annotation (Placement(transformation(extent={{-94,74},{-74,94}})));
        MeursHemodynamics.Components.ElasticCompartment elasticCompartment(
          elastance_NonSI=1/1.55,
          unstressedVolume_NonSI=850,
          externalPressure_NonSI=0,
          V0_NonSI=1000)
          annotation (Placement(transformation(extent={{50,18},{70,38}})));
        MeursHemodynamics.Components.ElasticCompartment elasticCompartment1(
          elastance_NonSI=1/82.5,
          unstressedVolume_NonSI=2950,
          externalPressure_NonSI=0,
          V0_NonSI=3200)
          annotation (Placement(transformation(extent={{-46,18},{-26,38}})));
        Physiolibrary.Hydraulic.Components.Pump heart1(useSolutionFlowInput=
              true)
          annotation (Placement(transformation(extent={{8,62},{28,82}})));
        Physiolibrary.Hydraulic.Components.Conductor resistance1(Conductance(
              displayUnit="ml/(mmHg.s)") = 6.2755151845753e-09)
          annotation (Placement(transformation(extent={{12,18},{32,38}})));
      equation
        connect(heart.q_out, arteries.q_in) annotation (Line(
            points={{14,-40},{46,-40},{46,-74}},
            thickness=1));
        connect(arteries.q_in, resistance.q_out) annotation (Line(
            points={{46,-74},{16,-74}},
            thickness=1));
        connect(resistance.q_in, veins.q_in) annotation (Line(
            points={{-4,-74},{-32,-74}},
            thickness=1));
        connect(veins.q_in, heart.q_in) annotation (Line(
            points={{-32,-74},{-32,-40},{-6,-40}},
            thickness=1));
        connect(pulse.y, heart.solutionFlow) annotation (Line(
            points={{-73,84},{-62,84},{-62,-26},{4,-26},{4,-33}},
            color={0,0,127}));
        connect(elasticCompartment1.bloodFlowInflow, heart1.q_in)
          annotation (Line(points={{-36,28},{-36,72},{8,72}}, color={0,0,0}));
        connect(heart1.q_out, elasticCompartment.bloodFlowInflow) annotation (
            Line(
            points={{28,72},{60,72},{60,28}},
            color={0,0,0},
            thickness=1));
        connect(heart1.solutionFlow, heart.solutionFlow) annotation (Line(
              points={{18,79},{18,84},{-62,84},{-62,-26},{4,-26},{4,-33}},
              color={0,0,127}));
        connect(elasticCompartment1.bloodFlowInflow, resistance1.q_in)
          annotation (Line(points={{-36,28},{-36,42},{6,42},{6,28},{12,28}},
              color={0,0,0}));
        connect(resistance1.q_out, elasticCompartment.bloodFlowInflow)
          annotation (Line(
            points={{32,28},{46,28},{46,42},{60,42},{60,28}},
            color={0,0,0},
            thickness=1));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics={                          Text(
                extent={{-40,-12},{80,-22}},
                lineColor={175,175,175},
                textString="Minimal circulation driven by cardiac output")}),
            Documentation(revisions="<html>
<p><i>2014</i></p>
<p>Marek Matejak, Charles University, Prague, Czech Republic </p>
</html>"),experiment(StopTime=5));
      end MinimalCirculation;
    end Model;
  end MeursHemodynamicsPhysiolibrary;
  annotation (uses(        Modelica(version="4.0.0"), Physiolibrary(version=
            "2.4.1")),
    version="3",
    conversion(noneFromVersion="", noneFromVersion="2"));
end MeursHemodynamics;
