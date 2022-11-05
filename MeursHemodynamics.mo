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
      parameter Real currentHeartRate=72 "heart rate in beats per min";
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
        V0_NonSI=204)
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
        V0_NonSI=526)
        annotation (Placement(transformation(extent={{20,-32},{40,-12}})));
      ElasticCompartment SystemicTissues(
        elastance_NonSI=0.262,
        unstressedVolume_NonSI=185,
        externalPressure_NonSI=0,
        V0_NonSI=283)
        annotation (Placement(transformation(extent={{-16,-32},{4,-12}})));
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
        V0_NonSI=1480)
        annotation (Placement(transformation(extent={{-78,-32},{-58,-12}})));
      ElasticCompartment extrathoracicVeins(
        elastance_NonSI=0.0169,
        unstressedVolume_NonSI=1000,
        externalPressure_NonSI=0,
        V0_NonSI=1530)
        annotation (Placement(transformation(extent={{-48,-32},{-28,-12}})));
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
          points={{40.4,0},{30,0},{30,-22}},
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
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Epa(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000106,
          ZeroPressureVolume=5e-05,
          ExternalPressure=-533.28954966,
          Elastance=31064116.267695)
          annotation (Placement(transformation(extent={{-84,-38},{-56,-10}})));
        Physiolibrary.Hydraulic.Components.Resistor Rpp(useConductanceInput=
              false, Resistance(displayUnit="(mmHg.s)/ml") = 14665462.61565)
          annotation (Placement(transformation(extent={{-54,-13},{-20,13}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Epv(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000518,
          ZeroPressureVolume=0.00035,
          ExternalPressure=-533.28954966,
          Elastance=6066168.6273825)
          annotation (Placement(transformation(extent={{-8,-40},{26,-12}})));
        Physiolibrary.Hydraulic.Components.Resistor Rlain(useConductanceInput=
              false, Resistance(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{36,-12},{66,12}})));
      equation
        connect(pulmonaryBloodInflow, Rpp.q_in) annotation (Line(
            points={{-94,0},{-54,0}},
            color={0,0,0},
            thickness=1));
        connect(Rpp.q_out, Rlain.q_in) annotation (Line(
            points={{-20,0},{36,0}},
            color={0,0,0},
            thickness=1));
        connect(Rlain.q_out, pulmonaryBloodOutflow) annotation (Line(
            points={{66,0},{92,0}},
            color={0,0,0},
            thickness=1));
        connect(Epv.q_in, Rlain.q_in) annotation (Line(
            points={{9,-26},{8,-26},{8,0},{36,0}},
            color={0,0,0},
            thickness=1));
        connect(Epa.q_in, Rpp.q_in) annotation (Line(
            points={{-70,-24},{-70,0},{-54,0}},
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
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eitha(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000204,
          ZeroPressureVolume=0.00014,
          ExternalPressure=-533.28954966,
          Elastance=190651014.00345)
          annotation (Placement(transformation(extent={{78,-36},{100,-14}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eetha(
          volume_start(displayUnit="ml") = 0.000526,
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          ZeroPressureVolume=0.00037,
          Elastance=74127247.40274)
          annotation (Placement(transformation(extent={{18,-36},{42,-14}})));
        Physiolibrary.Hydraulic.Components.Resistor Retha(useConductanceInput=
              false, Resistance(displayUnit="(mmHg.s)/ml") = 7999343.2449)
          annotation (Placement(transformation(extent={{34,-10},{54,10}})));
        Physiolibrary.Hydraulic.Components.Inertia inertia(I(displayUnit=
                "mmHg.s2/ml") = 226648.0586055, volumeFlow_start(displayUnit=
                "ml/min") = 0)                                                                                                                                             annotation(Placement(transformation(extent={{-10,-10},
                  {10,10}},                                                                                                    rotation = 180, origin={72,0})));
        Physiolibrary.Hydraulic.Components.Resistor Rsart(useConductanceInput=
              false, Resistance(displayUnit="(mmHg.s)/ml") = 106657909.932)
                                                                   annotation (
            Placement(transformation(
              extent={{12,-10},{-12,10}},
              origin={16,0})));
        Physiolibrary.Hydraulic.Components.Resistor Rsven(useConductanceInput=
              false, Resistance(displayUnit="(mmHg.s)/ml") = 26664477.483)
                                                                  annotation (
            Placement(transformation(
              extent={{13,-10},{-13,10}},
              origin={-15,0})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Est(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.000283,
          ZeroPressureVolume=0.000185,
          Elastance=34930465.50273)
          annotation (Placement(transformation(extent={{-12,-34},{12,-12}})));
        Physiolibrary.Hydraulic.Components.Resistor Rethv(useConductanceInput=
              false, Resistance(displayUnit="(mmHg.s)/ml") = 11999014.86735)
          annotation (Placement(transformation(extent={{-34,-10},{-58,10}})));
        Physiolibrary.Hydraulic.Components.Resistor Rrain(useConductanceInput=
              false, Resistance(displayUnit="(mmHg.s)/ml") = 399967.162245)
          annotation (Placement(transformation(extent={{-64,-10},{-90,10}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eithv(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.00148,
          ZeroPressureVolume=0.00119,
          ExternalPressure=-533.28954966,
          Elastance=2426467.450953)
          annotation (Placement(transformation(extent={{-72,-34},{-50,-12}})));
        Physiolibrary.Hydraulic.Components.ElasticVesselElastance Eethv(
          useV0Input=false,
          useExternalPressureInput=false,
          useComplianceInput=false,
          volume_start=0.00153,
          ZeroPressureVolume=0.001,
          Elastance=2253148.3473135)
          annotation (Placement(transformation(extent={{-42,-34},{-20,-12}})));
      equation
        connect(systemicBloodInflow, inertia.q_in) annotation (Line(
            points={{100,0},{82,-1.77636e-15}},
            color={0,0,0},
            thickness=1));
        connect(Eitha.q_in, inertia.q_in) annotation (Line(
            points={{89,-25},{89,0},{82,0},{82,-1.77636e-15}},
            color={0,0,0},
            thickness=1));
        connect(inertia.q_out, Retha.q_out) annotation (Line(
            points={{62,4.44089e-16},{59,4.44089e-16},{59,0},{54,0}},
            color={0,0,0},
            thickness=1));
        connect(Rsart.q_in, Retha.q_in) annotation (Line(
            points={{28,0},{34,0}},
            color={0,0,0},
            thickness=1));
        connect(Eetha.q_in, Rsart.q_in) annotation (Line(
            points={{30,-25},{30,2.22045e-16},{28,2.22045e-16}},
            color={0,0,0},
            thickness=1));
        connect(Est.q_in, Rsart.q_out) annotation (Line(
            points={{0,-23},{0,-1.66533e-15},{4,0}},
            color={0,0,0},
            thickness=1));
        connect(Rsven.q_in, Rsart.q_out) annotation (Line(
            points={{-2,0},{2,0},{2,-1.66533e-15},{4,0}},
            color={0,0,0},
            thickness=1));
        connect(systemicBloodOutflow, Rrain.q_out) annotation (Line(
            points={{-100,0},{-100,2.22045e-16},{-90,2.22045e-16}},
            color={0,0,0},
            thickness=1));
        connect(Rrain.q_in, Rethv.q_out) annotation (Line(
            points={{-64,0},{-58,0}},
            color={0,0,0},
            thickness=1));
        connect(Rethv.q_in, Rsven.q_out) annotation (Line(
            points={{-34,0},{-28,0}},
            color={0,0,0},
            thickness=1));
        connect(Eithv.q_in, Rethv.q_out) annotation (Line(
            points={{-61,-23},{-61,0},{-58,0}},
            color={0,0,0},
            thickness=1));
        connect(Eethv.q_in, Rsven.q_out) annotation (Line(
            points={{-31,-23},{-31,0},{-28,0}},
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
    end Components;

    package Model

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
        MeursHemodynamics.Components.BloodResistor
                      extrathoracicArterialResistance(bloodResistance_NonSI=
              0.06)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={150,-26})));
        MeursHemodynamics.Components.Inductor
                 aorticFlowInertia(inertance_NonSI=0.0017) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={198,-26})));
        MeursHemodynamics.Components.BloodResistor
                      systemicArteriolarResistance(bloodResistance_NonSI=0.8)
          annotation (Placement(transformation(
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
        MeursHemodynamics.Components.BloodResistor
                      smallVenuleResistance(bloodResistance_NonSI=0.2)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-40,-26})));
        MeursHemodynamics.Components.BloodResistor
                      venousResistance(bloodResistance_NonSI=0.09) annotation (
            Placement(transformation(
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
        MeursHemodynamics.Components.BloodResistor
                      centralVenousResistance(bloodResistance_NonSI=0.003)
          annotation (Placement(transformation(
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
        MeursHemodynamics.Components.BloodResistor
                      pulmonaryResistance(bloodResistance_NonSI=0.11)
          annotation (Placement(transformation(extent={{-20,54},{0,74}})));
        MeursHemodynamics.Components.ElasticCompartment
                           pulmonaryVeins(
          elastance_NonSI=0.0455,
          unstressedVolume_NonSI=350,
          externalPressure_NonSI=-4,
          V0_NonSI=518)
          annotation (Placement(transformation(extent={{10,34},{30,54}})));
        MeursHemodynamics.Components.BloodResistor
                      pulmonaryVenousResistance(bloodResistance_NonSI=0.003)
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
        extends
          Physiolibrary.Icons.CardioVascular;
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
        PhysiolibraryExpantion.ElasticVesselWithInnerVolume doubleWallElastic(
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
          annotation (Line(points={{26,-42.8},{26,-44},{6,-44},{6,-28},{34,-28},
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
        PhysiolibraryExpantion.SerialElasticConnections
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
        PhysiolibraryExpantion.ElasticVesselWithInnerVolume Cw(
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
        PhysiolibraryExpantion.SerialElasticConnections
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
        PhysiolibraryExpantion.SerialElasticConnections
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
        PhysiolibraryExpantion.ElasticVesselWithInnerVolume
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
          annotation (Line(points={{40,-0.8},{20,-0.8},{20,14},{48,14},{48,26}},
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
        PhysiolibraryExpantion.SerialElasticConnections
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
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=false,
          hardElastance=true)
          annotation (Placement(transformation(extent={{-8,-104},{12,-84}})));
        PhysiolibraryExpantion.ElasticVesselWithInnerVolume Cw(
          ZeroPressureVolume=0.0023,
          Compliance(displayUnit="l/cmH2O") = 2.0394324259559e-06,
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
          annotation (Placement(transformation(extent={{24,70},{44,90}})));
        PhysiolibraryExpantion.ElasticVessel CL2(
          volume_start=0.0023,
          ZeroPressureVolume=0.0023,
          Compliance(displayUnit="l/cmH2O") = 5.0985810648896e-06,
          useExternalPressureInput=true,
          hardElastance=true)
          annotation (Placement(transformation(extent={{54,70},{74,90}})));
        Physiolibrary.Hydraulic.Sources.UnlimitedVolume unlimitedVolume2(
            usePressureInput=true)
          annotation (Placement(transformation(extent={{-28,80},{-8,100}})));
        PhysiolibraryExpantion.OuterElasticVessel outerElasticVessel(
          outer_volume_start=0,
          useV0Input=true,
          ZeroPressureVolume=0.0023,
          useComplianceInput=true,                                   Compliance(
              displayUnit="l/cmH2O") = 2.0394324259559e-06,
          useExternalPressureInput=true,                    hardElastance=true)
          annotation (Placement(transformation(extent={{56,44},{76,64}})));
        Physiolibrary.Hydraulic.Sensors.PressureMeasure pressureMeasure
          annotation (Placement(transformation(extent={{72,78},{92,98}})));
        Modelica.Blocks.Math.Feedback feedback
          annotation (Placement(transformation(extent={{86,56},{106,76}})));
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
            points={{18,78},{24,78},{24,80}},
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
            points={{44,80},{64,80}},
            color={0,0,0},
            thickness=1));
        connect(outerElasticVessel.InnerVolume, CL2.volume) annotation (Line(
              points={{72.2,52},{70,52},{70,70}}, color={0,0,127}));
        connect(outerElasticVessel.pressure, CL2.externalPressure) annotation (
            Line(points={{78.1,52.9},{78.1,92},{72,92},{72,88}}, color={0,0,127}));
        connect(CL2.q_in, pressureMeasure.q_in) annotation (Line(
            points={{64,80},{72,80},{72,82},{78,82}},
            color={0,0,0},
            thickness=1));
        connect(feedback.u1, CL2.externalPressure) annotation (Line(points={{88,66},
                {76.1,66},{76.1,90},{72,90},{72,88}},     color={0,0,127}));
        connect(feedback.u2, pressureMeasure.pressure) annotation (Line(points=
                {{96,58},{96,52},{110,52},{110,84},{88,84}}, color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=4,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end TestSimpleRespirationMechnaicsExtens;
    end Model;

    package PhysiolibraryExpantion
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
</html>",   info="<html>
<p>Pressure can be generated by an elastic tissue surrounding some accumulated volume. Typically there is a threshold volume, below which the relative pressure is equal to external pressure and the wall of the blood vessels is not stressed. But if the volume rises above this value, the pressure increases proportionally. The slope in this pressure-volume characteristic is called &ldquo;Compliance&rdquo;.</p>
<ul>
<li>Increassing volume above ZeroPressureVolume (V0) generate positive pressure (greater than external pressure) lineary dependent on excess volume.</li>
<li>Decreasing volume below CollapsingPressureVolume (V00) generate negative pressure (lower than external pressure) logarithmicaly dependent on volume.</li>
<li>Otherwise external pressure is presented as pressure inside ElasticVessel.</li>
</ul>
<p><br><img src=\"modelica://Physiolibrary/Resources/Images/UserGuide/ElasticVessel_PV.png\"/></p>
</html>"));
      end ElasticVesselWithInnerVolume;

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
        parameter Physiolibrary.Types.Pressure MinimalCollapsingPressure(min=-
              Modelica.Constants.inf)=-101325;
          parameter Physiolibrary.Types.Volume CollapsingPressureVolume(min=-Modelica.Constants.inf)=
           1e-12 "Maximal volume, which generate negative collapsing pressure";
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
    end PhysiolibraryExpantion;
  end MeursHemodynamicsPhysiolibrary;
  annotation (uses(        Modelica(version="4.0.0"), Physiolibrary(version=
            "2.4.1")),
    version="3",
    conversion(noneFromVersion="", noneFromVersion="2"));
end MeursHemodynamics;
