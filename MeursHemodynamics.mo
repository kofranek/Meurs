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
        HP = HR/60;
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
        atrialElmax=0.06,
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
       Modelica.Blocks.Interfaces.RealOutput Pm "realtive pressure of braeth muscle (from 0 to 1)" annotation (Placement(transformation(extent={{100,-10},
                {120,10}}),     iconTransformation(extent={{100,-10},{120,10}})));
      Real HB(start=0) "heart period - duration of breath cycle in sec";
      Boolean b;

      Real Ti "length of inspiration";
      Real Te "length of expiration";
      Real T0 "start time of current breath in sec";
    equation
      b=time - pre(T0) >= pre(HB);
     when {initial(),b} then
        T0 = time;
        HB = RR/60;
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
        atrialElmax=0.06,
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
          atrialElmax=0.06,
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

    end Components;

    package Model
      model MeursModel
        Components.HeartPhysiolibrary heartPhysiolibrary
          annotation (Placement(transformation(extent={{-44,-32},{38,40}})));
        Components.PulmonaryCirculation pulmonaryCirculation
          annotation (Placement(transformation(extent={{-62,24},{60,116}})));
        Components.SystemicCirculation systemicCirculation
          annotation (Placement(transformation(extent={{-60,-104},{60,2}})));
      equation
        connect(heartPhysiolibrary.pulmonaryArteryOutflow, pulmonaryCirculation.pulmonaryBloodInflow)
          annotation (Line(
            points={{-26.78,14.8},{-86,14.8},{-86,70},{-62,70}},
            color={0,0,0},
            thickness=1));
        connect(systemicCirculation.systemicBloodOutflow, heartPhysiolibrary.RightAtriumInflow)
          annotation (Line(
            points={{-60,-51},{-76,-51},{-76,-52},{-88,-52},{-88,-3.2},{-26.78,
                -3.2}},
            color={0,0,0},
            thickness=1));
        connect(systemicCirculation.systemicBloodInflow, heartPhysiolibrary.AortaOutflow)
          annotation (Line(
            points={{60,-51},{84,-51},{84,-4.64},{24.06,-4.64}},
            color={0,0,0},
            thickness=1));
        connect(heartPhysiolibrary.leftAtriumInflow, pulmonaryCirculation.pulmonaryBloodOutflow)
          annotation (Line(
            points={{24.06,12.64},{86,12.64},{86,70.92},{60,70.92}},
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
    end Model;
  end MeursHemodynamicsPhysiolibrary;
  annotation (uses(        Modelica(version="4.0.0"), Physiolibrary(version=
            "2.4.1")),
    version="3",
    conversion(noneFromVersion="", noneFromVersion="2"));
end MeursHemodynamics;
