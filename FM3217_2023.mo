within ;
package FM3217_2023 "Collection of models as created in FM3217"

  package Tutorial1

    model SimplePendulum "Model of a simple pendulum"
      import      Modelica.Units.SI;
      constant SI.Acceleration g = 9.81 "Gravitional constant";
      parameter Modelica.Units.SI.Length L = 1 "Length of the Pendulum";

      // Now come the variables
      Modelica.Units.SI.Angle Theta(start=0.1, fixed=true);
      /* start of comment
  some comment
  */
      Modelica.Units.SI.AngularVelocity ThetaDot;
    equation
      ThetaDot = der(Theta);
      der(ThetaDot) = - g/L * sin(Theta);
    end SimplePendulum;
  end Tutorial1;

  package Tutorial2

    model Motor
      Modelica.Electrical.Analog.Basic.Resistor resistor
        annotation (Placement(transformation(extent={{-46,32},{-26,52}})));
      Modelica.Electrical.Analog.Basic.Inductor inductor
        annotation (Placement(transformation(extent={{-2,32},{18,52}})));
      Modelica.Electrical.Analog.Basic.RotationalEMF emf
        annotation (Placement(transformation(extent={{22,-10},{42,10}})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-68,-58},{-48,-38}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-58,-2})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia
        annotation (Placement(transformation(extent={{62,-10},{82,10}})));
      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-140,-22},{-100,18}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
        "Flange of right shaft"
        annotation (Placement(transformation(extent={{94,-10},{114,10}})));
    equation
      connect(signalVoltage.p, resistor.p)
        annotation (Line(points={{-58,8},{-58,42},{-46,42}}, color={0,0,255}));
      connect(resistor.n, inductor.p)
        annotation (Line(points={{-26,42},{-2,42}}, color={0,0,255}));
      connect(inductor.n, emf.p)
        annotation (Line(points={{18,42},{32,42},{32,10}}, color={0,0,255}));
      connect(emf.n, ground.p) annotation (Line(points={{32,-10},{32,-24},{-58,
              -24},{-58,-38}}, color={0,0,255}));
      connect(signalVoltage.n, ground.p)
        annotation (Line(points={{-58,-12},{-58,-38}}, color={0,0,255}));
      connect(emf.flange, inertia.flange_a)
        annotation (Line(points={{42,0},{62,0}}, color={0,0,0}));
      connect(signalVoltage.v, u)
        annotation (Line(points={{-70,-2},{-120,-2}}, color={0,0,127}));
      connect(inertia.flange_b, flange)
        annotation (Line(points={{82,0},{104,0}}, color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Motor;

    model MotorDrive
      Motor motor(
        resistor(R=0.5),
        inductor(L=0.05),
        inertia(J=0.001))
        annotation (Placement(transformation(extent={{8,-8},{28,12}})));
      Modelica.Blocks.Sources.Step step(startTime=0.5)
        annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
      Modelica.Blocks.Math.Feedback feedback
        annotation (Placement(transformation(extent={{-62,-10},{-42,10}})));
      Modelica.Blocks.Continuous.PID PID(
        k=10,
        Ti=2,
        Td=0.01)
        annotation (Placement(transformation(extent={{-30,-6},{-10,14}})));
      Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
        annotation (Placement(transformation(extent={{46,-8},{66,12}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
        annotation (Placement(transformation(extent={{74,-8},{94,12}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
         Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=180,
            origin={72,-34})));
    equation
      connect(PID.y, motor.u) annotation (Line(points={{-9,4},{10,4},{10,1.8},{
              6,1.8}}, color={0,0,127}));
      connect(feedback.y, PID.u) annotation (Line(points={{-43,0},{-30,0},{-30,
              4},{-32,4}}, color={0,0,127}));
      connect(step.y, feedback.u1)
        annotation (Line(points={{-73,0},{-60,0}}, color={0,0,127}));
      connect(motor.flange, idealGear.flange_a)
        annotation (Line(points={{28.4,2},{46,2}}, color={0,0,0}));
      connect(idealGear.flange_b, inertia.flange_a)
        annotation (Line(points={{66,2},{74,2}}, color={0,0,0}));
      connect(angleSensor.flange, inertia.flange_b) annotation (Line(points={{
              82,-34},{96,-34},{96,2},{94,2}}, color={0,0,0}));
      connect(angleSensor.phi, feedback.u2) annotation (Line(points={{61,-34},{
              -52,-34},{-52,-8}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=100));
    end MotorDrive;
  end Tutorial2;

  package Tutorial3
    package Components
      model Motor
        Modelica.Electrical.Analog.Basic.Resistor resistor
          annotation (Placement(transformation(extent={{-46,32},{-26,52}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor
          annotation (Placement(transformation(extent={{-2,32},{18,52}})));
        Modelica.Electrical.Analog.Basic.RotationalEMF emf
          annotation (Placement(transformation(extent={{22,-10},{42,10}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-68,-58},{-48,-38}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
          annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=90,
              origin={-58,-2})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{62,-10},{82,10}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-140,-22},{-100,18}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
          "Flange of right shaft"
          annotation (Placement(transformation(extent={{94,-10},{114,10}})));
      equation
        connect(signalVoltage.p, resistor.p)
          annotation (Line(points={{-58,8},{-58,42},{-46,42}}, color={0,0,255}));
        connect(resistor.n, inductor.p)
          annotation (Line(points={{-26,42},{-2,42}}, color={0,0,255}));
        connect(inductor.n, emf.p)
          annotation (Line(points={{18,42},{32,42},{32,10}}, color={0,0,255}));
        connect(emf.n, ground.p) annotation (Line(points={{32,-10},{32,-24},{-58,
                -24},{-58,-38}}, color={0,0,255}));
        connect(signalVoltage.n, ground.p)
          annotation (Line(points={{-58,-12},{-58,-38}}, color={0,0,255}));
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{42,0},{62,0}}, color={0,0,0}));
        connect(signalVoltage.v, u)
          annotation (Line(points={{-70,-2},{-120,-2}}, color={0,0,127}));
        connect(inertia.flange_b, flange)
          annotation (Line(points={{82,0},{104,0}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Motor;

      model DCMachine

        parameter Modelica.Units.SI.Resistance R=0.5
          "Resistance of the armature" annotation (Dialog(group="Electrical"));

        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)
          annotation (Placement(transformation(extent={{-46,32},{-26,52}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)
          annotation (Placement(transformation(extent={{-2,32},{18,52}})));
        Modelica.Electrical.Analog.Basic.RotationalEMF emf
          annotation (Placement(transformation(extent={{22,-12},{42,8}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-80,-76},{-60,-56}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J)
          annotation (Placement(transformation(extent={{62,-10},{82,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
          "Flange of right shaft"
          annotation (Placement(transformation(extent={{94,-10},{114,10}})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p
          "Positive electrical pin" annotation (Placement(transformation(extent=
                 {{-144,44},{-88,100}}), iconTransformation(extent={{-144,44},{
                  -88,100}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n
          "Negative electrical pin" annotation (Placement(transformation(extent=
                 {{-142,-100},{-86,-44}}), iconTransformation(extent={{-142,
                  -100},{-86,-44}})));
        parameter Modelica.Units.SI.Inductance L=0.1
          "Inductance of the DC Machine";
        parameter Modelica.Units.SI.Inertia J=0.001
          "Internal inertia of the DC machine"
          annotation (Dialog(tab="Mechanical"));
      equation
        connect(resistor.n, inductor.p)
          annotation (Line(points={{-26,42},{-2,42}}, color={0,0,255}));
        connect(inductor.n, emf.p)
          annotation (Line(points={{18,42},{32,42},{32,8}},  color={0,0,255}));
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{42,-2},{52,-2},{52,0},{62,0}},
                                                   color={0,0,0}));
        connect(inertia.flange_b, flange)
          annotation (Line(points={{82,0},{104,0}}, color={0,0,0}));
        connect(resistor.p, p) annotation (Line(points={{-46,42},{-78,42},{-78,
                72},{-116,72}}, color={0,0,255}));
        connect(emf.n, n) annotation (Line(points={{32,-12},{32,-72},{-114,-72}},
              color={0,0,255}));
        connect(emf.n, ground.p) annotation (Line(points={{32,-12},{32,-56},{
                -70,-56}}, color={0,0,255}));
        connect(n, n)
          annotation (Line(points={{-114,-72},{-114,-72}}, color={0,0,255}));
        connect(p, p) annotation (Line(points={{-116,72},{-116,72},{-116,72},{
                -116,72}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Bitmap(extent={{94,-48},{-92,74}}, fileName=
                    "modelica://FM3217_2023/Resources/Images/dc-motor.jpg")}),
                                                                       Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          Documentation(info="<html>
<h4>This is a model of a simple DC machine.</h4>
<p><img src=\"modelica://FM3217_2023/Resources/Images/dc-motor.jpg\"/></p>
</html>"));
      end DCMachine;

      model Rload "Resistive load"
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=2*R_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={0,22})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p1
                      "Positive electrical pin"
          annotation (Placement(transformation(extent={{-10,90},{10,110}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n1
                      "Negative electrical pin"
          annotation (Placement(transformation(extent={{-10,-112},{10,-92}})));
        parameter Modelica.Units.SI.Resistance R_load=0.5
          "Ohmic value of the resistive load";
        Modelica.Electrical.Analog.Basic.Resistor resistor1(R=R_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={0,-24})));
      equation
        connect(resistor.p, p1) annotation (Line(points={{1.77636e-15,32},{
                1.77636e-15,64},{0,64},{0,100}}, color={0,0,255}));
        connect(resistor.n, resistor1.p) annotation (Line(points={{-1.77636e-15,
                12},{-1.77636e-15,-16},{1.77636e-15,-16},{1.77636e-15,-14}},
              color={0,0,255}));
        connect(resistor1.n, n1) annotation (Line(points={{-1.77636e-15,-34},{
                -1.77636e-15,-72},{0,-72},{0,-102}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Rload;

      model RLload
        extends Rload;
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-40,0})));
        parameter Modelica.Units.SI.Inductance L_load=0.1 "Load inductance";
      equation
        connect(inductor.p, p1) annotation (Line(points={{-40,10},{-40,78},{0,
                78},{0,100}}, color={0,0,255}));
        connect(inductor.n, n1) annotation (Line(points={{-40,-10},{-40,-70},{0,
                -70},{0,-102}}, color={0,0,255}));
      end RLload;

      model RLCload
        extends RLload;
        Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=C_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={34,0})));
        parameter Modelica.Electrical.Spice3.Types.Capacitance C_load=0.001
          "Load capacitance";
      equation
        connect(capacitor.n, n1) annotation (Line(points={{34,-10},{34,-70},{0,
                -70},{0,-102}}, color={0,0,255}));
        connect(capacitor.p, p1) annotation (Line(points={{34,10},{34,78},{0,78},
                {0,100}}, color={0,0,255}));
      end RLCload;

      model test
        extends RLCload(
          R_load=2,
          L_load=0.2,
          C_load=0.03);
      end test;

      model Turbine
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J_t)
          annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
            tau_constant=T_t)
          annotation (Placement(transformation(extent={{90,-10},{70,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
            Placement(transformation(rotation=0, extent={{-110,-10},{-90,10}})));
        parameter Modelica.Units.SI.Inertia J_t=2 "Turbine inertia";
        parameter Modelica.Units.SI.AngularMomentum T_t=10 "Turbine torque";
      equation
        connect(constantTorque.flange, inertia.flange_b)
          annotation (Line(points={{70,0},{-14,0}}, color={0,0,0}));
        connect(flange_a, inertia.flange_a)
          annotation (Line(points={{-100,0},{-34,0}}, color={0,0,0}));
        annotation (Icon(graphics={Bitmap(extent={{-100,-100},{100,98}},
                  fileName=
                    "modelica://FM3217_2023/Resources/Images/Turbine.png")}));
      end Turbine;
    end Components;

    package Tests
      model MotorDrive
        Tutorial2.Motor motor(
          resistor(R=0.5),
          inductor(L=0.05),
          inertia(J=0.001))
          annotation (Placement(transformation(extent={{8,-8},{28,12}})));
        Modelica.Blocks.Sources.Step step(startTime=0.5)
          annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
        Modelica.Blocks.Math.Feedback feedback
          annotation (Placement(transformation(extent={{-62,-10},{-42,10}})));
        Modelica.Blocks.Continuous.PID PID(
          k=10,
          Ti=2,
          Td=0.01)
          annotation (Placement(transformation(extent={{-30,-6},{-10,14}})));
        Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
          annotation (Placement(transformation(extent={{46,-8},{66,12}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
          annotation (Placement(transformation(extent={{74,-8},{94,12}})));
        Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
           Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={72,-34})));
      equation
        connect(PID.y, motor.u) annotation (Line(points={{-9,4},{10,4},{10,1.8},{
                6,1.8}}, color={0,0,127}));
        connect(feedback.y, PID.u) annotation (Line(points={{-43,0},{-30,0},{-30,
                4},{-32,4}}, color={0,0,127}));
        connect(step.y, feedback.u1)
          annotation (Line(points={{-73,0},{-60,0}}, color={0,0,127}));
        connect(motor.flange, idealGear.flange_a)
          annotation (Line(points={{28.4,2},{46,2}}, color={0,0,0}));
        connect(idealGear.flange_b, inertia.flange_a)
          annotation (Line(points={{66,2},{74,2}}, color={0,0,0}));
        connect(angleSensor.flange, inertia.flange_b) annotation (Line(points={{
                82,-34},{96,-34},{96,2},{94,2}}, color={0,0,0}));
        connect(angleSensor.phi, feedback.u2) annotation (Line(points={{61,-34},{
                -52,-34},{-52,-8}}, color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=100));
      end MotorDrive;

      model DCMachineTest
        Components.DCMachine dCMachine(R=1)
          annotation (Placement(transformation(extent={{4,-6},{30,24}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
          annotation (Placement(transformation(
              extent={{16,-16},{-16,16}},
              rotation=90,
              origin={-42,8})));
        Modelica.Blocks.Sources.Step step
          annotation (Placement(transformation(extent={{-92,-2},{-72,18}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{38,0},{58,20}})));
      equation
        connect(signalVoltage.p, dCMachine.p) annotation (Line(points={{-42,24},
                {-14,24},{-14,19.8},{1.92,19.8}}, color={0,0,255}));
        connect(signalVoltage.n, dCMachine.n) annotation (Line(points={{-42,-8},
                {-12,-8},{-12,-1.8},{2.18,-1.8}}, color={0,0,255}));
        connect(step.y, signalVoltage.v)
          annotation (Line(points={{-71,8},{-61.2,8}}, color={0,0,127}));
        connect(dCMachine.flange, inertia.flange_a)
          annotation (Line(points={{30.52,9},{38,9},{38,10}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCMachineTest;

      model DCGeneratorTest
        Components.DCMachine dCMachine(R=1)
          annotation (Placement(transformation(extent={{4,-6},{30,24}})));
        Components.RLCload rLCload(R_load=1)
          annotation (Placement(transformation(extent={{-46,0},{-26,20}})));
        Components.Turbine turbine annotation (Placement(transformation(
                rotation=0, extent={{50,0},{70,20}})));
      equation
        connect(dCMachine.flange,turbine.flange_a)
          annotation (Line(points={{30.52,9},{50,9},{50,10}}, color={0,0,0}));
        connect(rLCload.p1, dCMachine.p) annotation (Line(points={{-36,20},{-36,
                40},{1.92,40},{1.92,19.8}}, color={0,0,255}));
        connect(rLCload.n1, dCMachine.n) annotation (Line(points={{-36,-0.2},{
                -36,-22},{2.18,-22},{2.18,-1.8}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCGeneratorTest;

      model DCGeneratorTest2copy
        Components.DCMachine dCMachine(R=1)
          annotation (Placement(transformation(extent={{4,-6},{30,24}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{38,0},{58,20}})));
        Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
            tau_constant=10)
          annotation (Placement(transformation(extent={{90,0},{70,20}})));
        Components.RLCload rLCload(R_load=2)
          annotation (Placement(transformation(extent={{-46,0},{-26,20}})));
      equation
        connect(dCMachine.flange, inertia.flange_a)
          annotation (Line(points={{30.52,9},{38,9},{38,10}}, color={0,0,0}));
        connect(constantTorque.flange, inertia.flange_b)
          annotation (Line(points={{70,10},{58,10}}, color={0,0,0}));
        connect(rLCload.p1, dCMachine.p) annotation (Line(points={{-36,20},{-36,
                40},{1.92,40},{1.92,19.8}}, color={0,0,255}));
        connect(rLCload.n1, dCMachine.n) annotation (Line(points={{-36,-0.2},{
                -36,-22},{2.18,-22},{2.18,-1.8}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCGeneratorTest2copy;

      model DCGeneratorTest2extend
        extends DCGeneratorTest(rLCload(
            R_load=2,
            L_load=0.2,
            C_load=0.03));
      end DCGeneratorTest2extend;
    end Tests;
  end Tutorial3;

  package Tutorial4
    model ElectricKettle
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=230^2/2000,
          useHeatPort=true) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=-90,
            origin={12,20})));
      Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(V=sqrt(2)*230,
          f=50) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-88,18})));
      Modelica.Electrical.Analog.Basic.Ground ground1
        annotation (Placement(transformation(extent={{-98,-48},{-78,-28}})));
      Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor
        annotation (Placement(transformation(extent={{-40,38},{-20,58}})));
      Modelica.Blocks.Math.Mean mean(f=50) annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=-90,
            origin={-40,26})));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor water(C=4.18e3*1.7,
          T(start=283.15, fixed=true))
        annotation (Placement(transformation(extent={{32,26},{52,46}})));
      Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor temperatureSensor
        annotation (Placement(transformation(extent={{60,12},{76,28}})));
      Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch
        annotation (Placement(transformation(extent={{-76,38},{-56,58}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor kettleWall(G=5)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={42,-2})));
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature roomTemperature(T=
            294.15)
        annotation (Placement(transformation(extent={{72,-44},{52,-24}})));
      Modelica.Blocks.Logical.OnOffController onOffController(bandwidth=3)
        annotation (Placement(transformation(extent={{80,16},{92,28}})));
      Modelica.Blocks.Sources.Constant const(k=95)
        annotation (Placement(transformation(extent={{64,36},{74,46}})));
    equation
      connect(resistor.n, sineVoltage.n) annotation (Line(points={{12,10},{12,
              -2},{-88,-2},{-88,8}}, color={0,0,255}));
      connect(ground1.p, sineVoltage.n)
        annotation (Line(points={{-88,-28},{-88,8}}, color={0,0,255}));
      connect(powerSensor.nc, resistor.p)
        annotation (Line(points={{-20,48},{12,48},{12,30}}, color={0,0,255}));
      connect(powerSensor.pv, resistor.p) annotation (Line(points={{-30,58},{
              -30,64},{12,64},{12,30}}, color={0,0,255}));
      connect(powerSensor.nv, sineVoltage.n) annotation (Line(points={{-30,38},
              {-30,-2},{-88,-2},{-88,8}}, color={0,0,255}));
      connect(powerSensor.power, mean.u)
        annotation (Line(points={{-40,37},{-40,33.2}}, color={0,0,127}));
      connect(water.port, resistor.heatPort)
        annotation (Line(points={{42,26},{42,20},{22,20}}, color={191,0,0}));
      connect(water.port, temperatureSensor.port)
        annotation (Line(points={{42,26},{42,20},{60,20}}, color={191,0,0}));
      connect(sineVoltage.p, switch.p) annotation (Line(points={{-88,28},{-88,
              48},{-76,48}}, color={0,0,255}));
      connect(switch.n, powerSensor.pc)
        annotation (Line(points={{-56,48},{-40,48}}, color={0,0,255}));
      connect(kettleWall.port_a, resistor.heatPort)
        annotation (Line(points={{42,8},{42,20},{22,20}}, color={191,0,0}));
      connect(kettleWall.port_b, roomTemperature.port) annotation (Line(points=
              {{42,-12},{42,-34},{52,-34}}, color={191,0,0}));
      connect(temperatureSensor.T, onOffController.u) annotation (Line(points={
              {76,20},{78,20},{78,18.4},{78.8,18.4}}, color={0,0,127}));
      connect(const.y, onOffController.reference) annotation (Line(points={{
              74.5,41},{78.8,41},{78.8,25.6}}, color={0,0,127}));
      connect(onOffController.y, switch.control) annotation (Line(points={{92.6,
              22},{96,22},{96,76},{-66,76},{-66,60}}, color={255,0,255}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        Documentation(info="<html>
<p>Eletric Kettle</p>
<ul>
<li>Volume of 1.7 L</li>
<li>230 V mains supply</li>
<li>Power of 2000 Watt</li>
</ul>
<p><br>Question: What should the resistance of the heating resistor be?</p>
<p><br>Power = Volage x Current</p>
<p>Resistance = Voltage / Current</p>
<p><br>This leads to:</p>
<p><br>For a 2000 Watt consuming resistor the resistance should be:</p>
<p>R = V / I = V / (P/V) = V^2 / P</p>
<p><br><h4>Heat capacity of Water</h4></p>
<p>1 calorty = heat energy to heat up one gram of water by 1 Kelvin</p>
<p>1 calorty = 4.18 J/(g k)</p>
</html>"),
        experiment(StopTime=600, __Dymola_NumberOfIntervals=5000));
    end ElectricKettle;
  end Tutorial4;

  package Tutorial5
    model ConnectingPipes
      HydroPower.HydroSystems.Pipe pipe(
        horizontalIcon=true,
        L=100,
        ZL=90,
        showDataInIcon=true)
        annotation (Placement(transformation(extent={{-10,70},{14,88}})));
      inner HydroPower.System_HPL system_HPL(steadyState=true,
          constantTemperature=true)
        annotation (Placement(transformation(extent={{-96,82},{-80,98}})));
      HydroPower.SinksAndSources.Fixed_pT source(paraOption=false)
        annotation (Placement(transformation(extent={{-60,72},{-46,86}})));
      HydroPower.SinksAndSources.Fixed_pT sink(paraOption=false)
        annotation (Placement(transformation(extent={{56,72},{42,86}})));
    equation
      connect(source.b, pipe.a)
        annotation (Line(points={{-45.3,79},{-11.2,79}},color={0,0,255}));
      connect(sink.b, pipe.b)
        annotation (Line(points={{41.3,79},{15.2,79}},color={0,0,255}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=600,
          Tolerance=1e-05,
          __Dymola_Algorithm="Radau"));
    end ConnectingPipes;

    model PipeWithValve
      extends ConnectingPipes;
      HydroPower.HydroSystems.PipeValve pipeValve1(
        m_dot_nom=123.5*1000,
        dp_nom=900000,
        ZL=90) annotation (Placement(transformation(extent={{-10,16},{18,40}})));
      HydroPower.SinksAndSources.Fixed_pT source1(paraOption=false)
        annotation (Placement(transformation(extent={{-48,20},{-32,36}})));
      HydroPower.SinksAndSources.Fixed_pT sink1(paraOption=false)
        annotation (Placement(transformation(extent={{64,18},{44,38}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=0.9,
        duration=100,
        offset=0.1,
        startTime=100)
        annotation (Placement(transformation(extent={{-82,38},{-62,58}})));
    equation
      connect(pipeValve1.a, source1.b)
        annotation (Line(points={{-11.4,28},{-31.2,28}}, color={0,0,255}));
      connect(pipeValve1.b, sink1.b)
        annotation (Line(points={{19.4,28},{43,28}}, color={0,0,255}));
      connect(ramp.y, pipeValve1.ValveCtrl)
        annotation (Line(points={{-61,48},{4,48},{4,41.2}}, color={0,0,127}));
      annotation (Documentation(info="<html>
<p>P = ro * g * Q * H</p>
<p>P = 100 MW</p>
<p>H = 90 m</p>
<p><br>Q = P/(ro*g*H) = 100e6 / (1e3 * 9.81 * 90)</p>
</html>"), experiment(StopTime=600));
    end PipeWithValve;

    model SimpleWaterWay
      extends PipeWithValve(ramp(
          height=0.99,
          duration=10,
          offset=0.01));
      HydroPower.SinksAndSources.Fixed_pT source2(paraOption=false)
        annotation (Placement(transformation(extent={{-94,-22},{-76,-4}})));
      HydroPower.SinksAndSources.Fixed_pT sink2(paraOption=false)
        annotation (Placement(transformation(extent={{92,-24},{70,-2}})));
      HydroPower.HydroSystems.Pipe pipe2(
        horizontalIcon=true,
        L=10000,
        ZL=100,
        ZR=90,
        showDataInIcon=true)
        annotation (Placement(transformation(extent={{-54,-22},{-30,-4}})));
      HydroPower.HydroSystems.PipeValve pipeValve2(
        m_dot_nom=123.5*1000,
        dp_nom=900000,
        ZL=90,
        ZR=0) annotation (Placement(transformation(extent={{14,-22},{40,-4}})));
      HydroPower.HydroSystems.HydroComponents.Containers.ClosedVolume
        closedVolume
        annotation (Placement(transformation(extent={{-20,-22},{0,-4}})));
    equation
      connect(pipe2.a, source2.b)
        annotation (Line(points={{-55.2,-13},{-75.1,-13}}, color={0,0,255}));
      connect(ramp.y, pipeValve2.ValveCtrl) annotation (Line(points={{-61,48},{
              -52,48},{-52,6},{27,6},{27,-3.1}}, color={0,0,127}));
      connect(pipeValve2.b, sink2.b)
        annotation (Line(points={{41.3,-13},{68.9,-13}}, color={0,0,255}));
      connect(pipe2.b, closedVolume.a)
        annotation (Line(points={{-28.8,-13},{-20,-13}}, color={0,0,255}));
      connect(closedVolume.b, pipeValve2.a)
        annotation (Line(points={{0,-13},{12.7,-13}}, color={0,0,255}));
      annotation (experiment(StopTime=6000, __Dymola_NumberOfIntervals=5000));
    end SimpleWaterWay;

    model WaterWayWithSurgeShaft
      extends SimpleWaterWay(ramp(height=-0.9, offset=1));
      HydroPower.HydroSystems.SurgeTank surgeTank3(
        D=100,
        deltZ=15,
        Vol=1000)
        annotation (Placement(transformation(extent={{-16,-74},{6,-52}})));
      HydroPower.SinksAndSources.Fixed_pT source3(paraOption=false)
        annotation (Placement(transformation(extent={{-92,-72},{-74,-54}})));
      HydroPower.SinksAndSources.Fixed_pT sink3(paraOption=false)
        annotation (Placement(transformation(extent={{94,-74},{72,-52}})));
      HydroPower.HydroSystems.Pipe pipe3(
        horizontalIcon=true,
        L=10000,
        ZL=100,
        ZR=90,
        showDataInIcon=true)
        annotation (Placement(transformation(extent={{-52,-72},{-28,-54}})));
      HydroPower.HydroSystems.PipeValve pipeValve3(
        m_dot_nom=123.5*1000,
        dp_nom=900000,
        ZL=90,
        ZR=0) annotation (Placement(transformation(extent={{16,-72},{42,-54}})));
    equation
      connect(pipe3.a,source3. b)
        annotation (Line(points={{-53.2,-63},{-73.1,-63}}, color={0,0,255}));
      connect(pipeValve3.b,sink3. b)
        annotation (Line(points={{43.3,-63},{70.9,-63}}, color={0,0,255}));
      connect(pipe3.b, surgeTank3.a)
        annotation (Line(points={{-26.8,-63},{-17.1,-63}}, color={0,0,255}));
      connect(pipeValve3.a, surgeTank3.b)
        annotation (Line(points={{14.7,-63},{7.1,-63}}, color={0,0,255}));
      connect(ramp.y, pipeValve3.ValveCtrl) annotation (Line(points={{-61,48},{
              -60,48},{-60,-46},{29,-46},{29,-53.1}}, color={0,0,127}));
      annotation (experiment(StopTime=600, __Dymola_NumberOfIntervals=5000));
    end WaterWayWithSurgeShaft;
  end Tutorial5;

  package Tutorial6
    model ReservoirBase
      inner HydroPower.System_HPL system_HPL(steadyState=true,
          constantTemperature=true)
        annotation (Placement(transformation(extent={{-92,76},{-72,96}})));
      HydroPower.HydroSystems.Reservoir headwater
        annotation (Placement(transformation(extent={{-66,20},{-46,40}})));
      HydroPower.HydroSystems.Reservoir tailwater
        annotation (Placement(transformation(extent={{50,12},{70,32}})));
      HydroPower.HydroSystems.Pipe conduit(horizontalIcon=true, L=1000)
        annotation (Placement(transformation(extent={{-36,14},{-16,34}})));
    equation
      connect(headwater.a2_pipe, conduit.a)
        annotation (Line(points={{-45,24},{-37,24}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end ReservoirBase;

    model TwoReservoirs
      extends Modelica.Icons.Example;
      extends ReservoirBase(conduit(ZL=100));
    equation
      connect(conduit.b, tailwater.a1_pipe) annotation (Line(points={{-15,24},{
              18,24},{18,16},{49,16}}, color={0,0,255}));
      annotation (experiment(StopTime=600, __Dymola_Algorithm="Radau"));
    end TwoReservoirs;

    model TwoReservoirsWithSource
      extends Modelica.Icons.Example;
      extends ReservoirBase(conduit(ZL=100));
      HydroPower.SinksAndSources.Fixed_HT constantWaterHead(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{-96,26},{-76,46}})));
      HydroPower.SinksAndSources.Fixed_HT constantTailWater(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{98,18},{78,38}})));
    equation
      connect(conduit.b, tailwater.a1_pipe) annotation (Line(points={{-15,24},{
              18,24},{18,16},{49,16}}, color={0,0,255}));
      connect(headwater.a1_open, constantWaterHead.b)
        annotation (Line(points={{-67,36},{-75,36}}, color={0,0,255}));
      connect(tailwater.a2_open, constantTailWater.b)
        annotation (Line(points={{71,28},{77,28}}, color={0,0,255}));
      annotation (experiment(StopTime=600, __Dymola_Algorithm="Radau"));
    end TwoReservoirsWithSource;

    model WaterWayRes
      extends Modelica.Icons.Example;
      extends ReservoirBase(conduit(
          L=10000,
          ZL=100,
          ZR=90));
      HydroPower.SinksAndSources.Fixed_HT constantWaterHead(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{-92,26},{-72,46}})));
      HydroPower.SinksAndSources.Fixed_HT constantTailWater(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{102,18},{82,38}})));
      HydroPower.HydroSystems.SurgeTank surgeTank(D=30, deltZ=100)
        annotation (Placement(transformation(extent={{-8,14},{12,34}})));
      HydroPower.HydroSystems.PipeValve pressureshaft(ZL=90)
        annotation (Placement(transformation(extent={{20,14},{40,34}})));
      Modelica.Blocks.Sources.Ramp ramp(duration=10, startTime=100)
        annotation (Placement(transformation(extent={{-2,54},{18,74}})));
    equation
      connect(tailwater.a2_open, constantTailWater.b)
        annotation (Line(points={{71,28},{81,28}}, color={0,0,255}));
      connect(headwater.a1_open, constantWaterHead.b)
        annotation (Line(points={{-67,36},{-71,36}}, color={0,0,255}));
      connect(conduit.b, surgeTank.a)
        annotation (Line(points={{-15,24},{-9,24}}, color={0,0,255}));
      connect(surgeTank.b, pressureshaft.a)
        annotation (Line(points={{13,24},{19,24}}, color={0,0,255}));
      connect(tailwater.a1_pipe, pressureshaft.b) annotation (Line(points={{49,
              16},{46,16},{46,24},{41,24}}, color={0,0,255}));
      connect(ramp.y, pressureshaft.ValveCtrl)
        annotation (Line(points={{19,64},{30,64},{30,35}}, color={0,0,127}));
      annotation (experiment(
          StopTime=600,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Radau"));
    end WaterWayRes;

    model WaterWayResClosingValve
      extends WaterWayRes(ramp(
          height=-1,
          duration=1,
          offset=1), system_HPL(Q_start=31.276));
      annotation (experiment(
          StopTime=600,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Radau"));
    end WaterWayResClosingValve;

    model SundsbarmWaterway
      extends Modelica.Blocks.Examples;
      inner HydroPower.System_HPL system_HPL(
        Q_start=24,
        steadyState=true,
        constantTemperature=true)
        annotation (Placement(transformation(extent={{-92,76},{-72,96}})));
      HydroPower.HydroSystems.Reservoir headwater(
        Hmax=ones(headwater.n)*(564 + 48 + 5),
        depth=ones(headwater.n)*(48 + 5),
        H_start=ones(headwater.n)*(564 + 48))
        annotation (Placement(transformation(extent={{-104,2},{-84,22}})));
      HydroPower.HydroSystems.Reservoir tailwater(
        Hmax=ones(tailwater.n)*(110 + 5 + 3),
        depth=ones(tailwater.n)*(5 + 3),
        H_start=ones(tailwater.n)*(110 + 5))
        annotation (Placement(transformation(extent={{86,-8},{106,12}})));
      HydroPower.HydroSystems.Pipe conduit(
        endD={5.8,5.8},
        L=6600,
        ZL=564,
        ZR=541.5,
        horizontalIcon=true)
        annotation (Placement(transformation(extent={{-70,-4},{-50,16}})));
      HydroPower.SinksAndSources.Fixed_HT constantWaterHead(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{-80,30},{-100,50}})));
      HydroPower.SinksAndSources.Fixed_HT constantTailWater(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{78,34},{98,54}})));
      HydroPower.HydroSystems.SurgeTank surgeTank(
        D=3.6,
        deltZ=150,
        H2L=0.87,
        Vol=100) annotation (Placement(transformation(extent={{-40,-4},{-20,16}})));
      HydroPower.HydroSystems.PipeValve pressureShaft(
        endD={3,3},
        m_dot_nom=24e3,
        dp_nom=4890000,
        L=724,
        ZL=541.5,
        ZR=112.5) annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=-0.9,
        duration=1,
        offset=1,
        startTime=100)
        annotation (Placement(transformation(extent={{-38,40},{-18,60}})));
      HydroPower.HydroSystems.Pipe tailRace(
        endD={5.8,5.8},
        L=10000,
        ZL=110.5,
        ZR=110,
        horizontalIcon=true)
        annotation (Placement(transformation(extent={{56,-14},{76,6}})));
      HydroPower.HydroSystems.HydroComponents.Containers.ClosedVolume turbineHouse(
          D=5.8, L=2)
        annotation (Placement(transformation(extent={{26,-4},{46,16}})));
    equation
      connect(headwater.a2_pipe,conduit. a)
        annotation (Line(points={{-83,6},{-71,6}},   color={0,0,255}));
      connect(tailwater.a2_open, constantTailWater.b) annotation (Line(points={{107,
              8},{110,8},{110,44},{99,44}}, color={0,0,255}));
      connect(headwater.a1_open, constantWaterHead.b) annotation (Line(points={{-105,
              18},{-106,18},{-106,40},{-101,40}}, color={0,0,255}));
      connect(conduit.b, surgeTank.a)
        annotation (Line(points={{-49,6},{-41,6}}, color={0,0,255}));
      connect(surgeTank.b, pressureShaft.a)
        annotation (Line(points={{-19,6},{-7,6}}, color={0,0,255}));
      connect(ramp.y, pressureShaft.ValveCtrl)
        annotation (Line(points={{-17,50},{4,50},{4,17}}, color={0,0,127}));
      connect(tailwater.a1_pipe, tailRace.b)
        annotation (Line(points={{85,-4},{77,-4}}, color={0,0,255}));
      connect(tailRace.a, turbineHouse.b)
        annotation (Line(points={{55,-4},{50,-4},{50,6},{46,6}}, color={0,0,255}));
      connect(pressureShaft.b, turbineHouse.a)
        annotation (Line(points={{15,6},{26,6}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
                {120,100}})), Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-120,-100},{120,100}})),
        experiment(
          StopTime=600,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Radau"));
    end SundsbarmWaterway;
  end Tutorial6;

  package Tutorial7
    model PlantConnectAndDisconnectToGrid
      extends HydroPower.Examples.PlantConnectAndDisconnectToGrid;
      annotation (experiment(
          StopTime=600,
          __Dymola_NumberOfIntervals=5000,
          Tolerance=1e-05,
          __Dymola_Algorithm="Radau"));
    end PlantConnectAndDisconnectToGrid;

    model Sundsbarm
      "Hydro plant model connecting to grid at t=150s and disconnecting at t=350s"
      extends Modelon.Icons.Experiment;

      HydroPower.HydroSystems.Reservoir reservoir1(
        n=4,
        L=500,
        Hmax=fill(105, reservoir1.n),
        H_start=fill(100, reservoir1.n),
        depthIntake={0,15},
        steadyState=false) annotation (Placement(transformation(extent={{-95,-42},
                {-75,-22}},
                         rotation=0)));
      HydroPower.HydroSystems.Reservoir river(
        n=4,
        L=500,
        nSegmentIntake={1,3},
        Hmax=fill(70, river.n),
        H_start=fill(60, river.n),
        depth={70,70,70,70},
        depthIntake={0,1},
        steadyState=false) annotation (Placement(transformation(extent={{73,-52},
                {93,-32}}, rotation=0)));
      HydroPower.HydroSystems.Pipe pressureShaft(
        L=200,
        ZL=90,
        ZR=40,
        endL={5,5},
        endD={5.5,5.5},
        Q_start=0.1,
        n=5,
        enable_dataVizPort_lower=true,
        p_start=540000) annotation (Placement(transformation(extent={{-8,-53},{
                12,-33}}, rotation=0)));
      HydroPower.MechanicalSystems.BasicTurbine turbine(
        np=12,
        H_nom=480,
        tableOnFile=true,
        LdraftTube=10,
        DavDraftTube=2,
        LscrollCase=5,
        DavScrollCasing=2,
        PUInFlowTables=true,
        QTableName="Qtab",
        Q_nom=24,
        H_start=564 + 48,
        H_start_draftTube=115,
        Ty=0.4,
        yvLim1=[-0.1, 0.1],
        yvLim2=[-0.2, 0.2],
        TurbineDataFile=Modelica.Utilities.Files.loadResource(HydroPower.TABLE_DIR
             + "TurbineDataFile.mat"),
        P_nom=103000000)
                        annotation (Placement(transformation(extent={{18,-72},{
                38,-52}},
                     rotation=0)));

      HydroPower.HydroSystems.Pipe downstream(
        n=4,
        endL={5,5},
        ZL=40,
        ZR=0,
        L=100,
        endD={5.5,5.5},
        Q_start=0.1,
        enable_dataVizPort_lower=true,
        p_start=400000) annotation (Placement(transformation(extent={{44,-50},{
                64,-30}}, rotation=0)));
      HydroPower.ElectricalSystems.PowerGrid powerGrid(
        startTime=1e6,
        unitsJ={122000,5.5e6,8000},
        NoLoadUnits={200,400,1000},
        distNoGen={-2,0,0},
        distTgen={150,1e6,1e6}) annotation (Placement(transformation(extent={{-89,
                40},{-69,60}}, rotation=0)));

      Modelica.Blocks.Sources.Ramp pwr_ref(
        duration=10,
        height=0,
        offset=45e6,
        startTime=1e6) annotation (Placement(transformation(extent={{-37,64},{-25,
                76}}, rotation=0)));
      HydroPower.ElectricalSystems.GeneratorAndMCB generator(
        np={12},
        Kdmp={0.05},
        f_start=0,
        J={212500},
        timeMCB_close={150},
        timeMCB_open={200},
        P_nom={103000000})
                          annotation (Placement(transformation(extent={{-59,40},{
                -39,60}},
                       rotation=0)));
      HydroPower.ControllersAndSensors.TurbineGovernorAnalog turbineGovernor(
        ep=1,
        DeadBand=0.001,
        Ki_load=0.1,
        Kd_load=0.5,
        Kd_noLoad=0.05,
        Ki_noLoad=0.025,
        K_noLoad=0.2,
        K_load=0.4,
        tRamp=40,
        P_generator_nom=generator.P_nom[1],
        enableRamp=false) annotation (Placement(transformation(extent={{-24,40},
                {-4,60}}, rotation=0)));
      HydroPower.Visualizers.RealValue turbinePower(precision=2, input_Value=
            turbine.summary.P_turbine*1e-6)
        annotation (Placement(transformation(extent={{64,10},{78,24}})));
      HydroPower.Visualizers.BooleanIndicator MCB(input_Value=turbineGovernor.summary.isMCB)
        annotation (Placement(transformation(extent={{64,73},{77,87}})));
      HydroPower.Visualizers.RealValue gridbalanceNum(precision=2, input_Value=
            generator.summary.P_grid_tot*1e-6)
        annotation (Placement(transformation(extent={{64,38},{78,52}})));
      inner HydroPower.System_HPL system_HPL(
        steadyState=true,
        pipeRoughness=0.1,
        T_start=293)
        annotation (Placement(transformation(extent={{-90,-80},{-70,-60}})));
      HydroPower.Visualizers.RealValue gridbalanceNum1(precision=2, input_Value=
           generator.summary.f[1])
        annotation (Placement(transformation(extent={{64,53},{78,67}})));
      HydroPower.Visualizers.RealValue gridbalanceNum2(precision=2, input_Value=
           generator.summary.P_generator[1]*1e-6)
        annotation (Placement(transformation(extent={{64,25},{78,39}})));
      HydroPower.HydroSystems.Pipe conduit(
        endD={5.8,5.8},
        L=6600,
        ZL=564,
        ZR=541.5,
        horizontalIcon=true)
        annotation (Placement(transformation(extent={{-69,-46},{-49,-26}})));
      HydroPower.HydroSystems.SurgeTank surgeTank(
        D=3.6,
        deltZ=150,
        H2L=0.87,
        Vol=100) annotation (Placement(transformation(extent={{-39,-55},{-19,
                -35}})));
      HydroPower.SinksAndSources.Fixed_HT constantWaterHead(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{-74,-13},{-94,7}})));
      HydroPower.SinksAndSources.Fixed_HT constantTailWater(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{71,-24},{91,-4}})));
    equation

      connect(powerGrid.f_grid, generator.f_grid) annotation (Line(
          points={{-68,43},{-60,43}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(powerGrid.P_grid_balance, generator.P_grid_balance) annotation (Line(
          points={{-68,57},{-60,57}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(pwr_ref.y, turbineGovernor.P_reference) annotation (Line(
          points={{-24.4,70},{-20,70},{-20,61}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(generator.f_out[1], turbineGovernor.f) annotation (Line(
          points={{-38,43},{-25,43}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(generator.onMCB, powerGrid.MCB) annotation (Line(
          points={{-49,61},{-49,81},{-79,81},{-79,61}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(generator.onMCB[1], turbineGovernor.isMCB) annotation (Line(
          points={{-49,61},{-49,81},{-14,81},{-14,61}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(generator.P_out[1], turbineGovernor.P_generator) annotation (Line(
          points={{-38,57},{-25,57}},
          color={0,0,127},
          smooth=Smooth.None));

      connect(generator.f_out, powerGrid.f) annotation (Line(
          points={{-38,43},{-32,43},{-32,9},{-93,9},{-93,43},{-90,43}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(turbine.a, pressureShaft.b) annotation (Line(
          points={{17,-62},{13,-62},{13,-43}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(turbine.b, downstream.a) annotation (Line(
          points={{39,-62},{43,-62},{43,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(downstream.b, river.a1_pipe) annotation (Line(
          points={{65,-40},{69,-40},{69,-48},{72,-48}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(powerGrid.J_grid, generator.J_grid)
        annotation (Line(points={{-68,50},{-68,50},{-60,50}}, color={0,0,127}));
      connect(turbineGovernor.y, turbine.yGV)
        annotation (Line(points={{-3,50},{34,50},{34,-51}},
                                                        color={0,0,127}));
      connect(generator.f_out[1], turbine.f_generator) annotation (Line(points={{-38,43},
              {-32,43},{-32,9},{22,9},{22,-51}},         color={0,0,127}));
      connect(turbine.TurbineData[1], generator.P_turbine[1]) annotation (Line(
            points={{28,-51.6667},{28,23},{-55,23},{-55,39}},            color={0,0,
              127}));
      connect(conduit.b,surgeTank. a)
        annotation (Line(points={{-48,-36},{-47,-36},{-47,-45},{-40,-45}},
                                                   color={0,0,255}));
      connect(reservoir1.a2_pipe, conduit.a) annotation (Line(points={{-74,-38},
              {-74,-36},{-70,-36}}, color={0,0,255}));
      connect(surgeTank.b, pressureShaft.a) annotation (Line(points={{-18,-45},
              {-18,-43},{-9,-43}}, color={0,0,255}));
      connect(constantWaterHead.b, reservoir1.a1_open) annotation (Line(points=
              {{-95,-3},{-97,-3},{-97,-26},{-96,-26}}, color={0,0,255}));
      connect(constantTailWater.b, river.a2_open) annotation (Line(points={{92,
              -14},{97,-14},{97,-36},{94,-36}}, color={0,0,255}));
      annotation (
        Diagram(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={1,1}), graphics={
            Rectangle(
              extent={{50,91},{90,7}},
              lineColor={215,215,215},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid,
              radius=2),           Text(
              extent={{54,13},{90,9}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="turbine power [MW]"),Text(
              extent={{57,73},{85,65}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="Main Circuit Breaker"),
                                Text(
              extent={{39,41},{103,37}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="grid power balance [MW]"),
                                Text(
              extent={{39,56},{103,52}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="generator frequency [Hz]"),
                                Text(
              extent={{38,27},{102,23}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="generator power [MW]")}),
        experiment(
          StopTime=600,
          Tolerance=1e-006,
          __Dymola_Algorithm="Radau"),
        Documentation(info="<html>
<p>This example illustrates a hydro power plant acting under no load and when connected to the power grid. </p>
<h4>Model experiment description</h4>
<p>When there is no load present the governor will only have the frequency error as input signal which will have the effect that the frequency of the hydro plant generator is controlled to equal the nominal frequency. This behaviour can be seen during the first 150s of simulation. </p>
<p>When 150s has passed and the frequency of the generator is synchronized to the grid frequency, the MCB is closed, the power reference is set to 45MW and new PID parameters are applied. </p>
<p>At time=350 load rejection takes place and the MCB opens once again. </p>
<h4>Simulation setup</h4>
<p>Simulate for 600s using solver Radau with a tolerance set to 1e-6.</p>
<h4>Output</h4>
<p>The most interesting variables are:</p>
<ul>
<li>generator frequency - generator.summary.f[1]</li>
<li>generated power - generator.summary.P_generator[1]</li>
</ul>
</html>",     revisions="<html>
<hr><p><font color=\"#E72614\"><b>Copyright &copy; 2004-2023, MODELON AB</b></font> <font color=\"#AFAFAF\"><br /><br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br /> This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br /> or by other means have their origin from any Modelon Library. </font></p>
</html>"),
        Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={1,1})),
        __Dymola_experimentSetupOutput);
    end Sundsbarm;
  end Tutorial7;
  annotation (uses(Modelica(version="4.0.0"), HydroPower(version="2.17"),
      Modelon(version="4.3")));
end FM3217_2023;
