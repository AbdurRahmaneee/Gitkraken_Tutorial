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
      extends PipeWithValve;
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
        m_dot_nom=110e3,
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
    end SimpleWaterWay;
  end Tutorial5;
  annotation (uses(Modelica(version="4.0.0"), HydroPower(version="2.17")));
end FM3217_2023;
