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
        Modelica.Electrical.Analog.Basic.Resistor resistor
          annotation (Placement(transformation(extent={{-46,32},{-26,52}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor
          annotation (Placement(transformation(extent={{-2,32},{18,52}})));
        Modelica.Electrical.Analog.Basic.RotationalEMF emf
          annotation (Placement(transformation(extent={{22,-10},{42,10}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-80,-76},{-60,-56}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{62,-10},{82,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
          "Flange of right shaft"
          annotation (Placement(transformation(extent={{94,-10},{114,10}})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p
          "Positive electrical pin" annotation (Placement(transformation(extent
                ={{-144,44},{-88,100}}), iconTransformation(extent={{-144,44},{
                  -88,100}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n
          "Negative electrical pin" annotation (Placement(transformation(extent
                ={{-142,-100},{-86,-44}}), iconTransformation(extent={{-142,
                  -100},{-86,-44}})));
      equation
        connect(resistor.n, inductor.p)
          annotation (Line(points={{-26,42},{-2,42}}, color={0,0,255}));
        connect(inductor.n, emf.p)
          annotation (Line(points={{18,42},{32,42},{32,10}}, color={0,0,255}));
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{42,0},{62,0}}, color={0,0,0}));
        connect(inertia.flange_b, flange)
          annotation (Line(points={{82,0},{104,0}}, color={0,0,0}));
        connect(resistor.p, p) annotation (Line(points={{-46,42},{-78,42},{-78,
                72},{-116,72}}, color={0,0,255}));
        connect(emf.n, n) annotation (Line(points={{32,-10},{32,-72},{-114,-72}},
              color={0,0,255}));
        connect(emf.n, ground.p) annotation (Line(points={{32,-10},{32,-56},{
                -70,-56}}, color={0,0,255}));
        connect(n, n)
          annotation (Line(points={{-114,-72},{-114,-72}}, color={0,0,255}));
        connect(p, p) annotation (Line(points={{-116,72},{-116,72},{-116,72},{
                -116,72}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics
              ={Bitmap(extent={{94,-48},{-92,74}}, fileName=
                    "modelica://FM3217_2023/Resources/Images/dc-motor.jpg")}),
                                                                       Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          Documentation(info="<html>
<h4>This is a model of a simple DC machine.</h4>
<p><img src=\"modelica://FM3217_2023/Resources/Images/dc-motor.jpg\"/></p>
</html>"));
      end DCMachine;
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
        Components.DCMachine dCMachine
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
    end Tests;
  end Tutorial3;
  annotation (uses(Modelica(version="4.0.0")));
end FM3217_2023;
