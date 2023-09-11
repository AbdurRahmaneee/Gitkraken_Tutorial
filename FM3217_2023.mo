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
      Motor motor
        annotation (Placement(transformation(extent={{-8,-8},{12,12}})));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end MotorDrive;
  end Tutorial2;
  annotation (uses(Modelica(version="4.0.0")));
end FM3217_2023;
