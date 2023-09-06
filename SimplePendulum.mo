within ;
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
