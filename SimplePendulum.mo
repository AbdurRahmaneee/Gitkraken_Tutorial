within ;
model SimplePendulum
  constant Modelica.Units.SI.Acceleration g = 9.81;
  parameter Modelica.Units.SI.Length L = 1;
  Modelica.Units.SI.Angle Theta(start=0.1, fixed=true);
  Modelica.Units.SI.AngularVelocity ThetaDot;
equation
  ThetaDot = der(Theta);
  der(ThetaDot) = - g/L * sin(Theta);
end SimplePendulum;
