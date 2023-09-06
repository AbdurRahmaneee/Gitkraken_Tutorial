within ;
model SimplePendulum
  constant Real g(unit="m/s2") = 9.81;
  parameter Real L(min=0, unit="m") = 1;
  Real Theta(start=0.1, fixed=true);
  Real ThetaDot;
equation
  ThetaDot = der(Theta);
  der(ThetaDot) = - g/L * sin(Theta);
end SimplePendulum;
