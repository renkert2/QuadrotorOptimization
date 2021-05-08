[r,speed_max,t,theta_max] = QROpt.Range('MaxPitch', true);
rotor_speed_max = QROpt.calcSteadyStateIO(1).RotorSpeed;
D = QROpt.Propeller.D.Value;
n = rotor_speed_max * (u.rad/u.rev); % rad/s to rev/s
J = speed_max*abs(sin(theta_max)) / (D*n)