vdc = 12;
R = 1;

iq = 1;
d = 0:0.01:2;

idc = sqrt(3/2).*d.*iq;
vq = 0.5*(-sqrt(6).*d.*iq.*R+sqrt(6).*d.*vdc);

eta = (iq.*vq)./(idc.*vdc);
Pout = iq.*vq;

plot(Pout,eta)