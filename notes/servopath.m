clear all;
close all;

s = linspace(-pi,pi,1000);
convWidth = (56/2)*2^(1/2);

p = cos(s/2);
n = 180/convWidth;

x = p.*sin(n.*s);
y = p.*cos(n.*s);
z = sin(s/2);

r = x.^2 + y.^2 + z.^2;
thetad = acosd(z./r);
phid = atand(y./x) + 90;

theta = deg2rad(thetad);
phi = deg2rad(phid); 
x1 = r.*sin(theta).*cos(phi);
y1 = r.*sin(theta).*sin(phi);
z1 = r.*cos(theta);

figure(1)
plot3(x,y,z)

figure(2)
plot3(x1,y1,z1)


