clear all;
close all;

s = linspace(-pi,pi,1000);
convWidth = (56/2)*2^(1/2);

p = cos(s/2);
n = 180/convWidth;

x = p.*sin(n.*s);
y = p.*cos(n.*s);
z = sin(s/2);

figure(1)
subplot(2,2,1)
plot3(x,y,z)
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')

%change to polar
r = (x.^2+y.^2+z.^2);
theta = acosd(z./r);
phi = atand(y./x);

%modify helix (1)
for i=1:length(x)
    if (x(i) < 0 && y(i) < 0)
        phi(i) = 180 - phi(i);
    elseif (x(i) > 0 && y(i) < 0)
        phi(i) = phi(i) * -1;
    elseif (x(i) < 0 && y(i) > 0)
        phi(i) = phi(i) + 180;
    end
end

theta = deg2rad(theta);
phi = deg2rad(phi); 
x1 = r.*sin(theta).*cos(phi);
y1 = r.*sin(theta).*sin(theta);
z1 = r.*cos(theta);
            

subplot(2,2,2)
plot3(x1,y1,z1)
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')

for i=1:length(x)
     if(z(i) > 0 && x(i) < 0)
         x(i) = -x(i);
     end
     if(z(i) < 0)
         if(x(i) >0)
             x(i) = -x(i);
         end
         z(i) = -z(i);
     end
end

subplot(2,2,3)
plot3(x,y,z)
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')

