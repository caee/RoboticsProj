%% Jacobian from P4 
clear,clc
j = linspace(0,36,37);
phi = linspace(0,2*pi,37);

p_0c = [150,0,120];
R=32;

p0 = zeros(37,3);
q_1 = zeros(37,1);
q_2 = zeros(37,1);
q_3 = zeros(37,1);
q_4 = zeros(37,1);

% horizontal stylus
varphi = 0;

% DH parameters
a_2 = 93; % [mm]
a_3 = 93; % [mm]
a_4 = 50; % [mm]

p0 = p0 + p_0c;


% det virker til at lade x-planet være konstant er good
for i = j
    p0(i+1,2) = p0(i+1,2) + R*cos(phi(i+1));
    p0(i+1,3) = p0(i+1,3) + R*sin(phi(i+1));
    
    x_s = p0(i+1,1);
    y_s = p0(i+1,2);
    z_s = p0(i+1,3)-50;
    x_hat = sqrt(x_s^2+y_s^2);

    p_03x_hat = x_hat - a_4*cos(varphi);
    p_03z = z_s - a_4*sin(varphi);
    x_test(i+1) = p_03x_hat;
    d = sqrt((p_03z)^2 + p_03x_hat^2);
    
    q_1(i+1,1) = atan2(y_s,x_s);
    
    %det her er "first option" for theta_3
    q_3(i+1,1) = pi - acos((a_2^2+a_3^2-d^2)/(2*a_2*a_3));

    % det her er "second option" for theta_3
    %q_3(i+1,1) = acos((-a_2^2-a_3^2+d^2)/(2*a_2*a_3));

    q_2(i+1,1) = atan2(p_03z,p_03x_hat) - atan2(a_3*sin(q_3(i+1,1)),a_2+a_3*cos(q_3(i+1,1)));

    q_4(i+1,1) = varphi - q_3(i+1,1) - q_2(i+1,1);

end

point = 28; % 1 = phi=0, 10= phi=pi/2, 19 = phi=pi, 28 = phi=3*pi/2, 

t_1 = q_1(point);
t_2 = q_2(point);
t_3 = q_3(point);
t_4 = q_4(point);
T01 = [cos(t_1) 0 sin(t_1) 0;
       sin(t_1) 0 -cos(t_1) 0;
       0 1 0 50;
       0 0 0 1];
T12 = [cos(t_2) -sin(t_2) 0 93*cos(t_2);
       sin(t_2) cos(t_2) 0 93*sin(t_2);
       0 0 1 0;
       0 0 0 1];
T23 = [cos(t_3) -sin(t_3) 0 93*cos(t_3);
       sin(t_3) cos(t_3) 0 93*sin(t_3);
       0 0 1 0;
       0 0 0 1];
T34 = [cos(t_4) -sin(t_4) 0 50*cos(t_4);
       sin(t_4) cos(t_4) 0 50*sin(t_4);
       0 0 1 0;
       0 0 0 1];
T45 = [-0.3162272401 -0.9486834734 0 -14.99997505;
        0.9486834734 -0.3162272401 0  45.00000831;
        0 0 1 0;
        0 0 0 1];

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;

z_0 = [0 0 1]';
z_1 = T01(1:3,3);
z_2 = T02(1:3,3);
z_3 = T03(1:3,3);

o_0 = [0 0 0]';
o_1 = T01(1:3,4);
o_2 = T02(1:3,4);
o_3 = T03(1:3,4);
o_4 = T04(1:3,4);
o_5 = T05(1:3,4);

J_stylus = zeros(6,4);
J_stylus(1:3,1) = cross(z_0,(o_4-o_0));
J_stylus(4:6,1) = z_0;
J_stylus(1:3,2) = cross(z_1,(o_4-o_1));
J_stylus(4:6,2) = z_1;
J_stylus(1:3,3) = cross(z_2,(o_4-o_2));
J_stylus(4:6,3) = z_2;
J_stylus(1:3,4) = cross(z_3,(o_4-o_3));
J_stylus(4:6,4) = z_3;

J_cam = zeros(6,4);
J_cam(1:3,1) = cross(z_0,(o_5-o_0));
J_cam(4:6,1) = z_0;
J_cam(1:3,2) = cross(z_1,(o_5-o_1));
J_cam(4:6,2) = z_1;
J_cam(1:3,3) = cross(z_2,(o_5-o_2));
J_cam(4:6,3) = z_2;
J_cam(1:3,4) = cross(z_3,(o_5-o_3));
J_cam(4:6,4) = z_3;


%% Force calc
%tau=J.F
%F=[Fx Fy Fz tau_xy tau_xz tau_yz]
F=[0 0 -1 0 0 0];
tau=J_stylus.F;