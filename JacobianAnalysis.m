clc
clear all
%% Calculate the forward kinematics of the arm for position
% t_1=0;  %0:360
% t_2=0;  %-60:250
% t_3=0;  %0:360
% t_4=0;  %-160:160
% t_5=0;  %0:360
% t_6=0;  %-160:160

syms t_1 t_2 t_3 t_4 t_5 t_6

d_01=1;
d_12=0;
d_23=0.5;
d_34=0.5;
d_45=0.5;
d_56=0.5;
d_6e=0.1;

T_01=[cosd(t_1) -sind(t_1)*cosd(90) sind(t_1)*sind(90) 0;
      sind(t_1) cosd(t_1)*cosd(90) -cosd(t_1)*sind(90) 0;
      0 sind(90) cosd(90) d_01;
      0 0 0 1]
  
T_12=[cosd(90+t_2) -sind(90+t_2)*cosd(90) sind(90+t_2)*sind(90) 0;
      sind(90+t_2) cosd(90+t_2)*cosd(90) -cosd(90+t_2)*sind(90) 0;
      0 sind(90) cosd(90) d_12;
      0 0 0 1]
  
T_23=[cosd(t_3) -sind(t_3)*cosd(-90) sind(t_3)*sind(-90) 0;
      sind(t_3) cosd(t_3)*cosd(-90) -cosd(t_3)*sind(-90) 0;
      0 sind(-90) cosd(-90) (d_23+d_34);
      0 0 0 1]
  
T_34=[cosd(t_4) -sind(t_4)*cosd(90) sind(t_4)*sind(90) 0;
      sind(t_4) cosd(t_4)*cosd(90) -cosd(t_4)*sind(90) 0;
      0 sind(90) cosd(90) 0;
      0 0 0 1]

T_45=[cosd(t_5) -sind(t_5)*cosd(-90) sind(t_5)*sind(-90) 0;
      sind(t_5) cosd(t_5)*cosd(-90) -cosd(t_5)*sind(-90) 0;
      0 sind(-90) cosd(-90) (d_45+d_56);
      0 0 0 1]
T_56=[cosd(-90+t_6) -sind(-90+t_6)*cosd(0) sind(-90+t_6)*sind(0) (d_6e)*cosd(-90+t_6);
      sind(-90+t_6) cosd(-90+t_6)*cosd(0) -cosd(-90+t_6)*sind(0) (d_6e)*sind(-90+t_6);
      0 sind(0) cosd(0) 0;
      0 0 0 1]
  
P = [0 0 0 1]'
T_0e = T_01*T_12*T_23*T_34*T_45*T_56*P
%this method didn't work. refer to angela sodeman video about using the
%rotaton matrix to determine jacobian
x = jacobian(T_0e, [t_1 t_2 t_3 t_4 t_5 t_6]);
x=simplify(x)
t_1n=0;  %0:360
t_2n=0;  %-60:250
t_3n=0;  %0:360
t_4n=0;  %-160:160
t_5n=0;  %0:360
t_6n=0;  %-160:160
jacobian_matrix = subs(x,[t_1 t_2 t_3 t_4 t_5 t_6],[t_1n t_2n t_3n t_4n t_5n t_6n])
t_dot = [1 0 0 0 0 0]';
ee_vel = jacobian_matrix*t_dot