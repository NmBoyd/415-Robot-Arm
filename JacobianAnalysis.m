clc
clear all
%% Calculate the forward kinematics of the arm for position
t_1=0;  %0:360
t_2=0;  %-60:250
t_3=0;  %0:360
t_4=0;  %-160:160
t_5=0;  %0:360
t_6=0;  %-160:160

% syms t_1 t_2 t_3 t_4 t_5 t_6

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
T_0e = T_01*T_12*T_23*T_34*T_45*T_56
%this method didn't work. refer to angela sodeman video about using the
%rotaton matrix to determine jacobian
R_00 = [1 0 0; 0 1 0; 0 0 1];
ddq0a = cross(R_00*[0;0;1],(T_0e(1:3,4)-[0;0;0]))
ddq0b = (R_00*[0;0;1])
ddq1a = cross(T_01(1:3,1:3)*[0;0;1],(T_0e(1:3,4)-T_01(1:3,4)))
ddq1b = (T_01(1:3,1:3)*[0;0;1])
T_02 = T_01*T_12;
ddq2a = cross(T_02(1:3,1:3)*[0;0;1],(T_0e(1:3,4)-T_02(1:3,4)))
ddq2b = (T_02(1:3,1:3)*[0;0;1])
T_03 = T_01*T_12*T_23;
ddq3a = cross(T_03(1:3,1:3)*[0;0;1],(T_0e(1:3,4)-T_03(1:3,4)))
ddq3b = (T_03(1:3,1:3)*[0;0;1])
T_04 = T_01*T_12*T_23*T_34;
ddq4a = cross(T_04(1:3,1:3)*[0;0;1],(T_0e(1:3,4)-T_04(1:3,4)))
ddq4b = (T_04(1:3,1:3)*[0;0;1])
T_05 = T_01*T_12*T_23*T_34;
ddq5a = cross(T_05(1:3,1:3)*[0;0;1],(T_0e(1:3,4)-T_05(1:3,4)))
ddq5b = (T_05(1:3,1:3)*[0;0;1])
T_06 = T_01*T_12*T_23*T_34*T_56;
ddq6a = cross(T_06(1:3,1:3)*[0;0;1],(T_0e(1:3,4)-T_06(1:3,4)))
ddq6b = (T_06(1:3,1:3)*[0;0;1])
%solve for jacobian matrix and end effector velocity
jacob_matrix = [ddq0a ddq1a ddq2a ddq3a ddq4a ddq5a; ddq0b ddq1b ddq2b ddq3b ddq4b ddq5b]
ee_vel = jacob_matrix*[0 0 0 0 0 0]'