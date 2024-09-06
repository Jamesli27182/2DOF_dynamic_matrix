%% trydynamic_matrix
%m1=1 m2=1 L1=1 L2=1 I1=1 I2=1 q1=0 q2=0 g=9.81 q1_dotdot=0 q1_dot=0
%q2_dotdot=0 q2_dot=0
%t1 = 2*mgL=2*1*1*9.81Nm=19.6200Nm
%t2 = 0.5*mg*L=0.5*1*9.81*1=4.9050Nm
%2try q1 = pi/2,q2 = 0; q1dot=...q2dotdot=0,t1=t2=0
%3try q1 = pi/4,q2 = pi/4; q1dot=...q2dotdot=0,t1=mgl/1.414+mgl/2.828=10.4066Nm t2=0

clear
m1=1; m2=1; L1=1; L2=1; I1=1; I2=1; q1=0; q2=0; g=9.81; 
%input q1 q2 q1dot q2dot q1dotdot q2dotdot
q1=pi/4;q2=pi/4;
q1_dotdot=0; q1_dot=0;
q2_dotdot=0; q2_dot=0;
q =[q1;q2];
q_dot=[q1_dot;q2_dot];
q_dotdot=[q1_dotdot;q2_dotdot];
M_i_1_1 = 2*I1 + 2*I2 + (L1^2*m1)/4 + L1^2*m2 + (L2^2*m2)/4 + L1*L2*m2*cos(q2);
M_i_1_2 = (m2*L2^2)/4 + (L1*m2*cos(q2)*L2)/2;
M_i_2_1 = ((m2*L2^2)/4 + (L1*m2*cos(q2)*L2)/2);
M_i_2_2 = ((L2^2*m2)/4);
M_i = [M_i_1_1,M_i_1_2;M_i_2_1,M_i_2_2;];
C_c_11 = (-L1*L2*m2*q2_dot*sin(q2));
C_c_12 = (L1*L2*m2*q2_dot*sin(q2))/2;
C_c_21 = ((L1*L2*m2*sin(q2))/2)*q1_dot;
C_c_22 = 0;
C_c = [C_c_11,C_c_12;C_c_21,C_c_22];
G_g1 = (L2*g*m2*cos(q1 + q2))/2 + (L1*g*m1*cos(q1))/2 + L1*g*m2*cos(q1);
G_g2 = (L2*g*m2*cos(q1 + q2))/2;
G_g = [G_g1;G_g2];
%output
torque = M_i*q_dotdot+C_c*q_dot+G_g