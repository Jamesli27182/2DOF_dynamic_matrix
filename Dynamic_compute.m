%% 2DoF of robotic arm Dynamic computation file created by James Li/2024/09/19
classdef Dynamic_compute
% syms m1 m2 L1 L2 I1 I2 q1 q2 g q1_dotdot q1_dot q2_dotdot q2_dot
% eq1 = I1*q1_dotdot + I2*q1_dotdot + I2*q2_dotdot + (L1^2*m1*q1_dotdot)/4 + L1^2*m2*q1_dotdot + (L2^2*m2*q1_dotdot)/4 + (L2^2*m2*q2_dotdot)/4 + (L2*g*m2*cos(q1 + q2))/2 + (L1*g*m1*cos(q1))/2 + L1*g*m2*cos(q1) - (L1*L2*m2*sin(q2)*q2_dot^2)/2 + L1*L2*m2*cos(q2)*q1_dotdot + (L1*L2*m2*cos(q2)*q2_dotdot)/2 - L1*L2*m2*sin(q2)*q1_dot*q2_dot;
% eq2 = I2*q1_dotdot + I2*q2_dotdot + (L2^2*m2*q1_dotdot)/4 + (L2^2*m2*q2_dotdot)/4 + (L2*g*m2*cos(q1 + q2))/2 + (L1*L2*m2*sin(q2)*q1_dot^2)/2 + (L1*L2*m2*cos(q2)*q1_dotdot)/2
% eq1_findq1dd = collect(eq1,q1_dotdot);
% eq1_findq2dd = collect(eq1,q2_dotdot);
% eq1_findq1d  = collect(eq1,q1_dot);
% eq2_findq1dd = collect(eq2,q1_dotdot);
% eq2_findq2dd = collect(eq2,q2_dotdot);
% eq2_findq1d  = collect(eq2,q1_dot);
    methods(Static)
        function Inertia_Matrix = M(m1,m2,I1,I2,L1,L2,q2)
            M_i_1_1 = (I1 + I2 + (L1^2*m1)/4 + L1^2*m2 + (L2^2*m2)/4 + L1*L2*m2*cos(q2));
            M_i_1_2 = ((m2*L2^2)/4 + (L1*m2*cos(q2)*L2)/2 + I2);
            M_i_2_1 = ((m2*L2^2)/4 + (L1*m2*cos(q2)*L2)/2 + I2);
            M_i_2_2 = ((m2*L2^2)/4 + I2);
            Inertia_Matrix = [M_i_1_1,M_i_1_2;M_i_2_1,M_i_2_2;];
        end
        function Coriolis_matrix=C_c(m2,L1,L2,q1_dot,q2,q2_dot)
            C_c_11 =  (-L1*L2*m2*q2_dot*sin(q2));
            C_c_12 = - (L1*L2*m2*q2_dot*sin(q2))/2;
            C_c_21 = ((L1*L2*m2*sin(q2))/2)*q1_dot;
            C_c_22 = 0;
            Coriolis_matrix = [C_c_11,C_c_12;C_c_21,C_c_22];
        end
        function gravity = G_g(m1,m2,L1,L2,q1,q2,g)
            G_g1 = (L2*g*m2*cos(q1 + q2))/2 + (L1*g*m1*cos(q1))/2 + L1*g*m2*cos(q1);
            G_g2 = (L2*g*m2*cos(q1 + q2))/2;
            gravity = [G_g1;G_g2];
        end
        
        function Jacobian_matrix = J(L1,L2,q1,q2)
            Jacobian_matrix = [- L1*sin(q1) - L2*sin(q1 + q2), -L2*sin(q1 + q2);
                               L2*cos(q1 + q2) + L1*cos(q1),  L2*cos(q1 + q2)];
        end
        function Jacobian_matrix_dot = J_dot(L1,L2,q1,q2,q1_dot,q2_dot)
        Jacobian_matrix_dot = [- L2*cos(q1 + q2)*(q1_dot + q2_dot) - L1*cos(q1)*q1_dot, -L2*cos(q1 + q2)*(q1_dot + q2_dot);
                               - L2*sin(q1 + q2)*(q1_dot + q2_dot) - L1*sin(q1)*q1_dot, -L2*sin(q1 + q2)*(q1_dot + q2_dot)];
        end

        function end_position = endXY(L1,L2,q1,q2)
            end_position=[L1*cos(q1)+L2*cos(q1+q2);
                L1*sin(q1)+L2*sin(q1+q2)];
        end

    end
end