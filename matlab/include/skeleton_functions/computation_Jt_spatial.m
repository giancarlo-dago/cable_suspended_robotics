close all
clear
clc

syms p_a1x p_a1y p_a1z p_a2x p_a2y p_a2z p_b1x p_b1y p_b1z p_b2x p_b2y p_b2z real
sympref('FloatingPointOutput',false);

p_a1 = [p_a1x; p_a1y; p_a1z];
p_a2 = [p_a2x; p_a2y; p_a2z];
p_b1 = [p_b1x; p_b1y; p_b1z];
p_b2 = [p_b2x; p_b2y; p_b2z];

u_a = simplify((p_a2-p_a1)/(norm(p_a2-p_a1)));
u_b = simplify((p_b2-p_b1)/(norm(p_b2-p_b1)));
k = simplify(u_a'*u_b);

index = 1;

switch index
    case 1 % Case 1: 0<t_ac<normA & 0<t_bc<normB
        t_ac = simplify(((p_b1-p_a1)'*(u_a-k*u_b))/(1-k^2));
        t_bc = simplify((t_ac-u_a'*(p_b1-p_a1))/k);
    case 2 % Case 2: t_ac<=0 & 0<t_bc<normB
        t_ac = 0;
        t_bc = simplify((t_ac-u_a'*(p_b1-p_a1))/k);
    case 3 % Case 3: t_ac>=normA & 0<t_bc<normB
        t_ac = norm(p_a2-p_a1);
        t_bc = simplify((t_ac-u_a'*(p_b1-p_a1))/k);
    case 4 % Case 4: 0<t_ac<normA & t_bc<=0
        t_ac = simplify(((p_b1-p_a1)'*(u_a-k*u_b))/(1-k^2));
        t_bc = 0;
    case 5 % Case 5: 0<t_ac<normA & t_bc>=normB
        t_ac = simplify(((p_b1-p_a1)'*(u_a-k*u_b))/(1-k^2));
        t_bc = norm(p_b2-p_b1);
    case 6 % Case 6: t_ac<=0 & t_bc<=0
        t_ac = 0;
        t_bc = 0;
    case 7 % Case 7: t_ac<=0 & t_bc>=normB
        t_ac = 0;
        t_bc = norm(p_b2-p_b1);
    case 8 % Case 8: t_ac>=normA & t_bc<=0
        t_ac = norm(p_a2-p_a1);
        t_bc = 0;
    case 9 % Case 9: t_ac>=normA & t_bc>=normB
        t_ac = norm(p_a2-p_a1);
        t_bc = norm(p_b2-p_b1);
end
        
p_ac = simplify(p_a1+t_ac*u_a);
p_bc = simplify(p_b1+t_bc*u_b);


% Jt_11 = jacobian(p_ac,p_a1)
% Jt_12 = jacobian(p_ac,p_a2)
% Jt_13 = jacobian(p_ac,p_b1)
% Jt_14 = jacobian(p_ac,p_b2)
% Jt_21 = jacobian(p_bc,p_a1)
% Jt_22 = jacobian(p_bc,p_a2)
% Jt_23 = jacobian(p_bc,p_b1)
% Jt_24 = jacobian(p_bc,p_b2)

% Jt = [Jt_11 Jt_12 Jt_13 Jt_14;
%       Jt_21 Jt_22 Jt_23 Jt_24]
 