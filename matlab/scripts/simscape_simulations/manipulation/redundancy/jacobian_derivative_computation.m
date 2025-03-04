clear
close all
clc

syms th1A th2A th3A th1B th2B th3B ...
     a1A a2A a3A a1B a2B a3B offA offB ...
     ml1A ml2A ml3A ml1B ml2B ml3B ...
     l1A l2A l3A l1B l2B l3B real
       
% Center of Mass of the arms
pl1A = [offA+l1A*sin(th1A), -l1A*cos(th1A)]';
pl2A = [offA+a1A*sin(th1A)+l2A*sin(th1A+th2A), -a1A*cos(th1A)-l2A*cos(th1A+th2A)]';
pl3A = [offA+a1A*sin(th1A)+a2A*sin(th1A+th2A)+l3A*sin(th1A+th2A+th3A), -a1A*cos(th1A)-a2A*cos(th1A+th2A)-l3A*cos(th1A+th2A+th3A)]';
pl1B = [offB+l1B*sin(th1B), -l1B*cos(th1B)]';
pl2B = [offB+a1B*sin(th1B)+l2B*sin(th1B+th2B), -a1B*cos(th1B)-l2B*cos(th1B+th2B)]';
pl3B = [offB+a1B*sin(th1B)+a2B*sin(th1B+th2B)+l3B*sin(th1B+th2B+th3B), -a1B*cos(th1B)-a2B*cos(th1B+th2B)-l3B*cos(th1B+th2B+th3B)]';

cmxA = (ml1A*pl1A(1)+ml2A*pl2A(1)+ml3A*pl3A(1))/(ml1A+ml2A+ml3A);
cmyA = (ml1A*pl1A(2)+ml2A*pl2A(2)+ml3A*pl3A(2))/(ml1A+ml2A+ml3A);
cmxB = (ml1B*pl1B(1)+ml2B*pl2B(1)+ml3B*pl3B(1))/(ml1B+ml2B+ml3B);
cmyB = (ml1B*pl1B(2)+ml2B*pl2B(2)+ml3B*pl3B(2))/(ml2B+ml2B+ml3B);
cmx = (cmxA + cmxB)/2;

% Cinematica diretta CLIK 1                                                            
xe = [ offA + a1A*sin(th1A) + a2A*sin(th1A + th2A) + a3A*sin(th1A + th2A + th3A);
            - a1A*cos(th1A) - a2A*cos(th1A + th2A) - a3A*cos(th1A + th2A + th3A);
       offB + a1B*sin(th1B) + a2B*sin(th1B + th2B) + a3B*sin(th1B + th2B + th3B);
            - a1B*cos(th1B) - a2B*cos(th1B + th2B) - a3B*cos(th1B + th2B + th3B)];
        
% Cinematica diretta CLIK 2
% xe = [ offA + a1A*sin(th1A) + a2A*sin(th1A + th2A) + a3A*sin(th1A + th2A + th3A);
%             - a1A*cos(th1A) - a2A*cos(th1A + th2A) - a3A*cos(th1A + th2A + th3A);
%        offB + a1B*sin(th1B) + a2B*sin(th1B + th2B) + a3B*sin(th1B + th2B + th3B);
%             - a1B*cos(th1B) - a2B*cos(th1B + th2B) - a3B*cos(th1B + th2B + th3B);
%                                                                             cmx];


q = [th1A th2A th3A th1B th2B th3B];
J = jacobian(xe,q);

syms t th1A(t) th2A(t) th3A(t) th1B(t) th2B(t) th3B(t) real 

% CLIK 1
Ja = [a2A*cos(th1A + th2A) + a1A*cos(th1A) + a3A*cos(th1A + th2A + th3A), a2A*cos(th1A + th2A) + a3A*cos(th1A + th2A + th3A), a3A*cos(th1A + th2A + th3A),                                                                  0,                                                  0,                           0;
      a2A*sin(th1A + th2A) + a1A*sin(th1A) + a3A*sin(th1A + th2A + th3A), a2A*sin(th1A + th2A) + a3A*sin(th1A + th2A + th3A), a3A*sin(th1A + th2A + th3A),                                                                  0,                                                  0,                           0;
                                                                       0,                                                  0,                           0, a2B*cos(th1B + th2B) + a1B*cos(th1B) + a3B*cos(th1B + th2B + th3B), a2B*cos(th1B + th2B) + a3B*cos(th1B + th2B + th3B), a3B*cos(th1B + th2B + th3B);
                                                                       0,                                                  0,                           0, a2B*sin(th1B + th2B) + a1B*sin(th1B) + a3B*sin(th1B + th2B + th3B), a2B*sin(th1B + th2B) + a3B*sin(th1B + th2B + th3B), a3B*sin(th1B + th2B + th3B)];

% CLIK 2
% Ja = [                                                                                                     a2A*cos(th1A + th2A) + a1A*cos(th1A) + l3A*cos(th1A + th2A + th3A),                                                               a2A*cos(th1A + th2A) + l3A*cos(th1A + th2A + th3A),                                 l3A*cos(th1A + th2A + th3A),                                                                                                                                                                       0,                                                                                                                0,                                                           0;
%                                                                                                            a2A*sin(th1A + th2A) + a1A*sin(th1A) + l3A*sin(th1A + th2A + th3A),                                                               a2A*sin(th1A + th2A) + l3A*sin(th1A + th2A + th3A),                                 l3A*sin(th1A + th2A + th3A),                                                                                                                                                                       0,                                                                                                                0,                                                           0;
%                                                                                                                                                                             0,                                                                                                                0,                                                           0,                                                                                                      a2B*cos(th1B + th2B) + a1B*cos(th1B) + l3B*cos(th1B + th2B + th3B),                                                               a2B*cos(th1B + th2B) + l3B*cos(th1B + th2B + th3B),                                 l3B*cos(th1B + th2B + th3B);
%                                                                                                                                                                             0,                                                                                                                0,                                                           0,                                                                                                      a2B*sin(th1B + th2B) + a1B*sin(th1B) + l3B*sin(th1B + th2B + th3B),                                                               a2B*sin(th1B + th2B) + l3B*sin(th1B + th2B + th3B),                                 l3B*sin(th1B + th2B + th3B);
%       (ml3A*(a2A*cos(th1A + th2A) + a1A*cos(th1A) + l3A*cos(th1A + th2A + th3A)) + ml2A*(l2A*cos(th1A + th2A) + a1A*cos(th1A)) + l1A*ml1A*cos(th1A))/(2*(ml1A + ml2A + ml3A)), (ml3A*(a2A*cos(th1A + th2A) + l3A*cos(th1A + th2A + th3A)) + l2A*ml2A*cos(th1A + th2A))/(2*(ml1A + ml2A + ml3A)), (l3A*ml3A*cos(th1A + th2A + th3A))/(2*(ml1A + ml2A + ml3A)), (ml3B*(a2B*cos(th1B + th2B) + a1B*cos(th1B) + l3B*cos(th1B + th2B + th3B)) + ml2B*(l2B*cos(th1B + th2B) + a1B*cos(th1B)) + l1B*ml1B*cos(th1B))/(2*(ml1B + ml2B + ml3B)), (ml3B*(a2B*cos(th1B + th2B) + l3B*cos(th1B + th2B + th3B)) + l2B*ml2B*cos(th1B + th2B))/(2*(ml1B + ml2B + ml3B)), (l3B*ml3B*cos(th1B + th2B + th3B))/(2*(ml1B + ml2B + ml3B))];
% 

% CLIK 3


%% Derivata dello Jacobiano analitico

Jad = diff(Ja,'t');

syms dth1A dth2A dth3A dth1B dth2B dth3B real

Jad_new = subs(Jad,diff(th1A(t),t),dth1A);
Jad_new = subs(Jad_new,diff(th2A(t),t),dth2A);
Jad_new = subs(Jad_new,diff(th3A(t),t),dth3A);
Jad_new = subs(Jad_new,diff(th1B(t),t),dth1B);
Jad_new = subs(Jad_new,diff(th2B(t),t),dth2B);
Jad_new = subs(Jad_new,diff(th3B(t),t),dth3B);

syms th1A_t th2A_t th3A_t th1B_t th2B_t th3B_t real
Jad_new = subs(Jad_new,th1A(t),th1A_t);
Jad_new = subs(Jad_new,th2A(t),th2A_t);
Jad_new = subs(Jad_new,th3A(t),th3A_t);
Jad_new = subs(Jad_new,th1B(t),th1B_t);
Jad_new = subs(Jad_new,th2B(t),th2B_t);
Jad_new = subs(Jad_new,th3B(t),th3B_t);

syms th1A th2A th3A th1B th2B th3B real
Jad_new = subs(Jad_new,th1A_t,th1A);
Jad_new = subs(Jad_new,th2A_t,th2A);
Jad_new = subs(Jad_new,th3A_t,th3A);
Jad_new = subs(Jad_new,th1B_t,th1B);
Jad_new = subs(Jad_new,th2B_t,th2B);
Jad_new = subs(Jad_new,th3B_t,th3B);
