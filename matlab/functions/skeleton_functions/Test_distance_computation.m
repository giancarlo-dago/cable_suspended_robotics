clear
close all
clc

p_a1 = [1 1 2]';
p_a2 = [1 4 3]';
p_b1 = [5 2 1]';
p_b2 = [3 3 1]';

figure();
plot3(p_a1(1),p_a1(2),p_a1(3),'o','Color','k'); hold on;
plot3(p_a2(1),p_a2(2),p_a2(3),'o','Color','k');
plot3(p_b1(1),p_b1(2),p_b1(3),'o','Color','k');
plot3(p_b2(1),p_b2(2),p_b2(3),'o','Color','k');
plot3([p_a1(1) p_a2(1)],[p_a1(2) p_a2(2)],[p_a1(3) p_a2(3)],'k')
plot3([p_b1(1) p_b2(1)],[p_b1(2) p_b2(2)],[p_b1(3) p_b2(3)],'k')
grid;

u_a = (p_a2-p_a1)/(norm(p_a2-p_a1));
u_b = (p_b2-p_b1)/(norm(p_b2-p_b1));
k = u_a'*u_b;
t_ac = ((p_b1-p_a1)'*(u_a-k*u_b))/(1-k^2);
t_bc = (t_ac-u_a'*(p_b1-p_a1))/k;

if (t_ac/norm(p_a2-p_a1)<=0)
    t_ac = 0;
    flagA = 2;
elseif (t_ac/norm(p_a2-p_a1)>=1)
    t_ac = norm(p_a2-p_a1);
    flagA = 3;
else
    flagA = 1;
end

if (t_bc/norm(p_b2-p_b1)<=0)
    t_bc = 0;
    flagB = 2;
elseif (t_bc/norm(p_b2-p_b1)>=1)
    t_bc = norm(p_b2-p_b1);
    flagB = 3;
else
    flagB = 1;
end

if flagA==1 && flagB==1             % Case 1: 0<t_ac<normA & 0<t_bc<normB
    index = 1;
elseif flagA==2 && flagB==1         % Case 2: t_ac<=0 & 0<t_bc<normB
    index = 2;
elseif flagA==3 && flagB==1         % Case 3: t_ac>=normA & 0<t_bc<normB
    index = 3;
elseif flagA==1 && flagB==2         % Case 4: 0<t_ac<normA & t_bc<=0
    index = 4;
elseif flagA==1 && flagB==3         % Case 5: 0<t_ac<normA & t_bc>=normB
    index = 5;
elseif flagA==2 && flagB==2         % Case 6: t_ac<=0 & t_bc<=0
    index = 6;
elseif flagA==2 && flagB==3         % Case 7: t_ac<=0 & t_bc>=normB
    index = 7;
elseif flagA==3 && flagB==2         % Case 8: t_ac>=normA & t_bc<=0
    index = 8;
elseif flagA==3 && flagB==3         % Case 9: t_ac>=normA & t_bc>=normB
    index = 9;
end

p_ac = p_a1+t_ac*u_a;
p_bc = p_b1+t_bc*u_b;

dmin = norm(p_ac-p_bc);
disp(index)
        
plot3(p_ac(1),p_ac(2),p_ac(3),'o','Color','r');
plot3(p_bc(1),p_bc(2),p_bc(3),'o','Color','r');
plot3([p_ac(1) p_bc(1)],[p_ac(2) p_bc(2)],[p_ac(3) p_bc(3)],'r')
