clear
clc
close all

set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')

lw = 2;

%% COMPARISON CARTESIAN POSITION ERROR
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
load reference_traj_25s.mat
tx_eb = 0.075;
tz_eb = 0.0275;
ti=0;
te=468;
%p_ref(313+1:te,1:3) = repmat(p_ref(313,1:3),te-313,1);
%o_ref(313+1:te,1:3) = repmat(o_ref(313,1:3),te-313,1);

p_ref(313+1:te,1:3) = repmat(p_ref(313,1:3),te-313,1);
o_ref(313+1:te,1:3) = repmat([1.57,0,0],te-313,1);

t = ti:0.005:(te-ti-1)*0.005;

cart = readmatrix('cart_pos.txt');
for i = 1:size(cart,1)
    ep_norm(i) = norm(cart(i,1:3)-p_ref(i,1:3)-[tx_eb,0,-tz_eb]);
    eo_norm(i) = norm(cart(i,4:6)-o_ref(i,1:3));
end
ep_norm(313:te) = ep_norm(313);
eo_norm(313:te) = eo_norm(313);


plot(t, ep_norm, 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(t, eo_norm, 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);

cart = readmatrix('cart_pos2.txt');
clear ep_norm eo_norm

for i = 1:size(cart,1)
    ep_norm(i) = norm(cart(i,1:3)-p_ref(i,1:3)-[tx_eb,0,-tz_eb]);
    eo_norm(i) = norm(cart(i,4:6)-o_ref(i,1:3));
end

plot(t, ep_norm, 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(t, eo_norm, 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);

legend('$e_{p,r}$', '$e_{o,r}$', '$e_{p,m}$', '$e_{o,m}$');
xlabel('t [s]')
ylabel('$\mathcal{E}$ [m or rad]')
set(gca, 'FontSize',16);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');
exportgraphics(h, 'rodyman_cartesian_position.pdf');


%% COMPARISON OBJECT DISPLACEMENT
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
load obj_pos.mat
ti=655;
te=ti+53-1;
for i = ti+1:te
    disp_norm(i-ti) = norm([obj_pos{i}.Pose.Position.X, obj_pos{i}.Pose.Position.Y, obj_pos{i}.Pose.Position.Z]-[obj_pos{ti}.Pose.Position.X, obj_pos{ti}.Pose.Position.Y, obj_pos{ti}.Pose.Position.Z]);
end
t = 0:0.062:(te-ti-1)*0.062;
plot(t,disp_norm, 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
hold on

load obj_pos2.mat
clear disp_norm
ti=655;
te=ti+53-1;
for i = ti+1:te
    disp_norm(i-ti) = norm([obj_pos{i}.Pose.Position.X, obj_pos{i}.Pose.Position.Y, obj_pos{i}.Pose.Position.Z]-[obj_pos{ti}.Pose.Position.X, obj_pos{ti}.Pose.Position.Y, obj_pos{ti}.Pose.Position.Z]);
end
t = 0:0.062:(te-ti-1)*0.062;
plot(t,disp_norm, 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);

legend('$d_r$', '$d_m$');
xlabel('t [s]')
ylabel('$\mathcal{D}$ [m]')
set(gca, 'FontSize',16);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');
exportgraphics(h, 'rodyman_object_displacement.pdf');