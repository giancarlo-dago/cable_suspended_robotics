run('DynamicsPlanar1RGravComp.m')

%% SINUSOIDAL

period = 4;
t = 0 : 0.04 : 2;
f = 1/period;
phi = 0;
A = 0.5;
lAz = 0.4157;
mA = 10.2 * 2;
g0 = 9.8;

colorlist = ["b";"r";"y";"m"];

values_vec = [1 0.8 0.5 0.3];
% values_vec = [0.4 0.3 0.25 0.2];

fig1 = figure('Renderer', 'painters', 'Position', [10 10 1800 500]);
fig2 = figure('Renderer', 'painters', 'Position', [10 10 500 500]);
fig3 = figure('Renderer', 'painters', 'Position', [10 10 1800 500]);

msg = "Amp";
labels = [msg+" 1", msg+" 0.8", msg+" 0.5", msg+" 0.3"];
% labels = [msg+" 0.4", msg+" 0.3", msg+" 0.25", msg+" 0.2"];


for h = 1:length(values_vec)

    A = values_vec(h);
%     lAz = values_vec(h);
%     mA = values_vec(h);
%     f = values_vec(h);
    
    qA_rest = A*cos(2*pi*f*t + phi);
    qA_left_ext = A*cos(2*pi*f*t + phi) - 1.0;
    qA_right_ext = A*cos(2*pi*f*t + phi) + 1.0;
    qAd = -(A*2*pi*f)*sin(2*pi*f*t + phi);
    qAdd = -A*(2*pi*f)^2*cos(2*pi*f*t + phi);

%     com_traj_y = lAz*sin(qA_left);
%     com_traj_z = -lAz*cos(qA_left);
    
    for k = 1:length(t)

        Fy_left_ext(k) = FyFun(lAz, mA/2, qA_left_ext(k), qAd(k), qAdd(k));
        Fy_right_ext(k) = FyFun(lAz, mA/2, qA_right_ext(k), qAd(k), qAdd(k));
        Fz_left_ext(k) = FzFun(lAz, mA/2, qA_left_ext(k), qAd(k), qAdd(k));
        Fz_right_ext(k) = FzFun(lAz, mA/2, qA_right_ext(k), qAd(k), qAdd(k));

        Fy_rest(k) = FyFun(lAz, mA, qA_rest(k), qAd(k), qAdd(k));
        Fz_rest(k) = FzFun(lAz, mA, qA_rest(k), qAd(k), qAdd(k));
        Fy_rest_sum(k) = 2*FyFun(lAz, mA/2, qA_rest(k), qAd(k), qAdd(k));
        Fz_rest_sum(k) = 2*FzFun(lAz, mA/2, qA_rest(k), qAd(k), qAdd(k));
       
    end

    figure(fig1)
    subplot(1,4,h)
    c = linspace(1,30,length(Fy_left_ext));
    scatter3(ones(1,51)*(-0.19), Fy_left_ext, Fz_left_ext, [], c, 'filled', 'LineWidth',2), hold on, grid on
    scatter3(ones(1,51)*0.19, Fy_right_ext, Fz_right_ext, [], c, 'filled', 'LineWidth',2), hold on, grid on
    scatter3(ones(1,51)*0.0, Fy_left_ext + Fy_right_ext, Fz_left_ext + Fz_right_ext, [], c, 'filled', 'LineWidth',2), hold on, grid on

    d = linspace(30,60,length(Fy_rest));
    scatter3(ones(1,51)*(0.0), Fy_rest, Fz_rest, [], 'filled', 'LineWidth',2), hold on, grid on
%     scatter3(-0.19, com_traj_y, com_traj_z, [], 'LineWidth',2), hold on, grid on
%     scatter3(0.19, com_traj_y, com_traj_z, [], 'LineWidth',2), hold on, grid on
    xlabel('Y [m]'), ylabel('Z [m]'), title(msg + " " + num2str(values_vec(h)))
    xlim([-0.3 0.3]), ylim([-25 25]), zlim([-25 25])

end


