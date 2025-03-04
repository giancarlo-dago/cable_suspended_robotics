run('DynamicsPlanar1R.m')

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

    com_traj_y = lAz*sin(qA);
    com_traj_z = -lAz*cos(qA);
    
    for k = 1:length(t)

        Fy_left_ext(k) = FyFun(lAz, mA/2, qA_left_ext(k), qAd(k), qAdd(k));
        Fy_right_ext(k) = FyFun(lAz, mA/2, qA_right_ext(k), qAd(k), qAdd(k));
        Fz_left_ext(k) = FzFun(g0, lAz, mA/2, qA_left_ext(k), qAd(k), qAdd(k));
        Fz_right_ext(k) = FzFun(g0, lAz, mA/2, qA_right_ext(k), qAd(k), qAdd(k));

        Fy_rest(k) = FyFun(lAz, mA, qA_rest(k), qAd(k), qAdd(k));
        Fz_rest(k) = FzFun(g0, lAz, mA, qA_rest(k), qAd(k), qAdd(k));
        Fy_rest_sum(k) = 2*FyFun(lAz, mA/2, qA_rest(k), qAd(k), qAdd(k));
        Fz_rest_sum(k) = 2*FzFun(g0, lAz, mA/2, qA_rest(k), qAd(k), qAdd(k));
        

%         Fy_left_ext(k) = FyFun(g0, lAz, mA/2, qA_left_ext(k), qAdd(k));
%         Fy_right_ext(k) = FyFun(g0, lAz, mA/2, qA_right_ext(k), qAdd(k));
%         Fz_left_ext(k) = FzFun(g0, lAz, mA/2, qA_left_ext(k), qAd(k));
%         Fz_right_ext(k) = FzFun(g0, lAz, mA/2, qA_right_ext(k), qAd(k));
% 
%         Fy_rest(k) = FyFun(g0, lAz, mA, qA_rest(k), qAdd(k));
%         Fz_rest(k) = FzFun(g0, lAz, mA, qA_rest(k), qAd(k));

% 
%         F_y(k) = FyFun(lAz, mA, qA(k), qAd(k), qAdd(k));
%         F_z(k) = FzFun(g0, lAz, mA, qA(k), qAd(k), qAdd(k));
%         F_mod(k) = sqrt(F_y(k).^2 + F_z(k).^2);
%         ang_par(k) = acos(F_y(k)/F_mod(k)) * 180/pi;
%         ang_par_2(k) = asin(F_z(k)/F_mod(k)) * 180/pi;
%         ang_tot(k) = atan2(F_y(k), F_z(k)) *180/pi;
%     
%         figure(fig1)    
%         subplot(1,4,h)
%         quiver(0, 0, F_y(k), F_z(k), 'Color', '[0.8 0.8 0.8]'), hold on
%         plot([0 com_traj_y(k)], [0 com_traj_z(k)], 'Color', '[0.4 0.4 0.4]'), hold on
%         xlim([-0.7 0.7]), ylim([-0.7+5 0.7+5]), axis square
%         xlim([-0.7 0.7]), ylim([-1.01 0.29]), axis square

%         if h==1
%             Fy_cont1(k) = -lAz*mA*(sin(qA(k))*qAd(k)^2);
%             Fy_cont2(k) = lAz*mA*(qAdd(k)*cos(qA(k)));
%             Fz_cont1(k) = mA*(lAz*cos(qA(k))*qAd(k)^2);
%             Fz_cont2(k) = mA*(lAz*qAdd(k)*sin(qA(k)));
%         end

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
    xlabel('Y [m]'), ylabel('Z [m]'), title(msg + " " + num2str(values_vec(h))), axis square
%     xlim([-0.3 0.3]), ylim([-25 25]), zlim([-25 25])


%     figure(fig1)    
%     subplot(1,4,h)
%     c = linspace(1,10,length(com_traj_z));
%     scatter(F_y, F_z, [],c,'filled', 'LineWidth',2), hold on, grid on
%     scatter(com_traj_y, com_traj_z,[], c, 'LineWidth',2), hold on, grid on
%     xlabel('Y [m]'), ylabel('Z [m]'), title(msg + " " + num2str(values_vec(h))), axis square
% 
%     figure(fig2)
%     plot([0 F_y], [0 F_z], 'LineWidth',2), hold on, xlim([-0.7 0.7]), ylim([-0.7 0.7]), grid on,
%     xlabel('Y [m]'), ylabel('Z [m]'), legend(labels), axis square

%     figure(3);
%     subplot 321
%     plot (t,qA, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{A}$','Interpreter','latex')
%     subplot 323
%     plot (t,F_y, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{y}$','Interpreter','latex')
%     subplot 325
%     plot (t,F_z, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{z}$','Interpreter','latex')
%     subplot 322
%     plot (t,F_mod, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{tot}$','Interpreter','latex')
%     subplot 324
%     plot (t,ang_par, 'LineWidth',2), hold on, grid on, xlabel('time')
%     subplot 326
%     plot (t,ang_tot, 'LineWidth',2), hold on, grid on, xlabel('time')
% 
%     figure(4);
%     subplot 311
%     plot (t,qA, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{A}$','Interpreter','latex')
%     subplot 312
%     plot (t,qAd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{A}$','Interpreter','latex')
%     subplot 313
%     plot (t,qAdd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\ddot{q}_{A}$','Interpreter','latex')


end
% 
% figure(fig3)
% subplot(1,4,1), axis square
% plot (t,Fy_cont1, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_y$ - Centripetal','Interpreter','latex'), legend('sinusoidal')
% subplot(1,4,2), axis square
% plot (t,Fy_cont2, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_y$ - Motion','Interpreter','latex'), legend('sinusoidal')
% subplot(1,4,3), axis square
% plot (t,Fz_cont1, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_z$ - Centripetal','Interpreter','latex'), legend('sinusoidal')
% subplot(1,4,4), axis square
% plot (t,Fz_cont2, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_z$ - Motion','Interpreter','latex'), legend('sinusoidal')

% exportgraphics(fig1, 'Planar1RGravComp_Amplitudes.pdf');
% exportgraphics(fig2, 'Planar1RGravComp_AmplitudesComparison.pdf');
%  
% 
% %% CUBIC 
% 
% addpath('../../../../functions/trajectory_generation_functions/')
% 
% 
% % close all
% clc
% 
% period = 4;
% dt = 0.04;
% lAz = 0.5;
% mA = 0.5;
% g0 = 9.8;
% A = 0.5;
% colorlist = ["b";"r";"y";"m"];
% 
% values_vec = [1 0.8 0.5 0.3];
% % values_vec = [0.4 0.3 0.25 0.2];
% 
% fig1 = figure('Renderer', 'painters', 'Position', [10 10 1800 500]);
% fig2 = figure('Renderer', 'painters', 'Position', [10 10 500 500]);
% 
% 
% msg = "Amp";
% labels = [msg+" 1", msg+" 0.8", msg+" 0.5", msg+" 0.3"];
% % labels = [msg+" 0.4", msg+" 0.3", msg+" 0.25", msg+" 0.2"];
% 
% 
% for h=1:length(values_vec)
% 
%     A = values_vec(h);
% %     lAz = values_vec(h);
% %     mA = values_vec(h);
% %     f = values_vec(h);
% %     period = 1/f;
% 
%     tTrajectories = period/2;
%     sViaPoints = [A; -A];
%     sDotViaPoints = [0; 0];
%     [qA, qAd, qAdd, t] = concatenatedMultiJointCubicTraj(dt, tTrajectories, sViaPoints, sDotViaPoints);
%     
%     t = t{1};
%     qA = qA{1};
%     qAd = qAd{1};
%     qAdd = qAdd{1};
%     
%     com_traj_y = lAz*sin(qA);
%     com_traj_z = -lAz*cos(qA);
%         
%     for k = 1:length(t)
%     
%         F_y(k) = FyFun(lAz, mA, qA(k), qAd(k), qAdd(k));
%         F_z(k) = FzFun(g0, lAz, mA, qA(k), qAd(k), qAdd(k));
%         F_mod(k) = sqrt(F_y(k).^2 + F_z(k).^2);
%         ang_par(k) = acos(F_y(k)/F_mod(k)) * 180/pi;
%         ang_par_2(k) = asin(F_z(k)/F_mod(k)) * 180/pi;
%         ang_tot(k) = atan2(F_y(k), F_z(k)) *180/pi;
%     
% %         figure(fig1)
% %         subplot(1,4,h)
% %         quiver(0, 0, F_y(k), F_z(k), 'Color', '[0.8 0.8 0.8]'), hold on
% %         plot([0 com_traj_y(k)], [0 com_traj_z(k)], 'Color', '[0.4 0.4 0.4]'), hold on
% %         xlim([-0.7 0.7]), ylim([-0.7 0.7]), axis square
% %         xlim([-0.7 0.7]), ylim([-1.01 0.29]), axis square
% 
%         if h == 1
%             Fy_cont1(k) = -lAz*mA*(sin(qA(k))*qAd(k)^2);
%             Fy_cont2(k) = lAz*mA*(qAdd(k)*cos(qA(k)));
%             Fz_cont1(k) = mA*(lAz*cos(qA(k))*qAd(k)^2);
%             Fz_cont2(k) = mA*(lAz*qAdd(k)*sin(qA(k)));
%         end
%     end
%     
% %     figure(fig1)    
% %     subplot(1,4,h)
% %     c = linspace(1,10,length(com_traj_z'));
% %     scatter(F_y, F_z, [], c,'filled', 'LineWidth',2), hold on, grid on
% %     scatter(com_traj_y, com_traj_z,[], c, 'LineWidth',2), hold on, grid on
% %     xlabel('Y [m]'), ylabel('Z [m]'), title(msg + " " + num2str(values_vec(h))), axis square
% % 
% %     figure(fig2)
% %     plot([0 F_y], [0 F_z], 'LineWidth',2), hold on, xlim([-0.7 0.7]), ylim([-0.7 0.7]), grid on,
% %     xlabel('Y [m]'), ylabel('Z [m]'), legend(labels), axis square
% 
% %         
% %     figure(3);
% %     subplot 321
% %     plot (t,qA, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{A}$','Interpreter','latex')
% %     subplot 323
% %     plot (t,F_y, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{y}$','Interpreter','latex')
% %     subplot 325
% %     plot (t,F_z, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{z}$','Interpreter','latex')
% %     subplot 322
% %     plot (t,F_mod, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{tot}$','Interpreter','latex')
% %     subplot 324
% %     plot (t,ang_par, 'LineWidth',2), hold on, grid on, xlabel('time')
% %     subplot 326
% %     plot (t,ang_tot, 'LineWidth',2), hold on, grid on, xlabel('time')
% %     
% %     figure(4);
% %     subplot 311
% %     plot (t,qA, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{A}$','Interpreter','latex')
% %     subplot 312
% %     plot (t,qAd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{A}$','Interpreter','latex')
% %     subplot 313
% %     plot (t,qAdd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\ddot{q}_{A}$','Interpreter','latex')
% % 
% %     figure(5)
% %     subplot 221
% %     plot (t,Fy_cont1, 'LineWidth',2), hold on, grid on, xlabel('time'), title('Fy_cont1 - Centripetal','Interpreter','latex')
% %     subplot 223
% %     plot (t,Fy_cont2, 'LineWidth',2), hold on, grid on, xlabel('time'), title('Fy_cont2 - Motion','Interpreter','latex')
% %     subplot 222
% %     plot (t,Fz_cont1, 'LineWidth',2), hold on, grid on, xlabel('time'), title('Fz_cont1 - Centripetal','Interpreter','latex')
% %     subplot 224
% %     plot (t,Fz_cont2, 'LineWidth',2), hold on, grid on, xlabel('time'), title('Fz_cont2 - Motion','Interpreter','latex')
% %     
% end
% 
% 
% figure(fig3)
% subplot(1,4,1), axis square
% plot (t,Fy_cont1, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_y$ - Centripetal','Interpreter','latex'), legend('sinusoidal', 'cubic')
% subplot(1,4,2), axis square
% plot (t,Fy_cont2, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_y$ - Motion','Interpreter','latex'), legend('sinusoidal', 'cubic')
% subplot(1,4,3), axis square
% plot (t,Fz_cont1, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_z$ - Centripetal','Interpreter','latex'), legend('sinusoidal', 'cubic')
% subplot(1,4,4), axis square
% plot (t,Fz_cont2, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_z$ - Motion','Interpreter','latex'), legend('sinusoidal', 'cubic')
% 
% 
% % exportgraphics(fig1, 'Planar1RGravComp_CubicAmplitudes.pdf');
% % exportgraphics(fig2, 'Planar1RGravComp_CubicAmplitudesComparison.pdf');
% % exportgraphics(fig3, 'ForcesDecomposition_SinCubic.pdf');
% 
% %% TRAPEZOIDAL
% 
% % close all
% clc
% 
% period = 4;
% dt = 0.04;
% lAz = 0.5;
% mA = 0.5;
% g0 = 0.0;
% accMax = 2.5;
% colorlist = ["b";"r";"y";"m"];
% 
% values_vec = [1 0.8 0.5 0.3];
% % values_vec = [0.4 0.3 0.25 0.2];
% 
% fig1 = figure('Renderer', 'painters', 'Position', [10 10 1800 500]);
% fig2 = figure('Renderer', 'painters', 'Position', [10 10 500 500]);
% 
% msg = "Amp";
% labels = [msg+" 1", msg+" 0.8", msg+" 0.5", msg+" 0.3"];
% % labels = [msg+" 0.4", msg+" 0.3", msg+" 0.25", msg+" 0.2"];
% 
% 
% for h=1:length(values_vec)
% 
%     A = values_vec(h);
% 
%     tFinal = period/2;
%     sInit = [A; -A];
%     sFinal = [-A; A]; 
%     accDes = [accMax; accMax];
% 
%     [qA, qAd, qAdd, t] = concatenatedMultiJointTrapVelTraj_tf(dt, tFinal, accDes, sInit, sFinal);
%     
%     t = t{1};
%     qA = qA{1};
%     qAd = qAd{1};
%     qAdd = qAdd{1};
%     
%     com_traj_y = lAz*sin(qA);
%     com_traj_z = -lAz*cos(qA);
%         
%     for k = 1:length(t)
%     
%         F_y(k) = FyFun(lAz, mA, qA(k), qAd(k), qAdd(k));
%         F_z(k) = FzFun(g0, lAz, mA, qA(k), qAd(k), qAdd(k));
%         F_mod(k) = sqrt(F_y(k).^2 + F_z(k).^2);
%         ang_par(k) = acos(F_y(k)/F_mod(k)) * 180/pi;
%         ang_par_2(k) = asin(F_z(k)/F_mod(k)) * 180/pi;
%         ang_tot(k) = atan2(F_y(k), F_z(k)) *180/pi;
%     
% %         figure(fig1)
% %         subplot(1,4,h)
% %         quiver(0, 0, F_y(k), F_z(k), 'Color', '[0.8 0.8 0.8]'), hold on
% %         plot([0 com_traj_y(k)], [0 com_traj_z(k)], 'Color', '[0.4 0.4 0.4]'), hold on
% %         xlim([-0.7 0.7]), ylim([-0.7 0.7]), axis square
% %         xlim([-0.7 0.7]), ylim([-1.01 0.29]), axis square
% 
%         if h == 1
%             Fy_cont1(k) = -lAz*mA*(sin(qA(k))*qAd(k)^2);
%             Fy_cont2(k) = lAz*mA*(qAdd(k)*cos(qA(k)));
%             Fz_cont1(k) = mA*(lAz*cos(qA(k))*qAd(k)^2);
%             Fz_cont2(k) = mA*(lAz*qAdd(k)*sin(qA(k)));
%         end
%     end
%     
% %     figure(fig1)    
% %     subplot(1,4,h)
% %     c = linspace(1,10,length(com_traj_z'));
% %     scatter(F_y, F_z, [], c,'filled', 'LineWidth',2), hold on, grid on
% %     scatter(com_traj_y, com_traj_z,[], c, 'LineWidth',2), hold on, grid on
% %     xlabel('Y [m]'), ylabel('Z [m]'), title(msg + " " + num2str(values_vec(h))), axis square
% % 
% %     figure(fig2)
% %     plot([0 F_y], [0 F_z], 'LineWidth',2), hold on, xlim([-0.7 0.7]), ylim([-0.7 0.7]), grid on,
% %     xlabel('Y [m]'), ylabel('Z [m]'), legend(labels), axis square
% %         
% %     figure(3);
% %     subplot 321
% %     plot (t,qA, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{A}$','Interpreter','latex')
% %     subplot 323
% %     plot (t,F_y, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{y}$','Interpreter','latex')
% %     subplot 325
% %     plot (t,F_z, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{z}$','Interpreter','latex')
% %     subplot 322
% %     plot (t,F_mod, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{tot}$','Interpreter','latex')
% %     subplot 324
% %     plot (t,ang_par, 'LineWidth',2), hold on, grid on, xlabel('time')
% %     subplot 326
% %     plot (t,ang_tot, 'LineWidth',2), hold on, grid on, xlabel('time')
% %     
% %     figure(4);
% %     subplot 311
% %     plot (t,qA, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{A}$','Interpreter','latex')
% %     subplot 312
% %     plot (t,qAd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{A}$','Interpreter','latex')
% %     subplot 313
% %     plot (t,qAdd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\ddot{q}_{A}$','Interpreter','latex')
%     
% end
% 
% figure(fig3)
% subplot(1,4,1), axis square
% plot (t,Fy_cont1, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_y$ - Centripetal','Interpreter','latex')
% legend('sinusoidal', 'cubic', 'trapezoidal','Location','SouthEast')
% subplot(1,4,2), axis square
% plot (t,Fy_cont2, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_y$ - Motion','Interpreter','latex')
% legend('sinusoidal', 'cubic', 'trapezoidal','Location','SouthEast')
% subplot(1,4,3), axis square
% plot (t,Fz_cont1, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_z$ - Centripetal','Interpreter','latex')
% legend('sinusoidal', 'cubic', 'trapezoidal','Location','SouthEast')
% subplot(1,4,4), axis square
% plot (t,Fz_cont2, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_z$ - Motion','Interpreter','latex')
% legend('sinusoidal', 'cubic', 'trapezoidal','Location','SouthEast')
% 
% % exportgraphics(fig1, 'Planar1RGravComp_TrapAmplitudes.pdf');
% % exportgraphics(fig2, 'Planar1RGravComp_CubicAmplitudesComparison.pdf');
% % exportgraphics(fig3, 'ForcesDecomposition_SinCubicTrap.pdf');
% 
% 
% %% QUINTIC
% 
% % close all
% clc
% 
% period = 4;
% dt = 0.04;
% lAz = 0.5;
% mA = 0.5;
% g0 = 0.0;
% accMax = 2.5;
% colorlist = ["b";"r";"y";"m"];
% 
% values_vec = [1 0.8 0.5 0.3];
% % values_vec = [0.4 0.3 0.25 0.2];
% 
% fig1 = figure('Renderer', 'painters', 'Position', [10 10 1800 500]);
% fig2 = figure('Renderer', 'painters', 'Position', [10 10 500 500]);
% 
% msg = "CoM";
% labels = [msg+" 1", msg+" 0.8", msg+" 0.5", msg+" 0.3"];
% % labels = [msg+" 0.4", msg+" 0.3", msg+" 0.25", msg+" 0.2"];
% 
% 
% for h=1:length(values_vec)
% 
%     A = values_vec(h);
% 
%     tTrajectories = period/2;
%     sViaPoints = [A; -A];
%     sDotViaPoints = [0; 0];
%     sDot2ViaPoints = [0; 0];
%     [qA, qAd, qAdd, t] = concatenatedMultiJointQuinticTraj(dt, tTrajectories, sViaPoints, sDotViaPoints, sDot2ViaPoints);
% 
%     t = t{1};
%     qA = qA{1};
%     qAd = qAd{1};
%     qAdd = qAdd{1};
%     
%     com_traj_y = lAz*sin(qA);
%     com_traj_z = -lAz*cos(qA);
%         
%     for k = 1:length(t)
%     
%         F_y(k) = FyFun(lAz, mA, qA(k), qAd(k), qAdd(k));
%         F_z(k) = FzFun(g0, lAz, mA, qA(k), qAd(k), qAdd(k));
%         F_mod(k) = sqrt(F_y(k).^2 + F_z(k).^2);
%         ang_par(k) = acos(F_y(k)/F_mod(k)) * 180/pi;
%         ang_par_2(k) = asin(F_z(k)/F_mod(k)) * 180/pi;
%         ang_tot(k) = atan2(F_y(k), F_z(k)) *180/pi;
%     
%         figure(fig1)
%         subplot(1,4,h)
%         quiver(0, 0, F_y(k), F_z(k), 'Color', '[0.8 0.8 0.8]'), hold on
%         plot([0 com_traj_y(k)], [0 com_traj_z(k)], 'Color', '[0.4 0.4 0.4]'), hold on
%         xlim([-0.85 0.85]), ylim([-0.6 1.1]), axis square
% %         xlim([-0.7 0.7]), ylim([-1.01 0.29]), axis square
% 
%         if h == 1
%             Fy_cont1(k) = -lAz*mA*(sin(qA(k))*qAd(k)^2);
%             Fy_cont2(k) = lAz*mA*(qAdd(k)*cos(qA(k)));
%             Fz_cont1(k) = mA*(lAz*cos(qA(k))*qAd(k)^2);
%             Fz_cont2(k) = mA*(lAz*qAdd(k)*sin(qA(k)));
%         end
%     end
%     
%     figure(fig1)    
%     subplot(1,4,h)
%     c = linspace(1,10,length(com_traj_z'));
%     scatter(F_y, F_z, [], c,'filled', 'LineWidth',2), hold on, grid on
%     scatter(com_traj_y, com_traj_z,[], c, 'LineWidth',2), hold on, grid on
%     xlabel('Y [m]'), ylabel('Z [m]'), title(msg + " " + num2str(values_vec(h))), axis square
% 
%     figure(fig2)
%     plot([0 F_y], [0 F_z], 'LineWidth',2), hold on, xlim([-0.85 0.85]), ylim([-0.6 1.1]), grid on,
%     xlabel('Y [m]'), ylabel('Z [m]'), legend(labels), axis square
%         
% %     figure(3);
% %     subplot 321
% %     plot (t,qA, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{A}$','Interpreter','latex')
% %     subplot 323
% %     plot (t,F_y, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{y}$','Interpreter','latex')
% %     subplot 325
% %     plot (t,F_z, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{z}$','Interpreter','latex')
% %     subplot 322
% %     plot (t,F_mod, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_{tot}$','Interpreter','latex')
% %     subplot 324
% %     plot (t,ang_par, 'LineWidth',2), hold on, grid on, xlabel('time')
% %     subplot 326
% %     plot (t,ang_tot, 'LineWidth',2), hold on, grid on, xlabel('time')
% %     
% %     figure(4);
% %     subplot 311
% %     plot (t,qA, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{A}$','Interpreter','latex')
% %     subplot 312
% %     plot (t,qAd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{A}$','Interpreter','latex')
% %     subplot 313
% %     plot (t,qAdd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\ddot{q}_{A}$','Interpreter','latex')
%     
% end
% 
% % figure(fig3)
% % subplot(1,4,1), axis square
% % plot (t,Fy_cont1, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_y$ - Centripetal','Interpreter','latex')
% % legend('sinusoidal', 'cubic', 'trapezoidal', 'quintic', 'Location','SouthEast')
% % subplot(1,4,2), axis square
% % plot (t,Fy_cont2, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_y$ - Motion','Interpreter','latex')
% % legend('sinusoidal', 'cubic', 'trapezoidal', 'quintic','Location','SouthEast')
% % subplot(1,4,3), axis square
% % plot (t,Fz_cont1, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_z$ - Centripetal','Interpreter','latex')
% % legend('sinusoidal', 'cubic', 'trapezoidal', 'quintic','Location','SouthEast')
% % subplot(1,4,4), axis square
% % plot (t,Fz_cont2, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$F_z$ - Motion','Interpreter','latex')
% % legend('sinusoidal', 'cubic', 'trapezoidal', 'quintic', 'Location','SouthEast')
% % 
% % 
% 
% exportgraphics(fig1, 'Planar1RGravComp_QuinticAmplitudes.pdf');
% % exportgraphics(fig2, 'Planar1RGravComp_QunticAmplitudesComparison.pdf');
% % exportgraphics(fig3, 'ForcesDecomposition_SinCubicTrapQuint.pdf');
% 
% 
