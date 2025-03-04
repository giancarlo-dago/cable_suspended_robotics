close all
clear
clc

timeWindow = 240; % [s]

[filename, path] = uigetfile({'*.*'}, 'MultiSelect', 'on');
fig0 = figure('Renderer', 'painters', 'Position', [0 0 1600 600]);
fig1 = figure('Renderer', 'painters');
fig2 = figure('Renderer', 'painters');
fig3 = figure('Renderer', 'painters');
fig4 = figure('Renderer', 'painters');
fig5 = figure('Renderer', 'painters');
colorList = ['y', 'r', 'b'];

%% Parameters
% Parameters cables
iCxx = 0;
LC = 4.53;
lC = 3.64;

% lC = 4;
mC = 206;
fvC = 0.0;
fvA = 0.0;
fsC = 0.0;
fsA = 0.0;

% Arms parameters
iAxx = 0.0;
lA = 0.4157;
mA = 10.2 * 2;

% Gravity
g0 = 9.8;

%% Data

for k=length(filename):-1:1
    simData = load(strcat(path,filename{k}));
    t = simData.out.tout;
    tShort = t(1:find(t>timeWindow,1,'first'));
    t = tShort;

    qC = reshape(simData.out.state_variable.Data(1,1,1:length(t)),length(t),1);
    qA = reshape(simData.out.state_variable.Data(2,1,1:length(t)),length(t),1);
    qCd = reshape(simData.out.state_variable.Data(3,1,1:length(t)),length(t),1);
    qAd = reshape(simData.out.state_variable.Data(4,1,1:length(t)),length(t),1);
    qCdd = reshape(simData.out.state_variable.Data(5,1,1:length(t)),length(t),1);
    qAdd = reshape(simData.out.state_variable.Data(6,1,1:length(t)),length(t),1);

    figure(fig0);
    subplot 321
    plot(t,qC, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('pos [rad]'), title('$q_{C}$','Interpreter','latex')
    [YUPPER,YLOWER] = envelope(qC);
    plot(t, [YUPPER,YLOWER], 'LineWidth',2, 'Color', colorList(k)), hold on, grid on, xlabel('time [s]'), ylabel('pos [rad]')    
    subplot 323
    plot(t,qCd, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('vel [rad/2]'), title('$\dot{q}_{C}$','Interpreter','latex')
    subplot 325
    plot(t,qCdd, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('acc [rad/s^2]'), title('$\ddot{q}_{C}$','Interpreter','latex')
    
    subplot 322
    plot(t,qA, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('pos [rad]'), title('$q_{A}$','Interpreter','latex')
    subplot 324
    plot(t,qAd, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('vel [rad/2]'), title('$\dot{q}_{A}$','Interpreter','latex')
    subplot 326
    plot(t,qAdd, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('acc [rad/s^2]'), title('$\dot{q}_{A}$','Interpreter','latex')

    figure(fig1);
    [YUPPER,YLOWER] = envelope(qC);
    plot(t, [YUPPER,YLOWER], 'LineWidth',2, 'Color', colorList(k)), hold on, grid on, xlabel('time [s]'), ylabel('pos [rad]')

    T = qCd.*(0.5000.*qCd.*(mA*LC^2 + 2*mA.*cos(qA)*LC*lA + mA*lA^2 + mC*lC^2 + iAxx + iCxx) + 0.5000.*qAd.*(mA*lA^2 + LC*mA*cos(qA)*lA + iAxx)) + qAd.*(0.5000*qAd.*(mA*lA^2 + iAxx) + 0.5000*qCd.*(mA*lA^2 + LC*mA*cos(qA)*lA + iAxx));
    T_motion1 = 0.5*(iCxx + mC*lC^2).*qCd.^2;
    T_motion1params2 = 0.5*(mA*LC^2 + mA*lA^2 + iAxx).*qCd.^2;
    T_motion1motion2 = (mA*lA^2 + iAxx).*qAd.*qCd;
    T_motion1motion2config2 = LC*mA*lA*cos(qA).*qAd.*qCd;
    T_motion1config2 = LC*mA*lA.*cos(qA).*qCd.^2;
    T_motion2 = 0.5*(mA*lA^2 + iAxx).*qAd.^2 + LC*mA*lA.*cos(qA).*qAd.^2;

    U = -mC*lC*g0*cos(qC) - mA*LC*g0*cos(qC) - mA*lA*g0*cos(qC+qA);
    U_config1 = -mC*lC*g0*cos(qC);
    U_config1param2 = - mA*LC*g0*cos(qC);
    U_config1config2 = - mA*lA*g0*cos(qC+qA);
    Ustar = -(mC*lC - mA*LC - mA*lA)*g0;

    E = T + U;

    figure(fig2);
    subplot 421
    plot(t,T, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$T$','Interpreter','latex')
    subplot 423
    plot(t,T_motion1 + T_motion1params2, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$T1$','Interpreter','latex')
    subplot 425
    plot(t,T_motion1motion2 + T_motion1motion2config2 + T_motion1config2, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$T12 + T2$','Interpreter','latex')
    subplot 422
    plot(t,U, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$U$','Interpreter','latex')
    subplot 424
    plot(t,U_config1 + U_config1param2, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$U1$','Interpreter','latex')
    subplot 426
    plot(t,U_config1config2, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$U12$','Interpreter','latex')
%     subplot(4,2,7)
%     plot(t,T_motion1motion2 + T_motion1motion2config2 + T_motion1config2, 'LineWidth',2, 'Color', colorList(k)), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$T12 + T2$','Interpreter','latex')
%     plot(t,U_config1config2 + mA*lA*g0, '--', 'LineWidth',2, 'Color', colorList(k)), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$T12 + T2$','Interpreter','latex')
    subplot(4,2,8)
    E_12 = T_motion1motion2 + T_motion1motion2config2 + T_motion1config2 + U_config1config2;
%     E_dot = (E(2:end)-E(1:end-1))./0.125;
    plot(t,E_12, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$E12$','Interpreter','latex')
    subplot(4,2,7)
    plot(t,E, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$E$','Interpreter','latex')

    figure(fig3);
    subplot 331
    plot(t,T_motion1, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$T motion1$','Interpreter','latex')
    subplot 334
    plot(t,T_motion1params2, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$T motion1, params2$','Interpreter','latex')
    subplot 333
    plot(t,T_motion1motion2, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$T motion1, motion2$','Interpreter','latex')
    subplot 336
    plot(t,T_motion1motion2config2, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$T motion1, motion2 ,config2$','Interpreter','latex')
    subplot 337
    plot(t,T_motion1config2, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$T motion1, config2$','Interpreter','latex')
    subplot 338
    plot(t,T_motion2, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$T motion2$','Interpreter','latex')
    subplot 332
    plot(t,U_config1, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$U config1$','Interpreter','latex')
    subplot 335
    plot(t,U_config1param2, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$U config1, param2$','Interpreter','latex')
    subplot 339
    plot(t,U_config1config2, 'LineWidth',2), hold on, grid on, xlabel('time [s]'), ylabel('[J]'), title('$U config1, config2$','Interpreter','latex')


    figure(fig5);
    nPoints = 80000;
    for k=1:nPoints
    
        QA = qA(k);
        QC = qC(k);
        QAd = qAd(k);
        QCd = qCd(k);
    
        B = [mA*LC^2 + 2*mA*cos(QA)*LC*lA + mA*lA^2 + mC*lC^2 + iAxx + iCxx, mA*lA^2 + LC*mA*cos(QA)*lA + iAxx;
                                          mA*lA^2 + LC*mA*cos(QA)*lA + iAxx,                    mA*lA^2 + iAxx];
        
        n = [- LC*lA*mA*sin(QA)*QAd^2 - 2*LC*lA*mA*QCd*sin(QA)*QAd + fvC*QCd + fsC*sign(QCd) + g0*lA*mA*sin(QA + QC) + LC*g0*mA*sin(QC) + g0*lC*mC*sin(QC);
                                                                                 LC*lA*mA*sin(QA)*QCd^2 + fvA*QAd + fsA*sign(QAd) + g0*lA*mA*sin(QA + QC)];
    
        tau(k,:) = B*[qCdd(k); qAdd(k)]+n;
        end
    plot(t(1:nPoints),qAd(1:nPoints).*tau(:,2), 'LineWidth',2), hold on

end


