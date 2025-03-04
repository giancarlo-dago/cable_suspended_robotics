close all
clc

timeWindow = 240; % [s]

[filename, path] = uigetfile({'*.*'}, 'MultiSelect', 'off');
% fig0 = figure('Renderer', 'painters', 'Position', [0 0 1600 600]);
% fig1 = figure('Renderer', 'painters');
% fig2 = figure('Renderer', 'painters');
% fig3 = figure('Renderer', 'painters');
% fig4 = figure('Renderer', 'painters');
% fig5 = figure('Renderer', 'painters');
% colorList = ['y', 'r', 'b'];

ans = load(filename);
t = ans.out.tout;    
tShort = t(1:find(t>timeWindow,1,'first'));
t = tShort;
qC = reshape(ans.out.state_variable.Data(1,1,1:length(t)),length(t),1);
qA = reshape(ans.out.state_variable.Data(2,1,1:length(t)),length(t),1);
qCd = reshape(ans.out.state_variable.Data(3,1,1:length(t)),length(t),1);
qAd = reshape(ans.out.state_variable.Data(4,1,1:length(t)),length(t),1);
qCdd = reshape(ans.out.state_variable.Data(5,1,1:length(t)),length(t),1);
qAdd = reshape(ans.out.state_variable.Data(6,1,1:length(t)),length(t),1);


% plot(reshape(out.state_variable.Data(1,:,:),1,:), reshape(out.state_variable.Data(3,:,:),1,:))
plot(qC, qCd), grid, axis square,
xlabel('Position $q_C$ [rad]','Interpreter','latex'), ylabel('Velocity $\dot{q}_C$ [rad/s]','Interpreter','latex')

