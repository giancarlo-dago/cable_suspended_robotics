%% Posizione iniziale e finale giunto prismatico
p0 = 0;
p1 = 16.98;
tf = 80;

%% Vettore delle posizioni e dei tempi
P = [p0 p1];                             % Vettore delle posizioni
t = [0 tf];                              % Vettore degli istanti temporali di passaggio per i punti

%% Campioni
Nc = 100;                                   % Numero dei campioni al secondo
T_vec = Nc * t;
Campioni = zeros(1,length(T_vec)-1);            % Vettore dei campioni
for j=1:length(T_vec)-1
    Campioni(j) = T_vec(j+1) - T_vec(j);
end

%% Calcolo della traiettoria in termini di posizione, velocità e accelerazione

for j = 1:length(Campioni)
    temp_diff = t(j+1)-t(j);
    time_interval(:,j) = 1/Nc : 1/Nc : temp_diff;
end

[p, pd, pdd] = lspb(P(1), P(2), time_interval(:,1));
for j = 2:length(Campioni)
    [p(end+1:end+Campioni(j)), pd(end+1:end+Campioni(j)), pdd(end+1:end+Campioni(j))] = lspb(P(j), P(j+1), time_interval(:,j));
end

%% Allungo il vettore p per vedere cosa accade a regime nei grafici
tempo_di_regime = 5; % secondi
Nc_regime = Nc*tempo_di_regime;

for h = length(p)+1:length(p)+Nc_regime
    p(h) = p(end);
    pd(h) = pd(end);
    pdd(h) = zeros(1);
end 

%% Trasformo posizioni e velocità in oggetti di tipo timeseries per portarli in simulink

total_time = 1/Nc : 1/Nc : length(p)/Nc;

d0_ref = timeseries(p', total_time);
dd0_ref = timeseries(pd', total_time);
ddd0_ref = timeseries(pdd', total_time);

%% Plot

figure(1);
subplot(3,1,1); plot(total_time,p,'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$d_0(t) [m]$','Interpreter','Latex'); legend('d0_{ref}'); grid;
subplot(3,1,2); plot(total_time,pd,'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{d}_0(t) [\frac{m}{s}]$','Interpreter','Latex');legend('dd0_{ref}'); grid; 
subplot(3,1,3); plot(total_time,pdd,'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{d}_0(t) [\frac{m}{s^2}]$','Interpreter','Latex'); legend('ddd0_{ref}'); grid;
sgtitle('Overhead crane motion')