
%% (A) Punti 

p1A = [-0.25 -1.207 0]'; % Relativo a q0 = [-pi/2 -pi/4 pi/2]
p2A = [0 -1 0]';
rho1 = 0.1; cent1 = [0 -1.1 0]; R1 = [0 -1 0; 1 0 0; 0 0 1]; ang1 = -2*pi;
p3A = [0 -1 0]'; 
p4A = [-0.25 -1.207 0]'; % Relativo a q0 = [-pi/2 -pi/4 pi/2]

%% (B) Punti

p1B = [0.25 -1.207 0]';   % Relativo a q0 = [-pi/2 pi/4 -pi/2]
p2B = [0.35 -1.3 0]';
p3B = [0.35 -1.3 0]';
p4B = [0.25 -1.207 0]';   % Relativo a q0 = [-pi/2 pi/4 -pi/2]

%% (A) Vettore delle posizioni, dei tempi e dei delta

PA = [p1A p2A p3A p4A];                   % Vettore delle posizioni

figure(1)
scatter(p1A(1), p1A(2),'r'); hold on
for k = 2:length(PA(1,:))
    scatter(PA(1,k), PA(2,k),'r')
end

tA = [0 10 25 35];                                      % Vettore degli istanti temporali di passaggio per i punti
dtA = [0 0 0 0];                                       % Vettore degli anticipi per i punti di via
Tipo_curva_A = [0 1 0 0];                              % 0=segmento, 1=circonferenza

rho_vec_A =  [0 rho1 0 0];                                % Vettore dei raggi dei tratti di circonferenza
cent_vec_A = [zeros(1,3) cent1 zeros(1,3) zeros(1,3)];     % Vettore delle posizioni centri dei tratti di circonferenza
ang_vec_A =  [0 ang1 0 0];                                % Vettore delle distanze angolari dei tratti circolari
R_vec_A =    [zeros(3) R1 zeros(3) zeros(3)];           % Vettore delle matrici di rotazione relative ai tratti di circonferenza

%% (B) Vettore delle posizioni, dei tempi e dei delta

PB = [p1B p2B p3B p4B];                   % Vettore delle posizioni

figure(1)
scatter(p1B(1), p1B(2),'b'); hold on
for k = 2:length(PB(1,:))
    scatter(PB(1,k), PB(2,k),'b')
end

tB = [0 10 25 35];                                      % Vettore degli istanti temporali di passaggio per i punti
dtB = [0 0 0 0];                                     % Vettore degli anticipi per i punti di via
Tipo_curva_B = [0 0 0 0];                             % 0=segmento, 1=circonferenza

rho_vec_B =  [0 0 0 0];                               % Vettore dei raggi dei tratti di circonferenza
cent_vec_B = [zeros(1,3) zeros(1,3) zeros(1,3) zeros(1,3)];             % Vettore delle posizioni centri dei tratti di circonferenza
ang_vec_B =  [0 0 0 0];                               % Vettore delle distanze angolari dei tratti circolari
R_vec_B =    [zeros(3) zeros(3) zeros(3) zeros(3)];                 % Vettore delle matrici di rotazione relative ai tratti di circonferenza

%% (A) Campioni

NcA = 100;                                                               % Numero dei campioni al secondo
TA = NcA * tA;
CampioniA = zeros(1,length(TA)-1);                                        % Vettore dei campioni
for j=1:length(TA)-1
    CampioniA(j) = TA(j+1) - TA(j);
end

%% (B) Campioni

NcB = 100;                                                               % Numero dei campioni al secondo
TB = NcB * tB;
CampioniB = zeros(1,length(TB)-1);                                        % Vettore dei campioni
for j=1:length(TB)-1
    CampioniB(j) = TB(j+1) - TB(j);
end

%% (A) Calcolo dei delta (da libro)

DtA(1) = 0;
for j=2:length(PA(1,:))
    DtA(j) = DtA(j-1) + dtA(j);
end

CtotA = sum(CampioniA)-sum(dtA);                       % Numero totale di campioni usati tenendo conto degli anticipi dovuti a pti di via

%% (B) Calcolo dei delta (da libro)

DtB(1) = 0;
for j=2:length(PB(1,:))
    DtB(j) = DtB(j-1) + dtB(j);
end

CtotB = sum(CampioniB)-sum(dtB);                       % Numero totale di campioni usati tenendo conto degli anticipi dovuti a pti di via

%% (A) Calcolo delle s e p (da libro) - Pianificazione traiettoria (posizione, velocità e accelerazione)

distA = norm(PA(:,2)-PA(:,1));

[spA, spdA, spddA] = lspb(0, distA, CampioniA(1));
for j = 3:length(PA(1,:))
    if Tipo_curva_A(j-1) == 0                         % tratto rettilineo
        distA = norm(PA(:,j)-PA(:,j-1));
        [spA(end+1:end+CampioniA(j-1)), spdA(end+1:end+CampioniA(j-1)), spddA(end+1:end+CampioniA(j-1))] = lspb(0, distA, CampioniA(j-1));
    else                                            % circonferenza
        distA = ang_vec_A(j-1)*rho_vec_A(j-1);
        [spA(end+1:end+CampioniA(j-1)), spdA(end+1:end+CampioniA(j-1)), spddA(end+1:end+CampioniA(j-1))] = lspb(0, distA, CampioniA(j-1));
    end
end

sA = zeros(CtotA,length(PA(1,:))-1);
sdA = zeros(CtotA,length(PA(1,:))-1);
sddA = zeros(CtotA,length(PA(1,:))-1);
for j = 2:length(PA(1,:))
    i1A = 1 : TA(j-1)-DtA(j);                          % intervallo 1
    i2A = TA(j-1)-DtA(j)+1 : TA(j)-DtA(j);               % intervallo 2
    i3A = TA(j)-DtA(j)+1 : TA(end)-DtA(end);             % intervallo 3
    
    if Tipo_curva_A(j-1) == 0                         % tratto rettilineo
        distA = norm(PA(:,j)-PA(:,j-1));
    else                                            % circonferenza
        distA = ang_vec_A(j-1)*rho_vec_A(j-1);
    end

    sA(i1A,j-1) = 0;                                  % Ascissa curvilinea posizioni
    sA(i2A,j-1) = spA(i2A+DtA(j));
    sA(i3A,j-1) = distA;
    
    sdA(i1A,j-1) = 0;                                 % Ascissa curvilinea velocità
    sdA(i2A,j-1) = spdA(i2A+DtA(j));
    sdA(i3A,j-1) = 0;
    
    sddA(i1A,j-1) = 0;                                % Ascissa curvilinea accelerazioni
    sddA(i2A,j-1) = spddA(i2A+DtA(j));
    sddA(i3A,j-1) = 0;
end

pA = PA(:,1);
pdA = zeros(3,1);
pddA = zeros(3,1);
for j = 2:length(PA(1,:))
    i1A = 1 : TA(j-1)-DtA(j);                          % intervallo 1
    i2A = TA(j-1)-DtA(j)+1 : TA(j)-DtA(j);               % intervallo 2
    i3A = TA(j)-DtA(j)+1 : TA(end)-DtA(end);             % intervallo 3
    
    if Tipo_curva_A(j-1) == 0                         % tratto rettilineo
        distA = PA(:,j)-PA(:,j-1);     
        if (norm(distA) ~= 0) 
            pA = pA + (sA(:,j-1) / norm(distA) * distA')';
            pdA = pdA + (sdA(:,j-1) / norm(distA) * distA')';
            pddA = pddA + (sddA(:,j-1) / norm(distA) * distA')';        
        else 
            pA = pA + (sA(:,j-1) * distA')';
            pdA = pdA + (sdA(:,j-1) * distA')';
            pddA = pddA + (sddA(:,j-1) * distA')';  
        end
    else                                            % circonferenza
        rhoA = rho_vec_A(j-1); 
        centA = cent_vec_A((j-1)*3-2:(j-1)*3);
        RA = R_vec_A(:,(j-1)*3-2:(j-1)*3);
        ppA = [rhoA*cos(sA(i2A,j-1)/rhoA) ...
              rhoA*sin(sA(i2A,j-1)/rhoA) ...
              zeros(length(sA(i2A,j-1)),1)];
        pA(:,i2A) = centA' + RA*ppA';
        ppdA = [-sdA(i2A,j-1).*(sin(sA(i2A,j-1)/rhoA))...
                sdA(i2A,j-1).*(cos(sA(i2A,j-1)/rhoA))...
                zeros(length(sA(i2A,j-1)),1)];
        pdA(:,i2A) = RA*ppdA';
        ppddA = [-(((sdA(i2A,j-1)).^2).*(cos(sA(i2A,j-1)/rhoA))/rhoA)-(sddA(i2A,j-1).*sin(sA(i2A,j-1)/rhoA))...
                -(((sdA(i2A,j-1)).^2).*(sin(sA(i2A,j-1)/rhoA))/rhoA)+(sddA(i2A,j-1).*cos(sA(i2A,j-1)/rhoA))...
                 zeros(length(sA(i2A,j-1)),1)];
        pddA(:,i2A) = RA*ppddA';
        
        for k = TA(j)-DtA(j)+1 : TA(end)-DtA(end)
            pA(:,k) = pA(:,i2A(end));
            pdA(:,k) = pdA(:,i2A(end));
            pddA(:,k) = 0;
        end
    end
end

%% (B) Calcolo delle s e p (da libro) - Pianificazione traiettoria (posizione, velocità e accelerazione)

distB = norm(PB(:,2)-PB(:,1));

[spB, spdB, spddB] = lspb(0, distB, CampioniB(1));
for j = 3:length(PB(1,:))
    if Tipo_curva_B(j-1) == 0                         % tratto rettilineo
        distB = norm(PB(:,j)-PB(:,j-1));
        [spB(end+1:end+CampioniB(j-1)), spdB(end+1:end+CampioniB(j-1)), spddB(end+1:end+CampioniB(j-1))] = lspb(0, distB, CampioniB(j-1));
    else                                            % circonferenza
        distB = ang_vec_B(j-1)*rho_vec_B(j-1);
        [spB(end+1:end+CampioniB(j-1)), spdB(end+1:end+CampioniB(j-1)), spddB(end+1:end+CampioniB(j-1))] = lspb(0, distB, CampioniB(j-1));
    end
end

sB = zeros(CtotB,length(PB(1,:))-1);
sdB = zeros(CtotB,length(PB(1,:))-1);
sddB = zeros(CtotB,length(PB(1,:))-1);
for j = 2:length(PB(1,:))
    i1B = 1 : TB(j-1)-DtB(j);                          % intervallo 1
    i2B = TB(j-1)-DtB(j)+1 : TB(j)-DtB(j);               % intervallo 2
    i3B = TB(j)-DtB(j)+1 : TB(end)-DtB(end);             % intervallo 3
    
    if Tipo_curva_B(j-1) == 0                         % tratto rettilineo
        distB = norm(PB(:,j)-PB(:,j-1));
    else                                            % circonferenza
        distB = ang_vec_B(j-1)*rho_vec_B(j-1);
    end

    sB(i1B,j-1) = 0;                                  % Ascissa curvilinea posizioni
    sB(i2B,j-1) = spB(i2B+DtB(j));
    sB(i3B,j-1) = distB;
    
    sdB(i1B,j-1) = 0;                                 % Ascissa curvilinea velocità
    sdB(i2B,j-1) = spdB(i2B+DtB(j));
    sdB(i3B,j-1) = 0;
    
    sddB(i1B,j-1) = 0;                                % Ascissa curvilinea accelerazioni
    sddB(i2B,j-1) = spddB(i2B+DtB(j));
    sddB(i3B,j-1) = 0;
end

pB = PB(:,1);
pdB = zeros(3,1);
pddB = zeros(3,1);
for j = 2:length(PB(1,:))
    i1B = 1 : TB(j-1)-DtB(j);                          % intervallo 1
    i2B = TB(j-1)-DtB(j)+1 : TB(j)-DtB(j);               % intervallo 2
    i3B = TB(j)-DtB(j)+1 : TB(end)-DtB(end);             % intervallo 3
    
    if Tipo_curva_B(j-1) == 0                         % tratto rettilineo
        distB = PB(:,j)-PB(:,j-1);
        if (norm(distB) ~= 0) 
            pB = pB + (sB(:,j-1) / norm(distB) * distB')';
            pdB = pdB + (sdB(:,j-1) / norm(distB) * distB')';
            pddB = pddB + (sddB(:,j-1) / norm(distB) * distB')';     
        else 
            pB = pB + (sB(:,j-1) * distB')';
            pdB = pdB + (sdB(:,j-1) * distB')';
            pddB = pddB + (sddB(:,j-1) * distB')';
        end
    else                                            % circonferenza
        rhoB = rho_vec_B(j-1); 
        centB = cent_vec_B((j-1)*3-2:(j-1)*3);
        RB = R_vec_B(:,(j-1)*3-2:(j-1)*3);
        ppB = [rhoB*cos(sB(i2B,j-1)/rhoB) ...
              rhoB*sin(sB(i2B,j-1)/rhoB) ...
              zeros(length(sB(i2B,j-1)),1)];
        pB(:,i2B) = centB' + RB*ppB';
        ppdB = [-sdB(i2B,j-1).*(sin(sB(i2B,j-1)/rhoB))...
                sdB(i2B,j-1).*(cos(sB(i2B,j-1)/rhoB))...
                zeros(length(sB(i2B,j-1)),1)];
        pdB(:,i2B) = RB*ppdB';
        ppddB = [-(((sdB(i2B,j-1)).^2).*(cos(sB(i2B,j-1)/rhoB))/rhoB)-(sddB(i2B,j-1).*sin(sB(i2B,j-1)/rhoB))...
                -(((sdB(i2B,j-1)).^2).*(sin(sB(i2B,j-1)/rhoB))/rhoB)+(sddB(i2B,j-1).*cos(sA(i2B,j-1)/rhoB))...
                 zeros(length(sB(i2B,j-1)),1)];
        pddB(:,i2B) = RB*ppddB';
        
        for k = TB(j)-DtB(j)+1 : TB(end)-DtB(end)
            pB(:,k) = pB(:,i2B(end));
            pdB(:,k) = pdB(:,i2B(end));
            pddB(:,k) = 0;
        end
    end
end

%% Adattare lunghezze delle traiettorie

Ctot = max([length(pA) length(pB)]);
Nc_regime = 1000;

% Allungo il vettore pA per vedere cosa accade a regime nei grafici
for h = length(pA)+1:length(pA)+Nc_regime
    pA(:,h) = pA(:,end);
    pdA(:,h) = pdA(:,end);
    pddA(:,h) = zeros(3,1);
end 
% Allungo il vettore pB per vedere cosa accade a regime nei grafici
for h = length(pB)+1:length(pB)+Nc_regime
    pB(:,h) = pB(:,end);
    pdB(:,h) = pdB(:,end);
    pddB(:,h) = zeros(3,1);
end 

if (length(pA) > length(pB))
    diff = length(pA) - length(pB);
    for h = length(pB)+1:length(pB)+diff
        pB(:,h) = pB(:,end);
        pdB(:,h) = pdB(:,end);
        pddB(:,h) = pddB(:,end);
    end
elseif (length(pA) < length(pB))
    diff = length(pB) - length(pA);
    for h = length(pA)+1:length(pA)+diff
        pA(:,h) = pA(:,end);
        pdA(:,h) = pdA(:,end);
        pddA(:,h) = pddA(:,end);
    end   
end

%% Traiettoria giunto prismatico di base (vettore di 0)

pBase = zeros(1,length(pA));
pdBase = zeros(1,length(pdA));
pddBase = zeros(1,length(pddA));

%% Unisco i vettori relativi ad A e B

p(1:2,:) = pA(1:2,:); 
p(3:4,:) = pB(1:2,:);
p(5,:) = pBase;
pd(1:2,:) = pdA(1:2,:); 
pd(3:4,:) = pdB(1:2,:);
pd(5,:) = pdBase;
pdd(1:2,:) = pddA(1:2,:); 
pdd(3:4,:) = pddB(1:2,:);
pdd(5,:) = pddBase;


%% Trasformo posizioni e velocità in oggetti di tipo timeseries per portarli in simulink
Nc = NcA;           % ??????

time = 1/Nc : 1/Nc : length(p)/Nc;

xd = timeseries(p', time);
xd_dot = timeseries(pd', time);
xd_ddot = timeseries(pdd', time);

%% (A) Plot

figure(1)
title('Traiettoria pianificata Manipolatore A')
g1 = plot(p(1,:), p(2,:),'r');
grid;

xlabel('x'); ylabel('y');
figure(2);
subplot(2,1,1); plot(time,pd(1,:),'r'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{x}(t) [m]$','Interpreter','Latex');legend('pd_x'); grid; 
subplot(2,1,2); plot(time,pd(2,:),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{y}(t) [m]$','Interpreter','Latex');legend('pd_y'); grid;
sgtitle('Velocità Manipolatore A')
figure(3);
subplot(2,1,1); plot(time,pdd(1,:),'r'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{x}(t) [m]$','Interpreter','Latex'); legend('pdd_x'); grid;
subplot(2,1,2); plot(time,pdd(2,:),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{y}(t) [m]$','Interpreter','Latex'); legend('pdd_y'); grid;
sgtitle('Accelerazione Manipolatore A')
figure(4);
subplot(2,1,1); plot(time,p(1,:),'r'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$x(t) [m]$','Interpreter','Latex'); legend('p_x'); grid;
subplot(2,1,2); plot(time,p(2,:),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$y(t) [m]$','Interpreter','Latex'); legend('p_y'); grid;
sgtitle('Posizione Manipolatore A')

%% (B) Plot

figure(1)
title('Traiettoria pianificata Manipolatore B')
g1 = plot(p(3,:), p(4,:),'b');

xlabel('x'); ylabel('y');
figure(5);
subplot(2,1,1); plot(time,pd(3,:),'r'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{x}(t) [m]$','Interpreter','Latex');legend('pd_x'); grid; 
subplot(2,1,2); plot(time,pd(4,:),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{y}(t) [m]$','Interpreter','Latex');legend('pd_y'); grid;
sgtitle('Velocità Manipolatore B')
figure(6);
subplot(2,1,1); plot(time,pdd(3,:),'r'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{x}(t) [m]$','Interpreter','Latex'); legend('pdd_x'); grid;
subplot(2,1,2); plot(time,pdd(4,:),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{y}(t) [m]$','Interpreter','Latex'); legend('pdd_y'); grid;
sgtitle('Accelerazione Manipolatore B')
figure(7);
subplot(2,1,1); plot(time,p(3,:),'r'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$x(t) [m]$','Interpreter','Latex'); legend('p_x'); grid;
subplot(2,1,2); plot(time,p(4,:),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$y(t) [m]$','Interpreter','Latex'); legend('p_y'); grid;
sgtitle('Posizione Manipolatore B')




