% close all
% clear
% clc

%% (A) Punti 

p1A = [-0.2319 -0.8386 0]';
p2A = [-0.15 -0.65 0]';
rho1 = 0.1; cent1 = [-0.15 -0.75 0]; R1 = [0 -1 0; 1 0 0; 0 0 1]; ang1 = -2*pi;
p3A = [-0.15 -0.65 0]'; 
p4A = [-0.19 -0.9 0]';
% p4A = [-0.19 -0.8386 0]';

% p1A = [-0.2174 -0.7539 0]';
% p4A = [-0.2174 -0.7539 0]';
% 9212

%% (B) Punti

p1B = [0.2319 -0.8386 0]';
p2B = [0.29 -0.8 0]';
p3B = [0.29 -0.6 0]';
p4B = [0.19 -0.9 0]';
% p4B = [0.19 -0.8386 0]';


% p1B = [0.2174 -0.7539 0]';
% p2B = [0.29 -0.9 0]';
% p3B = [0.29 -0.75 0]';
% p4B = [0.2174 -0.7539 0]';

% p1B = [0.2319 -0.8386 0]';
% p2B = [0.3 -0.8 0]';
% p3B = [0.09413 -0.3 0]';
% p4B = [0.09413 -0.3 0]';

% p3B = [0.07386 -0.4554 0]';
% p4B = [0.07386 -0.5 0]';

%% Orientamenti 
OA = [pi/6 pi/4-0.3 pi/4-0.2 0];               % Vettore degli orientamenti
OB = [-pi/6 -0.1 -pi/4 0];                     % Vettore degli orientamenti

%% (A) Vettore delle posizioni, dei tempi e dei delta

PA = [p1A p2A p3A p4A];                        % Vettore delle posizioni

% tA = [0 5 12.5 17.5];                                     % Vettore degli istanti temporali di passaggio per i punti
tA = [0 10 35 40];                                          % Vettore degli istanti temporali di passaggio per i punti

dtA = [0 0 0 0];                                            % Vettore degli anticipi per i punti di via
Tipo_curva_A = [0 1 0 0];                                   % 0=segmento, 1=circonferenza

rho_vec_A =  [0 rho1 0 0];                                  % Vettore dei raggi dei tratti di circonferenza
cent_vec_A = [zeros(1,3) cent1 zeros(1,3) zeros(1,3)];      % Vettore delle posizioni centri dei tratti di circonferenza
ang_vec_A =  [0 ang1 0 0];                                  % Vettore delle distanze angolari dei tratti circolari
R_vec_A =    [zeros(3) R1 zeros(3) zeros(3)];               % Vettore delle matrici di rotazione relative ai tratti di circonferenza

%% (B) Vettore delle posizioni, dei tempi e dei delta

PB = [p1B p2B p3B p4B];                              % Vettore delle posizioni

% tB = [0 5 12.5 17.5];                                   % Vettore degli istanti temporali di passaggio per i punti
tB = [0 10 35 40];                                   % Vettore degli istanti temporali di passaggio per i punti

dtB = [0 0 0 0];                                     % Vettore degli anticipi per i punti di via
Tipo_curva_B = [0 0 0 0];                            % 0=segmento, 1=circonferenza

rho_vec_B =  [0 0 0 0];                              % Vettore dei raggi dei tratti di circonferenza
cent_vec_B = [zeros(1,3) zeros(1,3) zeros(1,3) zeros(1,3)];  % Vettore delle posizioni centri dei tratti di circonferenza
ang_vec_B =  [0 0 0 0];                              % Vettore delle distanze angolari dei tratti circolari
R_vec_B =    [zeros(3) zeros(3) zeros(3) zeros(3)];  % Vettore delle matrici di rotazione relative ai tratti di circonferenza

%% (A) Campioni

NcA = 100;                                                               % Numero dei campioni al secondo
TA = NcA * tA;
CampioniA = zeros(1,length(TA)-1);                                       % Vettore dei campioni
for j=1:length(TA)-1
    CampioniA(j) = TA(j+1) - TA(j);
end

%% (B) Campioni

NcB = 100;                                                               % Numero dei campioni al secondo
TB = NcB * tB;
CampioniB = zeros(1,length(TB)-1);                                       % Vettore dei campioni
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
    if Tipo_curva_A(j-1) == 0                          % tratto rettilineo
        distA = norm(PA(:,j)-PA(:,j-1));
        [spA(end+1:end+CampioniA(j-1)), spdA(end+1:end+CampioniA(j-1)), spddA(end+1:end+CampioniA(j-1))] = lspb(0, distA, CampioniA(j-1));
    else                                               % circonferenza
        distA = ang_vec_A(j-1)*rho_vec_A(j-1);
        [spA(end+1:end+CampioniA(j-1)), spdA(end+1:end+CampioniA(j-1)), spddA(end+1:end+CampioniA(j-1))] = lspb(0, distA, CampioniA(j-1));
    end
end

sA = zeros(CtotA,length(PA(1,:))-1);
sdA = zeros(CtotA,length(PA(1,:))-1);
sddA = zeros(CtotA,length(PA(1,:))-1);
for j = 2:length(PA(1,:))
    i1A = 1 : TA(j-1)-DtA(j);                         % intervallo 1
    i2A = TA(j-1)-DtA(j)+1 : TA(j)-DtA(j);            % intervallo 2
    i3A = TA(j)-DtA(j)+1 : TA(end)-DtA(end);          % intervallo 3
    
    if Tipo_curva_A(j-1) == 0                         % tratto rettilineo
        distA = norm(PA(:,j)-PA(:,j-1));
    else                                              % circonferenza
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
    i1A = 1 : TA(j-1)-DtA(j);                         % intervallo 1
    i2A = TA(j-1)-DtA(j)+1 : TA(j)-DtA(j);            % intervallo 2
    i3A = TA(j)-DtA(j)+1 : TA(end)-DtA(end);          % intervallo 3
    
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

%% (A) Pianificazione orientamento

distA = norm(OA(2)-OA(1));
[sopA, sopdA, sopddA] = lspb(0, distA, CampioniA(1));

for j = 3:length(OA)
    distA = norm(OA(j)-OA(j-1));
    [sopA(end+1:end+CampioniA(j-1)), sopdA(end+1:end+CampioniA(j-1)), sopddA(end+1:end+CampioniA(j-1))] = lspb(0, distA, CampioniA(j-1));
end

soA = zeros(CtotA,length(OA)-1);
sodA = zeros(CtotA,length(OA)-1);
soddA = zeros(CtotA,length(OA)-1);
for j = 2:length(OA)
    i1A = 1 : TA(j-1)-DtA(j);                          % intervallo 1
    i2A = TA(j-1)-DtA(j)+1 : TA(j)-DtA(j);               % intervallo 2
    i3A = TA(j)-DtA(j)+1 : TA(end)-DtA(end);             % intervallo 3
    
    distA = norm(OA(j)-OA(j-1));

    soA(i1A,j-1) = 0;                                  % Ascissa curvilinea posizioni
    soA(i2A,j-1) = sopA(i2A+DtA(j));
    soA(i3A,j-1) = distA;
    
    sodA(i1A,j-1) = 0;                                 % Ascissa curvilinea velocità
    sodA(i2A,j-1) = sopdA(i2A+DtA(j));
    sodA(i3A,j-1) = 0;
    
    soddA(i1A,j-1) = 0;                                % Ascissa curvilinea accelerazioni
    soddA(i2A,j-1) = sopddA(i2A+DtA(j));
    soddA(i3A,j-1) = 0;
end

oA = OA(1);
odA = 0;
oddA = 0;
for j = 2:length(OA)
    i1A = 1 : TA(j-1)-DtA(j);                          % intervallo 1
    i2A = TA(j-1)-DtA(j)+1 : TA(j)-DtA(j);               % intervallo 2
    i3A = TA(j)-DtA(j)+1 : TA(end)-DtA(end);             % intervallo 3
    
    distA = OA(j)-OA(j-1);
    oA = oA + (soA(:,j-1) / norm(distA) * distA')';
    odA = odA + (sodA(:,j-1) / norm(distA) * distA')';
    oddA = oddA + (soddA(:,j-1) / norm(distA) * distA')';
end

%% (B) Pianificazione orientamento

distB = norm(OB(2)-OB(1));
[sopB, sopdB, sopddB] = lspb(0, distB, CampioniB(1));
for j = 3:length(OB)
    distB = norm(OB(j)-OB(j-1));
    [sopB(end+1:end+CampioniB(j-1)), sopdB(end+1:end+CampioniB(j-1)), sopddB(end+1:end+CampioniB(j-1))] = lspb(0, distB, CampioniB(j-1));
end

soB = zeros(CtotB,length(OB)-1);
sodB = zeros(CtotB,length(OB)-1);
soddB = zeros(CtotB,length(OB)-1);
for j = 2:length(OB)
    i1B = 1 : TB(j-1)-DtB(j);                          % intervallo 1
    i2B = TB(j-1)-DtB(j)+1 : TB(j)-DtB(j);               % intervallo 2
    i3B = TB(j)-DtB(j)+1 : TB(end)-DtB(end);             % intervallo 3
    
    distB = norm(OB(j)-OB(j-1));

    soB(i1B,j-1) = 0;                                  % Ascissa curvilinea posizioni
    soB(i2B,j-1) = sopB(i2B+DtB(j));
    soB(i3B,j-1) = distB;
    
    sodB(i1B,j-1) = 0;                                 % Ascissa curvilinea velocità
    sodB(i2B,j-1) = sopdB(i2B+DtB(j));
    sodB(i3B,j-1) = 0;
    
    soddB(i1B,j-1) = 0;                                % Ascissa curvilinea accelerazioni
    soddB(i2B,j-1) = sopddB(i2B+DtB(j));
    soddB(i3B,j-1) = 0;
end

oB = OB(1);
odB = 0;
oddB = 0;
for j = 2:length(OB)
    i1B = 1 : TB(j-1)-DtB(j);                            % intervallo 1
    i2B = TB(j-1)-DtB(j)+1 : TB(j)-DtB(j);               % intervallo 2
    i3B = TB(j)-DtB(j)+1 : TB(end)-DtB(end);             % intervallo 3
    
    distB = OB(j)-OB(j-1);
    oB = oB + (soB(:,j-1) / norm(distB) * distB')';
    odB = odB + (sodB(:,j-1) / norm(distB) * distB')';
    oddB = oddB + (soddB(:,j-1) / norm(distB) * distB')';
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
% Allungo il vettore oA per vedere cosa accade a regime nei grafici
for h = length(oA)+1:length(oA)+Nc_regime
    oA(:,h) = oA(:,end);
    odA(:,h) = odA(:,end);
    oddA(:,h) = 0;
end 
% Allungo il vettore oB per vedere cosa accade a regime nei grafici
for h = length(oB)+1:length(oB)+Nc_regime
    oB(:,h) = oB(:,end);
    odB(:,h) = odB(:,end);
    oddB(:,h) = 0;
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
xd_full = timeseries([p(1:2,:)' oA' p(3:4,:)' oB'], time);
xd_full_dot = timeseries([pd(1:2,:)' odA' pd(3:4,:)' odB'], time);
xd_full_ddot = timeseries([pdd(1:2,:)' oddA' pdd(3:4,:)' oddB'], time);

%% Plot

figure(1)
plot(p(1,:), p(2,:)-4.333,'r'); hold on;
plot(p(3,:), p(4,:)-4.333,'b');
grid;
xlim([-0.35 0.35]); ylim([-1.1039-4.333 -0.4039-4.333]);
axis square
xlabel('x [m]'); ylabel('y [m]');
legend('Arm 1 trajectory reference','Arm 2 trajectory reference','AutoUpdate','off')

scatter(p1A(1), p1A(2)-4.333,'r'); hold on
for k = 2:length(PA(1,:))
    scatter(PA(1,k), PA(2,k)-4.333,'r')
end
scatter(p1B(1), p1B(2)-4.333,'b'); hold on
for k = 2:length(PB(1,:))
    scatter(PB(1,k), PB(2,k)-4.333,'b')
end

text(-0.265,-0.8386-4.333,'A');
text(-0.15,-0.62-4.333,'B=C');
text(-0.173,-0.9-4.333,'D');
text(0.25,-0.8386-4.333,'E');
text(0.305,-0.8-4.333,'F');
text(0.305,-0.6-4.333,'G');
text(0.205,-0.9-4.333,'H');

figure(2)
plot(time,oA,'r'); grid; xlabel('[s]'); ylabel('[rad]'); xlim([time(1) time(end)]); hold on; grid; ylim([-1 1]); axis square
plot(time,oB,'b'); grid; xlabel('[s]'); ylabel('[rad]'); xlim([time(1) time(end)]);
legend('Arm 1 orientation reference','Arm 2 orientation reference','Location','northeast')

figure(3);
subplot(2,1,1); plot(time,p(1,:),'r'); xlabel('[s]'); ylabel('[m]');legend('$x$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);
subplot(2,1,2); plot(time,p(2,:)-4.333,'b'); xlabel('[s]'); ylabel('[m]');legend('$y$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);
figure(4);
subplot(2,1,1); plot(time,pd(1,:),'r'); xlabel('[s]'); ylabel('[m/s]');legend('$\dot{x}$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);
subplot(2,1,2); plot(time,pd(2,:),'b'); xlabel('[s]'); ylabel('[m/s]');legend('$\dot{y}$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);
xlim([time(1) time(end)]);
figure(5);
subplot(2,1,1); plot(time,pdd(1,:),'r'); xlabel('[s]'); ylabel('[m/s^2]');legend('$\ddot{x}$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);
subplot(2,1,2); plot(time,pdd(2,:),'b'); xlabel('[s]'); ylabel('[m/s^2]');legend('$\ddot{y}$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);


% % sgtitle('Velocità Manipolatore A')
% figure(3);
% subplot(2,1,1); plot(time,pdd(1,:),'r'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{x}(t) [m]$','Interpreter','Latex'); legend('pdd_x'); grid;
% subplot(2,1,2); plot(time,pdd(2,:),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{y}(t) [m]$','Interpreter','Latex'); legend('pdd_y'); grid;
% xlim([time(1) time(end)]);
% sgtitle('Accelerazione Manipolatore A')
% figure(2);
% subplot(2,1,1); plot(time,p(1,:),'r'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$x(t) [m]$','Interpreter','Latex'); legend('p_x'); grid;
% subplot(2,1,2); plot(time,p(2,:),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$y(t) [m]$','Interpreter','Latex'); legend('p_y'); grid;
% xlim([time(1) time(end)]);
% % sgtitle('Posizione Manipolatore A')

figure(6)
subplot(2,1,1); plot(time,p(3,:),'r'); xlabel('[s]'); ylabel('[m]');legend('$x$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);
subplot(2,1,2); plot(time,p(4,:)-4.333,'b'); xlabel('[s]'); ylabel('[m]');legend('$y$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);
figure(7);
subplot(2,1,1); plot(time,pd(3,:),'r'); xlabel('[s]'); ylabel('[m/s]');legend('$\dot{x}$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);
subplot(2,1,2); plot(time,pd(4,:),'b'); xlabel('[s]'); ylabel('[m/s]');legend('$\dot{y}$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);
xlim([time(1) time(end)]);
figure(8);
subplot(2,1,1); plot(time,pdd(3,:),'r'); xlabel('[s]'); ylabel('[m/s^2]');legend('$\ddot{x}$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);
subplot(2,1,2); plot(time,pdd(4,:),'b'); xlabel('[s]'); ylabel('[m/s^2]');legend('$\ddot{y}$','Interpreter','Latex'); grid; xlim([time(1) time(end)]);

% figure(5);
% subplot(2,1,1); plot(time,pd(3,:),'r'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{x}(t) [m]$','Interpreter','Latex');legend('pd_x'); grid; 
% subplot(2,1,2); plot(time,pd(4,:),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{y}(t) [m]$','Interpreter','Latex');legend('pd_y'); grid;
% sgtitle('Velocità Manipolatore B')
% figure(6);
% subplot(2,1,1); plot(time,pdd(3,:),'r'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{x}(t) [m]$','Interpreter','Latex'); legend('pdd_x'); grid;
% subplot(2,1,2); plot(time,pdd(4,:),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{y}(t) [m]$','Interpreter','Latex'); legend('pdd_y'); grid;
% sgtitle('Accelerazione Manipolatore B')
% figure(7);
% subplot(2,1,1); plot(time,p(3,:),'r'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$x(t) [m]$','Interpreter','Latex'); legend('p_x'); grid;
% subplot(2,1,2); plot(time,p(4,:),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$y(t) [m]$','Interpreter','Latex'); legend('p_y'); grid;
% sgtitle('Posizione Manipolatore B')




