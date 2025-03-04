
%% QUESTO E' IL MODELLO CON LA PRIMA INERZIA, IL SISTEMA E' STRONG INERTIALLY COUPLED

close all
clear
clc

load('B.mat', 'B')
cCode(B,'B_Planar2R_',false,true)

load('n.mat', 'n')
cCode(n,'n_Planar2R_',false,true)

Bbm = B(1,2);  % e' sempre >0 per ogni q2

%% QUESTO E' IL MODELLO CON LA SECONDA INERZIA, IL SISTEMA NON E' STRONG INERTIALLY COUPLED

close all
clear
clc

load('B_v2.mat', 'B')
cCode(B,'B_Planar2R_v2_',false,true)

load('n_v2.mat', 'n')
cCode(n,'n_Planar2R_v2_',false,true)

Bbm = B(1,2);  % NON e' sempre >0 per ogni q2
