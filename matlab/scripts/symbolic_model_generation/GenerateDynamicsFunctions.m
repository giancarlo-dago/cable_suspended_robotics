%%
% disp('Bfile');
% Bfun = matlabFunction(B,'File','B_reduced_f');
% disp('nfile');
% nfun = matlabFunction(n,'File','n_reduced_f');

%%

if ispc % Windows
    addpath ('cranebot\cranebot_with_pilz\')
else % Linux
    addpath ('cranebot/cranebot_with_pilz/')
end

load('B.mat')
load('n.mat')

disp('Bfile');
Bfun = matlabFunction(B,'File','B_planarY_arms_f');
disp('nfile');
nfun = matlabFunction(n,'File','n_planarY_arms_f');