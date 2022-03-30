clear
clc

addpath(genpath('MLC_tools\'))
addpath(genpath('SpringMassSystem'))

generations = 30;

% mlc = MLC('MLC_ex_LQR_problem'); % demo by original developers
mlc = MLC('MLC_springmass_problem');

mlc.go(generations);

mlc.show_best_indiv