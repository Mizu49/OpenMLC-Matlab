clear
clc

addpath(genpath('MLC_tools\'))

generations = 10;

mlc = MLC('MLC_ex_LQR_problem')
mlc.go(generations);

mlc.show_best_indiv