clear
clc

addpath(genpath('MLC_tools\'))

mlc = MLC('MLC_ex_LQR_problem')
mlc.go(10);

mlc.show_best_indiv