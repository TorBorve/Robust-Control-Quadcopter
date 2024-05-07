close all; clear all; clc;
% Add folders to path
thisfile = which(mfilename);
exercisefolder = fileparts(thisfile);
cd(exercisefolder);
addpath(genpath(exercisefolder));
clear thisfile exercisefolder;