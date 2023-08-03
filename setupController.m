% [Project] Cartesian Impedance Controller based on Elastic Potential
% You must run this setup.m file to run the .m scripts.
%
% Authors                       Email
%   [1] Johannes Lachner        jlachner@mit.edu
%

%% Cleaning up + Environment Setup
clear; close all; clc;

% Include all the subdirectories
    cur_path = [ fileparts(pwd), '/controller' ];
    addpath( cur_path );
    addpath( genpath( cur_path ) );

disp( 'Impedance Control library added.' )