%--------------------------------------------------------------------------
%                           ISR-TrafSim v2.0
%                        Copyright (C) 2010-2013
%
%--------------------------------------------------------------------------
% This Matlab file is part of the ISR-TrafSim: a Matlab
% library for traffic simulation and pose estimation in Urban environments,
% namely roundabouts and crossroads.
%
% http://www.isr.uc.pt/~conde/isr-trafsim/
%
%-CITATION---------------------------------------------------------------------------
% If you use this software please cite one of the following papers:
% 1) L.C.Bento, R.Parafita, S.Santos and U.Nunes, An Intelligent Traffic Management
% at Intersections legacy mode for vehicles not equipped with V2V and V2I Communications,
% 16th IEEE Int.Conf. Intelligent Transportation Systems, Netherlands, 2013.
% 2) L.C.Bento, R.Parafita and U.Nunes, Inter-vehicle sensor fusion for accurate vehicle
% localization supported by V2V and V2I communications, 15th IEEE Int.Conf. Intelligent
% Transportation Systems, USA, 2012.
% 3) L.C.Bento, R.Parafita and U.Nunes, Intelligent traffic management at intersections
% supported by V2V and V2I communications, 15th IEEE Int.Conf. Intelligent
% Transportation Systems, USA, 2012.
%
%-DESCRIPTION--------------------------------------------------------------
%
% Simulator initialization
%
%-DISCLAIMER---------------------------------------------------------------
% This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY;
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
% You can use this source code without licensing fees only for NON-COMMERCIAL research
% and EDUCATIONAL purposes only.
% You cannot repost this file without prior written permission from the authors.
%
%-AUTHORS------------------------------------------------------------------
%   Urbano Nunes*
%   Luis Conde Bento**
%   Ricardo Parafita*
%   Sergio Santos*
%
%  *Institute of Systems and Robotics   - University of Coimbra
% **School of Technology and Management - Polytechnic Institute of Leiria
%--------------------------------------------------------------------------

% Clear all previous information and compile necessary functions
clear all; close all; clc;

% Add path to GPS/INS/KalmanFilter toolbox
addpath check commu commu\lib environmental func fusion
addpath gps2 graphic mag_rule matdata odometry reserveM
addpath savedata spreadsheet statistics steer
%addpath gps1 gps3 inertial lidar

% Mex files (if is necessary)
% mex lidar\map2.c; clear mex;

% Clear all previous vehicles log files in folder savedata\car
if(exist('savedata\car.txt'))
    rmdir('savedata\car','s');
end
if(~exist('savedata\car'))
    mkdir('savedata\car');
end
if(exist('savedata\history.txt'))
    diary OFF
    delete('savedata\history.txt')
end
diary('savedata\history.txt')
    
% Initializes all the variables necessary for the simulator
global s; s=configsim_isr_trafsim('init');

% Initializes global variables to store data to run simulation in offline mode (video)
global d1 d2; d1=[]; d2=[];

% Initializes all the variables necessary for the GPS simulation
if(s.gps_type==1 && s.gps>1)
    global gps_data; gps_data=gps_isr_trafsim('init');
end

% Initializes all the variables necessary for the INS simulation
if(s.ins>1)
    global ins_data; ins_data=ins_isr_trafsim('init');
end

% Initializes all the variables necessary for the LIDDAR simulation
if(s.las>1)
    global las_data; las_data=las_isr_trafsim('init');
end

% Load initial configurations
s.in=ones(12,8);
s.in_flux(1:12)=s.tlaunch;
s.in_extra=zeros(12,3); 
s.in_extra(1:12,3)=[1:12'];
s.full_cars=3;
s.full=0;

% Initializes users interface
%s.hinterface=isr_tfs_interface();
run_isr_trafsim

