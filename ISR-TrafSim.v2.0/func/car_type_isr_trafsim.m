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
%-DESCRIPTION------------------------------------------------------------------------
%
% Runs the main program of the Traffic Simulator
%
%-USE----------------------------------------------------------------------
%
% car_type_isr_tfs(type)
%
% -> Input(s)
%   » type      -   type of vehicles
%
% -> Output(s)
%   » Car physical parameters
%       length  -   [m]
%       width   -   [m]
%       Wsbw    -   [m] (space between wheels)
%       mass    -   Weigth [Kg]
%       c_d     -   Aerodynamic Drag Coefficient
%       c_r     -   Rolling Resistance Coefficient
%       A       -   Frontal Area [m^2]
%       D       -   % 205 / 55 R16
%       r_dyn   -   Tire Radius [m]
%
%-DISCLAIMER-------------------------------------------------------------------------
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

function [ parameters ] = car_type_isr_trafsim( type )

% Car type 1 - European Hatchback (Mégane III 1.4 TCe 130cv Dynamique)
if(type==1)
    length=4.295;    width=1.808;     mass=1205; Wsbw=2.641;
    c_d=0.326;      A=2.21;         c_r=0.013;
    D=((205*0.55 * 2) + 16 * 25.4)/1000; % 205 / 55 R16
    r_dyn=D/2;
    % Car type 2 - European light truck or minivan  (TRAFIC PASSENGER FP Passenger 9 lugares L1H1 1.0T 2.0 dCi 90cv )
else
    length=4.782;    width=1.904;     mass=1912; Wsbw=3.490;
    c_d=0.362;      A=3.38;         c_r=0.013;
    D=((205*0.65 * 2) + 16 * 25.4)/1000; % 205 / 65 R16
    r_dyn=D/2;
end

% Exit parameters
parameters.length=length;
parameters.width=width;
parameters.mass=mass;
parameters.Wsbw=Wsbw;
parameters.c_r=c_r;
parameters.c_d=c_d;
parameters.A=A;
parameters.r_dyn=r_dyn;

end

