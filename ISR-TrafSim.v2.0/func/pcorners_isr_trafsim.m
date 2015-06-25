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
% Rotate coordinate points of vehicle corners according to orientation
% angle (without translation)
%
%-USE----------------------------------------------------------------------
%
% -> Input(s)
%   » t     -   Orientation angle (rads)
%   » type  -   Type of vehicle
%
% -> Output(s)
%   » Array with [cornX cornY]
%       cornX   -   Coordinates in X-axis of corners
%       cornY   -   Coordinates in Y-axis of corners
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

function [cornX, cornY] = pcorners_isr_trafsim(t,length,width)

cpry=[-width/2 width/2 width/2 -width/2]; cprx=[length/2 length/2 -length/2 -length/2];

% Rotation matrix
m_rot=[cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];

% New points of robot
p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];


cornX=[p1t(1) p2t(1) p3t(1) p4t(1)];
cornY=[p1t(2) p2t(2) p3t(2) p4t(2)];
end

