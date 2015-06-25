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
%
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

function [M,L] = write_MM_isr_trafsim(M,L,time,x,y,t,~,width,length,margin_w,margin_l,map_res,map_dimx,map_dimy)

% Select the correct layer
layer=find(L==round2(time,0.01));

if(isempty(layer))
    [M,L]=create_L(M,L,round2(time,0.01),map_res,map_dimx,map_dimy);
    layer=length(L);
end

% Rotation matrix
m_rot=[cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];

% Calculate car relative points
widht=width+margin_w; length=length+margin_l;
cy=[-widht/2 widht/2 widht/2 -widht/2];
cx=[length/2 length/2 -length/2 -length/2];

% New points of robot
p1t=m_rot*[cx(1);cy(1);1]; p2t=m_rot*[cx(2);cy(2);1]; p3t=m_rot*[cx(3);cy(3);1]; p4t=m_rot*[cx(4);cy(4);1];
cornX=[p1t(1) p2t(1) p3t(1) p4t(1)];
cornY=[p1t(2) p2t(2) p3t(2) p4t(2)];

% Define polygon points
xbox=[cornX cornX(1)]+x;
ybox=[cornY cornY(1)]+y;

% Merge the mask and the current occupation layer
mask = poly2mask(xbox*map_res,ybox*map_res,size(M,1),size(M,2));
M(:,:,layer)=M(:,:,layer)|mask;

end