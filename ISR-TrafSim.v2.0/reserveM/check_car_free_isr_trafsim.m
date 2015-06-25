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
% Use to verify the space free in front/back of a vehicle
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

function out = check_car_free_isr_trafsim(amp,n)
global s c

% Car information
px=c.car(n).pos.x;  % Road setpoint to analyze
py=c.car(n).pos.y;  % Road setpoint to analyze
t=c.car(n).pos.t;       % Road orientation

[cornX, cornY] = isr_tfs_pcorners(t,c.car(n).length,c.car(n).width);
cornX=cornX+px; cornY=cornY+py;


% Advance car corners for front in amp(2) meters
cornX(1)=cornX(1)+amp(2)*cos(t); cornY(1)=cornY(1)+amp(2)*sin(t);
cornX(2)=cornX(2)+amp(2)*cos(t); cornY(2)=cornY(2)+amp(2)*sin(t);

% Back car corners for back in amp(1) meters
cornX(3)=cornX(3)-amp(1)*cos(t); cornY(3)=cornY(3)-amp(1)*sin(t);
cornX(4)=cornX(4)-amp(1)*cos(t); cornY(4)=cornY(4)-amp(1)*sin(t);

% Compute box
xbox=[cornX cornX(1)];
ybox=[cornY cornY(1)];

mask = poly2mask(xbox*s.occup_res,ybox*s.occup_res,size(s.ocmap,1),size(s.ocmap,2));
out = s.ocmap(:,:).*uint8(mask);
if(isempty(find(out~=0 & out~=c.car(n).id)));   % If dont exist obstacles in selected space
    out=1;
else
    out=0;
end
end

