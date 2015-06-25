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
% Use to verify the space free in a road with a confidence interval
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

function occ=check_roundabout_entrance_isr_trafsim(roadID)
global s
% Roads information
px=s.ri(roadID,3);
py=s.ri(roadID,4);
t=s.ri(roadID,7);       % Road orientation
[cornX, cornY] = pcorners_isr_trafsim(t,s.rw,s.rw);
cornX=cornX+px; cornY=cornY+py;

% Advance car corners for front in amp(2) meters
cornX(1)=cornX(1)+s.amplitudeRoundControl(2)*cos(t); cornY(1)=cornY(1)+s.amplitudeRoundControl(2)*sin(t);
cornX(2)=cornX(2)+s.amplitudeRoundControl(2)*cos(t); cornY(2)=cornY(2)+s.amplitudeRoundControl(2)*sin(t);

% Back car corners for back in amp(1) meters
cornX(3)=cornX(3)-s.amplitudeRoundControl(1)*cos(t); cornY(3)=cornY(3)-s.amplitudeRoundControl(1)*sin(t);
cornX(4)=cornX(4)-s.amplitudeRoundControl(1)*cos(t); cornY(4)=cornY(4)-s.amplitudeRoundControl(1)*sin(t);

% Compute box
xbox=[cornX cornX(1)];
ybox=[cornY cornY(1)];
mask = poly2mask(xbox*s.occup_res,ybox*s.occup_res,size(s.ocmap,1),size(s.ocmap,2));
out = s.ocmap(:,:).*uint8(mask);
resMask=find(out~=0);
occ=[];
if(isempty(resMask) || length(resMask)==1)   % If not exist obstacles in selected space
    s.intersectCarRound(:,roadID)=Inf;
    s.lights(2,roadID)=0;
else
    % Get IDs from occupation space
    carIDs=getCarIDFromMap_isr_trafsim(resMask);
    occ=[occ carIDs];
end

end
