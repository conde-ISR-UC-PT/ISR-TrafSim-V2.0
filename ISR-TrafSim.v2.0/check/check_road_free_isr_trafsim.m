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
% Check if road is clear in a distance (1-Clear, 0-Road filled)
% If distance is zero, all road is verified
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

function [out] = check_road_free_isr_trafsim(r,d)
global s

% If distance argumemt is zero, all road is verified
if(d==0)
    d=s.ri(r,6);    % d is the road length
end

% Roads information (road)[1xi 2yi 3xf 4yf 5nsetp 6length 7ang 8intype 9endtype 10nextlaunch 11countdelay]
px=s.ri(r,1);
py=s.ri(r,2);
t=s.ri(r,7);
l=s.rw/2;
c1x=px+l*cos(t-pi/2);
c1y=py+l*sin(t-pi/2);
c2x=px+l*cos(t+pi/2);
c2y=py+l*sin(t+pi/2);
px_=px+d*cos(t);
py_=py+d*sin(t);
c4x=px_+l*cos(t-pi/2);
c4y=py_+l*sin(t-pi/2);
c3x=px_+l*cos(t+pi/2);
c3y=py_+l*sin(t+pi/2);

% Checks which the pixels that are within the polygon
xbox=[c1x c2x c3x c4x c1x];
ybox=[c1y c2y c3y c4y c1y];

% 1000x com inpolygon Elapsed time is 22.108233 seconds.
% 1000x com a mask    Elapsed time is 0.608906 seconds.
mask = poly2mask(xbox*s.occup_res,ybox*s.occup_res,size(s.ocmap,1),size(s.ocmap,2));
out = ~logical(nnz(s.ocmap(:,:) & mask));

end

