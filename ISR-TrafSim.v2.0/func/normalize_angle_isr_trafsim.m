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
% Normalize the input angle in accordance with selected options
%
%-USE----------------------------------------------------------------------
%
% -> Input(s)
%   » ang   -   Input angle
%   » in    -   Type of input angle ('rad'/'deg')
%   » out   -   Type of output angle ('rad'/'deg')
%   » mode  -   Option
%       If 1 convert ang to 0<ang<360
%       If 2 convert ang to -180<ang<180
%
% -> Output(s)
%   » ang   -   Angle converted according to specifications
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

function [ ang ] = normalize_angle_isr_trafsim(ang,mode,in,out)

if(in=='rad')
    ang=ang*180/pi;
elseif(in=='deg')
    
else
    disp('Is not possible convert the angle')
end

if(mode==1)
    while(ang>360)
        ang=ang-360;
    end
    while(ang<0)
        ang=ang+360;
    end
elseif(mode==2)
    while(ang>180)
        ang=ang-360;
    end
    while(ang<-180)
        ang=ang+360;
    end
else
    disp('Is not possible convert the angle')
end


if(out=='rad')
    ang=ang*pi/180;
elseif(out=='deg')
else
    disp('Is not possible convert the angle')
end
end

