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
% Print vehicles/ghosts on replay mode
%
%-USE----------------------------------------------------------------------
%
% -> Input(s)
%   » data  -   Array with vehicle data
%
% -> Output(s)
%   » Array with handlers to the patchs used
%
%-DISCLAIMER---------------------------------------------------------------%-DISCLAIMER---------------------------------------------------------------
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

function [h] = printoffline_isr_trafsim(data)
global s

h=[];

% Isolate input arguments data=[x,y,t,type,dri]
x=data(1); y=data(2); t=data(3); cartype=data(4); dritype=data(5);

% Calculates the positions of the corners
par=car_type(cartype);
[cornX, cornY]=isr_tfs_pcorners(t,par.length,par.width);

if(dritype==1), color=s.cd1; elseif(dritype==2), color=s.cd2; elseif(dritype==3), color=s.cd3; end

% Make patch
h_=patch('XData',cornX+x,'YData',cornY+y,'FaceColor',color);
h=[h h_];

ind=6;
if(s.gps==2)
    gpsx=data(ind); ind=ind+1;
    gpsy=data(ind); ind=ind+1;
    h_=patch('XData',cornX+gpsx,'YData',cornY+gpsy,'FaceColor','none','EdgeColor','b');
    h=[h h_];
end
if(s.ins==2)
    insx=data(ind); ind=ind+1;
    insy=data(ind); ind=ind+1;
    inst=data(ind); %ind=ind+1;
    [cornX cornY]=corners(inst, cartype);
    h_=patch('XData',cornX+insx,'YData',cornY+insy,'FaceColor','none','EdgeColor','g');
    h=[h h_];
end

end

