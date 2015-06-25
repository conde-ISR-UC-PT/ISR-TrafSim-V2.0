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
% View cross cels occupied by the trajectory
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

function [crosscels] = view_cross_cels_isr_trafsim(n)
global c s

crosscels=[];
if(isempty(find(c.car(n).traj(3,:)==40))==0)
    ncels=12;
    ymin=s.crosszone(4);
    ymax=s.crosszone(3);
    xmin=s.crosszone(1);
    xmax=s.crosszone(2);
    delta=(ymax-ymin)/ncels;
    
    for i=1:c.car(n).trajn
        
        if(c.car(n).traj(3,i)==40)
            type=c.car(n).type;
            
            x=c.car(n).traj(1,i);
            y=c.car(n).traj(2,i);
            t=c.car(n).traj(5,i);
            [cornX cornY] = pcorners_isr_trafsim(c.car(n).pos.t,c.car(n).length,c.car(n).width);
            cornX=cornX+x;
            cornY=cornY+y;
            
            for j=1:4
                if(cornX(j)>xmin && cornX(j)<xmax && cornY(j)>ymin && cornY(j)<ymax)
                    %                         plot(cornX(j),cornY(j),'*')
                    xcel=floor((cornX(j)-xmin)/delta)+1;
                    ycel=floor((cornY(j)-ymin)/delta)+1;
                    cel=(ycel-1)*ncels+xcel;
                    if(cel>=0 && cel<=(ncels^2))
                        if(isempty(find(crosscels==cel)))
                            crosscels=[crosscels cel];
                        end
                    end
                end
            end
        end
    end
end

end