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

function [  ] = add_car_notify_cross_isr_trafsim(  )
global s c

if(size(s.list_cross_proximity,1)>1)
    for r=1:size(s.crosslist,2)
        for i=2:size(s.list_cross_proximity,1)
            carID=s.list_cross_proximity(i,r);
            if(isempty(s.list_cross_without_commu))
                already_reserved=0;
            else
                already_reserved=s.list_cross_without_commu(1)==carID;
            end
            if(carID~=0 && ~already_reserved)
                n=find(c.listactive==carID);                           % Add car to list of cars with velocity profile
                if(isempty(find(carID==s.cross_notify_list(1,:))) && isempty(c.car(n).vel_prof) && isempty(find(carID==s.list_cross_without_commu)) && isempty(find(carID==s.list_cross_already_reserve)))
                    if(dist_isr_trafsim(s.cross_center(1),s.cross_center(2),c.car(n).pos.x,c.car(n).pos.y)<s.control_radius_cross)
                        isr_tfs_ota_commu([carID 90 s.time],'notify_cross');
                        s.cross_notify_list(4,end)=s.cross_notify_list(4,end);
                        % Put semaphore red
                        s.cross_notify_list(9:11,end)=NaN;
                    end
                end
            end
        end
    end
end
end

