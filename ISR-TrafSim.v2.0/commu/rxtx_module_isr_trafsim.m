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
% Communication procedure for a car
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

function [] = rxtx_module_isr_trafsim(n)
global c s

if(c.car(n).commu==1)
    % Attempts to notify the roundabout of his presence until get velocity profile
    if(s.round_gest==1 && s.otacommu==1)
        if(round2(mod(round2(c.car(n).timeinsim,0.001),(1/s.cartxf)),0.0001)==0)
            if(~isempty(intersect(c.car(n).traj(3,c.car(n).setp),s.roundlist)) )
                if(c.car(n).commu_vel_prof==-1 )
                    if( isempty(c.car(n).vel_prof) )
                        ota_commu_isr_trafsim([c.car(n).id 90 s.time],'notify_round');
                    end
                elseif(c.car(n).commu_vel_prof==1 && isnan(c.car(n).vel_prof(1,1)))
                    ota_commu_isr_trafsim([c.car(n).id 90 s.time],'notify_round');
                end
            end
        end
    end
    
    % Attempts to notify the crossroads of his presence until get velocity profile
    if(s.cross_mode==2 && s.otacommu==1)
        if(round2(mod(round2(c.car(n).timeinsim,0.001),(1/s.cartxf)),0.0001)==0)
            if(~isempty(intersect(c.car(n).traj(3,c.car(n).setp),s.crosslist)) )
                if(c.car(n).commu_vel_prof2==-1 )
                    if( isempty(c.car(n).vel_prof2) )
                        ota_commu_isr_trafsim([c.car(n).id 90 s.time],'notify_cross');
                    end
                elseif(c.car(n).commu_vel_prof2==1 && isnan(c.car(n).vel_prof2(1,1)))
                    ota_commu_isr_trafsim([c.car(n).id 90 s.time],'notify_cross');
                end
            end
        end
    end
end

end

