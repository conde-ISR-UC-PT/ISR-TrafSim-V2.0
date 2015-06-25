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
% To view the map (help on debug)
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

function [] = viewM_isr_trafsim(M,L,arg,map_res,n,local)
global s c
if(s.makeVideo_Reserve)
    carID = num2str(c.car(n).id);
    timeC = num2str(s.time);
    movieName=strcat('videosReserve\reserve','_',timeC,'_',local,'_',carID);
    writerObj = VideoWriter(movieName,'MPEG-4');
    set(writerObj,'FrameRate',10)
    open(writerObj)
end

if(arg==0)
    n=size(M,3);
end
figure(40)
for j=1:n
    scalemag= 400*1/map_res;
    imshow(flipud(M(:,:,j)*1000),'InitialMagnification', scalemag);
    if(s.makeVideo_Reserve)
        % Store frame to video
        frame=getframe;
        writeVideo(writerObj,frame);
    end
    pause(0.1)
end
if(s.makeVideo_Reserve)
    % Close video
    close(writerObj);
end
close(40)

end

