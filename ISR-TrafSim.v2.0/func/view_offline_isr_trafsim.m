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
% Play a video based on save data
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

function [] = view_offline_isr_trafsim()
global s  d1 d2

% Open file
writerObj = VideoWriter('isr_tfs','MPEG-4');
open(writerObj);

s.hfigsim = figure(99);
set(s.hfigsim,'Position',[500 500 300 100],'NumberTitle','off','Name','Dialog box','Resize','off');
uicontrol('Position',[20 20 260 60],'String','Press to view the results of simulation',...
    'Callback','uiresume(gcbf)');
uiwait(gcf);
close(s.hfigsim);

disp(' ')
disp('Simulation mode offline')
str=sprintf('Recording Interval %1.2f (s)',s.rec);
disp(str)

s.mode=1;   s.time=0;   ind=1; flag=0; aux=1;
s.hsema=[]; s.lights=[];
[lin,~]=size(d1); h=[];

% Initialization of the graphical environment
isr_tfs_graphical_environment()
set(s.hfigsim,'NumberTitle','off','Name','Offline simulation','Position',[100 100 s.dimx*6 s.dimy*6],'Resize','off');

% Plot roads
isr_tfs_printset(); pause(1);


for time=0:(1/s.frec):s.duration
    
    % Print traffic lights
    s.lights=[];
    s.lights(:,:)=d2(:,:,aux);
    aux=aux+1;
    isr_tfs_trafficlights();
    
    % Refresh time
    s.time=time;
    
    % Print current time in command window
    if(mod(time,1)==0)
        str=sprintf('Time= %d (s)',s.time);
        disp(str)
    end
    
    % Plot time in simulator window
    if(s.printtime==1 && s.mode==1)
        %isr_tfs_print('time',0)
    end
    
    tic
    % Print all cars
    if(flag==0)
        while(round2(d1(ind,1),0.001)==round2(time,0.001))
            d_=[d1(ind,2),d1(ind,3),d1(ind,4),d1(ind,5),d1(ind,6)];
            ii=7;
            if(s.gps==2)
                d_=[d_ d1(ind,ii) d1(ind,ii+1)];
                ii=ii+2;
            end
            if(s.ins==2)
                d_=[d_ d(ind,ii) d1(ind,ii+1) d1(ind,ii+2)];
                ii=ii+3;
            end
            if(s.fus==2)
                d_=[d_ d1(ind,ii) d1(ind,ii+1) d1(ind,ii+2)];
                ii=ii+3;
            end
            curh=isr_tfs_printoffline(d_);
            h=[h curh];
            ind=ind+1;
            if(ind==lin)
                flag=1;
                break
            end
        end
    end
    drawnow
    frame=getframe;
    writeVideo(writerObj,frame);
    
    a=toc;
    if((1/s.frec)-a>0)
        pause((1/s.frec)-a)
    else
        disp('delay')
    end
    delete(h); h=[];
    
end
disp('End of offline mode')
close(s.hfigsim)
close(writerObj);
end
