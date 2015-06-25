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
% Open the window to online simulation environment
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

function [] = graphical_environment_isr_trafsim()
global s

if(s.mode==1)
    s.hfigsim=figure(s.figsim);
    %set(s.hfigsim,'Renderer','OpenGL')             % don't speedUP
    %set(0, 'DefaultFigureRenderer', 'OpenGL');     % don't speedUP
    
    set(s.hfigsim,'NumberTitle','off','Name','Online simulation','Position',[100 100 s.dimx*6 s.dimy*6],'Resize','off');
    
    s.hsim=subplot(1,1,1);
    axis([0+5 s.dimx-5 0+5 s.dimy-5]);
    set(s.hsim,'Position',[0.03 0.03 0.95 0.95],'Color',[0.8 0.8 0.8]);
    
    hold on
    
    if(s.printtime==1)
        isr_tfs_print('timeinit',0);
    end
    
    if(s.offline==0)
        if(s.make_video==0)
            uicontrol('Style', 'pushbutton','String', 'Pause','Position', [810 410 60 20],'Callback', 'pause_isr_trafsim');
        end
        if(s.printonlineinfo==1)
            isr_tfs_print('infoinit',0);
            uicontrol('Style', 'pushbutton','String', 'Cross','Position', [810-30 410+30 40 20],'Callback', {@help_fun,'3'});
            uicontrol('Style', 'pushbutton','String', 'Round','Position', [810+50 410+30 40 20],'Callback', {@help_fun,'2'});
            uicontrol('Style', 'pushbutton','String', 'All','Position', [810+20 410+30 20 20],'Callback', {@help_fun,'1'});
        end
    end
end
end
