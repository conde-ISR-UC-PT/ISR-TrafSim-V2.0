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
% Print vehicles/ghost on simulator window (only online mode) and the
% selected information
%
%-USE----------------------------------------------------------------------
%
% -> Input(s)
%   » arg   -   Option
%       car         -   Print selected car
%       time        -   Print updated time
%       timeinit    -   Print time = 0 seconds
%   » n     -   Position of vehicle on the structure
%   » data  -   Not used
%
% -> Output(s)
%   » None
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

function [] = print_isr_trafsim(arg,n)

if(strcmp(arg,'car'))
    pcar(n);
elseif(strcmp(arg,'time'))
    ptime(1);
elseif(strcmp(arg,'info'))
    pinfo(1);
elseif(strcmp(arg,'timeinit'))
    ptime(0);
elseif(strcmp(arg,'infoinit'))
    pinfo(0);
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Print car/info ang ghosts (on-line)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = pcar(n)
global s c

% Delete handlers if are needed
if(~isempty(c.car(n).aux))
    delete(c.car(n).aux);
    c.car(n).aux=[];
end

% Make array to rotation matrix
widht=c.car(n).width; length=c.car(n).length;
cpry=[-widht/2 widht/2 widht/2 -widht/2];
cprx=[length/2 length/2 -length/2 -length/2];

% Startup patches
if(c.car(n).hcar==-1),c.car(n).hcar=patch('XData',s.cprx-300,'YData',s.cpry-300,'FaceColor',c.car(n).dri.color);end
if(s.gps>1),if(c.car(n).hgpspos==-1),c.car(n).hgpspos=patch('XData',s.cprx-300,'YData',s.cpry-300,'FaceColor','none','EdgeColor','b');end,end
if(s.ins>1),if(c.car(n).hinspos==-1),c.car(n).hinspos=patch('XData',s.cprx-300,'YData',s.cpry-300,'FaceColor','none','EdgeColor','g');end,end
if(s.fus>1),if(c.car(n).hfuspos==-1),c.car(n).hfuspos=patch('XData',s.cprx-300,'YData',s.cpry-300,'FaceColor','none','EdgeColor','r');end,end
if(s.fus2>1),if(c.car(n).hfus2pos==-1),c.car(n).hfus2pos=patch('XData',s.cprx-300,'YData',s.cpry-300,'FaceColor','none','EdgeColor','w');end,end
if(s.fus3>1),if(c.car(n).hfus3pos==-1),c.car(n).hfus3pos=patch('XData',s.cprx-300,'YData',s.cpry-300,'FaceColor','none','EdgeColor','c');end,end
if(s.fus4>1),if(c.car(n).hfus4pos==-1),c.car(n).hfus4pos=patch('XData',s.cprx-300,'YData',s.cpry-300,'FaceColor','none','EdgeColor','m');end,end
if(s.odo>1),if(c.car(n).hodopos==-1),c.car(n).hodopos=patch('XData',s.cprx-300,'YData',s.cpry-300,'FaceColor','none','EdgeColor','y');end,end

% Move patch to a new position
set(c.car(n).hcar,'XData',c.car(n).cornX+c.car(n).pos.x,'YData',c.car(n).cornY+c.car(n).pos.y,'FaceColor',c.car(n).dri.color);

% Print security vectors
if(s.printsecurity==1)
    %             dbstop at 107 in isr_tfs_print.m
    if(c.car(n).hcarsec~=-1)
        delete(c.car(n).hcarsec);
    end
    if(c.car(n).hcarsecl~=-1)
        delete(c.car(n).hcarsecl);
    end
    c.car(n).hcarsec=plot([c.car(n).pos.x c.car(n).pos.x+s.sd*cos(c.car(n).pos.t+2*c.car(n).steerval)],[c.car(n).pos.y c.car(n).pos.y+s.sd*sin(c.car(n).pos.t+2*c.car(n).steerval)]);
    c.car(n).hcarsecl=plot([c.car(n).pos.x+s.lsd*cos(c.car(n).pos.t+pi/2) c.car(n).pos.x+s.lsd*cos(c.car(n).pos.t-pi/2)],[ c.car(n).pos.y+s.lsd*sin(c.car(n).pos.t+pi/2) c.car(n).pos.y+s.lsd*sin(c.car(n).pos.t-pi/2) ]);
end

% Move GPS patch to a new position
if(s.gps>1)
    set(c.car(n).hgpspos,'XData',c.car(n).cornX+c.car(n).gps(1,1),'YData',c.car(n).cornY+c.car(n).gps(1,2),'FaceColor','none','EdgeColor','b');
end

% Move INS patch to a new position
if(s.ins>1)
    m_rot=[cos(c.car(n).ins(1,3)) -sin(c.car(n).ins(1,3)) 0; sin(c.car(n).ins(1,3)) cos(c.car(n).ins(1,3)) 0; 0 0 1];
    if(c.car(n).type==1)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    elseif(c.car(n).type==2)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    end
    set(c.car(n).hinspos,'XData',[p1t(1) p2t(1) p3t(1) p4t(1)]+c.car(n).ins(1,1),'YData',[p1t(2) p2t(2) p3t(2) p4t(2)]+c.car(n).ins(1,2),'FaceColor','none','EdgeColor','g');
end

% Move Fusion patch to a new position
if(s.fus>1)
    m_rot=[cos(c.car(n).fus(1,3)) -sin(c.car(n).fus(1,3)) 0; sin(c.car(n).fus(1,3)) cos(c.car(n).fus(1,3)) 0; 0 0 1];
    if(c.car(n).type==1)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    elseif(c.car(n).type==2)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    end
    set(c.car(n).hfuspos,'XData',[p1t(1) p2t(1) p3t(1) p4t(1)]+c.car(n).fus(1,1),'YData',[p1t(2) p2t(2) p3t(2) p4t(2)]+c.car(n).fus(1,2),'FaceColor','none','EdgeColor','r');
end

% Move Fusion 2 patch to a new position
if(s.fus2>1)
    m_rot=[cos(c.car(n).fus2(1,3)) -sin(c.car(n).fus2(1,3)) 0; sin(c.car(n).fus2(1,3)) cos(c.car(n).fus2(1,3)) 0; 0 0 1];
    if(c.car(n).type==1)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    elseif(c.car(n).type==2)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    end
    set(c.car(n).hfus2pos,'XData',[p1t(1) p2t(1) p3t(1) p4t(1)]+c.car(n).fus2(1,1),'YData',[p1t(2) p2t(2) p3t(2) p4t(2)]+c.car(n).fus2(1,2),'FaceColor','none','EdgeColor','w');
end

% Move Fusion 3 patch to a new position
if(s.fus3>1)
    m_rot=[cos(c.car(n).fus3(1,3)) -sin(c.car(n).fus3(1,3)) 0; sin(c.car(n).fus3(1,3)) cos(c.car(n).fus3(1,3)) 0; 0 0 1];
    if(c.car(n).type==1)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    elseif(c.car(n).type==2)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    end
    set(c.car(n).hfus3pos,'XData',[p1t(1) p2t(1) p3t(1) p4t(1)]+c.car(n).fus3(1,1),'YData',[p1t(2) p2t(2) p3t(2) p4t(2)]+c.car(n).fus3(1,2),'FaceColor','none','EdgeColor','c');
end

% Move Fusion 4 patch to a new position
if(s.fus4>1)
    m_rot=[cos(c.car(n).fus4(1,3)) -sin(c.car(n).fus4(1,3)) 0; sin(c.car(n).fus4(1,3)) cos(c.car(n).fus4(1,3)) 0; 0 0 1];
    if(c.car(n).type==1)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    elseif(c.car(n).type==2)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    end
    set(c.car(n).hfus4pos,'XData',[p1t(1) p2t(1) p3t(1) p4t(1)]+c.car(n).fus4(1,1),'YData',[p1t(2) p2t(2) p3t(2) p4t(2)]+c.car(n).fus4(1,2),'FaceColor','none','EdgeColor','m');
end

% Move Odometry patch to a new position
if(s.odo>1)
    m_rot=[cos(c.car(n).odo(1,3)) -sin(c.car(n).odo(1,3)) 0; sin(c.car(n).odo(1,3)) cos(c.car(n).odo(1,3)) 0; 0 0 1];
    if(c.car(n).type==1)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    elseif(c.car(n).type==2)
        p1t=m_rot*[cprx(1);cpry(1);1]; p2t=m_rot*[cprx(2);cpry(2);1]; p3t=m_rot*[cprx(3);cpry(3);1]; p4t=m_rot*[cprx(4);cpry(4);1];
    end
    set(c.car(n).hodopos,'XData',[p1t(1) p2t(1) p3t(1) p4t(1)]+c.car(n).odo(1,1),'YData',[p1t(2) p2t(2) p3t(2) p4t(2)]+c.car(n).odo(1,2),'FaceColor','none','EdgeColor','y');
end

% To print car info
if(s.printinfo==1)
    if(c.car(n).hinfo~=-1)
        delete(c.car(n).hinfo)  % Delete handler if exist
    end
    
    if(s.infotype==1)           % Origin and Destination
        str=sprintf('%d,%d',c.car(n).roadori,c.car(n).roaddes);
    elseif(s.infotype==2)       % Car ID
        str=sprintf('%d',c.car(n).id);
    elseif(s.infotype==3)       % Car linear speed (m/s)
        str=sprintf('%2.0f Km/h',c.car(n).dyn(1)*3.6);
    elseif(s.infotype==4)       % (v,w) Linear Speed and Angular Speed
        str=sprintf('%d,%d',c.car(n).dyn(1),c.car(n).dyn(2));
    elseif(s.infotype==5)       % Steering Angle (Deg)
        str=sprintf('%2.0f Deg',c.car(n).steerval*180/pi);
    elseif(s.infotype==6)       % GPS HDOP
        str=sprintf('HDOP= %2.0f',c.car(n).gps(1,3));
    elseif(s.infotype==7)
        str=sprintf(' %s (%1.0f) - %s',num2str(c.car(n).id), c.car(n).cmdacc, num2str(c.car(n).list));
    elseif(s.infotype==8)
        str=sprintf('%2.0f Km/h %1.0f',c.car(n).dyn(1)*3.6,c.car(n).cmdacc);
    elseif(s.infotype==9)
        str=sprintf('%1.0f',c.car(n).cmdacc);
    elseif(s.infotype==10)
        str=sprintf('ID-%d, %s',c.car(n).id,num2str(c.car(n).list));
    elseif(s.infotype==11)
        str=sprintf('%d',c.car(n).traj(3,c.car(n).setp));
    elseif(s.infotype==12)
        str=sprintf('(%1.0f) %d - %s ',c.car(n).cmdacc,c.car(n).id,num2str(c.car(n).list));
    elseif(s.infotype==13)
        str=sprintf('%d (%1.0f)',c.car(n).id,c.car(n).cmdacc);
    elseif(s.infotype==14)
        if(isnan(c.car(n).vel_prof) )
            str1=sprintf('W');
        elseif(isempty(c.car(n).vel_prof))
            str1=sprintf('D');
        else
            str1=sprintf('M');
        end
        if(isnan(c.car(n).vel_prof2) )
            str2=sprintf('/W ');
        elseif(isempty(c.car(n).vel_prof2))
            str2=sprintf('/D ');
        else
            str2=sprintf('/M ');
        end
        str=[str1 str2 num2str(c.car(n).id)];
    elseif(s.infotype==15)
        if(isnan(c.car(n).vel_prof) )
            str=sprintf('W');
        elseif(isempty(c.car(n).vel_prof))
            str=sprintf('D');
        else
            str=sprintf('M');
        end
    elseif(s.infotype==16)
        str=sprintf('NC-%d',find(s.otalist==c.car(n).id));
    elseif(s.infotype==17)
        str=sprintf('(%d) %d',c.car(n).id,c.car(n).cause);
    elseif(s.infotype==18)  % ( 18 - ID/(Mode->D/W/M)/(causeID)/v(Km/h))
        str1=num2str(c.car(n).id);
        if(isnan(c.car(n).vel_prof))
            str2=sprintf('W');
        elseif(isempty(c.car(n).vel_prof))
            str2=sprintf('D');
        else
            str2=sprintf('M');
        end
        
        if(isnan(c.car(n).vel_prof2))
            str3=sprintf('/W ');
        elseif(isempty(c.car(n).vel_prof2))
            str3=sprintf('/D ');
        else
            str3=sprintf('/M ');
        end
        if(c.car(n).cause==0),str4='       '; else, str4=sprintf('%03d ',c.car(n).cause); end
        str5=sprintf('%2.0f Km/h',c.car(n).dyn(1)*3.6);
        str=[str1 ' ' str2 str3 str4 str5];
    end
    
    
    c.car(n).hinfo=text(c.car(n).pos.x+1,c.car(n).pos.y+1,str,'color','k','BackgroundColor','w','Fontsize',5);
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Print online time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = ptime(mode)
global s c
if(mode==1)
    delete(s.hott)
    s.hott=text(147,90,num2str(s.time),'color',s.otc,'BackgroundColor',s.obtc,'HorizontalAlignment','right');
else
    text(128,90,'Time ','color',s.otc,'BackgroundColor',s.obtc);
    s.hott=text(145,90,num2str(s.time),'color',s.otc,'BackgroundColor',s.obtc,'HorizontalAlignment','right');
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Print online information
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = pinfo(mode)
global s c
% dbstop at 275 in isr_tfs_print.m
if(mode==0)     % Initialize handlers
    s.hoinfo=[];
    fill([127 127 150 150],[93 65 65 93],'k')
    text(128,85,'Simulated ','color',s.otc,'BackgroundColor',s.obtc);
    text(128,80,'Waiting ','color',s.otc,'BackgroundColor',s.obtc);
else            % Refresh information
    delete(s.hoinfo)
    h1=text(147,85,num2str(size(c.listactive,2)),'color',s.otc,'BackgroundColor',s.obtc,'HorizontalAlignment','right');
    h2=text(147,80,num2str(sum(s.rl(2,:))),'color',s.otc,'BackgroundColor',s.obtc,'HorizontalAlignment','right');
    s.hoinfo=[h1 h2];
end

end
