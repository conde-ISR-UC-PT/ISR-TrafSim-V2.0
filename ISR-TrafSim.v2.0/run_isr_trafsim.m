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
% Simulation Run (main cycle)
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

function [] = run_isr_trafsim()
global s c d1 d2

% To relauch simulator
if(isfield(s, 'original'))
    s=s.original;
else
    s.original=s;
end

% Verify if is necessary clear files
if(size(dir('savedata\car'),1)>2)
    rmdir('savedata\car','s')
    mkdir('savedata\car')
end

% Initialization of the graphical environment
graphical_environment_isr_trafsim();

% Initializes places with oil (slip places)
slip_isr_trafsim();

% Generate trajectory points and print roads
printset_isr_trafsim();

% Initializes all the variables necessary for the cars
c=datacar_isr_trafsim('init');

% Initialization of the magnets
global mag_data
mag_data=magnets_isr_trafsim('init');    % Initialization of the graphical environment
graphical_environment_isr_trafsim();

% Group the indices cells per road for generate map accumulator
if(s.ocmap_accum_graph)
    grid_isr_trafsim('cells');
end

% Memory reservation for laser simulation
if(s.las)
    global las_data
    map2(s.dimx,s.dimy,las_data.res,[0 0 0 0 0]',las_data.rays,las_data.range,las_data.open);
end

% Load conf settings
s.ri(1:12,8)=s.in(1:12,8);

% Initialization of the road launch table
launch_isr_trafsim('init');

% Generate trajectory points and print roads
printset_isr_trafsim();

% Initialization of the magnets
mag_data=magnets_isr_trafsim('init');

% Initialization of stops
s.stoplist=[];
if(s.cross_mode==0 || s.cross_mode==1 || s.cross_mode==2)
    s.stoplist=[s.stoplist s.crosslist];
end
if(s.round_gest==1)
    s.stoplist=[s.stoplist s.roundlist];
end
s.lights=zeros(3,28);
for i=1:length(s.stoplist)
    s.lights(1,s.stoplist(1,i))=1;
end

% Initializes all the variables necessary for the communication ovr-the-air
if(s.otacommu==1 && s.otacommuemulation==1),script_init; ota_commu_isr_trafsim('add_node',90);end

if(s.mode==0), s.hbar=waitbar(0,'Running simulation, please wait...'); end
if(s.mode==2)
    openSocket();
end

% To add acumulator table at specific time
in_extra=[];
for i=1:size(s.in_extra,1)
    if(nnz(s.in_extra(i,:))==3)
        in_extra=[in_extra ; s.in_extra(i,:)];
    end
end
if(~isempty(in_extra))
    in_extra=sortrows(in_extra,1);
end


% To fill scenario with vehicles (in each road entrace)
% For each road
s.ri(1:12,8)=s.in(1:12,8);
if(s.full)
    for r=1:s.rn
        sx=s.ri(r,1);           % Start X point
        sy=s.ri(r,2);           % Start Y point
        ang=s.ri(r,7);          % Orientation angle
        entrance=s.ri(r,8);     % Start Y point
        d=0;
        if(entrance==1)     % if is a entrance road
            for n=1:s.full_cars;
                % Insert car in scenario
                px=sx+d*cos(ang); py=sy+d*sin(ang); % Place whre vehivle will be deployed
                [~,n]=launch_car_isr_tfs(r,0);     % Launch car
                c.car(n).pos.x=px;                  % Set car news position
                c.car(n).pos.y=py;                  % Set car news position
                c.car(n).pos.t=ang;                 % Set car news position
                [c.car(n).cornX, c.car(n).cornY]=pcorners_isr_trafsim(c.car(n).pos.t,c.car(n).length,c.car(n).width);
                c.car(n).dyn=[0 0];                 % Set car speed and
                
                % Refresh current setpoint
                dist=zeros(1,size(c.car(n).traj,2));
                for i=1:size(c.car(n).traj,2)
                    dist(1,i)=dist_isr_trafsim(c.car(n).pos.x,c.car(n).pos.y,c.car(n).traj(1,i),c.car(n).traj(2,i));
                end
                [~,col]=min(dist);
                c.car(n).setp=col+3;
                
                % Prepare next launch
                d=d+6;                              % To next launch
            end
        end
    end
end

if(s.mode==1)                       % Online
    p=length(c.listactive);
    if(p~=0)
        for j=1:p
            print_isr_trafsim('car',j)
        end
    end
end
drawnow
s.collisions=grid_isr_trafsim('update');            % Update occupation 'grid' map

% To save video file
if(s.make_video)
    writerObj = VideoWriter('savedata\isr_tfs','MPEG-4');
    set(writerObj,'FrameRate',100)
    open(writerObj)
end

% Init list to keep track of cars that already reserv
s.list_round_already_reserve=s.roundlist;
s.list_cross_already_reserve=s.crosslist;

% Run all necessary cycles
comptime=0;
for time=0:s.step_time:(s.duration-s.step_time)
    tic
    
    % To add acumulator table at specific time
    if(~isempty(in_extra))
        if(round2(time,0.01)==in_extra(1,1))
            road=in_extra(1,3);
            amount_cars=in_extra(1,2);
            col=find(s.rl(1,:)==road);
            s.rl(2,col)=s.rl(2,col)+amount_cars;
            in_extra(1,:)=[];
        end
    end
    
    if(s.mode==0)
        waitbar(s.time/s.duration,s.hbar)
    end
    
    % Refresh the stored time
    s.time=time;
    if(mod(time,1)==0)
        str=sprintf('Time= %d/%d (s) | Cars created= %d/%d | Cars active= %d | Computation time= %f (s)',s.time,s.duration,s.curNcars,s.ncars,size(c.listactive,2),comptime);
        disp(str)
        comptime=0;
    end
    
    % Plot online information on ISR-TFS windows
    if(s.mode==1)
        if(s.printtime==1)
            print_isr_trafsim('time',0)
        end
        if(s.printonlineinfo==1)
            print_isr_trafsim('info',0)
        end
    end
    
    % Checks the appearance of new cars
    launch_isr_trafsim('check')
    
    % Move all cars in simulator
    move_isr_trafsim();
    
    % Simulate laser for all cars
    if(s.las>0)
        las_isr_trafsim('func');
    end
    
    if(s.collision_detect)
        
        % Updates the occupation map
        s.collisions=grid_isr_trafsim('update');
        
        if(mod(round2(time,0.001),1/s.fres)==0)
            % Compute information to plot graph of occupation of roads versus time
            if(s.ocmap_accum_graph)
                grid_isr_trafsim('compute');
            end
            
            % Increment accumulation map
            if(s.ocmap_accum)
                s.occup_map_accu=s.occup_map_accu+double(logical(s.ocmap));
                aux=(double(s.ocmap)>0).*s.time;
                s.ocmap3=s.ocmap3+((s.ocmap3==0).*aux);
            end
            
            if(s.overtake==1)
                % Change lane
                overtake_isr_tfs();
            end
        end
        
        % Analyzes collisions and stop cars envolved
        ncolli=length(s.collisions);
        if(ncolli>0)
            for i=1:ncolli
                if(s.collisions(i)>=100)
                    aux=find(c.listactive==s.collisions(i));
                    c.car(aux).cmdacc=-999;
                end
            end
        end
    end
    
    % Delete cars from simulator and erase information and handlers are no longer needed
    delete_task_isr_trafsim();
    list_round_already_reserve_gest_isr_trafsim();  % To delete car from that list
    list_cross_already_reserve_gest_isr_trafsim();  % To delete car from that list
    
    % Control traffic lights from crossroads
    if(s.cross_mode==0 || s.cross_mode==1)
        trafficlights_cross_vision_isr_trafsim();
    elseif(s.cross_mode==2 )
        trafficlights_cross_isr_trafsim();
    end
    
    % Control traffic lights from roundabout
    if(s.round_gest==1)
        trafficlights_round_isr_trafsim();
    end
    
    % Analyzes the occupation map to make the list of cars nearby
    grid_isr_trafsim('func');
    
    % Generate the acelaration command for all cars
    s.list_ignore_round=[];
    s.list_ignore_cross=[];
    for n=1:length(c.listactive)
        if(s.round_gest==1 && s.otacommu==1 && c.car(n).commu_vel_prof==1 && ~isempty(c.car(n).vel_prof) ) % If car receive velocity profile
            
            % If car receive a NaN velocity profile -> Wait
            if( isnan(c.car(n).vel_prof) )
                c.car(n).cmdacc=-1;
                c.car(n).cause=0;
                % If exist a velocity profile to follow
            else
                if(c.car(n).vel_prof(1,c.car(n).vp)<s.time)
                    c.car(n).vp=c.car(n).vp+1;
                end
                
                % Decides to tell the driver if he should slow down or accelerate
                if( c.car(n).dyn(1) > c.car(n).vel_prof(2,c.car(n).vp) )
                    c.car(n).cmdacc=-1;
                elseif( c.car(n).dyn(1) < c.car(n).vel_prof(2,c.car(n).vp) )
                    c.car(n).cmdacc=1;
                end
                
                % Stops tell the driver if he should slow down or accelerate
                if(c.car(n).vp>=c.car(n).vpn )%|| c.car(n).traj(3,c.car(n).setp)==c.car(n).rar)
                    c.car(n).vel_prof=[];
                end
            end
        elseif(s.cross_mode==2 && s.otacommu==1 && c.car(n).commu_vel_prof2==1 && ~isempty(c.car(n).vel_prof2) ) % If car receive velocity profile
            
            % If car receive a NaN velocity profile -> Wait
            if( isnan(c.car(n).vel_prof2) )
                c.car(n).cmdacc=-1;
                c.car(n).cause=0;
                % If exist a velocity profile to follow
            else
                if(c.car(n).vel_prof2(1,c.car(n).vp2)<s.time)
                    c.car(n).vp2=c.car(n).vp2+1;
                end
                
                % Decides to tell the driver if he should slow down or accelerate
                if( c.car(n).dyn(1) > c.car(n).vel_prof2(2,c.car(n).vp2) )
                    c.car(n).cmdacc=-1;
                elseif( c.car(n).dyn(1) < c.car(n).vel_prof2(2,c.car(n).vp2) )
                    c.car(n).cmdacc=1;
                end
                
                % Stops tell the driver if he should slow down or accelerate
                if(c.car(n).vp2>=c.car(n).vpn2 )%|| c.car(n).traj(3,c.car(n).setp)==c.car(n).rar)
                    c.car(n).vel_prof2=[];
                end
            end
        else
            [c.car(n).cmdacc, c.car(n).cause]=traffic_gestion_isr_trafsim(n);
        end
        
        % to cars without V2I ignore other traffic inside roundabout
        if(c.car(n).road>=30 && c.car(n).road<40 && c.car(n).dri_ignore_trf_round==1)
            c.car(n).cmdacc=1;
            s.list_ignore_round(end+1)=c.car(n).id;
        end
        
        % to cars without V2I ignore other traffic inside crossroads
        if(c.car(n).road==40 && c.car(n).dri_ignore_trf_cross==1)
            c.car(n).cmdacc=1;
            s.list_ignore_cross(end+1)=c.car(n).id;
        end
    end
    
    % Send GPS information over OTA communication
    if(s.otacommu==1)
        
        % Refresh positions for OTA communications
        if(s.otacommuemulation==1),ota_commu_isr_trafsim('refresh');end % NAO ENTRA!!!
        
        % Broadcast GPS information
        if(mod(round2(time,0.001),1/s.otafgps)==0 && s.otagps==1)
            ota_commu_isr_trafsim([90 0 s.time],'gps_info');    % 90=SenderID (roundabout), 0=ReceiverID (Broadcast)
        end
        
        % Process events of OTA communications
        if(s.otacommuemulation)
            if(~isempty(Event_list))
                ota_commu_run(s.time+s.step_time, [log_file, num2str(Nnodes)])
            end
        end
    end
    
    % Refresh list of cars ear to the roundabout % crossroads
    round_proximity_isr_trafsim();
    cross_proximity_isr_trafsim();
    
    
    % Roundabout gestion module
    if(s.round_gest==1 && s.otacommu==1 && ~mod(round2(s.time,0.001),(1/s.f_traffic_gest)) )
        round_gest_isr_trafsim();
    end
    
    % CrossRoad gestion module
    if(s.cross_mode==2 && s.otacommu==1 && ~mod(round2(s.time,0.001),(1/s.f_traffic_gest)) )
        cross_gest_isr_trafsim();
    end
    
    % Free Slots on registation tables
    for col=1:size(s.rg_count_tab,2)    % For roundabout
        road=s.rg_count_tab(1,col);
        for lin=3:10
            if(s.rg_count_tab(lin,col)~=0)
                carID=s.rg_count_tab(lin,col);
                pos=find(c.listactive==carID);
                if(~isnan(carID))
                    if(isempty(pos))                                            % If car already exit from simulator
                        s.rg_count_tab(lin,col)=0;                                 % Erase car ID from table
                        s.rg_count_tab(2,col)=s.rg_count_tab(2,col)-1;             % Decrease somatory from table
                    elseif(c.car(pos).roadprev==road && c.car(pos).road~=road)  % If car leaves the road and exits another road before
                        s.rg_count_tab(lin,col)=0;                                 % Erase car ID from table
                        s.rg_count_tab(2,col)=s.rg_count_tab(2,col)-1;             % Decrease somatory from table
                    end
                end
            end
        end
    end
    
    for col=1:size(s.rg_count_tab2,2)    % for crossroads
        road=s.rg_count_tab2(1,col);
        for lin=3:10
            if(s.rg_count_tab2(lin,col)~=0)
                carID=s.rg_count_tab2(lin,col);
                pos=find(c.listactive==carID);
                if(~isnan(carID))
                    if(isempty(pos))                                            % If car already exit from simulator
                        s.rg_count_tab2(lin,col)=0;                                 % Erase car ID from table
                        s.rg_count_tab2(2,col)=s.rg_count_tab2(2,col)-1;             % Decrease somatory from table
                    elseif(c.car(pos).roadprev==road && c.car(pos).road~=road)  % If car leaves the road and exits another road before
                        s.rg_count_tab2(lin,col)=0;                                 % Erase car ID from table
                        s.rg_count_tab2(2,col)=s.rg_count_tab2(2,col)-1;             % Decrease somatory from table
                    end
                end
            end
        end
    end
    
    % Print all cars active
    if(s.mode==1)
        p=length(c.listactive);
        if(p~=0)
            for j=1:p
                print_isr_trafsim('car',j)
            end
        end
    elseif(s.mode==2)                   % C++
        tab=zeros(100,9);   % [On/off x y teta length width cR cG cB] max=100cars
        for j=1:length(c.listactive);
            tab(j,:)=[1 c.car(j).pos.x c.car(j).pos.y c.car(j).pos.t c.car(j).length c.car(j).width c.car(j).dri.color(1) c.car(j).dri.color(2) c.car(j).dri.color(3)];
        end
        %tab=single(tab);
        pkt=reshape(tab',1,900);
        fwrite(s.net,pkt,'double');
        a=whos('pkt');
        %a.bytes
        %keyboard
        %disp('#')
    end
    
    % Adjust the window to follow a particular car ID, if is necessary
    if(s.mode==1)
        if(s.followcar>0)
            aux=find(c.listactive==s.followcar);
            if(~isempty(aux))
                axis([c.car(aux).pos.x-(s.dimx/5) c.car(aux).pos.x+(s.dimx/5) c.car(aux).pos.y-(s.dimy/5) c.car(aux).pos.y+(s.dimy/5)]);
            else
                axis([0+5 s.dimx-5 0+5 s.dimy-5]);
            end
        else
            axis([0+5 s.dimx-5 0+5 s.dimy-5]);
        end
        % Draw all cars
        drawnow
        % Store frame to video
        if(s.make_video)
            frame=getframe;
            writeVideo(writerObj,frame);
        end
    end
    
    % Acumulate computation time
    comptime=comptime+toc;
    
    if(s.flag_view_3D~=0)
        if(~isempty(s.ocmap_3D))
            %keyboard
            view_3D_map(s.flag_view_3D)
            disp('Close figure window to continue')
            while(~isempty(findobj('type','figure','name','3D Reservation')))
                pause(1)
            end
            s.flag_view_3D=0;
        end
    end
end

% Close video
if(s.make_video)
    close(writerObj);
end

if(s.mode==0), close(s.hbar); end
if(s.mode==1), close(s.hfigsim); end
if(s.mode==2)
    tab=zeros(100,9);   % [On/off x y teta length width cR cG cB] max=100cars
    tab(1,1)=single(999);
    pkt=reshape(tab',1,900);
    fwrite(s.net,pkt,'double');
end

% save pre data
save savedata\predata.mat

% End of simulation
disp('End of simulation');
disp(' ');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Display cars stats in command line
if(s.disp_res)
    compute_stats_isr_trafsim();
    if(s.round_gest==0)
        str1='No Gestion';
    elseif(s.round_gest==1 && s.round_method==1)
        str1='RIMSw';
    elseif(s.round_gest==1 && s.round_method==2)
        str1='RIMSe';
    end
    if(s.cross_mode==0)
        str2='No Gestion';
    elseif(s.cross_mode==1)
        str2='CIVS';
    elseif(s.cross_mode==2 && s.cross_method==1)
        str2='CIMSw';
    elseif(s.cross_mode==2 && s.cross_method==2)
        str2='CIMSe';
    end
    disp(['Roundabout: ' str1 '   /    Crossroad: ' str2])
    disp(' ')
end

% Save data in directory 'save'
if(s.save_data==1)
    d = clock; yy=num2str(d(1),'%4.0f'); mm=num2str(d(2),'%02.0f'); dd=num2str(d(3),'%02.0f'); h=num2str(d(4),'%02.0f'); m=num2str(d(5),'%02.0f'); name=strcat(yy,mm,dd,h,m);
    cd savedata\
    save(name)
    cd ..
end

% Plot occupation map accumulator
if(s.ocmap_accum)
    s.hgraph2=figure(4);
    set(s.hgraph2,'NumberTitle','off','Name','Occupation Map','Position',[100 100 s.dimx*6 s.dimy*6],'Resize','off');
    auxmap=s.occup_map_accu;
    M=max(max(auxmap));
    for i=1:size(s.occup_map_accu,1)
        for j=1:size(s.occup_map_accu,2)
            if(auxmap(i,j)>0)
                if((auxmap(i,j)/M)>=0.15)
                    auxmap(i,j)=auxmap(i,j)/M;
                else
                    auxmap(i,j)=0.15;
                end
            end
        end
    end
    hold on;
    surf(auxmap,'EdgeColor','none');
    load isr_tfs_colormap
    set(s.hgraph2,'Colormap',mycmap);
    caxis([0 1]);
    axis('equal')
    t=colorbar;
    set(get(t,'ylabel'),'String', 'Occupation level');
    xlabel('X-Axis [m]')
    ylabel('Y-Axis [m]')
    saveas(s.hgraph2,'savedata\ocmap_accum.fig')
    % shading interp
end

% Plot occupation map accumulator over the time
if(s.ocmap_accum_graph)
    s.hgraph3=figure(5);
    set(s.hgraph3,'NumberTitle','off','Name','Occupation Map over the time','Position',[100 100 s.dimx*6 s.dimy*6],'Resize','off');
    hold on; axis([0 s.ocmap_time(1,end) 0 max(max(s.ocmap_time(2:end,:)))])
    for i=1:s.rn
        plot(s.ocmap_time(1,:),s.ocmap_time(i+1,:))
    end
end

% Display paths velocity
if(s.disp_res2)
    load isr_tfs_colormap
    [z]=size(s.ocmap4,3);
    for r=1:6
        if(r==1)
            load path1(2_20).mat
            aux=fliplr(path1);
            str='savedata\path1(2_20).fig';
        elseif(r==2)
            load path2(1_27).mat
            aux=fliplr(path2);
            str='savedata\path2(1_27).fig';
        elseif(r==3)
            load path3(5_25).mat
            aux=fliplr(path3);
            str='savedata\path3(5_25).fig';
        elseif(r==4)
            load path4(6_24).mat
            aux=fliplr(path4);
            str='savedata\path4(6_24).fig';
        elseif(r==5)
            load path5(1_25).mat
            aux=fliplr(path5);
            str='savedata\path5(1_25).fig';
        elseif(r==6)
            load path6(7_19).mat
            aux=fliplr(path6);
            str='savedata\path6(7_19).fig';
        end
        [n]=size(aux,2);
        vel=zeros(z,n);
        for i=1:z
            for j=1:n
                vel(i,j)=s.ocmap4(aux(1,j),aux(2,j),i);
                if(vel(i,j)>0 && vel(i,j)<2/3.6)
                    vel(i,j)=2/3.6;
                end
            end
        end
        h=figure(10+r);
        hold on;
        surf([1:size(vel,2)],[1:size(vel,1)]/s.fres2,vel*3.6)
        load isr_tfs_colormap2
        set(h,'Colormap',mycmap);
        caxis([0 60]);
        M=size(vel,2);
        m=size(vel,1);
        view(2)
        axis('square')
        axis([0 size(vel,2) 0 size(vel,1)/s.fres2]);
        t=colorbar;
        set(get(t,'ylabel'),'String', 'Velocity [Km/h]');
        shading interp
        xlabel('Cells')
        ylabel('Time [s]')
        saveas(h,str)
    end
end

end


