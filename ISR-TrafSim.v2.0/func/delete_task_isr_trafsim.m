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
% Delete cars from simulator
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

function [] = delete_task_isr_trafsim()
global s c
% dbstop at 456 in isr_tfs_run

[~, p]=size(c.listactive);     n=0;
while(n~=p)
    n=n+1;
    if(c.car(n).setp>=c.car(n).trajn)
        
        % to delete car from list from cars without commu
        % because car can exit from simulator before longest time
        % calculated to exit (ex: car exit on roundabout first exit and
        % exits from simulator before wrost time calculated
        if(~isempty(s.car_round_without_commu))
            cond1=c.car(n).id==s.car_round_without_commu;
        else
            cond1=0;
        end
        
        if(~isempty(s.car_round_without_commu))
            cond2=c.car(n).id==s.car_cross_without_commu;
        else
            cond2=0;
        end
        
        if(cond1)
            disp('#####')%keyboard
            id=c.car(n).id;
            if(~isempty(find(s.round_notify_list==id)))
                pos=find(s.round_notify_list(1,:)==id);
                s.round_notify_list(:,pos)=[];
            end
            
            if(~isempty(find(s.list_round_already_reserve==id)))
                [ll, cc]=find(s.list_round_already_reserve==id);
                s.list_round_already_reserve(ll,:)=[];
            end
            
            for i=1:size(s.rg_count_tab,2)
                for pos=3:10
                    if(isnan(s.rg_count_tab(pos,i)))
                        s.rg_count_tab(pos,i)=0;
                        s.rg_count_tab(2,i)=s.rg_count_tab(2,i)-1;
                        break
                    end
                end
            end
            
            s.car_round_without_commu=[];
            s.time_round_without_commu=-1;
        end
        
        if(cond2)
            disp('#####')%keyboard
            id=c.car(n).id;
            if(~isempty(find(s.crodd_notify_list==id)))
                pos=find(s.cross_notify_list(1,:)==id);
                s.cross_notify_list(:,pos)=[];
            end
            
            if(~isempty(find(s.list_cross_already_reserve==id)))
                [ll, cc]=find(s.list_cross_already_reserve==id);
                s.list_cross_already_reserve(ll,:)=[];
            end
            
            for i=1:size(s.rg_count_tab2,2)
                for pos=3:10
                    if(isnan(s.rg_count_tab2(pos,i)))
                        s.rg_count_tab2(pos,i)=0;
                        s.rg_count_tab2(2,i)=s.rg_count_tab2(2,i)-1;
                        break
                    end
                end
            end
            
            s.car_cross_without_commu=[];
            s.time_cross_without_commu=-1;
        end
        
        % Increase cars that exit the simulator
        s.cars_exit=s.cars_exit+1;
        
        % Save individual car data
        if(s.save_car==1)
            aux=c.car(n);
            save(['savedata\car\car_' num2str(s.cars_exit)],'aux')
        end
        
        % Display results from fusion 3
        if(s.disp_fus3_res==1 && s.fus3>=1)
            %                 dbstop at 473 in isr_tfs_run
            figure
            orgllh = [s.lt*pi/180 s.lg*pi/180 0];
            for i = 1:size(c.car(n).est_lat,1)
                insxyz(i,:)     = llh2xyz([c.car(n).est_lat(i,1)  c.car(n).est_lon(i,1)  c.car(n).est_height(i,1) ]);
                insgpsxyz(i,:)  = llh2xyz([c.car(n).est_lat_(i,1) c.car(n).est_lon_(i,1) c.car(n).est_height_(i,1)]);
                insgpsKFxyz(i,:)= llh2xyz([c.car(n).est_lat_KF(1,i) c.car(n).est_lon_KF(1,i) c.car(n).est_height_KF(i,1)]);
                insenu(i,:)     = xyz2enu(insxyz(i,:),llh2xyz(orgllh));
                insgpsenu(i,:)  = xyz2enu(insgpsxyz(i,:),llh2xyz(orgllh));
                insgpsKFenu(i,:)= xyz2enu(insgpsKFxyz(i,:),llh2xyz(orgllh));
            end
            hold on
            plot3(c.car(n).TRUEpos(:,1),c.car(n).TRUEpos(:,2),zeros(size(c.car(n).TRUEpos(:,2)),1),'b')   % Real position
            plot3(insenu(:,1),insenu(:,2),insenu(:,3),'g')                       % INS position
            plot3(insgpsenu(:,1),insgpsenu(:,2),insgpsenu(:,3),'r')              % INS + GPS position
            plot3(insgpsKFenu(:,1),insgpsKFenu(:,2),insgpsKFenu(:,3),'k')        % INS + GPS + KF position
            plot3(c.car(n).gps_data(:,1),c.car(n).gps_data(:,2),zeros(size(c.car(n).gps_data(:,2),1),1),'m+')
            legend('Real','INS','INS+GPS+ERR','INS+GPS+KF','GPS')
            title('Car Positions'); xlabel('ENU X-axis (m)'); ylabel('ENU Y-axis (m)'); zlabel('ENU Z-axis (m)'); view(2)
            hold off
            axis('equal')
            isr_tfs_plot_KF_fb(n)
        end
        % Plot graphic with lateral errors
        if(s.lateral_errors)
            s.hgraph1=figure(3);
            if(c.car(n).type==1),str='car';else str='truck';end
            set(s.hgraph1,'NumberTitle','off','Name',['Position Errors of car ID ' num2str(c.car(n).id) ' (Type - ' str ' )'] ,'Position',[100 100 1000 500]);
            aux1=subplot(2,1,1);
            hold on
            plot([1:1:size(c.car(n).traj_err(1,:),2)]*(s.step_time),c.car(n).traj_err(1,:),'r')
            plot([1:1:size(c.car(n).traj_err(3,:),2)]*(s.step_time),c.car(n).traj_err(3,:),'g')
            xlabel('Time (s)'); ylabel('Distance Error (m)'); legend(aux1,'Mass center','Lock-ahead');
            aux2=subplot(2,1,2);
            hold on
            array1=c.car(n).traj_err(2,:)*(180/pi);
            array2=c.car(n).traj_err(4,:)*(180/pi);
            for i=1:size(array1,2)
                if(array1(1,i)>180)
                    array1(1,i)=array1(1,i)-360;
                elseif(array1(1,i)<-180)
                    array1(1,i)=array1(1,i)+360;
                end
                if(array2(1,i)>180)
                    array2(1,i)=array2(1,i)-360;
                elseif(array2(1,i)<-180)
                    array2(1,i)=array2(1,i)+360;
                end
            end
            plot([1:1:size(c.car(n).traj_err(2,:),2)]*(s.step_time),array1,'r')
            plot([1:1:size(c.car(n).traj_err(4,:),2)]*(s.step_time),array2,'g')
            xlabel('Time (s)'); ylabel('Angular Error (deg)'); legend(aux2,'Mass center','Lock-ahead');
            
        end
        
        if(s.disp_car_res)
            
            % Plot GPS and Magnets errors
            s.hgraph4=figure(6);
            if(c.car(n).type==1),str='car';else str='truck';end
            set(s.hgraph4,'NumberTitle','off','Name',['GPS and Permanent Magnets Localization Errors of car with ID ' num2str(c.car(n).id) ' (Type - ' str ' )'] ,'Position',[100 100 1000 500]);
            aux1=subplot(2,1,1);
            hold on
            plot(c.car(n).data(2,:),c.car(n).data(3,:),'r')
            plot(c.car(n).data(2,:),c.car(n).data(4,:),'g')
            xlabel('Time (s)'); ylabel('Distance Error (m)'); legend(aux1,'GPS error in X-Axis','GPS error in Y-Axis');
            aux2=subplot(2,1,2);
            hold on
            plot(c.car(n).data(2,:),c.car(n).data(5,:),'o-r')
            plot(c.car(n).data(2,:),c.car(n).data(6,:),'o-g')
            xlabel('Time (s)'); ylabel('Distance Error (m)'); legend(aux2,'Magnets error in X-Axis','Magnets error in Y-Axis');
            
            % Plot INS and Sensorial Fusion errors
            s.hgraph5=figure(7);
            if(c.car(n).type==1),str='car';else str='truck';end
            set(s.hgraph5,'NumberTitle','off','Name',['INS and Sensorial Fusion Localization Errors of car with ID ' num2str(c.car(n).id) ' (Type - ' str ' )'] ,'Position',[100 100 1000 500]);
            aux1=subplot(2,1,1);
            hold on
            plot(c.car(n).data(2,:),c.car(n).data(7,:),'r')
            plot(c.car(n).data(2,:),c.car(n).data(8,:),'g')
            plot(c.car(n).data(2,:),c.car(n).data(10,:),'b')
            plot(c.car(n).data(2,:),c.car(n).data(11,:),'c')
            xlabel('Time (s)'); ylabel('Distance Error (m)'); legend(aux1,'INS error in X-Axis','INS error in Y-Axis','Sensorial fusion error in X-Axis','Sensorial fusion error in Y-Axis');
            aux2=subplot(2,1,2);
            hold on
            
            array1=c.car(n).data(9,:)*(180/pi);
            array2=c.car(n).data(12,:)*(180/pi);
            for i=1:size(array1,2)
                if(array1(1,i)>180)
                    array1(1,i)=array1(1,i)-360;
                elseif(array1(1,i)<-180)
                    array1(1,i)=array1(1,i)+360;
                end
                if(array2(1,i)>180)
                    array2(1,i)=array2(1,i)-360;
                elseif(array2(1,i)<-180)
                    array2(1,i)=array2(1,i)+360;
                end
            end
            plot(c.car(n).data(2,:),array1,'r')
            plot(c.car(n).data(2,:),array2,'g')
            xlabel('Time (s)'); ylabel('Orientation Error (deg)'); legend(aux2,'INS orientation error','Sensorial Fusion orientation error');
            
            % Plot odometry errors
            s.hgraph6=figure(8);
            if(c.car(n).type==1),str='car';else str='truck';end
            set(s.hgraph6,'NumberTitle','off','Name',['Odometry Localization Errors of car with ID ' num2str(c.car(n).id) ' (Type - ' str ' )'] ,'Position',[100 100 1000 500]);
            aux1=subplot(2,1,1);
            hold on
            plot(c.car(n).data(2,:),c.car(n).data(13,:),'r')
            plot(c.car(n).data(2,:),c.car(n).data(14,:),'g')
            xlabel('Time (s)'); ylabel('Distance Error (m)'); legend(aux1,'Odometry error in X-Axis','Odometry error in Y-Axis');
            aux2=subplot(2,1,2);
            hold on
            array1=c.car(n).data(15,:)*(180/pi);
            for i=1:size(array1,2)
                if(array1(1,i)>180)
                    array1(1,i)=array1(1,i)-360;
                elseif(array1(1,i)<-180)
                    array1(1,i)=array1(1,i)+360;
                end
            end
            plot(c.car(n).data(2,:),array1,'r')
            xlabel('Time (s)'); ylabel('Orientation Error (deg)'); legend(aux2,'Odometry orientation error');
            
            % Plot Euclidean distance errors
            s.hgraph7=figure(9);
            if(c.car(n).type==1),str='car';else str='truck';end
            set(s.hgraph7,'NumberTitle','off','Name',['Euclidean Distance Localization Errors of car with ID ' num2str(c.car(n).id) ' (Type - ' str ' )'] ,'Position',[100 100 1000 500]);
            aux1=subplot(2,1,1); hold on
            plot(c.car(n).data(2,:),sqrt(c.car(n).data(3,:).^2+c.car(n).data(4,:).^2),'-r','LineWidth',2)
            plot(c.car(n).data(2,:),sqrt(c.car(n).data(7,:).^2+c.car(n).data(8,:).^2),'-g','LineWidth',2)
            plot(c.car(n).data(2,:),sqrt(c.car(n).data(10,:).^2+c.car(n).data(10,:).^2),'-y','LineWidth',2)
            plot(c.car(n).data(2,:),sqrt(c.car(n).data(13,:).^2+c.car(n).data(14,:).^2),'-c','LineWidth',2)
            plot(c.car(n).data(2,:),sqrt(c.car(n).data(5,:).^2+c.car(n).data(6,:).^2),'ob','LineWidth',2)
            xlabel('Time (s)'); ylabel('Distance Error (m)'); legend(aux1,'GPS error','INS error','Sensorial fusion error','Odometry error','Magnets error');
            aux2=subplot(2,1,2); hold on
            plot(c.car(n).data(2,:),c.car(n).data(16,:),'-g','LineWidth',2)
            xlabel('Time (s)'); ylabel('Distance (m)'); legend(aux2,'LIDAR minimum detection distance');
            
            
            % [time timeinsim gpsEX gpsEY magEX magEY insEX insEY insET fusEX fusEY fusET odoEX odoEY odoET minLaser velLinear road]
        end
        
        % To plot satellites position
        if(s.printsateliteposition==1)
            %                 dbstop at 600 in isr_tfs_run
            haux1=figure('Name',['Satellites positions for car ' num2str(c.car(n).id)]);
            hold on
            for i=1:round(size(c.car(n).gps_sat_pos,3)/10):size(c.car(n).gps_sat_pos,3)
                skyplot(c.car(n).gps_sat_pos(:,:,i),c.car(n).gps_sat_num(:,:,i),c.car(n).gps_usrxyz(:,:,i),0,1)
            end
            pause
            close(haux1);
        end
        
        % To plot trajectory and estimated trajectory using odometry, KF and laser (and/or magnets)
        if(s.disp_fus2_res==1 && s.fus2>=1)
            haux7=figure('Name',['Errors for car ' num2str(c.car(n).id)]);
            haux8=figure('Name',['Errors for car ' num2str(c.car(n).id)]);
            %                 dbstop at 641 in isr_tfs_run
            %                 dbstop at 647 in isr_tfs_run
            SerrorNoise=sqrt( (c.car(n).posReal(:,1)-c.car(n).posNoise(:,1)).^2 +(c.car(n).posReal(:,2)-c.car(n).posNoise(:,2)).^2 );
            WerrorNoise=dist_isr_trafsim_ang(c.car(n).posReal(:,3),c.car(n).posNoise(:,3));
            
            SerrorCt=sqrt( (c.car(n).posReal(:,1)-c.car(n).posCt(:,1)).^2 +(c.car(n).posReal(:,2)-c.car(n).posCt(:,2)).^2 );
            WerrorCt=dist_isr_trafsim_ang(c.car(n).posReal(:,3),c.car(n).posCt(:,3));
            
            SerrorMag=sqrt( (c.car(n).posReal(:,1)-c.car(n).posMag(:,1)).^2 +(c.car(n).posReal(:,2)-c.car(n).posMag(:,2)).^2 );
            WerrorMag=dist_isr_trafsim_ang(c.car(n).posReal(:,3),c.car(n).posMag(:,3));
            
            aux1=size(c.car(n).posReal,1);
            aux2=size(c.car(n).fus2_pos,1);
            if(aux1~=aux2)
                c.car(n).fus2_pos=c.car(n).fus2_pos(1:aux1,:);
            end
            Serrorfus2=sqrt( (c.car(n).posReal(:,1)-c.car(n).fus2_pos(:,1)).^2 +(c.car(n).posReal(:,2)-c.car(n).fus2_pos(:,2)).^2 );
            Werrorfus2=dist_isr_trafsim_ang(c.car(n).posReal(:,3),c.car(n).fus2_pos(:,3));
            
            figure(haux7); hold on;
            time=[0:size(c.car(n).deltaS_Noise,2)-1]/100;
            plot(time,SerrorNoise,'g',time,SerrorCt,'b',time,SerrorMag,'r',time,Serrorfus2,'m')
            
            mag3=[]; mag4=[];
            if(~isnan(c.car(n).timePos_MAG_aux4) )      % When detect laser + his own Mag
                for i=1:size(c.car(n).timePos_MAG_aux4,2)
                    index=round(c.car(n).timePos_MAG_aux4(i)/s.step_time);
                    mag3(end+1)=time(index); mag4(end+1)=Serrorfus2(index) ;
                end
                plot(mag3,mag4,'or')
            end
            
            mag1=[]; mag2=[];
            if(~isnan(c.car(n).fus2_mag_xpos) )         % When detect laser + Mag
                for i=1:size(c.car(n).fus2_mag_tpos,2)
                    index=round(c.car(n).fus2_mag_tpos(i)/s.step_time);
                    mag1(end+1)=time(index); mag2(end+1)=Serrorfus2(index);
                end
                plot(mag1,mag2,'ok')
            end
            
            las1=[]; las2=[];
            if(~isnan(c.car(n).fus2_las_xpos) )         %  When detect laser + RTK
                for i=1:size(c.car(n).fus2_las_tpos,2)
                    index=round(c.car(n).fus2_las_tpos(i)/s.step_time);
                    las1(end+1)=time(index); las2(end+1)=Serrorfus2(index);
                end
                plot(las1,las2,'+k')
            end
            grid; ylabel('Distance error [m]'); xlabel('Time [m]'); legend('Noise','KF-CT','Magnets','Laser','Self Mag','Mag','GPS','Location','NorthWest')
            
            
            
            
            
            figure(haux8); hold on;
            plot(time,WerrorNoise,'g',time,WerrorCt,'b',time,WerrorMag,'r',time,Werrorfus2,'m')
            
            mag3=[]; mag4=[];
            if(~isnan(c.car(n).timePos_MAG_aux4) )      % When detect laser + his own Mag
                for i=1:size(c.car(n).timePos_MAG_aux4,2)
                    index=round(c.car(n).timePos_MAG_aux4(i)/s.step_time);
                    mag3(end+1)=time(index); mag4(end+1)=Werrorfus2(index) ;
                end
                plot(mag3,mag4,'or')
            end
            
            mag1=[]; mag2=[];
            if(~isnan(c.car(n).fus2_mag_xpos) )         % When detect laser + Mag
                for i=1:size(c.car(n).fus2_mag_tpos,2)
                    index=round(c.car(n).fus2_mag_tpos(i)/s.step_time);
                    mag1(end+1)=time(index); mag2(end+1)=Werrorfus2(index);
                end
                plot(mag1,mag2,'ok')
            end
            
            las1=[]; las2=[];
            if(~isnan(c.car(n).fus2_las_xpos) )         %  When detect laser + RTK
                for i=1:size(c.car(n).fus2_las_tpos,2)
                    index=round(c.car(n).fus2_las_tpos(i)/s.step_time);
                    las1(end+1)=time(index); las2(end+1)=Werrorfus2(index);
                end
                plot(las1,las2,'+k')
            end
            grid; ylabel('Orientation error [rad]'); xlabel('Time [m]'); legend('Noise','KF-CT','Magnets','Laser','Self Mag','Mag','GPS','Location','NorthWest')
            
            
            pause
            close(haux7,haux8)
        end
        
        % To plot trajectory and estimated trajectory using odometry and magnets
        if(s.disp_fus4_res==1 && s.fus4>=1)
            %                 dbstop at 680 in isr_tfs_run
            haux1=figure('Name',['Delta S for car ' num2str(c.car(n).id)]);
            hold on
            plot([0:size(c.car(n).deltaS_Noise,2)-1]/100,c.car(n).deltaS_Noise,'g')
            plot([0:size(c.car(n).deltaS_Noise,2)-1]/100,c.car(n).deltaS_Real,'k')
            plot([0:size(c.car(n).deltaS_Noise,2)-1]/100,c.car(n).deltaS_KF_CT,'r')
            %                 title('Delta S')
            legend('Noise','Real','KF-CT')
            xlabel('Time [s]')
            ylabel('\DeltaS [m]')
            grid
            
            haux2=figure('Name',['Delta W for car ' num2str(c.car(n).id)]);
            hold on
            plot([0:size(c.car(n).deltaS_Noise,2)-1]/100,c.car(n).deltaW_Noise,'g')
            plot([0:size(c.car(n).deltaS_Noise,2)-1]/100,c.car(n).deltaW_Real,'k')
            plot([0:size(c.car(n).deltaS_Noise,2)-1]/100,c.car(n).deltaW_KF_CT,'r')
            %                 title('Delta W')
            legend('Noise','Real','KF-CT')
            xlabel('Time [s]')
            ylabel('\DeltaW [rad]')
            grid
            
            haux3=figure('Name',['Trajectory for car ' num2str(c.car(n).id)]);
            hold on
            if(c.car(n).mag_rule==1)
                plot(c.car(n).posMag(:,1),c.car(n).posMag(:,2),'r.','LineWidth',2)
                plot(c.car(n).xPos_MAG_aux4,c.car(n).yPos_MAG_aux4,'*k')
            end
            plot(c.car(n).posReal(:,1),c.car(n).posReal(:,2),'k')
            plot(c.car(n).posNoise(:,1),c.car(n).posNoise(:,2),'g')
            plot(c.car(n).posCt(:,1),c.car(n).posCt(:,2),'b')
            axis('equal')
            %                 title('Trajectory')
            if(c.car(n).mag_rule==1)
                legend('KF-CT + Magnets','Magnets','Real','Noise','KF-CT','Location','NorthWest')
            else
                legend('Real','Noise','KF-CT','Location','NorthWest')
            end
            xlabel('X [m]'); ylabel('Y [m]')
            grid
            
            haux4=figure('Name',['Trajectory for car ' num2str(c.car(n).id)]);
            hold on
            if(c.car(n).mag_rule==1)
                plot(c.car(n).posMag(:,1),c.car(n).posMag(:,2),'--r','LineWidth',2)
                plot(c.car(n).xPos_MAG_aux4,c.car(n).yPos_MAG_aux4,'*k')
            end
            plot(c.car(n).posReal(:,1),c.car(n).posReal(:,2),'k')
            plot(c.car(n).posNoise(:,1),c.car(n).posNoise(:,2),'g')
            plot(c.car(n).posCt(:,1),c.car(n).posCt(:,2),'b')
            axis('equal')
            %                 title('Trajectory')
            if(c.car(n).mag_rule==1)
                legend('KF-CT + Magnets','Magnets','Real','Noise','KF-CT','Location','NorthWest')
            else
                legend('Real','Noise','KF-CT','Location','NorthWest')
            end
            xlabel('X [m]'); ylabel('Y [m]')
            grid
            
            haux5=figure('Name',['Errors for car ' num2str(c.car(n).id)]);
            haux6=figure('Name',['Errors for car ' num2str(c.car(n).id)]);
            hold on
            if(c.car(n).mag_rule==1)
                SerrorNoise=sqrt( (c.car(n).posReal(:,1)-c.car(n).posNoise(:,1)).^2 +(c.car(n).posReal(:,2)-c.car(n).posNoise(:,2)).^2 );
                WerrorNoise=dist_isr_trafsim_ang(c.car(n).posReal(:,3),c.car(n).posNoise(:,3));
                SerrorCt=sqrt( (c.car(n).posReal(:,1)-c.car(n).posCt(:,1)).^2 +(c.car(n).posReal(:,2)-c.car(n).posCt(:,2)).^2 );
                WerrorCt=dist_isr_trafsim_ang(c.car(n).posReal(:,3),c.car(n).posCt(:,3));
                SerrorMag=sqrt( (c.car(n).posReal(:,1)-c.car(n).posMag(:,1)).^2 +(c.car(n).posReal(:,2)-c.car(n).posMag(:,2)).^2 );
                WerrorMag=dist_isr_trafsim_ang(c.car(n).posReal(:,3),c.car(n).posMag(:,3));
                figure(haux5)
                plot([0:size(c.car(n).deltaS_Noise,2)-1]/100,SerrorNoise,'g',[0:size(c.car(n).deltaS_Noise,2)-1]/100,SerrorCt,'b',[0:size(c.car(n).deltaS_Noise,2)-1]/100,SerrorMag,'r')
                grid; ylabel('\DeltaS [m]'); xlabel('Time [m]'); legend('Noise','KF-CT','Magnets','Location','NorthWest')
                figure(haux6)
                plot([0:size(c.car(n).deltaS_Noise,2)-1]/100,WerrorNoise,'g',[0:size(c.car(n).deltaS_Noise,2)-1]/100,WerrorCt,'b',[0:size(c.car(n).deltaS_Noise,2)-1]/100,WerrorMag,'r')
                axis('equal'); grid; ylabel('\DeltaW [rad]'); xlabel('Time [m]'); legend('Noise','KF-CT','Magnets','Location','NorthWest')
                
            else
                SerrorNoise=sqrt( (c.car(n).posReal(:,1)-c.car(n).posNoise(:,1)).^2 +(c.car(n).posReal(:,2)-c.car(n).posNoise(:,2)).^2 );
                WerrorNoise=dist_isr_trafsim_ang(c.car(n).posReal(:,3),c.car(n).posNoise(:,3));
                SerrorCt=sqrt( (c.car(n).posReal(:,1)-c.car(n).posCt(:,1)).^2 +(c.car(n).posReal(:,2)-c.car(n).posCt(:,2)).^2 );
                WerrorCt=dist_isr_trafsim_ang(c.car(n).posReal(:,3),c.car(n).posCt(:,3));
                figure(haux5)
                plot([0:size(c.car(n).deltaS_Noise,2)-1]/100,SerrorNoise,'g',[0:size(c.car(n).deltaS_Noise,2)-1]/100,SerrorCt,'b')
                grid; ylabel('\DeltaS [m]'); xlabel('Time [m]'); legend('Noise','KF-CT','Location','NorthWest')
                figure(haux6)
                plot([0:size(c.car(n).deltaS_Noise,2)-1]/100,WerrorNoise,'g',[0:size(c.car(n).deltaS_Noise,2)-1]/100,WerrorCt,'b')
                axis('equal'); grid; ylabel('\DeltaW [rad]'); xlabel('Time [m]'); legend('Noise','KF-CT','Location','NorthWest')
                
            end
            pause
            close(haux5,haux6)
            pause
            close(haux1,haux2,haux3,haux4)
            figure(s.hfigsim)
        end
        
        if(s.lateral_errors || s.disp_car_res)
            if(s.lateral_errors)
                delete(s.hgraph1)
            end
            if(s.disp_car_res)
                delete(s.hgraph7,s.hgraph6,s.hgraph5,s.hgraph4)
            end
        end
        
        % Calculate car stats
        if(s.disp_res)
            cont=0; aux=[];
            s.cars_time_insim=[s.cars_time_insim (c.car(n).timeinsim/c.car(n).traj(4,end)) ];
            for i=1:size(c.car(n).data,2)
                if(c.car(n).data(17,i)==0)
                    cont=cont+1;
                else
                    aux=[aux c.car(n).data(17,i)];
                end
            end
            s.cars_time_stop=[s.cars_time_stop cont*(1/s.fres)];
            s.cars_time_stop_perc=[s.cars_time_stop_perc ((cont*(1/s.fres))/c.car(n).timeinsim)*100 ];
            s.cars_time_run=[s.cars_time_run (c.car(n).timeinsim-cont*(1/s.fres))];
            s.cars_time_run_perc=[s.cars_time_run_perc ((c.car(n).timeinsim-cont*(1/s.fres))/c.car(n).timeinsim)*100 ];
            s.cars_mean_vel=[s.cars_mean_vel (sum(c.car(n).data(17,:))/size(c.car(n).data(17,:),2)) ];
            s.cars_mean_run_vel=[s.cars_mean_run_vel (sum(aux)/size(aux,2))];
        end
        
        % Delete handlers if simulator is in on-line mode
        if(s.mode==1)
            delete(c.car(n).hcar)
            if(c.car(n).hinfo~=-1) delete(c.car(n).hinfo); end
            if(c.car(n).hgpspos~=-1) delete(c.car(n).hgpspos); end
            if(c.car(n).hinspos~=-1) delete(c.car(n).hinspos); end
            if(c.car(n).hfuspos~=-1) delete(c.car(n).hfuspos); end
            if(c.car(n).hsetp~=-1) delete(c.car(n).hsetp); end
            if(c.car(n).hcarsec~=-1) delete(c.car(n).hcarsec); end
            if(c.car(n).hcarsecl~=-1) delete(c.car(n).hcarsecl); end
        end
        
        % Delete car from list of communication nodes
        if(s.otacommu==1 && s.otacommuemulation==1)
            isr_tfs_ota_commu('del_node',c.car(n).id);
        end
        
        id=c.car(n).id; % disp(['Delete car ' num2str(c.car(n).id)])
        % Find car Id on the lists
        cond1=ismember(id,s.cross_notify_list);
        if(cond1)
            keyboard
            [~, cc]=find(s.cross_notify_list(1,:)==id)
            s.cross_notify_list(:,cc)=[];
            cond1=0;
        end
        cond2=ismember(id,s.list_cross_already_reserve);
        if(cond2)
            [ll, cc]=find(s.list_cross_already_reserve==id);
            s.list_cross_already_reserve(ll,:)=[];
            cond2=0;
        end
        cond3=ismember(id,s.list_cross_without_commu);
        if(cond3)
            s.list_cross_without_commu=[];
            cond3=0;
        end
        
        cond5=ismember(id,s.round_notify_list);
        if(cond5)
            [~, cc]=find(s.round_notify_list(1,:)==id)
            s.round_notify_list(:,cc)=[];
            cond5=0;
        end
        cond6=ismember(id,s.list_round_already_reserve);
        if(cond6)
            [ll, cc]=find(s.list_round_already_reserve==id);
            s.list_round_already_reserve(ll,:)=[];
            cond6=0;
        end
        cond7=ismember(id,s.list_round_without_commu);
        if(cond7)
            s.list_round_without_commu=[];
            cond7=0;
        end
        
        if(cond1 || cond2 || cond3 || cond5 || cond6 || cond7)
            disp('Element stays in table')
            keyboard
        end
        
        % Delete car of car's struct
        datacar_isr_trafsim('delc',c.car(n).id);
        p=p-1; n=n-1;
    end
end
end

