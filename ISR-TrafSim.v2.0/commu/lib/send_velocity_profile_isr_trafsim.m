function [res] = send_velocity_profile(n,aux,id,vel_prof,rafter,interssection_type)
global s c

    if(strcmp(interssection_type,'round'))
        c.car(n).vel_prof=vel_prof;
        c.car(n).vpn=size(vel_prof,2);
        ota_commu_isr_trafsim([90 id s.time],'vel_prof');   % Send velocity profile

        % Add car to list already reserved
        if(isempty(find(s.list_round_already_reserve==id)))
            [ll cc]=find(s.list_round_already_reserve(1,:)==s.round_notify_list(12,aux));
            s.list_round_already_reserve(end+1,cc)=id;
        end
        % Actualizes table that will count how many cars are sent to a particular output
        if( ~isnan(vel_prof) )
            col=find(s.rg_count_tab(1,:)==rafter);
            s.rg_count_tab(2,col)=s.rg_count_tab(2,col)+1;  %[road sumatory IDcar1 IDcar2 ...]
            for i=3:10
                if(s.rg_count_tab(i,col)==0)
                    s.rg_count_tab(i,col)=id;       % Write carID on table on first free space
                    break
                end
            end
        end
        % Delete notification
        s.round_notify_list(:,aux)=[];

    elseif(strcmp(interssection_type,'cross'))
        c.car(n).vel_prof2=vel_prof;
        c.car(n).vpn2=size(vel_prof,2);
        ota_commu_isr_trafsim([90 id s.time],'vel_prof2');   % Send velocity profile
        
        % Add car to list already reserved
        if(isempty(find(s.list_cross_already_reserve==id)))
            [ll cc]=find(s.list_cross_already_reserve(1,:)==s.cross_notify_list(12,aux));
            s.list_cross_already_reserve(end+1,cc)=id;
        end
        % Actualizes table that will count how many cars are sent to a particular output
        if( ~isnan(vel_prof) )
            col=find(s.rg_count_tab2(1,:)==rafter);
            s.rg_count_tab2(2,col)=s.rg_count_tab2(2,col)+1;  %[road sumatory IDcar1 IDcar2 ...]
            for i=3:10
                if(s.rg_count_tab2(i,col)==0)
                    s.rg_count_tab2(i,col)=id;       % Write carID on table on first free space
                    break
                end
            end
        end
        % Delete notification
        s.cross_notify_list(:,aux)=[];
    end
end

