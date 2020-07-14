clear
close all

bag = rosbag('velocidad3.bag');

%%Velocidades lineales
% odom = select(bag,'Topic','/diff_drive_controller/odom'); %Odometry
% cmd_vel = select(bag,'Topic','/cmd_vel'); %Twist

%%Velocidades angulares
% cmd_wheel= select(bag,'Topic','/cmd_wheel'); %JointState
wheel_state= select(bag,'Topic','/wheel_state'); %JointState


%%%%%Estraer información de los topic tipo JointState%%%%%

%%Crear estructuras para las velocidades angulares
% cmd_wheel_struct = readMessages(cmd_wheel);
wheel_state_struct=readMessages(wheel_state);

%%Wheel_state%%

%wheel_state_data=zeros(length(wheel_state_struct)/2,6);
k=1;
x=1;

%%Indice de los datos
%LEFT time :1
%LEFT data: 2
%RIGHT time :3
%RIGHT data: 4
%LEFT seq: 5
%RIGHT seq: 6

for i=1:length(wheel_state_struct)
    
    if(strcmp(wheel_state_struct{i,1}.Name{1,1},'LEFT'))
        wheel_state_data(k,1)=wheel_state_struct{i,1}.Header.Stamp.Sec + wheel_state_struct{i,1}.Header.Stamp.Nsec*10^-9;
        wheel_state_data(k,2)=wheel_state_struct{i,1}.Position;
        wheel_state_data(k,5)=wheel_state_struct{i,1}.Header.Seq;
        k=k+1;
        
    elseif (strcmp(wheel_state_struct{i,1}.Name{1,1},'RIGHT'))
                  
        wheel_state_data(x,3)=wheel_state_struct{i,1}.Header.Stamp.Sec + wheel_state_struct{i,1}.Header.Stamp.Nsec*10^-9;
        wheel_state_data(x,4)=wheel_state_struct{i,1}.Position;
        wheel_state_data(x,6)=wheel_state_struct{i,1}.Header.Seq;
        x=x+1;
        
    end
    
end

%Elimina el ultimo elemento, produce errores con datos vacios
wheel_state_data=wheel_state_data(1:end-1,:);


%%cmd_wheel%%

%%Indice de los datos
%time :1
%LEFT data: 2
%RIGHT data: 3

% cmd_wheel_data=zeros(length(cmd_wheel_struct),3);
% 
% for i=1:length(cmd_wheel_struct)
%     cmd_wheel_data(i,2:3)=cmd_wheel_struct{i,1}.Velocity;
%     cmd_wheel_data(i,1)=cmd_wheel_struct{i,1}.Header.Stamp.Sec+cmd_wheel_struct{i,1}.Header.Stamp.Nsec*10^-9;
%     
% end


%%Mostrar información de la velocidad angular de las ruedas
fig = figure;
plot (wheel_state_data(:,1),wheel_state_data(:,2));
hold
plot (wheel_state_data(:,3),wheel_state_data(:,4));
% plot (cmd_wheel_data(:,1),cmd_wheel_data(:,2));
legend('Izquierda','Derecha')
% legend('cmd wheel')
title('Velocidades angulares')
xlabel('tiempo(ns)')
ylabel('velocidad angular (rad/s)')
hold off


% fig = figure;
% plot (wheel_state_data(:,3),wheel_state_data(:,4));
% hold
% plot (cmd_wheel_data(:,1),cmd_wheel_data(:,3));
% legend('wheel state')
% legend('cmd wheel')
% title('Rueda derecha')
% xlabel('tiempo(s)')
% ylabel('velocidad angular (rad/s)')


%%%%%%%%Analizar las estructuras de velocidad y posición lineales%%%%%%%%

% %Posicion
% odom_x = timeseries(odom,'Pose.Pose.Position.X');
% odom_y =  timeseries(odom,'Pose.Pose.Position.Y');
% 
% %Velocidad
% odom_vx = timeseries(odom,'Twist.Twist.Linear.X');
% odom_vz = timeseries(odom,'Twist.Twist.Angular.Z');
% 
% cmd_vx = timeseries(cmd_vel,'Linear.X');
% cmd_vz = timeseries(cmd_vel,'Angular.Z');
% 
% figure
% plot(odom_vx.Time,odom_vx.data)
% hold
% plot(cmd_vx.Time,cmd_vx.data)
% legend('Odometría','Consigna')
% title('Vx')
% xlabel('tiempo(s)')
% ylabel('velocidad lineal (m/s)')
% 
% 
% figure
% plot(odom_vz.Time,odom_vz.data)
% hold
% plot(cmd_vz.Time,cmd_vz.data)
% legend('Odometría','Consigna')
% title('Velocidad de giro')
% xlabel('tiempo(s)')
% ylabel('velocidad giro (rad/s)')
% 
% 
% figure
% plot(odom_x.data,odom_y.data)
% title('PATH')
% xlabel('x(m)')
% ylabel('y(m)')
