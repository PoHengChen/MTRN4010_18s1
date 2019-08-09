% Author: Po-Heng Chen , Z5014392
% 26/03/2018   Wednesday    Week5
% Program: Solution for RD, S1.2018, Project02.PartC
%..........................................................................

function MyProgram()
    clear ; clc() ; close all;
    load('Speed_dataC.mat');
    load('IMU_dataC.mat');
    load('Laser__2C.mat');
    %.....................................................................
    N = Vel.N;
    speed = Vel.speeds;
    times = double(Vel.times)/10000;
    times = times - times(1);
    %.....................................................................
    ii = times < 20 ;
    bias = mean(IMU.DATAf(6,ii)) ;
    yaw_rate = IMU.DATAf(6,:) - bias ;
    %.....................................................................
%     figure(8) ; clf() ; hold on ; grid on ; zoom on ;
%     xlabel('X (m)') ; ylabel('Y (m)') ; axis([-3.5 1.5 -1 5]) ;
    %MyHandle.handle4 = plot(0,0,'k.','MarkerSize',15);
    Pose=zeros(3,N-1);
    flag1 = 0;
    %..................................
    X0 = [0;0;pi/2] ; %Initial Condition (x, y, heading), in(meters, meters, radians)
    X=X0;
    for i = 1:1:N-1 %iterations / Loop
        X = PredictVehiclePose(X, yaw_rate(i), speed(i) ,times(i+1)-times(i));
        Pose(:,i)=X ;
        scale = 0.2;
        x = [Pose(1,i) Pose(1,i) + scale * cos(Pose(3,i))];
        y = [Pose(2,i) Pose(2,i) + scale * sin(Pose(3,i))];
        %PlotPose(Pose,MyHandle,i);
        if flag1
            set(MyHandle.handle4,'xdata',x ,'ydata',y);
        else
            flag1 = 1;
            figure(8) ; clf() ; hold on ; grid on ; zoom on ;
            xlabel('X (m)') ; ylabel('Y (m)') ; axis([-3.5 1.5 -1 5]) ;
            MyHandle.handle4 = line('xdata',x,'ydata',y,'Color','b','LineWidth',2);
            %MyHandle.handle4 = quiver(0,0,cos(Pose(3,i)),sin(Pose(3,i)), 0.5 ,'LineWidth', 2,'Color','r');
        end
        %try not to use quiver
        %if Vel.times(i) > dataL.times ; end

        s=sprintf('(Dynamic plotting...)\nShowing scan #[%d]/[%d]\r',i,N);
        title(s);
        pause(0.000000001) ;
    end;
return;
end

 function X = PredictVehiclePose(X0,yaw_rate,speed,dt)
    X=X0 ;
    dL = dt*speed ;
    X(3) = X0(3) + dt * (-yaw_rate); % yawA = -yawB
    X(1:2) = X0(1:2)+dL*[ cos(X0(3));sin(X0(3))] ;
return ;
 end
 
 function PlotPose(Pose,mh,i)
    set(mh.handle4,'xdata',Pose(1,i) ,'ydata',Pose(2,i)); 
%     set(mh.handle4,'xdata',Pose(1,i) ,'ydata',Pose(2,i) );
%     set(mh.handle4,'xdata',Pose(1,:) ,'ydata',Pose(2,:) );
 end
