% Author: Po-Heng Chen , Z5014392

% 26/03/2018   Wednesday    Week5

% Program: Solution for RD, S1.2018, Project02.PartC
%..........................................................................

function MyProgram()

clear ; clc() ; close all;
load('Speed_dataC.mat');
load('IMU_dataC.mat');
load('Laser__2C.mat');
yaw = PartA('IMU_dataC.mat');

figure(8) ; clf() ; hold on ; grid on ; zoom on ;
xlabel('X (m)'); ylabel('Y (m)');
MyHandle.handle4 = plot(0,0,'k.','MarkerSize',15);
axis([-3.5 1.5 -1 5]);
%.....................................................................
N = Vel.N;
speed = Vel.speeds;
% omega = IMU.DATAf(6,:);
times = double(Vel.times)/10000;
times = times - times(1);
kk = pi/180;
steering = (yaw) * kk;
%.....................................................................

Pose=zeros(3,N-1);

X0 = [0;0;pi/2] ; %Initial Condition (x, y, heading), in(meters, meters, radians)
X=X0;
for i = 1:1:N-1 %iterations / Loop
    X = PredictVehiclePose(X, steering(i), speed(i) ,times(i+1)-times(i));
    Pose(:,i)=X ;
    PlotPose(Pose,MyHandle,i);
        
    s=sprintf('(Dynamic plotting...)\nShowing scan #[%d]/[%d]\r',i,N);
    title(s);
    pause(0.00005) ;
%     if mod(i,100) == 0
%         PlotPose(Pose,MyHandle,i);
%         pause(0.01) ;                   % wait for ~10ms
%     end
end;

figure(1) ; clf() ; 
plot(Pose(1,:),Pose(2,:)) ; xlabel('X (m)'); ylabel('Y (m)'); title('Position');
grid on ;

return;
end

 function X = PredictVehiclePose(X0,steering,speed,dt)
    X=X0 ;
    dL = dt*speed ;
    X(3) = steering;
    X(1:2) = X0(1:2)+dL*[ cos(X0(3));sin(X0(3))] ;
return ;
 end
 
 function PlotPose(Pose,mh,i)
%    set(mh.handle4,'xdata',Pose(1,i) ,'ydata',Pose(2,i) );
    set(mh.handle4,'xdata',Pose(1,:) ,'ydata',Pose(2,:) );
 end
