%Author: Po-Heng Chen , Z5014392

%Program: Solution for RD, S1.2018, Project02.PartB

%..........................................................................
function main()

MyFile = 'Speed_dataC';
load(MyFile);

kk = pi/180;
yaw = PartA('IMU_dataC.mat');
%.....................................
speed = Vel.speeds;
N     = Vel.N ;

steering = (yaw) * kk;

times = double(Vel.times)/10000;
times = times - times(1);
%.....................................

Pose=zeros(3,N-1); % use N-1 instead of N will prevent starting point link to end point

X0 = [0;0;pi/2] ; %Initial Condition (x, y, heading), in(meters, meters, radians)
X=X0;
for i = 1:N-1 %iterations / Loop
    X = PredictVehiclePose(X, steering(i), speed(i) ,times(i+1)-times(i));
    Pose(:,i)=X ;
end;

% Show the results.
figure(1) ; clf() ; grid on ;
plot(Pose(1,:),Pose(2,:)) ; xlabel('X (m)'); ylabel('Y (m)'); title('Position');

%figure(2) ; clf ; plot(Pose(3,:)) ; %heading

return
end
%......................................................................%

function X = PredictVehiclePose(X0,steering,speed,dt)
    % Remember: state vector X = [x; y; heading]
    LCar = 0.46 ; %in this example L = 0.46meters
    X=X0 ;
    dL = dt*speed ;
    %X(3) = X0(3)+ tan(steering)*dL/LCar ;
    %X(3) = X0(3) + dt*IMU.DATAf(6,i);
    X(3) = steering;
    X(1:2) = X0(1:2)+dL*[ cos(X0(3));sin(X0(3))] ;
return ;
end
%.......................................................................