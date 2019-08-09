% Author: Po-Heng Chen , Z5014392

% 11/04/2018   Wednesday    Week6

% Program: Solution for AAS, S1.2018, Project02.PartD
%..........................................................................
function MyProgram()
    clear all; clc() ; close all;
    load('Speed_dataC.mat') ; load('IMU_dataC.mat') ; load('Laser__2C.mat') ;
    
    % Storing value.......................................................
    N_imu = Vel.N;
    speed = Vel.speeds;
    times_imu = double(Vel.times)/10000;
    times_imu = times_imu - times_imu(1);
    
    N_laser = dataL.N;
    times_laser = double(dataL.times)/10000;
    times_laser = times_laser - times_laser(1);
    
    % Initilize plotting...................................................
    figure(8) ; clf() ; hold on ; grid on ; zoom on ;
    xlabel('X (m)') ; ylabel('Y (m)') ; axis([-10 7 -1 8]) ;
    MyGUIHandle.handle1 = line(0,0,'Color','magenta','LineWidth',2); % robot position
    MyGUIHandle.handle2 = plot(0,0,'g*'); % obstacle
    MyGUIHandle.handle3 = plot(0,0,'k+','MarkerSize',15); % landmark
    MyGUIHandle.handle4 = plot(0,0,'b.'); % all scan
    MyGUIHandle.handle5 = plot(0,0,'r*'); % bright OOIs
    MyGUIHandle.handle6 = title('');      % empty title (update synchronizly with pole update)
    MyGUIHandle.handle7 = text(0,0,'');
    MyGUIHandle.handle8 = text(0,0,'');
    MyGUIHandle.handle9 = text(0,0,'');
    MyGUIHandle.handle10 = text(0,0,'');
    MyGUIHandle.handle11 = text(0,0,'');
    
    % Manage yaw_rate bias.................................................
    ii = times_imu < 20 ;
    bias = mean(IMU.DATAf(6,ii)) ;
    yaw_rate = IMU.DATAf(6,:) - bias ;
    
    Pose = zeros(3,N_imu-1);
    Pose(:,1) = [0 0 pi/2]; 
    skip = 1;
    j = 1;                  % Index of Laser Scan
    for i = 1:skip:N_imu-1  % Index of Vehicel Pose
        Pose(:,i+1) = PredictVehiclePose(Pose(:,i), yaw_rate(i), speed(i) ,times_imu(i+1)-times_imu(i));
        if times_imu(i) > times_laser(j)
            if j == 1
                landmark = ProcessFirstScan( dataL.Scans(:,j), MyGUIHandle, j, Pose(:,i) );
            else
                ProcessScan( dataL.Scans(:,j), MyGUIHandle, j, Pose(:,i), landmark );
            end
            
            PlotPose( Pose, MyGUIHandle, speed, i, N_imu ); % plot it synchornizly with scan will loop faster
            j = j + 1 ; 
            pause(0.01) ;
        end
    end
return
end

% =================== Process Laser =======================================
function landmark = ProcessFirstScan(scan,mh,j,Pose)

% Declare two Mask to extract ranges and intensities
mask1FFF = uint16(2^13-1);
maskE000 = bitshift(uint16(7),13)  ; % shift 0111 to left 13 bits

intensities = bitand(scan,maskE000);
ranges    = single(bitand(scan,mask1FFF))*0.01; 

% Transfer Polar form to Cartesian form
angles = [0:360]'*0.5* pi/180 ;       
X = -cos(angles).*ranges;
Y = sin(angles).*ranges;  

landmark = ExtractOOIs(ranges,intensities, X, Y) ;  % local coordinate
landmark = PlotOOIs(mh,landmark,intensities,j,Pose);% global coordinate

return;
 end

function ProcessScan(scan,mh,j,Pose,landmark)

% Declare two Mask to extract ranges and intensities
mask1FFF = uint16(2^13-1);
maskE000 = bitshift(uint16(7),13)  ; % shift 0111 to left 13 bits

intensities = bitand(scan,maskE000);
ranges    = single(bitand(scan,mask1FFF))*0.01; 

% Transfer Polar form to Cartesian form
angles = [0:360]'*0.5* pi/180 ;       
X = -cos(angles).*ranges;
Y = sin(angles).*ranges;  

OOIs = ExtractOOIs(ranges,intensities, X, Y) ;
if OOIs.N > 0
    GOOIs = PlotOOIs(mh,OOIs,intensities,j,Pose);
    min = 1;
    index = 0;
%     for k = 1:GOOIs.N
    for h = 1:landmark.N
        X = [landmark.Centers(:,h)'; GOOIs.Centers(:,1)'];
        temp = pdist(X);
        if temp < min
           min = temp;
           index = h;
        end
        %disp(index);
    end
    if  index == 1 && min < 0.4
       set(mh.handle7,'Position',[double(landmark.Centers(1,index)),double(landmark.Centers(2,index))],'String',index);
    else
        set(mh.handle7,'Position',[0,0]);
    end
end

return;
end

function r = ExtractOOIs(ranges,intensities,X,Y)
    r.N = 0;
    r.Centers = [];
    r.Sizes   = []; % diameter
    r.Color = [];
    count = 0;

    for i = 1:length(ranges)-1
        if abs(ranges(i)-ranges(i+1)) > 0.05 
            x = mean(X((i-count):i));
            y = mean(Y((i-count):i));
            diameter = sqrt((X(i-count)-X(i))^2+(Y(i-count)-Y(i))^2);
            
            color = 0;
            if sum(intensities((i-count):i))~=0, color = 1; end
            
            if  diameter <= 0.2 && diameter >= 0.05 && color == 1
                if isempty(r.Centers)       %If value is empty
                    r.Centers(1,:) = x;
                    r.Centers(2,:) = y;
                    r.Color = color;
                    r.Sizes = diameter;
                else
                    r.Centers = [r.Centers(1,:),x;r.Centers(2,:),y];
                    r.Color = [r.Color,color];
                    r.Sizes = [r.Sizes,diameter];
                end
                r.N = r.N + 1;
            end
            count = 0;    
        else
            count = count + 1;
        end
    end
 return;
 end

function GOOIs = PlotOOIs(mh,OOIs,intensities,i,Pose)
    if OOIs.N<1,  return ; end;
    OOIs_HR = find(OOIs.Color~=0);
    
    L = 0.46;
    xx = OOIs.Centers(1,OOIs_HR);
    yy = OOIs.Centers(2,OOIs_HR) + L;
    
    % Rotation
    heading = Pose(3);
    rotate = heading - pi/2 ; %disp(rotate);
    x = xx*cos(rotate)-yy*sin(rotate);
    y = xx*sin(rotate)+yy*cos(rotate);
    
    % Translation
    x = x + Pose(1);
    y = y + Pose(2);
    
    % return global coordinate
    GOOIs.Centers = [x;y];
    GOOIs.N = OOIs.N;
    
    set(mh.handle2,'xdata',x ,'ydata',y );
    if i==1, set(mh.handle3,'xdata',x(OOIs_HR) ,'ydata',y(OOIs_HR) );end
return;
end

% =================== Process Vehicle =====================================
function X = PredictVehiclePose(X0,yaw_rate,speed,dt)
    dL = dt*speed ;
    X(3) = X0(3) + dt * (-yaw_rate); % yawA = -yawB
    X(1:2) = X0(1:2)+dL*[ cos(X0(3));sin(X0(3))] ;
return ;
end
 
function PlotPose(Pose,mh,speed,i,N_imu)
    scale = 0.46;
    x = [Pose(1,i) Pose(1,i) + (speed(i)+scale) * cos(Pose(3,i))];
    y = [Pose(2,i) Pose(2,i) + (speed(i)+scale) * sin(Pose(3,i))];
    set(mh.handle1,'xdata',x ,'ydata',y);
%   set(mh.handle1,'xdata',Pose(1,i) ,'ydata',Pose(2,i) );
%    set(mh.handle1,'xdata',Pose(1,:) ,'ydata',Pose(2,:) );

    s = sprintf('(Dynamic plotting...)\nShowing scan #[%d]/[%d]\r',i,N_imu);
    set(mh.handle6,'string',s);
end
