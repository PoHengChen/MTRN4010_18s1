% Basic example, for reading UDP messages
% for MTRN4010.

% read document "StructureMessages-UDP_4010[2018].pdf" for details
% about the message format.

% Run "PossumSimulator09_UDP.exe", if you want to play data back, for
% testing your program.



function main()
  clc;clear;
  fprintf('Running an example UPD reader, for MTRN41010\n');  
  
  x = instrfind('type','udp'); delete(x);  %delete all UDP instances.
  % you may try to delete just those having conflicting input port
    
 PortRx = 1112;    
 uRx = udp('127.0.0.1','LocalPort',PortRx);
 set(uRx,'InputBufferSize',10000,'Timeout',1.2,'DatagramTerminateMode','on'); 
 fopen(uRx); 
 warning('off', 'instrument:fread:unsuccessfulRead');
    
cx1=0; t1=0; cx2=0;
tic();
initial();
initialPlot();
while 1
    xr=uint8(fread(uRx,10000,'uint8'));
    if (numel(xr)>0)     %number of array element > 0
        t1=0;  cx1=cx1+1; 
        ProcessData(xr);         %<----------- you process each message, here.
    else
        t1=t1+1.2;
        if (t1>4), fprintf('Too much time not having activity...\n');   break ; end; %timeout
    end;    
    
    if (toc()>1)   % print, at ~1HZ
        tic();  cx2=cx2+cx1; 
        if (cx1>0) 
            fprintf('Messages: new=[%d],total=[%d]\n',cx1,cx2);
        else
            fprintf('Messages: NO new arrivals.\n');
        end;
        
        cx1=0;
    end;    
    
end;    

fprintf('Closing UDP  port; BYE\n');
fclose(uRx) ; delete(uRx);
return;

end


function ProcessData(x)
    global cur;
    cur.UDP_DATA_ID = double(uint8(x(2)));
    
    time = double(typecast(uint8(x(9:12)),'uint32')) / 10000;
    if cur.UDP_DATA_ID == 31       % Dead-Reckoning (DR)(Longitudinal velocity and GyroZ).
%         cur.speed = double(uint8(x(13)))/1000;
%         cur.yaw_rate = -double(uint8(x(14)))*0.02*(pi/180);
        cur.speed = double(typecast((x(13:14)),'int16'))/1000;
       cur.yaw_rate = double(typecast((x(15:16)),'int16'))*0.02*(pi/180);
       cur.yaw_rate = cur.yaw_rate + pi/180; %- cur.bias;
        if cur.checkTimeFlag == 0
            cur.initialTime = time;
            cur.times_imu = time;
            cur.checkTimeFlag = 1;
        end
        cur.Dt = time - cur.times_imu;
        cur.times_imu = time;
    elseif cur.UDP_DATA_ID == 26   % LIDAR
        scan = [];
        for i = 1:361
           if isempty(scan)
               scan(1,:) = typecast(uint8(x(15:16)),'uint16');
           else
               scan = [scan(1,:),typecast(uint8(x(13+2*i:14+2*i)),'uint16')];
           end
        end
        cur.scan = scan';
        cur.times_laser = time;
    else
    end
    p5temptemp();
%     kk();

end

function initialPlot()
    figure(8) ; clf() ; hold on ; grid on ; zoom on ;
    xlabel('X (m)') ; ylabel('Y (m)') ; axis([-3.5 1.5 -1 5]);%axis([-10,10,0,20]) ;%axis([-10 7 -1 8]) ;
    global MyGUIHandle;
    figure(8) ; clf() ; hold on ; grid on ; zoom on ;
    xlabel('X (m)') ; ylabel('Y (m)') ; axis([-10 7 -1 8]) ;
    
    MyGUIHandle.handle3 = plot(0,0,'k+','MarkerSize',15); % landmark
    
%     MyGUIHandle.handle1 = quiver3(0,0,0,0,'Color','r','LineWidth',2); % robot position
    MyGUIHandle.handle1 = plot(0,0,'k.','MarkerSize',6);
    MyGUIHandle.handle2 = plot(0,0,'r*','MarkerSize',6); % obstacle
%     MyGUIHandle.handle0 = plot(0,0,'m.');
    
    MyGUIHandle.handle89 = quiver3(0,0,0,0,'Color','b','LineWidth',2); % robot position
    MyGUIHandle.handle88 = plot(0,0,'b*','MarkerSize',6); % obstacle
%     MyGUIHandle.handle87 = plot(0,0,'.');
    
    MyGUIHandle.handle6 = title('');      % empty title (update synchronizly with pole update)
    MyGUIHandle.handle7 = text(0,0,'');
    MyGUIHandle.handle8 = text(0,0,'');
    MyGUIHandle.handle9 = text(0,0,'');
    MyGUIHandle.handle10 = text(0,0,'');
    MyGUIHandle.handle11 = text(0,0,'');
return
end

function initial()
    global cur;
    cur.Xdr = [0; 0; pi/2];
    cur.Xdr_History = cur.Xdr;
    cur.bias = -0.0169;
    cur.checkTimeFlag = 0;
    
    cur.P = zeros(3,3);
    cur.stdDevGyro = 1.5*pi/180 ;        
    cur.stdDevSpeed = 0.05 ; 
    cur.sdev_rangeMeasurement = 0.15 ; 
    cur.sdev_BearingMeasurment = 1*pi/180;
    cur.Q_Process_Model = diag( [ (0.01)^2 ,(0.01)^2 , (1*pi/180)^2]) ; % Q_model 
    cur.Q_Input = diag([ cur.stdDevSpeed*cur.stdDevSpeed , cur.stdDevGyro*cur.stdDevGyro]);
    cur.Xe = [0; 0; pi/2];
    cur.Xe_History = cur.Xe;
    cur.bias = -0.0169;
    cur.landmarkSetFlag = 0;
    cur.i = 0;
    cur.N_laser = 8292;
    cur.N_imu = 44578;
    cur.speed_a = [];
    cur.yaw_rate_a = [];
    
    return
end

function unused()
    global cur;
    cur.i = 1;
    cur.P = zeros(3,3);
    cur.stdDevGyro = 1.5*pi/180 ;        
    cur.stdDevSpeed = 0.05 ; 
    cur.sdev_rangeMeasurement = 0.15 ; 
    cur.sdev_BearingMeasurment = 1*pi/180;
    cur.Q_Process_Model = diag( [ (0.01)^2 ,(0.01)^2 , (1*pi/180)^2]) ; % Q_model 
    cur.Q_Input = diag([ cur.stdDevSpeed*cur.stdDevSpeed , cur.stdDevGyro*cur.stdDevGyro]);
    
    cur.N_laser = 8292;
    cur.N_imu = 44578;
    cur.Xe = [ 0; 0; pi/2 ] ;
    cur.Xe_History = cur.Xe ;
%     cur.Xe_History(:,1) = cur.Xe ;
% ========================================================================
    cur.Xdr_History = zeros(3,cur.N_imu-1);
    cur.Xdr = [0; 0; pi/2];
%     cur.Xdr_History(:,1) = cur.Xdr;
    cur.time = 0;
    cur.checkTimeFlag = 0;
%     cur.ranges = [];
%     cur.intensities = [];
    cur.landmarkSetFlag = 0;
    cur.d = 0.46;
    cur.bias = -0.0169;
return
end
