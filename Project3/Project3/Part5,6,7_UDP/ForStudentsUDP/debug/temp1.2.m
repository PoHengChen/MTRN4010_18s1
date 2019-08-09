% Author: Po-Heng Chen , Z5014392
% 11/04/2018   Wednesday    Week9
% Program: Solution for AAS, S1.2018, Project03.Part3
% ======================== Main function ==================================
function MyProgram()
    global speedx;
    global yawratex;
    global cur;
    persistent i;
    if isempty(i)
        i = 1;
    end
    speedx(i) = cur.speed;
    yawratex(i) = cur.yaw_rate;
    i = i + 1;
    global MyGUIHandle;
    global landmark;
    global OOIs;
%     disp(cur.Dt);
    cur.Xdr = PredictVehiclePose(cur.Xdr, cur.yaw_rate, cur.speed ,cur.Dt); cur.Xdr = cur.Xdr';
        
 %% =============== Laser =================
    if cur.UDP_DATA_ID == 26 
        if cur.landmarkSetFlag == 0 %% ============ Landmark ===============
            landmark = ProcessFirstScan( cur.scan, MyGUIHandle ); 
            [OOIs] = ProcessScan( cur.scan, MyGUIHandle, cur.Xdr, landmark ,0);
            cur.landmarkSetFlag = 1;
        end     
        [OOIs] = ProcessScan( cur.scan, MyGUIHandle, cur.Xdr, landmark, 1 );
                 ProcessScan( cur.scan, MyGUIHandle, cur.Xe , landmark, 2 );
        pause(0.001) ;
    end
    if cur.landmarkSetFlag == 0, return; end

 %% =================== EKF =====================
    J = [ 1,  0,  -cur.Dt*cur.speed*sin(cur.Xe(3)) ;  0,  1,   cur.Dt*cur.speed*cos(cur.Xe(3)) ;   0,  0,  1  ] ; % Jacobian matrix of model
   Jn = [ cur.Dt*cos(cur.Xe(3)), 0  ;   cur.Dt*sin(cur.Xe(3)), 0  ; 0 , cur.Dt ] ;  % Jacobian matrix of input       
    % 
    Q = cur.Dt^2*cur.Q_Process_Model + Jn*cur.Q_Input*Jn';
    cur.P = J*cur.P*J'+ Q ; % new covariance, P(K+1|K) = J*P(K|K)*J'+Q ;

    % ===== Prediction =====
    cur.Xe = PredictVehiclePose(cur.Xe,cur.yaw_rate,cur.speed,cur.Dt) ; cur.Xe = cur.Xe';
    [MasuredRanges, MasuredBearings] = MeasurementsFromLocalFrame(OOIs);

    if OOIs.N > 0
        for u = 1:length(OOIs.ID)
            ID = OOIs.ID(u);
            % some auxiliary variables.
            eDX = landmark.Centers(1,ID) - cur.Xe(1) - 0.46*cos(cur.Xe(3));      % (xu-x)
            eDY = landmark.Centers(2,ID) - cur.Xe(2) - 0.46*sin(cur.Xe(3));      % (yu-y)
            eDD = sqrt( eDX*eDX + eDY*eDY ) ; %   so : sqrt( (xu-x)^2+(yu-y)^2 ) 
            ExpectedRange = eDD ; ExpectedBearing = (atan2(eDY,eDX)) - cur.Xe(3) + pi/2 ;

            H = [  -eDX/eDD , -eDY/eDD      , 0 ; eDY/(eDD^2), -eDX/(eDD^2)  ,-1 ] ;
            z = [   MasuredRanges(u) - ExpectedRange ;  MasuredBearings(u) - ExpectedBearing ]; z(2) = wrapToPi(z(2));
            R = [ cur.sdev_rangeMeasurement*cur.sdev_rangeMeasurement,       0; 0,    cur.sdev_BearingMeasurment*cur.sdev_BearingMeasurment];

            % === Intermediate step ===
            S = R + H*cur.P*H';
            iS = inv(S);
            K = cur.P*H'*iS ;
            % ===== Observation =====
            cur.Xe = cur.Xe + K*z ; % update the  expected value
            cur.P = cur.P - K*H*cur.P ; % update the  Covariance % i.e. "P = P-P*H'*iS*H*P"  )
        end
    end
    cur.Xe_History = [cur.Xe_History,cur.Xe] ;
    cur.Xdr_History = [cur.Xdr_History,cur.Xdr] ;
    cur.i = cur.i+1;
 %% =========== Plot ====================
    PlotPose( cur.Xdr_History, cur.Xe_History,MyGUIHandle, cur.i, cur.N_imu ); % plot it synchornizly with scan will loop faster
return
end

% =========================================================================
function [ranges,bearings] = MeasurementsFromLocalFrame(OOIs)
    ID = OOIs.ID;
    X = [0 0 pi/2];
    if OOIs.N > 0
        dx = OOIs.Centers(1,:) - X(1);
        dy = OOIs.Centers(2,:) - X(2);
        ranges = sqrt((dx.*dx + dy.*dy));
        bearings = atan2(dy,dx) - X(3) + pi/2 ;
%         IDs = [1 : GOOIs.N];
    else
        IDs=[];ranges=[];bearings=[];
    end;
return ;
end

% ======================== Process Laser ==================================
function landmark = ProcessFirstScan(scan,mh)
mask1FFF = uint16(2^13-1);
maskE000 = bitshift(uint16(7),13); % shift 0111 to left 13 bits

intensities = bitand(scan,maskE000);
ranges = single(bitand(scan,mask1FFF))*0.01; 

% Transfer Polar form to Cartesian form
angles = [0:360]'*0.5* pi/180 ;       
X = -cos(angles).*ranges;
Y = sin(angles).*ranges;

landmark = ExtractOOIs(ranges,intensities, X, Y);
L = 0.46;

    if landmark.N < 1,  return ; end;
    landmark_HR = find(landmark.Color~=0);
%   landmark.Centers(1,landmark_HR);
    landmark.Centers(2,landmark_HR) = landmark.Centers(2,landmark_HR) + L;
    set(mh.handle3,'xdata',landmark.Centers(1,landmark_HR) ,'ydata',landmark.Centers(2,landmark_HR) );
return;
end

function [OOIs] = ProcessScan(scan,mh,Pose,landmark,v)
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
    OOIs = PlotOOIs(mh,OOIs,Pose,v); % Obtain Global coordinate
    OOIs = PrintID(mh,OOIs,landmark); % Data association to obtain right OOIs
 else
    OOIs.ID = [];
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

% Return OOIs as global frame
function OOIs = PlotOOIs(mh,OOIs,Pose,v) 
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
    OOIs.Centers(3:4,:) = [x;y];
    
    if v == 1
        set(mh.handle2,'xdata',x ,'ydata',y ); % OOIs (dr)
    elseif v == 2
        set(mh.handle88,'xdata',x ,'ydata',y ); % OOIs (ekf)
    else
        % landmark
    end
    
return;
end

function OOIs = PrintID(mh,OOIs,landmark)
    abc = [];
    tempCenter = [];
    for k = 1:OOIs.N % start with certain ooi(green plot)
        
        min = 0.4;
        index = 0;
        for h = 1:landmark.N % check the distance with each landmark respectively
            
            temp = pdist([landmark.Centers(:,h)'; OOIs.Centers(3:4,k)']);
            if temp < min 
               min = temp;
               index = h;
               if isempty(tempCenter)
                   tempCenter = OOIs.Centers(:,k);
               else
                   tempCenter = [tempCenter,OOIs.Centers(:,k)];
               end
            end
        end
        
        if index~=0, abc = real([abc,index]);end 
        
    end
    OOIs.Centers = tempCenter;
    if ~isempty(abc)
       if sum(ismember(abc,1))==1
           set(mh.handle7,'Position',[double(landmark.Centers(1,1)),double(landmark.Centers(2,1))],'String',1); 
       else 
           set(mh.handle7,'Position',[0,0],'String',0);
       end
       if sum(ismember(abc,2))==1
           set(mh.handle8,'Position',[double(landmark.Centers(1,2)),double(landmark.Centers(2,2))],'String',2); 
       else 
           set(mh.handle8,'Position',[0,0],'String',0);
       end
       if sum(ismember(abc,3))==1
           set(mh.handle9,'Position',[double(landmark.Centers(1,3)),double(landmark.Centers(2,3))],'String',3); 
       else 
           set(mh.handle9,'Position',[0,0],'String',0);
       end
       if sum(ismember(abc,4))==1
           set(mh.handle10,'Position',[double(landmark.Centers(1,4)),double(landmark.Centers(2,4))],'String',4); 
       else 
           set(mh.handle10,'Position',[0,0],'String',0);
       end
       if sum(ismember(abc,5))==1
           set(mh.handle11,'Position',[double(landmark.Centers(1,5)),double(landmark.Centers(2,5))],'String',5); 
       else 
           set(mh.handle11,'Position',[0,0],'String',0);
       end
    end
    OOIs.ID = abc;
    OOIs.N = length(abc);
return;
end
% ========================= Process Vehicle ===============================
function X = PredictVehiclePose(X0,yaw_rate,speed,dt)
    dL = dt*speed ;
    X(3) = X0(3) + dt * (-yaw_rate); % yawA = -yawB
    X(1:2) = X0(1:2) + dL*[ cos(X0(3));sin(X0(3))] ;
return ;
end

function PlotPose(Xdr_History, Xe_History, mh, i, N_imu)
    L = 0.46;
%     s = sprintf('(Dynamic plotting...)\nShowing scan #[%d]/[%d]\r',i,N_imu);
%     set(mh.handle6,'string',s);
    
%     set(mh.handle0,'xdata',Xdr(1,:),'ydata',Xdr(2,:)); 
%     set(mh.handle1,'xdata',Xdr(1,i),'ydata',Xdr(2,i),'udata',L * sin(Xdr(3,i)),'vdata',L * cos(Xdr(3,i)));
    set(mh.handle1,'xdata',Xdr_History(1,:) ,'ydata',Xdr_History(2,:) );
%     set(mh.handle87,'xdata',Xe_History(1,:),'ydata',Xe_History(2,:));  
%     set(mh.handle89,'xdata',Xe_History(1,i),'ydata',Xe_History(2,i),'udata',L * sin(Xe_History(3,i)),'vdata',L * cos(Xe_History(3,i)));    

end
% =========================================================================
    