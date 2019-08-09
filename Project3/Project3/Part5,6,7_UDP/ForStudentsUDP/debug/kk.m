% Author: Po-Heng Chen , Z5014392
% 11/04/2018   Wednesday    Week9
% Program: Solution for AAS, S1.2018, Project03.Part3
% ======================== Main function ==================================
function kk()
global cur MyGUIHandles;
% ======= local plot =====
% ProcessScan(cur.scan,MyGUIHandles);

% ======= vehicle plot ======
cur.Xdr = PredictVehiclePose(cur.Xdr,(cur.yaw_rate),cur.speed,cur.Dt);
cur.Xdr_History = [cur.Xdr_History,cur.Xdr];
set(MyGUIHandles.handle4,'xdata',cur.Xdr_History(1,:) ,'ydata',cur.Xdr_History(2,:) );

% cur.Xdr = PredictVehiclePose(cur.Xdr, cur.yaw_rate, cur.speed ,cur.Dt); cur.Xdr = cur.Xdr';
% cur.Xdr_History = [cur.Xdr_History,cur.Xdr] ;
% % set(MyGUIHandle.handle1,'xdata',cur.Xdr(1),'ydata',cur.Xdr(2),'udata',0.46 * cos(cur.Xdr(3)),'vdata',0.46 * sin(cur.Xdr(3)));
% set(MyGUIHandle.handle1,'xdata',cur.Xdr_History(1,:),'ydata',cur.Xdr_History(2,:));
% 
% s = sprintf('(Dynamic plotting...)\nShowing scan #[%d]/[%d]\r',cur.i,cur.N_imu);
% set(MyGUIHandle.handle6,'string',s);
% cur.i = cur.i + 1;
pause(0.005) ;
return
end

function ProcessScan(scan,mh)

% Declare two Mask to extract ranges and intensities
mask1FFF = uint16(2^13-1);
maskE000 = bitshift(uint16(7),13)  ; % shift 0111 to left 13 bits

intensities = bitand(scan,maskE000);
ranges    = single(bitand(scan,mask1FFF))*0.01; 

% Transfer Polar form to Cartesian form
angles = [0:360]'*0.5* pi/180 ;       
X = cos(angles).*ranges;
Y = sin(angles).*ranges;   

set(mh.handle1,'xdata',X,'ydata',Y);
ii = find(intensities~=0);
set(mh.handle2,'xdata',X(ii),'ydata',Y(ii));


OOIs = ExtractOOIs(ranges,intensities, X, Y) ;
PlotOOIs(mh,OOIs,intensities);

return;
end

function r = ExtractOOIs(ranges,intensities,X,Y)
    r.N = 0;
    r.Centers = [];
    r.Sizes   = []; 
    r.Color = [];
    count = 0;

    for i = 1:length(ranges)-1

        if abs(ranges(i)-ranges(i+1)) > 0.05 %&&count >2
            
            % Center
            x = mean(X((i-count):i));
            y = mean(Y((i-count):i));
            
            % Diameter (Size)
            diameter = sqrt((X(i-count)-X(i))^2+(Y(i-count)-Y(i))^2);
            
            % Color
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
 
function PlotOOIs(mh,OOIs,intensities)
    if OOIs.N<1, return ; end;
    %iii = find();
    OOIs_HR = find(OOIs.Color~=0);
    x = OOIs.Centers(1,:);
    y = OOIs.Centers(2,:);
    set(mh.handle3,'xdata',x(OOIs_HR) ,'ydata',y(OOIs_HR) );
return;
end

function X = PredictVehiclePose(X0,yaw_rate,speed,dt)
    X = X0;
    dL = dt*speed ;
    X(3) = X0(3) + dt * (-yaw_rate); % yawA = -yawB
    X(1:2) = X0(1:2) + dL*[ cos(X0(3));sin(X0(3))] ;
return ;
end