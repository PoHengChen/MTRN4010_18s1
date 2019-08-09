% Edited by - Po-Heng Chen    
% 21/03/2018   Wednesday    Week4

% This program put all cluster in "Cluster" matrix, then filter the
% brilliant cluster before plotting.
% In this way, it is slower than edition 3

function MyProgram(DataFileName)

clc(); close all;
if ~exist('DataFileName','var'), DataFileName ='Laser__2.mat'; end;
load(DataFileName);                                      

figure(7); clf(); hold on; zoom on; axis([-10,10,0,20]);
title('Cartesian Plotting'); xlabel(''); ylabel('');

MyGUIHandles.handle1 = plot(0,0,'b.');
MyGUIHandles.handle2 = plot(0,0,'r*');
MyGUIHandles.handle3 = plot(0,0,'g+');

N = dataL.N;  
disp('Dynamic plotting');

for i=1:1:N,     % i representing which scan it is          
    scan_i = dataL.Scans(:,i);
    ProcessScan(scan_i,MyGUIHandles,i);
    
    s=sprintf('(Dynamic plotting...)\nShowing scan #[%d]/[%d]\r',i,N);
    title(s);
    
    pause(0.01) ;                   % wait for ~10ms
end;
disp('Done. Bye.');

return;
end

function ProcessScan(scan,mh,i)

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
    store = 0;

    for i = 1:length(ranges)-1

        if abs(ranges(i)-ranges(i+1)) > 0.05 %&&count >2
        
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
                
                store = store + 1;
            end
            count = 0;    
        else
            count = count + 1;
        end
    end
    
    r.N = store;

    
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