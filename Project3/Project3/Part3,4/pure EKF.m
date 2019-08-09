
            % ================= KF ==================
            
            % Jacobian matrix of model
            J = [ [ 1,  0,  -Dt*speed(i)*sin(Xe(3))  ] ; 
                  [ 0,  1,   Dt*speed(i)*cos(Xe(3))  ] ;    
                  [ 0,  0,      1                    ] ] ; 

            % Jacobian matrix of input
           Jn = [ [ Dt*cos(Xe(3)), 0 ] ; 
                  [ Dt*sin(Xe(3)), 0 ] ;    
                  [ 0            , Dt ] ] ;        
            
            % 
            Q = Dt^2*Q_Process_Model + Jn*Q_Input*Jn';
            
            % new covariance, P(K+1|K) = J*P(K|K)*J'+Q ;
            P = J*P*J'+ Q ;
            
            % ===== Prediction =====
            Xe = PredictVehiclePose(Xe,yaw_rate(i),speed(i),Dt) ;
            Xe = Xe';
            
            % ????????????????????????????
            [MasuredRanges, MasuredBearings] = MeasurementsFromLocalFrame(OOIs);
%             disp(i);
            if OOIs.N > 0
                for u = 1:length(OOIs.ID)
                    
                    ID = OOIs.ID(u);
                    
                    % some auxiliary variables.
                    eDX = landmark.Centers(1,ID) - Xe(1) - d*cos(Xe(3));      % (xu-x)
                    eDY = landmark.Centers(2,ID) - Xe(2) - d*sin(Xe(3));      % (yu-y)
                    eDD = sqrt( eDX*eDX + eDY*eDY ) ; %   so : sqrt( (xu-x)^2+(yu-y)^2 ) 
                    ExpectedRange = eDD ;
                    ExpectedBearing = (atan2(eDY,eDX)) - Xe(3) + pi/2 ;
                    
                    H = [  -eDX/eDD , -eDY/eDD      , 0 ;
                         eDY/(eDD^2), -eDX/(eDD^2)  ,-1 ] ;
                    
                    z = [ MasuredRanges(u) - ExpectedRange ; 
                        MasuredBearings(u) - ExpectedBearing ];
                    z(2) = wrapToPi(z(2));
                        
                    R = [ sdev_rangeMeasurement*sdev_rangeMeasurement,       0;
                              0,    sdev_BearingMeasurment*sdev_BearingMeasurment];
                    
                    % === Intermediate step ===
                    S = R + H*P*H';
                    iS = inv(S);
                    K = P*H'*iS ;
                    % ===== Observation =====
                    % update the  expected value
%                     Xe = Xe + K*z ;   
                    % update the  Covariance % i.e. "P = P-P*H'*iS*H*P"  )
                    P = P - K*H*P ;       
                end
            end
            Xe_History(:,i) = Xe ;
            % =======================================