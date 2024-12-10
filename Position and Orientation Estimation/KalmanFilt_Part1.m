clear; % Clear variables
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
Z = sampledVicon(1:6,:);%all the measurements that you need for the update
% Set initial condition
print
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1));
print(Uprev)% Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %J ust for saving state his.
prevTime = 0; %last time step in real time
%write your code here calling the pred_step.m and upd_step.m functions
for i = 1:length(sampledTime)
       angVel= sampledData(i).omg;
   %Extracts the angular velocity (omg) from the sampledData structure at index i

   acc = sampledData(i).acc;
   %Extracts the acceleration (acc) from the sampledData structure at index i

   currTime = sampledTime(i);
   %Retrieves the current time from the sampledTime vector at index i

   dt  = sampledTime(i) - prevTime;
   %Calculates the time difference between the current time (sampledTime(i)) and the previous time (prevTime)

   prevTime = currTime;
   %Updates the previous time to the current time for the next iteration

   z_t = Z(:,i);
   %Extracts a column vector from the matrix Z at column index i

   [covarEst,uEst] =  pred_step(uPrev,covarPrev,angVel,acc,dt);
   %Calls a function pred_step with parameters uPrev, covarPrev, angVel, acc, and dt. 
   %This function appears to performs the prediction step and returns covariance estimate (covarEst) and mean estimate (uEst).

   [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
   % Calls a function upd_step with parameters z_t, covarEst, and uEst 
   % This function appears to perform an update step based on the measurement z_t, and 
   % returns an updated estimate (uCurr) and updated covariance (covar_curr)

   uPrev = uCurr;
   %Updates the previous mean estimate for the next iteration

   covarPrev = covar_curr;
   %Updates the previous covariance for the next iteration

   savedStates(:,i) = uCurr;
   %Saves the current mean estimate uCurr into the savedStates matrix at column index i
end

plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);