%increment of Mean Square Error for pose feature based SLAM
%the result is (some difference of X_Estimation and X_state_gt).^2/m, 
%m is the number of tests
%coded by Yang SONG

function MSE_add=posefeature_MSE_add(X_Estimation,Xstate_gt,m)

T_steps=size(X_Estimation,2);%the number of time steps

%N=round(N/4)-1;             %the number of the features
MSE_add.RobotRotation=zeros(1,T_steps);
MSE_add.RobotPosition=zeros(1,T_steps);
MSE_add.RobotPose=zeros(1,T_steps);
MSE_add.FeatureRotation=zeros(1,T_steps);
MSE_add.FeaturePosition=zeros(1,T_steps);
MSE_add.FeaturePose=zeros(1,T_steps);


for i=1:T_steps
    error=(poseState_minus(X_Estimation{i}.state,Xstate_gt{i})).^2/m;
    N=round((size(error,1))/6)-1;
    MSE_add.RobotRotation(i)=sum(error(1:3));
    MSE_add.RobotPosition(i)=sum(error(3*N+4:3*N+6));
    MSE_add.RobotPose(i)=MSE_add.RobotRotation(i)+MSE_add.RobotPosition(i);
    MSE_add.FeatureRotation(i)=sum(error(4:3*N+3));
    MSE_add.FeaturePosition(i)=sum(error(3*N+7:6*N+6));
    MSE_add.FeaturePose(i)=MSE_add.FeatureRotation(i)+MSE_add.FeaturePosition(i);
end

MSE_add.Total=MSE_add.RobotPose+MSE_add.FeaturePose;