function [MSE,NEES,MSE_Std,NEES_Std,MSE_Ideal,NEES_Ideal]=stattest(Xstate_gt,U,z_expectation,z_expectation0,X0,P0,Index,T_steps,m,ODOM_noise,OBSV_noise)
%MSE
MSE.RobotRotation=zeros(1,T_steps);
MSE.RobotPosition=zeros(1,T_steps);
% MSE.RobotPose=zeros(1,T_steps);
MSE.FeatureRotation=zeros(1,T_steps);
MSE.FeaturePosition=zeros(1,T_steps);
% MSE.FeaturePose=zeros(1,T_steps);
% MSE.Total=zeros(1,T_steps);

%NEES
NEES.RobotRotation=zeros(1,T_steps);
NEES.RobotPosition=zeros(1,T_steps);
NEES.RobotPose=zeros(1,T_steps);
NEES.FeatureRotation=zeros(1,T_steps);
NEES.FeaturePosition=zeros(1,T_steps);
NEES.FeaturePose=zeros(1,T_steps);
% NEES.Total=zeros(1,T_steps);

%MSE_Std
MSE_Std.RobotRotation=zeros(1,T_steps);
MSE_Std.RobotPosition=zeros(1,T_steps);
% MSE_Std.RobotPose=zeros(1,T_steps);
MSE_Std.FeatureRotation=zeros(1,T_steps);
MSE_Std.FeaturePosition=zeros(1,T_steps);
% MSE_Std.FeaturePose=zeros(1,T_steps);
% MSE_Std.Total=zeros(1,T_steps);

%NEES_Std
NEES_Std.RobotRotation=zeros(1,T_steps);
NEES_Std.RobotPosition=zeros(1,T_steps);
NEES_Std.RobotPose=zeros(1,T_steps);
NEES_Std.FeatureRotation=zeros(1,T_steps);
NEES_Std.FeaturePosition=zeros(1,T_steps);
NEES_Std.FeaturePose=zeros(1,T_steps);
% NEES_Std.Total=zeros(1,T_steps);

%MSE_Ideal
MSE_Ideal.RobotRotation=zeros(1,T_steps);
MSE_Ideal.RobotPosition=zeros(1,T_steps);
% MSE_Ideal.RobotPose=zeros(1,T_steps);
MSE_Ideal.FeatureRotation=zeros(1,T_steps);
MSE_Ideal.FeaturePosition=zeros(1,T_steps);
% MSE_Ideal.FeaturePose=zeros(1,T_steps);
% MSE_Ideal.Total=zeros(1,T_steps);

%NEES_Ideal
NEES_Ideal.RobotRotation=zeros(1,T_steps);
NEES_Ideal.RobotPosition=zeros(1,T_steps);
NEES_Ideal.RobotPose=zeros(1,T_steps);
NEES_Ideal.FeatureRotation=zeros(1,T_steps);
NEES_Ideal.FeaturePosition=zeros(1,T_steps);
NEES_Ideal.FeaturePose=zeros(1,T_steps);
% NEES_Ideal.Total=zeros(1,T_steps);

for i=1:m

    U_noise=UaddNoise_FirstOrderInte(U,ODOM_noise);
    [z_noise,z0]=zaddNoise(z_expectation,z_expectation0,OBSV_noise);
    
    X_Estimation=posefeature_RIEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);
    X_Estimation_std=posefeature_StdEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);
    X_Estimation_ideal=posefeature_idealEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise,Xstate_gt);
    
    %Mean Absolute Error
%     MAE_add=posefeature_MAE_add(X_Estimation,Xstate_gt,m);
%     MAE.RobotRotation=MAE.RobotRotation+MAE_add.RobotRotation;
%     MAE.RobotPosition=MAE.RobotPosition+MAE_add.RobotPosition;
%     MAE.RobotPose=MAE.RobotPose+MAE_add.RobotPose;
%     MAE.FeatureRotation=MAE.FeatureRotation+MAE_add.FeatureRotation;
%     MAE.FeaturePosition=MAE.FeaturePosition+MAE_add.FeaturePosition;
%     MAE.FeaturePose=MAE.FeaturePose+MAE_add.FeaturePose;
%     MAE.Total=MAE.Total+MAE_add.Total;
    
    %MSE
%     MSE_add=posefeature_MSE_add(X_Estimation,Xstate_gt,m);
    MSE_add=posefeature_MSEstd_add(X_Estimation,Xstate_gt,m);
    MSE.RobotRotation=MSE.RobotRotation+MSE_add.RobotRotation;
    MSE.RobotPosition=MSE.RobotPosition+MSE_add.RobotPosition;
%     MSE.RobotPose=MSE.RobotPose+MSE_add.RobotPose;
    MSE.FeatureRotation=MSE.FeatureRotation+MSE_add.FeatureRotation;
    MSE.FeaturePosition=MSE.FeaturePosition+MSE_add.FeaturePosition;
%     MSE.FeaturePose=MSE.FeaturePose+MSE_add.FeaturePose;
%     MSE.Total=MSE.Total+MSE_add.Total;
    
    %NEES
    NEES_add=posefeature_NEES_add(X_Estimation,Xstate_gt,m);
    NEES.RobotRotation=NEES.RobotRotation+NEES_add.RobotRotation;
    NEES.RobotPosition=NEES.RobotPosition+NEES_add.RobotPosition;
    NEES.RobotPose=NEES.RobotPose+NEES_add.RobotPose;
    NEES.FeatureRotation=NEES.FeatureRotation+NEES_add.FeatureRotation;
    NEES.FeaturePosition=NEES.FeaturePosition+NEES_add.FeaturePosition;
    NEES.FeaturePose=NEES.FeaturePose+NEES_add.FeaturePose;
%     NEES.Total=NEES.Total+NEES_add.Total;
    
    %MSE_Std
%     MSE_Std_add=posefeature_MSE_add(X_Estimation_std,Xstate_gt,m);
    MSE_Std_add=posefeature_MSEstd_add(X_Estimation_std,Xstate_gt,m);
    MSE_Std.RobotRotation=MSE_Std.RobotRotation+MSE_Std_add.RobotRotation;
    MSE_Std.RobotPosition=MSE_Std.RobotPosition+MSE_Std_add.RobotPosition;
%     MSE_Std.RobotPose=MSE_Std.RobotPose+MSE_Std_add.RobotPose;
    MSE_Std.FeatureRotation=MSE_Std.FeatureRotation+MSE_Std_add.FeatureRotation;
    MSE_Std.FeaturePosition=MSE_Std.FeaturePosition+MSE_Std_add.FeaturePosition;
%     MSE_Std.FeaturePose=MSE_Std.FeaturePose+MSE_Std_add.FeaturePose;
%     MSE_Std.Total=MSE_Std.Total+MSE_Std_add.Total;
    
    %NEES_Std
    NEES_Std_add=posefeature_NEES_Std_add(X_Estimation_std,Xstate_gt,m);
    NEES_Std.RobotRotation=NEES_Std.RobotRotation+NEES_Std_add.RobotRotation;
    NEES_Std.RobotPosition=NEES_Std.RobotPosition+NEES_Std_add.RobotPosition;
    NEES_Std.RobotPose=NEES_Std.RobotPose+NEES_Std_add.RobotPose;
    NEES_Std.FeatureRotation=NEES_Std.FeatureRotation+NEES_Std_add.FeatureRotation;
    NEES_Std.FeaturePosition=NEES_Std.FeaturePosition+NEES_Std_add.FeaturePosition;
    NEES_Std.FeaturePose=NEES_Std.FeaturePose+NEES_Std_add.FeaturePose;
%     NEES_Std.Total=NEES_Std.Total+NEES_Std_add.Total;
    
    
    %MSE_Ideal
%     MSE_Ideal_add=posefeature_MSE_add(X_Estimation_ideal,Xstate_gt,m);
    MSE_Ideal_add=posefeature_MSEstd_add(X_Estimation_ideal,Xstate_gt,m);
    MSE_Ideal.RobotRotation=MSE_Ideal.RobotRotation+MSE_Ideal_add.RobotRotation;
    MSE_Ideal.RobotPosition=MSE_Ideal.RobotPosition+MSE_Ideal_add.RobotPosition;
%     MSE_Ideal.RobotPose=MSE_Ideal.RobotPose+MSE_Ideal_add.RobotPose;
    MSE_Ideal.FeatureRotation=MSE_Ideal.FeatureRotation+MSE_Ideal_add.FeatureRotation;
    MSE_Ideal.FeaturePosition=MSE_Ideal.FeaturePosition+MSE_Ideal_add.FeaturePosition;
%     MSE_Ideal.FeaturePose=MSE_Ideal.FeaturePose+MSE_Ideal_add.FeaturePose;
%     MSE_Ideal.Total=MSE_Ideal.Total+MSE_Ideal_add.Total;
    
    %NEES_Ideal
    NEES_Ideal_add=posefeature_NEES_Std_add(X_Estimation_ideal,Xstate_gt,m);
    NEES_Ideal.RobotRotation=NEES_Ideal.RobotRotation+NEES_Ideal_add.RobotRotation;
    NEES_Ideal.RobotPosition=NEES_Ideal.RobotPosition+NEES_Ideal_add.RobotPosition;
    NEES_Ideal.RobotPose=NEES_Ideal.RobotPose+NEES_Ideal_add.RobotPose;
    NEES_Ideal.FeatureRotation=NEES_Ideal.FeatureRotation+NEES_Ideal_add.FeatureRotation;
    NEES_Ideal.FeaturePosition=NEES_Ideal.FeaturePosition+NEES_Ideal_add.FeaturePosition;
    NEES_Ideal.FeaturePose=NEES_Ideal.FeaturePose+NEES_Ideal_add.FeaturePose;
%     NEES_Ideal.Total=NEES_Ideal.Total+NEES_Ideal_add.Total;
% %     NEES.ERROR=NEES.ERROR+NEES_add.ERROR;
    
end