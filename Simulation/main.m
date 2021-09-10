
%RMSE and NEES of RIEKF, Std-EKF, and Ideal EKF
%coded by Yang SONG

clear all
clc
addpath('./Math/')

load('data.mat');
m=50; %the number of tests
P0=0*P0;








%% settings 
ODOM_noise = 1*diag([0.1^2,0.1^2,0.1^2, 0.1^2,0.1^2,0.1^2]); % odometry noise cov matrix should be 6*6
OBSV_noise=diag([1*0.1^2 1*0.1^2 1*0.1^2 1*0.1^2 1*0.1^2 1*0.1^2]);

%%
[MSE,NEES,MSE_Std,NEES_Std,MSE_Ideal,NEES_Ideal]=stattest(Xstate_gt,U,z_expectation,z_expectation0,X0,P0,Index,T_steps,m,ODOM_noise,OBSV_noise);

%%
RMSE.RobotRotation=sqrt(MSE.RobotRotation);
RMSE.RobotPosition=sqrt(MSE.RobotPosition);
RMSE.FeatureRotation=sqrt(MSE.FeatureRotation);
RMSE.FeaturePosition=sqrt(MSE.FeaturePosition);

RMSE_Std.RobotRotation=sqrt(MSE_Std.RobotRotation);
RMSE_Std.RobotPosition=sqrt(MSE_Std.RobotPosition);
RMSE_Std.FeatureRotation=sqrt(MSE_Std.FeatureRotation);
RMSE_Std.FeaturePosition=sqrt(MSE_Std.FeaturePosition);

RMSE_Ideal.RobotRotation=sqrt(MSE_Ideal.RobotRotation);
RMSE_Ideal.RobotPosition=sqrt(MSE_Ideal.RobotPosition);
RMSE_Ideal.FeatureRotation=sqrt(MSE_Ideal.FeatureRotation);
RMSE_Ideal.FeaturePosition=sqrt(MSE_Ideal.FeaturePosition);















%%

name=RMSE;
name1=RMSE_Std;
name2=RMSE_Ideal;




figure(10)
plot(1:T_steps,name.RobotRotation);hold on
plot(1:T_steps,name1.RobotRotation);hold on
plot(1:T_steps,name2.RobotRotation)
xlabel('Time Steps')
legend('RI-EKF','Std-EKF','Ideal-EKF')
title('RMSE for RobotRotation')
saveas(gca,['RMSE_RobotRotation_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.png'])
saveas(gca,['RMSE_RobotRotation_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.fig'])

figure(11)
plot(1:T_steps,name.RobotPosition);hold on
plot(1:T_steps,name1.RobotPosition);hold on
plot(1:T_steps,name2.RobotPosition)
xlabel('Time Steps')
legend('RI-EKF','Std-EKF','Ideal-EKF')
title('RMSE for RobotPosition')
saveas(gca,['RMSE_RobotPosition_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.png'])
saveas(gca,['RMSE_RobotPosition_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.fig'])


figure(13)
plot(1:T_steps,name.FeatureRotation);hold on
plot(1:T_steps,name1.FeatureRotation);hold on
plot(1:T_steps,name2.FeatureRotation)
xlabel('Time Steps')
legend('RI-EKF','Std-EKF','Ideal-EKF')
title('RMSE for FeatureRotation')
saveas(gca,['RMSE_FeatureRotation_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.png'])
saveas(gca,['RMSE_FeatureRotation_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.fig'])

figure(14)
plot(1:T_steps,name.FeaturePosition);hold on
plot(1:T_steps,name1.FeaturePosition);hold on
plot(1:T_steps,name2.FeaturePosition)
xlabel('Time Steps')
legend('RI-EKF','Std-EKF','Ideal-EKF')
title('RMSE for FeaturePosition')
saveas(gca,['RMSE_FeaturePosition_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.png'])
saveas(gca,['RMSE_FeaturePosition_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.fig'])

%%
name=NEES;
name1=NEES_Std;
name2=NEES_Ideal;

figure(16)
plot(1:T_steps,name.RobotPose,'LineWidth',0.7);hold on
plot(1:T_steps,name1.RobotPose,'LineWidth',0.7);hold on
plot(1:T_steps,name2.RobotPose,'LineWidth',0.7);hold off
xlabel('Time Steps')
% axis([-0.5 T_steps+1 0 11])
title('NEES for RobotPose')
legend('RI-EKF','Std-EKF','Ideal-EKF')
saveas(gca,['NEES_RobotPose_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.png'])
saveas(gca,['NEES_RobotPose_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.fig'])

figure(17)
plot(1:T_steps,name.RobotRotation);hold on
plot(1:T_steps,name1.RobotRotation);hold on
plot(1:T_steps,name2.RobotRotation);hold off
xlabel('Time Steps')
title('NEES for RobotRotation')
legend('RI-EKF','Std-EKF','Ideal-EKF')
saveas(gca,['NEES_RobotRotation_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.png'])
saveas(gca,['NEES_RobotRotation_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.fig'])

figure(18)
plot(1:T_steps,name.RobotPosition);hold on
plot(1:T_steps,name1.RobotPosition);hold on
plot(1:T_steps,name2.RobotPosition);hold off
xlabel('Time Steps')
title('NEES for RobotPosition')
legend('RI-EKF','Std-EKF','Ideal-EKF')
saveas(gca,['NEES_RobotPosition_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.png'])
saveas(gca,['NEES_RobotPosition_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.fig'])

figure(19)
plot(1:T_steps,name.FeaturePose,'LineWidth',1.5);hold on
plot(1:T_steps,name1.FeaturePose,'LineWidth',1.5);hold on
plot(1:T_steps,name2.FeaturePose,'LineWidth',1.5);hold off
xlabel('Time Steps')
title('NEES for FeaturePose')
legend('RI-EKF','Std-EKF','Ideal-EKF')
saveas(gca,['NEES_FeaturePose_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.png'])
saveas(gca,['NEES_FeaturePose_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.fig'])

figure(20)
plot(1:T_steps,name.FeatureRotation);hold on
plot(1:T_steps,name1.FeatureRotation);hold on
plot(1:T_steps,name2.FeatureRotation);hold off
xlabel('Time Steps')
title('NEES for FeatureRotation')
legend('RI-EKF','Std-EKF','Ideal-EKF')
saveas(gca,['NEES_FeatureRotation_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.png'])
saveas(gca,['NEES_FeatureRotation_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.fig'])

figure(21)
plot(1:T_steps,name.FeaturePosition);hold on
plot(1:T_steps,name1.FeaturePosition);hold on
plot(1:T_steps,name2.FeaturePosition);hold off
xlabel('Time Steps')
title('NEES for FeaturePosition')
legend('RI-EKF','Std-EKF','Ideal-EKF')
saveas(gca,['NEES_FeaturePosition_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.png'])
saveas(gca,['NEES_FeaturePosition_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.fig'])



%%
Robot_Ori_x_gt=zeros(3,T_steps);
Robot_position_gt=zeros(3,T_steps);

for i=1:T_steps
    Robot_Ori_x_gt(:,i)=Xstate_gt{i}(1:3,1);
    Nn=size(Xstate_gt{i},2)/4;
    Robot_position_gt(:,i)=Xstate_gt{i}(1:3,3*Nn+1);
    
end

Rp_x_gt=zeros(3,N);
Rp_y_gt=zeros(3,N);
Rp_z_gt=zeros(3,N);

for i=1:N
    Rp_x_gt(:,i)=Xstate_gt{T_steps}(1:3,3*i+1);
    Rp_y_gt(:,i)=Xstate_gt{T_steps}(1:3,3*i+2);
    Rp_z_gt(:,i)=Xstate_gt{T_steps}(1:3,3*i+3);
end

p_gt=Xstate_gt{T_steps}(1:3,3*N+5:4*N+4);



figure
plot3(Robot_position_gt(1,:),Robot_position_gt(2,:),Robot_position_gt(3,:),'Color',[0.6 0.6 0.6],'LineWidth',2);hold on
quiver3(p_gt(1,:),p_gt(2,:),p_gt(3,:),Rp_x_gt(1,:),Rp_x_gt(2,:),Rp_x_gt(3,:),0.15,'Color',[0.6350 0.0780 0.1840],'LineWidth',2);hold on
quiver3(p_gt(1,:),p_gt(2,:),p_gt(3,:),Rp_y_gt(1,:),Rp_y_gt(2,:),Rp_y_gt(3,:),0.15,'Color',[0 0.4470 0.7410],'LineWidth',2);hold on
quiver3(p_gt(1,:),p_gt(2,:),p_gt(3,:),Rp_z_gt(1,:),Rp_z_gt(2,:),Rp_z_gt(3,:),0.15,'Color',[0.4660 0.6740 0.1880],'LineWidth',2);hold on
quiver3(0,0,0,1,0,0,0.5,'k','MaxHeadSize',0.05,'AutoScaleFactor',0.89,'AutoScale','off');
plot3(0,0,0,'*k','LineWidth',2);hold on
xlabel('x (m)','Fontname', 'Times New Roman','FontSize',20)
ylabel('y (m)','Fontname', 'Times New Roman','FontSize',20)
zlabel('z (m)','Fontname', 'Times New Roman','FontSize',20)
title('Experiment Map','Fontname', 'Times New Roman','FontSize',20)
saveas(gca,['Experiment Map_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.fig'])
saveas(gca,['Experiment Map_m',num2str(m),'_N',num2str(N),'_T',num2str(T_steps),'.png'])
