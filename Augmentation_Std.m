function [X_new,P_new]=Augmentation_Std(X,P,z,omega)
% X=(R,Rp,x,p) is the states with the previous features
% P is the estimated covariance of X
% z=[z1_new_p1,..,z1_new_pk,z2_new_p1,...,z2_new_pk] is the observation of
% the k new features.
%z1_pi is a rotation matrix,
%z2_pi is a position vector.
%omega is the observaation noise for each (z1_pi,z2_pi). It should be a 6*6
%matrix.
% coded by Yang SONG
N1=size(X,2);
N2=size(P,2);
N1=round(N1/4)-1;
N2=round(N2/6)-1;
if N1==N2
    N=N1; %N is the num of previous features
else
    warning('The dimensions of the inputs do not match in function "Augmentation"!');
end

%N_new=round(size(z,2)/4);%the num of new features
N_new=round(size(z.position,2));
R=X(1:3,1:3);
%new features
f1=zeros(3,3*N_new); %f1 is the rotation part for the new features
f2=zeros(3,N_new); %f2 is the rotation part for the new features
for i=1:N_new
    f1(1:3,3*i-2:3*i)=R*z.rotation(1:3,3*i-2:3*i);
    f2(1:3,i)=X(1:3,3*N+4)+R*z.position(1:3,i);
%     f1(1:3,3*i-2:3*i)=z(1:3,3*i-2:3*i)*R;
%     f2(1:3,i)=X(1:3,3*N+4)+R*z(1:3,3*N_new+i);
end

%new state
X_new=[X(1:3,1:3*N+3) f1 X(1:3,3*N+4:4*N+4) f2];

%new covariance
P=(P+P')/2;
P11=P(1:3*N+3,1:3*N+3);
P12=P(1:3*N+3,3*N+4:6*N+6);
P21=P12';
P22=P(3*N+4:6*N+6,3*N+4:6*N+6);

M1=zeros(3*N_new,3*N+3);
M2=zeros(3*N_new,3*N+3);
M3=zeros(3*N_new,3*N+3);
Omega11=zeros(3*N_new,3*N_new);
Omega12=zeros(3*N_new,3*N_new);
Omega22=zeros(3*N_new,3*N_new);
R_blk=zeros(3*N_new,3*N_new);
for i=1:N_new
    M1(3*i-2:3*i,1:3)=eye(3);
    M2(3*i-2:3*i,1:3)=-skew(R*z.position(1:3,i));
    M3(3*i-2:3*i,1:3)=eye(3);
    Omega11(3*i-2:3*i,3*i-2:3*i)=omega(1:3,1:3);
    Omega12(3*i-2:3*i,3*i-2:3*i)=omega(1:3,4:6);
    Omega22(3*i-2:3*i,3*i-2:3*i)=omega(4:6,4:6);
    R_blk(3*i-2:3*i,3*i-2:3*i)=R;
end
Omega21=Omega12';

P_new=[P11 P11*M1' P12 P11*M2'+P12*M3';...
    M1*P11 M1*P11*M1'+R_blk*Omega11*R_blk' M1*P12 M1*P11*M2'+M1*P12*M3'+R_blk*Omega12*R_blk';...
    P21 P21*M1' P22 P21*M2'+P22*M3';...
    M2*P11+M3*P21 M2*P11*M1'+M3*P21*M1'+R_blk*Omega21*R_blk' M2*P12+M3*P22 M2*P11*M2'+M2*P12*M3'+M3*P21*M2'+M3*P22*M3'+R_blk*Omega22*R_blk'];
P_new=(P_new+P_new')/2;



