function xi=poseState_minus(X1,X2)
%X1 and X2 are the state with pose feature, the form of them are
%(R,R^{p1},...,R^{pN},x,p1,...,pN), R is the rotation of the robot, R^{pi} is
%the rotation of the i-th pose, x is the position of the robot, and pi is
%the position of the i-th pose. 
%the dimension of X1 and X2 should be the same.
%xi is in the lie algebra. Its form is (xi_R;xi_Rp1;...;xi_RpN;xi_x;xi_p1;...;xi_pN)
%coded by Yang SONG

N1=size(X1,2);
N2=size(X2,2);

if N1==N2
    N=round(N1/4); %N is the num of features +1
else
    warning('The dimensions of the inputs do not match in poseState_minus!');
end

R1=X1(:,1:3);
R2=X2(:,1:3);
X=zeros(3,4*N);

for i = 1:N
    X(:,3*i-2:3*i)=X1(:,3*i-2:3*i)*X2(:,3*i-2:3*i)';
    X(:,3*N+i)=-R1*R2'*X2(:,3*N+i)+X1(:,3*N+i);
end

xi=zeros(6*N,1);

for i=1:N
    xi(3*i-2:3*i,1)= so3_log(X(:,3*i-2:3*i));
    
    normV=norm(xi(3*i-2:3*i,1));
    if normV<1.0e-20
        xi(3*i-2:3*i,1)=zeros(3,1);
    else
        normvec=xi(3*i-2:3*i,1)/normV;
        if normV>=2*pi
            normV=mod(normV,2*pi);
            if normV>pi
                normV=normV-2*pi;
            end
        end
        xi(3*i-2:3*i,1)=normV*normvec;
    end
    if norm(X(:,3*N+i))<1.0e-20
        xi(3*N+3*i-2:3*N+3*i,1)=zeros(3,1);
    else
%         xi(3*N+3*i-2:3*N+3*i,1)=jacor_inverse(-xi(1:3,1))*X(:,3*N+i);
        xi(3*N+3*i-2:3*N+3*i,1)=jaco_r(-xi(1:3,1))\X(:,3*N+i);
    end
end









