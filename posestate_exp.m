function X=posestate_exp(xi)
%xi is an element in lie algebra, whose form is a 6(N+1)*1 column (xi_R,xi_Rp,xi_x,xi_p)
%coded by Yang SONG

N=round(size(xi,1))/6-1;
X=zeros(3,4*N+4);
X(:,1:3)=so3_exp( xi(1:3,1) );
X(:,3*N+4)=jaco_r(-xi(1:3,1))*( xi(3*N+4:3*N+6,1) );


for i =1:N
    X(:,3*i+1:3*i+3)=so3_exp( xi(3*i+1:3*i+3,1) );
    X(:,3*N+4+i)=jaco_r(-xi(1:3,1))*(xi(3*N+4+3*i:3*N+6+3*i,1));
end







