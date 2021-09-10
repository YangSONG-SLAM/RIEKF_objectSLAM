%coded by Yang SONG
function U_noise=UaddNoise_FirstOrderInte(U,ODOM_noise)
T_steps=size(U,2);
U_noise=cell(1,T_steps);

W=mvnrnd(zeros(1,6),ODOM_noise,T_steps)'; %each column is a noise vector in lie algebra
%exp_W=zeros(3*T_steps,4);

for i =1:T_steps
    normW=norm(W(1:3,i));
    normvec=W(1:3,i)/normW;
    if normW>=2*pi
        normW=mod(normW,2*pi);
        if normW>pi
            normW=normW-2*pi;
        end
    end
    W(1:3,i)=normW*normvec;
    exp_W=so3_exp(W(1:3,i));
    %exp_W(3*i-2:3*i,1:4)=se3_exp(W(:,i));
    U_noise{i}.rotation=exp_W(1:3,1:3)*U{i}.rotation;
    U_noise{i}.position=U{i}.position+W(4:6,i);
end
