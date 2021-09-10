function [X,P]=removeUnobser(Xn,Pn,Index)
%coded by Yang SONG

N=round(size(Xn,2)/4-1);
n=size(Index,2);
if n<1
    X=Xn;
    P=Pn;
else
    X=zeros(3,4*n+4);
    X(1:3,1:3)=Xn(1:3,1:3);
    X(1:3,3*n+4)=Xn(1:3,3*N+4);
    P=zeros(6*n+6,6*n+6);
    P(1:3,1:3)=Pn(1:3,1:3);
    P(1:3,3*n+4:3*n+6)=Pn(1:3,3*N+4:3*N+6);
    P(3*n+4:3*n+6,1:3)=Pn(3*N+4:3*N+6,1:3);
    P(3*n+4:3*n+6,3*n+4:3*n+6)=Pn(3*N+4:3*N+6,3*N+4:3*N+6);

    for i=1:n
        X(1:3,3*i+1:3*i+3)=Xn(1:3,3*Index(i)+1:3*Index(i)+3);%here is something wrong
        X(1:3,3*n+4+i)=Xn(1:3,3*N+4+Index(i));

        P(1:3,3*i+1:3*i+3)=Pn(1:3,3*Index(i)+1:3*Index(i)+3);
        P(1:3,3*n+4+3*i:3*n+6+3*i)=Pn(1:3,3*N+4+3*Index(i):3*N+3*Index(i)+6);
        P(3*i+1:3*i+3,1:3)=Pn(3*Index(i)+1:3*Index(i)+3,1:3);
        P(3*n+4+3*i:3*n+6+3*i,1:3)=Pn(3*N+4+3*Index(i):3*N+3*Index(i)+6,1:3);

        P(3*n+4:3*n+6,3*i+1:3*i+3)=Pn(3*N+4:3*N+6,3*Index(i)+1:3*Index(i)+3);
        P(3*n+4:3*n+6,3*n+4+3*i:3*n+6+3*i)=Pn(3*N+4:3*N+6,3*N+4+3*Index(i):3*N+3*Index(i)+6);
        P(3*i+1:3*i+3,3*n+4:3*n+6)=Pn(3*Index(i)+1:3*Index(i)+3,3*N+4:3*N+6);
        P(3*n+4+3*i:3*n+6+3*i,3*n+4:3*n+6)=Pn(3*N+4+3*Index(i):3*N+3*Index(i)+6,3*N+4:3*N+6);
        for j=1:n
            P(3*i+1:3*i+3,3*j+1:3*j+3)=Pn(3*Index(i)+1:3*Index(i)+3,3*Index(j)+1:3*Index(j)+3);
            P(3*n+4+3*i:3*n+6+3*i,3*n+4+3*j:3*n+6+3*j)=Pn(3*N+4+3*Index(i):3*N+6+3*Index(i),3*N+4+3*Index(j):3*N+6+3*Index(j));


        end

    end
end




        