function lamda = Get_Lamda(L,X,beta,delta,lamda,N,Y_,P0,Q,C,i,mov_obs_num,sta_obs_num,Sj,Sl)
    lamda(:,N) = (-Y_(:,N)'*P0*C)';
    lamda(:,i) = lamda(:,i+1)'*fx(L,X,beta,delta,i)-Y_(:,i)'*Q*C; 
    for k=1:mov_obs_num
        result1=0.1*Sj(k,i)*li(Sj,k,i); %0.1 -> obstacle sensitivity
    end
    for k=1:sta_obs_num
        result2=0.1*Sl(k,i)*li(Sl,k,i);  %0.1 -> obstacle sensitivity
    end
    lamda(i)= (lamda(i)+result1+result2)'; %i did transpose here!
end

function result=fx(L,X,beta,delta,i)
    result(1,1:4)=[0; 0; -X(4,i)*sin(X(3,i)+beta(i)); cos(X(3,i)+beta(i))];
    result(2,1:4)=[0; 0; +X(4,i)*cos(X(3,i)+beta(i)); sin(X(3,i)+beta(i))];
    result(3,1:4)=[0; 0; 0; X(4,i)/L*cos(beta(i))*tan(delta(i))];
    result(4,1:4)=[0; 0; 0; 0];
end
function result=li(f,k,i)
    if f(k,i)<=0
        result=0;
    else
        result=1;
    end
end
