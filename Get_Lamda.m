%result of this function is lamda
function lamda = Get_Lamda(L,X,beta,delta,lamda,N,Y_,P0,Q,C,i,mov_obs_num,sta_obs_num,Sj,Sl,front_car,adj_behind_car,adj_front_car,C_star)
    if i==N+1
        lamda(:,N+1) = (-Y_(:,N+1)'*P0*C)';
    else
        lamda(:,i) = lamda(:,i+1)'*fx(L,X,beta,delta,i)-Y_(:,i)'*Q*C; 
    end
    for k=1:mov_obs_num
        result1=0.1*Sj(k,i)*li(Sj,k,i)*Sjx(C_star,X,front_car,adj_behind_car,adj_front_car,k,i); %0.1 -> obstacle sensitivity
    end
    for k=1:sta_obs_num
        result2=0.1*Sl(k,i)*li(Sl,k,i)*Slx(C_star,X,i);  %0.1 -> obstacle sensitivity
    end
    lamda(:,i)= (lamda(:,i)+result1+result2)'; %i did transpose here!
end

%function for partial derivative of Sj about x states 
function result= Sjx(C_star,X,front_car,adj_behind_car,adj_front_car,k,i)
    if k==1
        obs=front_car;
    else if k==2
            obs=adj_behind_car;
        else if k==3 %moving obstacle num
                obs=adj_front_car;
            end
        end
    end

   result=-C_star'*(C_star*X(:,i)-[obs.x; obs.y; 0])/norm(C_star*X(:,i)-[obs.x; obs.y; 0]);

end

%function for partial derivative of Sl about x states 
function result = Slx(C_star,X,i)
    result=-C_star'*(C_star*X(:,i)-[10; 0; 0])/norm(C_star*X(:,i)-[10; 0; 0]);
end

        
%output of this function is derivative of f(X,U)
function result=fx(L,X,beta,delta,i)
    result(1,1:4)=[0; 0; -X(4,i)*sin(X(3,i)+beta(i)); cos(X(3,i)+beta(i))];
    result(2,1:4)=[0; 0; +X(4,i)*cos(X(3,i)+beta(i)); sin(X(3,i)+beta(i))];
    result(3,1:4)=[0; 0; 0; X(4,i)/L*cos(beta(i))*tan(delta(i))];
    result(4,1:4)=[0; 0; 0; 0];
end
