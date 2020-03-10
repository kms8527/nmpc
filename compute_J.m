function J=compute_J(P0,Q,R,lamda,X,qv,qdel,Sj,Sl,Y_,U,delta,beta,move_obs_num,sta_obs_num,N,L)
    result1=1/2*Y_(:,N)'*P0*Y_(:,N);
    for i=1:N
        result2=0.1*li(qv,1,i)*qv(i);
        result3=0.1*li(qdel,1,i)*qdel(i);
    for k=1:move_obs_num
        result4(k)=0.1*Sj(k,i)*li(Sj,k,i);
    end
    for k=1:sta_obs_num
        result5(k)=0.1*Sl(k,i)*li(Sl,k,i);
    end
    J=result1+result2+result3+sum(result4)+sum(result5)+1/2*Y_(:,i)'*Q*Y_(:,i)+1/2*U(:,i)'*R*U(:,i)+lamda(:,i+1)'*(f(X,U,beta,delta,i,L)-X(:,i+1));
    end
end
function result=f(X,U,beta,delta,i,L)
    result(1,1)=X(4,i)*cos(X(3,i)+beta(i));
    result(2,1)=X(4,i)*sin(X(3,i)+beta(i));
    result(3,1)=X(4,i)*cos(beta(i))/L*tan(delta(i));
    result(4,1)=U(1,i);
end