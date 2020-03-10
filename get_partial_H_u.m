function partial_H_u=get_partial_H_u(U,R,lamda,i,qv,qdel)
    partial_H_u(:,i)=(U(:,i)'*R+lamda(:,i+1)'*fu()+0.1*qv(i)*li(qv,1,i)*qvu(qv,i)+0.1*qdel(i)*li(qdel,1,i)*qdelu(qdel,i))';
end
function result=fu()%(U,R,lamda,qv,qdel)
    result=zeros(4,2);
end

%function for derivative of qv w.r.t u
function result = qvu(qv,i)
    if qv(i)>=0
        result=[1 0];
    else
        result=[-1 0];
    end
end
function result = qdelu(qdel,i)
   if qdel(i)>=0
      result= [1 0];
   else
       result=[-1 0];
   end
end
