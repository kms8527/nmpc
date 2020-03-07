function [X,beta,delta,qv,qdel,Sj,Sl,Y,Y_]=Get_Next_States(C,X,U,L,delt,beta,delta,i,umax,Rc,Ro,Ru,front_car,adj_behind_car,adj_front_car,Y,Y_,Yd,qv,qdel,Sj,Sl)
%F 이 함수의 요약 설명 위치
%   update state and constraint function 
delta(i+1)=delta(i)+delt*U(1,i);
beta(i+1)=atan2(1/2*tan(delta(i)),1);
X(1,i+1)=X(1,i)+delt*X(4,i)*cos(X(3,i)+beta(i));%x
X(2,i+1)=X(2,i)+delt*X(4,i)*sin(X(3,i)+beta(i));%y
X(3,i+1)=X(3,i)+delt*X(4,i)/L*cos(beta(i))*tan(delta(i));%psi
X(4,i+1)=X(4,i)+delt*U(2,i);%v

Y(:,i+1)=C*X(:,i+1);
Y_(:,i+1)=Y(:,i+1)-Yd;
qv(1,:)=abs(U(2,:))-umax(2);
qdel(1,:)=abs(U(1,:))-umax(1);
Sj(1,i+1)=Rc-norm(Y(1:2,i+1)-[front_car.x; front_car.y]);
Sj(2,i+1)=Rc-norm(Y(1:2,i+1)-[adj_behind_car.x; adj_behind_car.y]);
Sj(3,i+1)=Rc-norm(Y(1:2,i+1)-[adj_front_car.x; adj_front_car.y]);
Sl(i+1)=Ro+Ru-norm(Y(1:2,i+1)-[10; 0]);
end