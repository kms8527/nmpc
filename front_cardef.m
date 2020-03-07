function [car,car_img]=front_cardef(width)

car.v=3;
car.a=0;
car.width=1.8;
car.L=4.57;
car.x=50;
car.y=1/2*width;
car_img=fill([car.x,car. x-car.L, car.x-car.L ,car.x], [[car.width/2, car.width/2, -car.width/2, -car.width/2] + car.y],'g');