% function for decision to calculate or not
function result=li(f,k,i)
    if f(k,i)<=0
        result=0;
    else
        result=1;
    end
end