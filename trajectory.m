function y=trajectory(t,tf,ths)
    if(t<tf)
        y=ths*(t/tf)-sin(2*pi*t/tf)/(2*pi);
    else
        y=ths
    end