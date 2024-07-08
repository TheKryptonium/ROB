function y=taf(t,tf,ths)
y=zeros(length(t),1);
    for i=1:length(t)
        if(t(i)<tf)
            y(i)=ths*((t(i)/tf)-((sin((2*pi*t(i))/tf))/(2*pi)));
        else
            y(i)=ths;
        end
    end