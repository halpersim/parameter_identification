function [Y_bar] = Y_bar_from_trajectory(a, b, duration, func, param_robot)
    w = 2 * pi / duration;

    dim = size(a);

    zero = zeros(dim(1), 1);

    s = size(func(zero, zero, zero, param_robot));
    
    Y_bar = zeros(dim(1)*1000, s(2));

    for m = 1:duration*100
        t = (m-1)/100;

        q = zero;
        qp = zero;
        qpp = zero;

        for i = 1:dim(1)
            for l = 1:dim(2)
                q(i) = q(i) + a(i,l) * sin(w * l * t) / (w*l) - b(i,l)*cos(w*l*t)/(w*l);
                qp(i) = qp(i) + a(i,l) * cos(w * l * t) + b(i,l)*sin(w*l*t);
                qpp(i) = qpp(i) - a(i,l) * sin(w * l * t) * (w*l) + b(i,l)*cos(w*l*t) * (w*l);
            end
        end

        Y_bar(((m-1)*7)+1 : m*7, :) = func(q, qp, qpp, param_robot);
    end
end