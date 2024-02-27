%loads a given trajectory and calculates all q, qp and qpp at a frequency of 100 Hz along that trajectory
function [q, qp, qpp] = load_trajectory(trajectory_filename, duration)
    w = 2 * pi / duration;

    traj = load(trajectory_filename);

    a = traj.best_a;
    b = traj.best_b;

    dim = size(a);

    q = zeros(dim(1), duration*100 +1);
    qp = zeros(dim(1), duration*100 +1);
    qpp = zeros(dim(1), duration*100 +1);

    for m = 1:duration*100
        t = (m-1)/100;

        for i = 1:dim(1)
            for l = 1:dim(2)
                q(i,m) = q(i,m) + a(i,l) * sin(w * l * t) / (w*l) - b(i,l)*cos(w*l*t)/(w*l);
                qp(i,m) = qp(i,m) + a(i,l) * cos(w * l * t) + b(i,l)*sin(w*l*t);
                qpp(i,m) = qpp(i,m) - a(i,l) * sin(w * l * t) * (w*l) + b(i,l)*cos(w*l*t) * (w*l);
            end
        end
    end
end