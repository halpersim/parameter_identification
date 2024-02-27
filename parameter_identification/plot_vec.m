% helper function that plots (n x 7) or (7 x n) double vectors
function plot_vec(vec, range, channels)

    if ~exist('channels', 'var')
        channels = 1:7;
    end

    if ~exist('range', 'var')
        range = 1:max(size(vec));
    end

    
    v_size = size(vec);
    ss = size(v_size);

    if ss(2) == 3         
        v = zeros(v_size(1), v_size(3));
        v(:, :) = vec(:, 1, :);
    elseif v_size(1) > v_size(2)
            v = permute(vec, [2 1]);
         else
            v = vec;
    end
    
    figure;
    for i = 1:7
        if any(channels == i) 
            %stem(v(i, range), 'LineStyle', 'none', 'DisplayName', int2str(i));
            plot(v(i, range));
            hold on;
        end
    end
    legend( num2cell(int2str(permute(channels, [2 1]))));
    hold off;
end