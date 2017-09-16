function [x, y, z] = diamondLinear(t, p, t_int)
if t <= 4 * t_int
    x = 0.25 * t;
else
    x = 0;
end

if t <= t_int
    y = (p / t_int) * t;
elseif t > t_int && t <= 3 * t_int
    y = p - p * ((t - t_int) / t_int);
elseif t > 3 * t_int && t <= 4 * t_int
    y = -p + p * ((t - 3*t_int) / t_int);
else
    y = 0;
end

if t <= 2 * t_int
    z = (p / t_int) * t;
elseif t > 2 * t_int && t <= 4 * t_int
    z = 2 * p * (1 - ((t - 2*t_int)/2*t_int));
else
    z = 0;
end

end