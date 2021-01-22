function limited_speed = limit_speed(original_speed,min_speed,max_speed)
if original_speed > max_speed
    limited_speed = max_speed;
elseif original_speed < -max_speed;
    limited_speed = -max_speed;
else
    limited_speed = original_speed;
end

if abs(limited_speed) < min_speed
    limited_speed = 0;
end
end