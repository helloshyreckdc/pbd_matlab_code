function visual_servo_callback(~, ~)

global sr300_data record_image_template
record_image_template = rosparam('get','/record_image_template');
persistent image_template

if isempty(image_template)
    image_template = sr300_data;
end

if record_image_template
    image_template = sr300_data;
    record_image_template = false;
end




end