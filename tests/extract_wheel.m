
% load('6_feb_test_3')

len_data_out = length(wheel_od);
xytheta = zeros(3, len_data_out);

for i = 1 : len_data_out
    
    inter_wheel = wheel_od{i};
    
    xytheta(1, i) = inter_wheel.x;
    xytheta(2, i) = inter_wheel.y;
    xytheta(3, i) = inter_wheel.yaw;
    
    
end

figure
plot(xytheta(1, :))