function [true_distance, bearing] = targetDetector(stereo_image)

husky_id =5;
config = GetHuskyConfig(husky_id);

undistortedImage = UndistortStereoImage(stereo_image, config.camera_model);
left = undistortedImage.left.rgb;
right = undistortedImage.right.rgb;
leftEllipse = findEllipse(left);
rightEllipse = findEllipse(right);
leftOffset = leftEllipse(2);
rightOffset = rightEllipse(2);
distance = config.camera_model.baseline * config.camera_model.left.fx/(leftOffset - rightOffset);
if distance < Inf
    true_distance = sqrt(distance^2 - 0.75^2);
    bearing1 = atand(2*leftOffset*tand(50)/512);
    bearing2 = atand(2*rightOffset*tand(50)/512);
    bearing = (bearing1 + bearing2)/2;
else
    true_distance = Inf;
    bearing = Inf;
end