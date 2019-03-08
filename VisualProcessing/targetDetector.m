function [true_distance, bearing] = targetDetector(stereo_image)

husky_id =5;
config = GetHuskyConfig(husky_id);

undistortedImage = UndistortStereoImage(stereo_image, config.camera_model);
left = undistortedImage.left.rgb;
right = undistortedImage.right.rgb;
leftEllipse = findEllipse(left);
rightEllipse = findEllipse(right);
leftOffset = leftEllipse(2)-256;
rightOffset = rightEllipse(2)-256;
disparity = leftOffset - rightOffset;
distance = 113.02545388436575 * disparity^-1.1373303662396406 ;
if distance < Inf
    true_distance = sqrt(abs(distance^2 - 0.75^2));
    bearing1 = atand(2*leftOffset*tand(50)/512);
    bearing2 = atand(2*rightOffset*tand(50)/512);
    bearing = (bearing1 + bearing2)/2;
else
    true_distance = Inf;
    bearing = Inf;
end