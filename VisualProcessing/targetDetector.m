function [distance, bearing] = targetDetector(stereo_image)

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
distance = 100.07482321516153 * disparity^-1.1292506152853168;
if distance < Inf && disparity > 0 || leftOffset ~= -256 || rightOffset ~= 256
    bearing1 = atand(2*leftOffset*tand(50)/512);
    bearing2 = atand(2*rightOffset*tand(50)/512);
    bearing = (bearing1 + bearing2)/2;
else
    bearing = Inf;
end