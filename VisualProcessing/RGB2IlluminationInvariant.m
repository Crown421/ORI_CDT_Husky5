function [ ii_image ] = RGB2IlluminationInvariant( image, alpha )
ii_image = 0.5 + log(image(:,:,2)) - alpha*log(image(:,:,3)) - (1-alpha)*log(image(:,:,1));