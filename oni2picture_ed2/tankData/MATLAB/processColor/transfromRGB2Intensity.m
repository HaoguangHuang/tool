function intensityMap = transformRGB2Intensity(RGBmap)
%     r = size(RGBmap,1);
%     c = size(RGBmap,2);
    R = double(RGBmap(:,:,1));
    G = double(RGBmap(:,:,2));
    B = double(RGBmap(:,:,3));
    intensityMap = uint8(sqrt(R.*R + G.*G + B.*B)/3);
end