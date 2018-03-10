% %------Washington dataset
% function dataProcessing
%     inputFile = '/home/hhg/Downloads/dataSet/RGBD/3D_Object_Reconstruction/RGBD object/rgbd-dataset/cell_phone/cell_phone_1';
%     outputFile = './input/cell_phone/data';
%     for i = 1:190
%         d = imread(sprintf('%s/cell_phone_1_1_%d_depth.png',inputFile,i));
%         c = imread(sprintf('%s/cell_phone_1_1_%d.png',inputFile,i));
%         mask = imread(sprintf('%s/cell_phone_1_1_%d_mask.png',inputFile,i));
%         
%         d_output = double(d) .* double(mask);
%         
%         imwrite(uint16(d_output),sprintf('%s/ppmpgm/d_%d.pgm',outputFile,i));
%         imwrite(uint16(d_output),sprintf('%s/png/d_%d.png',outputFile,i));
%         
%         imwrite(uint8(c),sprintf('%s/ppmpgm/c_%d.ppm',outputFile,i));
%         imwrite(uint8(c),sprintf('%s/png/c_%d.png',outputFile,i))
%         disp(i);
%     end
% end

%------UCB dataset
function dataProcessing
    inputFile = '/home/hhg/Downloads/dataSet/RGBD/3D_Object_Reconstruction/RGBD object/UCB/detergent/detergent';
    outputFile = './input/detergent/data';
    
    H_NP1_from_NP5 = h5read(sprintf('%s/calibration.h5',inputFile),'/H_NP1_from_NP5');
    H_NP1_ir_from_NP5 = h5read(sprintf('%s/calibration.h5',inputFile),'/H_NP1_ir_from_NP5');
    NP1_depth_K = h5read(sprintf('%s/calibration.h5',inputFile),'/NP1_depth_K');
%     NP1_depth_bias = h5read(sprintf('%s/calibration.h5',inputFile),'/NP1_depth_bias');
%     NP1_depth_scale = h5read(sprintf('%s/calibration.h5',inputFile),'/NP1_depth_scale');
%     NP1_ir_d = h5read(sprintf('%s/calibration.h5',inputFile),'/NP1_ir_d');
    NP1_ir_K = h5read(sprintf('%s/calibration.h5',inputFile),'/NP1_ir_K');
    
%     extRec = inv(H_NP1_ir_from_NP5)*H_NP1_from_NP5;  %error
%     extRec = inv(H_NP1_from_NP5)*H_NP1_ir_from_NP5; %error

    rgb2d = [0.999749,0.00518867,0.0217975,0.0243073;
-0.0051649,0.999986,-0.0011465,-0.000166518;
-0.0218031,0.00103363,0.999762,0.0151706;
0,0,0,1];
      d2rgb = inv(rgb2d);
      R = d2rgb(1:3,1:3); t = d2rgb(1:3,4);
       
%     extRec_invt = extRec';
%     R = extRec_invt(1:3,1:3); t = extRec_invt(1:3,4);
    
    for i = 1:120
        c_fname = imread(sprintf('%s/NP1_%d.jpg',inputFile,(i-1)*3));
        d_fname = h5read(sprintf('%s/NP1_%d.h5',inputFile,(i-1)*3),'/depth'); %need to be inverted
        d_fname = double(d_fname');
        d_fname = d_fname ./ 10; %mm
        mask = imread(sprintf('%s/masks/NP1_%d_mask.pbm',inputFile,(i-1)*3));
        mask_resz = imresize(mask,[480,640])==0;
        c = imresize(c_fname,[480,640]);
        c_y = rgb2ycbcr(c);
        
        [H,W] = size(d_fname);
%         aligned_depth = zeros(size(c_fname(:,:,1)));
        aligned_depth = zeros(size(d_fname));
        [H_align, W_align] = size(aligned_depth);
        for u = 1:W
            for v = 1:H
                d = d_fname(v,u);
                if d==0, continue; end
                
                xyz_d(1) = (u-NP1_depth_K(3,1))*d/NP1_depth_K(1,1);
                xyz_d(2) = (v-NP1_depth_K(3,2))*d/NP1_depth_K(2,2);
                xyz_d(3) = d;
                
                %---transform from depth to color coo
                xyz_c = R*xyz_d'+t;
                
                uvd_c(1) = xyz_c(1)*NP1_depth_K(1,1)/xyz_c(3)+NP1_depth_K(3,1);
                uvd_c(2) = xyz_c(2)*NP1_depth_K(2,2)/xyz_c(3)+NP1_depth_K(3,2);
                uvd_c(3) = xyz_c(3);
                
                uvd_c(1) = round(uvd_c(1));
                uvd_c(2) = round(uvd_c(2));
                
                if uvd_c(1) > 0 && uvd_c(1) <= W_align && uvd_c(2) > 0 && uvd_c(2) <= H_align && uvd_c(3)>0 
                    aligned_depth(uvd_c(2),uvd_c(1)) = uvd_c(3);
                end
            end
        end
        
%         figure(66),imshow(aligned_depth,[])
%         
%         I(:,:,1) = mat2gray(aligned_depth>0)*255;
%         I(:,:,2) = mask_resz*255;
%         I(:,:,3) = c(:,:,1);
%         figure(55),imshow(uint8(I));
%         
%         d_mask = double(aligned_depth) .* double(mask_resz);
%         figure(66),imshow(d_mask,[]);

        output_d = double(aligned_depth) .* double(mask_resz);
        output_c = c;
        
        imwrite(uint16(output_d),sprintf('%s/ppmpgm/d_%d.pgm',outputFile,i));
        imwrite(uint16(output_d),sprintf('%s/png/d_%d.png',outputFile,i));
        
        imwrite(uint8(output_c),sprintf('%s/ppmpgm/c_%d.ppm',outputFile,i));
        imwrite(uint8(output_c),sprintf('%s/png/c_%d.png',outputFile,i))
        disp(i);
    end
end


function getAlignedDepth()
    
end
