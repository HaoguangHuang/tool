# tool
各种工具

oni2picture_ed2/main.cpp: split real-time video in .oni format into a segment of depthMap and colorMap in .png/.jpg format
oni2picture_ed2/tankData/MATLAB/*.m:preprocess data(mainly extract object from depthMap and colorMap)
oni2picture_ed2/tankData/MATLAB/processColor/*.m:extract object from colorMap by depthMap mask and background mask

# master and branch
master: process data with rgb2intensity, but without subtracting fusioned color foreground from fusioned color background
branch1: process data with rgb2ycbcr, with subtracting fusioned color foreground from fusioned color background
branch1_1:add guided bilateral filter in branch1
branch_ccy:branch of branch1_1. add guided_JBF
