set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "centroid.dat" pointsize 5,-1.918457*x+-0.253616*y+2.769531,"pointcloud.dat" every 5
