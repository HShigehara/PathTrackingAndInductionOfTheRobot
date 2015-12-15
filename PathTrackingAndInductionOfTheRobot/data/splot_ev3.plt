set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "centroid.dat" pointsize 5,0.120688*x+-0.033798*y+68.114395,"pointcloud.dat" every 5
