set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "centroid.dat" pointsize 5,0.102802*x+-0.037606*y+785.263855,"pointcloud.dat" every 5
