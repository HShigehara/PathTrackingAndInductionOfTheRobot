set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "centroid.dat" pointsize 5,-0.058499*x+-0.037331*y+450.877319,"pointcloud.dat" every 5
