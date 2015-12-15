set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "centroid.dat" pointsize 5,-0.090546*x+0.077362*y+1098.710938,"pointcloud.dat" every 5
