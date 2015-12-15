set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "centroid.dat" pointsize 5,-0.021879*x+0.485281*y+0.286171,"pointcloud.dat" every 5
