set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "centroid.dat" pointsize 5,-1.940914*x+-0.224388*y+6.318726,"pointcloud.dat" every 5
