set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "centroid.dat" pointsize 5,0.103691*x+-0.023670*y+1153.869873,"pointcloud.dat" every 5
