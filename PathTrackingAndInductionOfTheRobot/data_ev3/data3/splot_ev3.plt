set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "centroid.dat" pointsize 5,1.881350*x+-0.096428*y+12.086426,"pointcloud.dat" every 5
