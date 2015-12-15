set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "centroid.dat" pointsize 5,0.199043*x+-0.001109*y+57.415161,"pointcloud.dat" every 5
