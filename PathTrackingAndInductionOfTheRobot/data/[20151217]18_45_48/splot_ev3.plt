set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "centroid.dat" pointsize 5,-1.#IND00*x+-1.#IND00*y+-1.#IND00,"pointcloud.dat" every 5
