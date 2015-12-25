set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "dof6-00.dat" pointsize 5,-1.#IND00*x+-1.#IND00*y+-1.#IND00,"point-00.dat" every 5
