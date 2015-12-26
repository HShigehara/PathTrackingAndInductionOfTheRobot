set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "dof6-01.dat" pointsize 5,-0.672889*x+2.619402*y+94.691406,"point-01.dat" every 5
