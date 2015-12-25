set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "dof6-00.dat" pointsize 5,0.074562*x+5.333862*y+775.812500,"point-00.dat" every 5
