set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "dof6-03.dat" pointsize 5,0.068100*x+0.171631*y+2441.062500,"point-03.dat" every 5
