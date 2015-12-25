set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "dof6-00.dat" pointsize 5,-0.164696*x+2.635986*y+1120.937500,"point-00.dat" every 5
