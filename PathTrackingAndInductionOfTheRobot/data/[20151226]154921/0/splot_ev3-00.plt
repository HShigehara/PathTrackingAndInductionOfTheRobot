set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "dof6-00.dat" pointsize 5,0.000320*x+0.443893*y+897.234863,"point-00.dat" every 5
