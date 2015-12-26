set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "dof6-02.dat" pointsize 5,-0.010659*x+2.163887*y+952.421875,"point-02.dat" every 5
