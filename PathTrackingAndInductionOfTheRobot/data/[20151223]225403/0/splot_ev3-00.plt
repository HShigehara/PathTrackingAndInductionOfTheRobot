set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "PointCloud Plane(LSM) Centroid"
splot "dof6-00.dat" pointsize 5,0.174723*x+2.206665*y+1002.242188,"point-00.dat" every 5
