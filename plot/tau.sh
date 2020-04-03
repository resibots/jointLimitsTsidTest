set datafile separator ","
set autoscale fix
set key outside right center
set title "Title"
#plot "log.csv" using 1 title "time" with points
#plot for [col=94:97] "log.csv" using 0:col with points title columnheader
plot "log.csv" using x4 with linespoints title columnheader
pause -1
