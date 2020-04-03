x1=94 #column pos number  arm left
x2=95 #column vel number  arm left
x3=96 #column acc number  arm left
x4=97 #column tau number  arm left
# x1=122 #column pos number  arm right
# x2=123 #column pos number  arm right
# x3=124 #column pos number  arm right
# x4=125 #column pos number  arm right

sed -i '/plot "log.csv" using x1 with linespoints title columnheader/c\plot "log.csv" using '${x1}' with linespoints title columnheader' pos.sh
sed -i '/plot "log.csv" using x2 with linespoints title columnheader/c\plot "log.csv" using '${x2}' with linespoints title columnheader' vel.sh
sed -i '/plot "log.csv" using x3 with linespoints title columnheader/c\plot "log.csv" using '${x3}' with linespoints title columnheader' acc.sh
sed -i '/plot "log.csv" using x4 with linespoints title columnheader/c\plot "log.csv" using '${x4}' with linespoints title columnheader' tau.sh
gnome-terminal -e "gnuplot pos.sh"
gnome-terminal -e "gnuplot vel.sh"
gnome-terminal -e "gnuplot acc.sh"
gnome-terminal -e "gnuplot tau.sh"
sed -i '/plot "log.csv" using '${x1}' with linespoints title columnheader/c\plot "log.csv" using x1 with linespoints title columnheader' pos.sh
sed -i '/plot "log.csv" using '${x2}' with linespoints title columnheader/c\plot "log.csv" using x2 with linespoints title columnheader' vel.sh
sed -i '/plot "log.csv" using '${x3}' with linespoints title columnheader/c\plot "log.csv" using x3 with linespoints title columnheader' acc.sh
sed -i '/plot "log.csv" using '${x4}' with linespoints title columnheader/c\plot "log.csv" using x4 with linespoints title columnheader' tau.sh

echo "Done"
