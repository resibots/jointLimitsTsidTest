# Joint Limit Test 

This is a code to test tsid TaskJointPosVelAccBounds with Talos


## How to use 

You need tsid, pinnochio and gnuplot

```
mkdir build
cd build
cmake ..
make
./test-lim
```

Press enter ask for the left hand to go up

The joint position will go from -1.45 and stop at the -1.5 limit

Type end and then enter to log the results
```
cp log.csv ../plot
cd ../plot
./plot.sh
```
