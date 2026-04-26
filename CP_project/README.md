# CP implementation for CPS

## compilation
To compile the cc files:
````
$ make
````

This creates the sdp executable file!

## execution
to run a single instance: 

````
$ ./sdp < instance.inp > out/out.txt
````

to run a range of instances:
````
$ ./run_test.sh from_range_value to_range_value
````

## note
In the sdp.cpp file, I added comments to explain each step of the process.
The main.pdf report does an overall explanation of thecnical concepts, but I hope that the code comments helps to understand my implementation!