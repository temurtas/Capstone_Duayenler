***compile***
gcc double_tof.c -o double_tof -lvl6180_pi -lwiringPi

***BAĞLANTILAR***
gpio0 sensor 1'in CE'sine
gpio1 sensor 2'in CE'sine
sensörlerin vin'i 3.3'e; scl,sda ve gnd ilgili yerlere
