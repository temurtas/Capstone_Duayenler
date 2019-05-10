***compile***
gcc double_tof.c -o double_tof -lvl6180_pi -lwiringPi

***BAĞLANTILAR***
gpio0 (pin11) sensor 1'in CE'sine (ön)
gpio1 (pin12) sensor 2'in CE'sine (arka)
sensörlerin vin'i 5V'e; scl,sda ve gnd ilgili yerlere
