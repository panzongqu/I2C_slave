# Preface
 Please do not waster your time on this code except following case:
* Yout must implement a [I2C](https://en.wikipedia.org/wiki/I%C2%B2C) slave device even thought target chip not I2C slave peripheral.
* Add I2C slave device feature for OS such as Linux.

# Introduction
Using I2C SDA pin as EXTI intrrupt source and detect I2C SCL/SDA pin for I2C protocol analyzers
I2C Speed depend on CPU main frequency
I2C clock stretching unsupport
CPU killer and low efficiency and the code not be tested yet

# Lisence
[WTFPL](https://en.wikipedia.org/wiki/WTFPL)