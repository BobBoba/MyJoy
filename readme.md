# What is it and why I build it?

I had had very old Logitech WingMan Extreme joystick for many years and after a while I got a noise on X and Y axis. It was very annoying to aim. Furhtermore it had very low resolution. I guess it had 8-bits ADC. So, I realized to build new firmware for STM32f103C8T6 I bought for $1.5.

For ADC I choose 16 bits ADS1115 with 4 channels and connect to STM32 using I2C bus.

Project for STM32CubeMX included.

Joystick has 4 axis, 4 position POV hat, 5 + 2 buttons.

Most important feature is that I use SS496A hall sensors attached directly to the axis potentiometers.

So now I have very preciuos aiming, with up to 2^16 steps for each axis.