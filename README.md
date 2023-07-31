This code has been used to convert a pressurrstat-controlled HX espresso machine
(Profitec 500) to custom PID control using an ESP32 microcontroller, an ADS1X15
ADC I2C board, a thermistor temperature probe, an SSD and an opto coupler.
The ADC is used to read the voltage from the thermistor circuit in differential
mode, the SSD drives the boiler's element, and the opto coupler replaces the
pressurestat and act as a kill switch if the SSD becomes defective.

The custom PID algorithm developed for this project controls a custom PWM
algorithm. The PWM algorithm has been optimised for the low frequency allowed by
AC and adjusts both the phase and the pulse period so that the output's accuracy is
maximized. The custom PID algorithm includes several features to prevent integral
windup and to optimize the response when the regime of the espresso machine
changes, such as when the pump turns on to extract a shot, or when either steam
or hot water is drawn.

The PID algorithm features include:
-Integral windup prevention
-Integral deadband
-Automatic derivative disabling, and switching to a Taylor series based control
algorithm when a large regime change is detected (to improve response, minimise
overshooting and undershooting)

The integral windup prevention algorithm uses an integral estimate based on a
recent average output calculation to reset the integral term, when appropriate.
