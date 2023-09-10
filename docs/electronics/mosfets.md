# N-channel MOSFETs

We need 4xMOSFET transistors also known as switches and used together with PWM to supply the current to the motors in peaks. Choosing them might be tricky and there are a couple of important points when choosing one for our application:

- Maximum drain current it can supply (Id max), which in this case should be around 3A to support the motor which can draw around 2.75A.
- The Vgs threshold voltage, which has to be low, maybe somewhere around 1V as li-po batteries voltage might drop to 3.4V at some point. It is always best to also check the drain current (Id) dependency on the threshold voltage (Vgs threshold) as each transistor will have a different response.
- This is provided under certain gate to sink bias Vgs. Normally expect this to be around 0.032Ohm, however the smaller the better. If you choose some MOSFET with Rds (on) equal to 0.3Ohm or so, due to motors running at 3A the voltage drop over the MOSFET will be 0.9V, thus motors will not get sufficient voltage drop over them.
- I have used si 2302 n-channel mosfets in my circuit as it fulfilled all these requirements.