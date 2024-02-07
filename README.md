# 3-Axis-Motion-Platform
A simple to build motion platform with a payload of about 150kg

While I was working on a Movie (SFX), there was a requirement for a table to rise into the air (magically) and wobble around.  We didn't have a simple solution to hand and in the end, VFX was used to animate the table.  However, I kept thinking about it!  After the movie production was finished, I built this in my own time.  I figured it could be useful for movies, but it could potentially form the basis of a home flight simulator too.

It's powered by 3 x NEMA23 Stepper motors with Drivers runniing on 48v. The steppers are coupled to 30:1 worm drive reductions. It's all driven by an ESP32 microcontroller (code attached).

I built it largely out of parts I had laying around.  If I were re-building it for a movie, I would use Hybrid Steppers & drives which have an encoder built in.  You can push these much harder without it loosing position, potentially increasing the load capacity significantly!  Without changing the design, the maximum length for the motors is 115mm

You'll need Solidworks 2023 to open the files.  Most laser cutters who can fold material too will accept Solidworks files natively.  If you're in the UK, I'd recommend LGM or Cirrus Laser.

The best place to buy the Worm Reduction boxes is Ali-Express.  Search for: "Worm Reducer NMRV030 11mm Input Shaft RV030 Worm Gearbox for NEMA 23 stepper motor" (https://www.aliexpress.com/item/32743752182.html)

I bought the motors, reductions and drivers altogether https://www.aliexpress.com/item/4000234604090.html

A reduction ratio of 30:1 will give you 100kg lift with high speed movement and reasonable dynamic load capability.  Increase the ratio if you require higher lift or dynamic loads.
The mechanism is strong enough to support 300kg with some factor of safety.  More if the components are welded together.
In addition to the NMRV30 reductions, you need to order three "Dual Output Shaft RV030". (https://www.aliexpress.com/item/1005002828888915.html)  This is a keyed shaft which engages with the laser cut components.


