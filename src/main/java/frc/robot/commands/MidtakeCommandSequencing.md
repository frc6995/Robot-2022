## Shooting
 -   Shooting should take priority over intaking. 
 -   If the robot has a ball, it should keep it ready to shoot.
     -   "Ready to shoot" means the lowest the ball can be while still breaking the top beam.
 -   If the robot is trying to shoot, it should:
     1. ensure there is a ball ready and stopped,
        -   (this check is part of the composited check for automatic shooting)
     1. do a short high-speed burst to feed it to the shooter 
         (interrupted either by clearing the top beam or after a ~0.15 s timeout)
     1. arm the midtake again while the shooter is recovering.
     1. report if and when there is another ball ready and stopped.
## Arming
 - When the midtake needs to prepare balls for shooting (e.g. after intake ends, or when intake is interrupted by a request to shoot), the midtake should stack the balls up at the top of the midtake.
    - If the top beam is broken, the midtake should reverse until it is not broken, to ensure all balls are below the top beam.
    - Once the top beam is not broken, the midtake can drive upwards until the beam is broken again, then stop. The midtake is now ready to shoot one ball.
## Idling
 - When the midtake is not otherwise in use, it should be perpetually driving up slowly while the top beam is not broken. This eventually brings any ball in the midtake up to the top beam, so that the midtake can detect it. 
    - Arming will still need to be done, but this ensures that in the idle state, the midtake can get an accurate count of its contents.
## Intake
 -   When the driver requests intake, the midtake should check its current ball count.
     -   If the midtake has two balls:
            -   (one should be ready and one should be backed up, breaking the lower beam)
            - it should stop the intake from deploying.
     -   If the midtake has one ball ready:
            - it should drive that ball down until it breaks the lower beam, then start the intake index action
         -   It can drive the ball while the intake is deploying, 
             but starting the intake spinning should be delayed until the ball breaks the beam. 
     -  If the midtake is empty:
         - the intake can start as soon as it is requested.
     - After intaking a ball, the midtake should check if it has two balls.
         - Checking for zero and one do not work while the balls are low, because the one ball would be in between the beams. With two, the top will trigger before the bottom clears, so both will be broken.
         - If both beams are broken, the intake should stop and retract.
 - When the intake stops (whether driver interruption or midtake being full), the midtake should arm.