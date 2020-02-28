MAE 144 UCSD 2019 FALL
Last Updated: 12/13/19

Author:Justin Chang
Groupmates: Basil Hu, Khang Nguyen


The following code controls the Mobile Inverted Pendulum (MiP). The controller design is
split into two halves: the outer loop and the inner loop. The inner loop controls the stability
of the MiP's body angle from the microcontroller's duty cycle. The outer loop controls the
position of the MiP so that it doesn't wander around in space. Gains of the controller may be changed
accordingly by changing the k1 and k2 which controls the gains of the inner and the outer loop respectively.

Hardware Required:
BeagleBone Blue
MiP Kit Assembled

How to use:
The following files should be in the folder:

Chang_Justin_Final.c
MakeFile
README

In the same directory, execute 'make' which makes the header and .o files. Then an executable
Chang_Justin_Final should be generated.

Execute "./Chang_Justin_Final" and the program should automatically run.

After executing, hold the MiP upright until the program finishes calibrating. The MiP will start to 
control and move itself with the motors turned on. Afterwards, let the MiP go and it will balance itself.
Ways to stop the program:
Either exiting through Ctrl-C through the terminal or long press the LED button for at least
on the BeagleBone. I also programmed the BeagleBone to stop once the body angle is over 1 radian
or around 58 degrees.

