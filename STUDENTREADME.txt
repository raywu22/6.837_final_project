Build Instructions (Athena with Makefile)
==================
$ cd build
$ cmake ..
$ make

Collaborators:
Or Oppenheimer
Discussed how takeStep should work, how to display blue ball at different positions.

References:
Lecture slides for equations.

Extra Credit:
None

Comments:
The math for this assignment was pretty straightforward (given equations from slides). Tuning the parameters took a bit of trial and error to find something that looked good. Overall, in terms of difficulty, I'd say this one was the easiest of the graded assignments, and the first one was the hardest.

It wasn't entirely clear to me what was the intent behind randomization. Did you want initial positions randomized? Initial velocities? Spring constants, or other constants? In the end, I decided to only randomize the initial position of the balls in the multi-particle chain. Since the spring constants and rest lengths were consistent for all springs, I didn't end up having an array containing spring data.
