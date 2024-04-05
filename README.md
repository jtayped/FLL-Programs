# FLL-Programs
CARGO CONNECT 2021-22 "EPIC LEGO BULLS" ROBOT_GAME

This is an FLL program made by the "Epic Lego Bulls" robot team from the province of Lleida.
With this program "loop.py" we can do 385 points on average, which classified us for the finals in Torremolinos.
__________________________________________________________________________________________________________

NOTES:
All of the speed values in the functions are converted to negative numbers, this is because our robot's
motors are turned so a negative speed makes the robot go forward the same thing as going backwards.

Some of the target_angles are adjusted for error, very common in run3 because the compliment is extremely
front heavy.

In the loop for the robot game, the IFs are stacked, this is due to the firmware of the robot, it can't 
do ANDs or ORs, apart from that it can't use threading.

Some of the functions specified in the Movement class are not used, but our team tested them extensively
in simulations (https://gears.aposteriori.com.sg/).
__________________________________________________________________________________________________________

I hope this has helped you understand the code a bit more because this year is the first year we have ever
programmed in Python with our robot. I'd like to wish the best of luck to all the teams that compete this
year!

