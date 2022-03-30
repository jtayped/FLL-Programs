# FLL-Programs
CARGO CONNECT 2021-22 "EPIC LEGO BULLS" ROBOT_GAME

This is an FLL program made by the "Epic Lego Bulls" robot team from the province of Lleida.
With this program "loop.py" we can do 385 points on avarage.
__________________________________________________________________________________________________________

NOTES:
Many of the speed values in the funcions are converted to negative numbers, this is because our robot's
motors are turned so a negative speed makes the robot go forward same thing for going backwards.

Some of the target_angles are adjusted for error, very common in run3 because the complement is extremely
front heavy.

In the loop for the robot game, the IFs are stacked, this is due to the firmware of the robot, it can't 
do ANDs or ORs, also apart from that it can't use threading (being able to do 2 things or more at once).

Some of the functions specified in the Movement class are not used, but our team tested them extensively
in the simulations we mention in our robot presentation (https://gears.aposteriori.com.sg/) a very
useful tool for other teams that program in blocks or in python code.
__________________________________________________________________________________________________________

I hope this has helped understand the code a bit more, because this year is the first year we have ever
programmed in python with our robot. I'd like to wish the best of luck to all the teams that compete this
year and stay safe!

