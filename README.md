# Obstacle-Avoidance
The algorithm makes a depth map from a stereo cam and uses this depth map to command a UAV to change velocities to avoid obstacles in an autonomous flight.

The algorithm that I developed divides the input frame into a 5x5 grid and then assigns weights depending on the average intensity of each block. Then a replusion is calculated which is directly proportional to this weight and inversely proportional to the distance of each box from the central cell.

If the central cell has average intensity less than a particular threshold, the drone keeps moving forward. 

Otherwise the vector sum of the replusions is calculated to assign a direction of velocity to the drone. If during any time the drone comes within a particular range of an object, a reverse velocity starts being given to the drone lest there should be collision.

Various edge cases have been taken care of too like if there was a long straight wall or a symmetric figure in front of the drone, then there is a microscopic possibility that the net repulsion would add upto zero and the drone would continue moving straight. In such a case, when there is an obstacle in the middle and the net repulsions are zero, a velocity in an arbitarary direction (in the code it is up and left) is given to the drone. The negative velocity given won't let the drone crash into the wall and this velocity in the arbitarary direction would help the drone avoid the wall.
Many more edge cases were found and dealt with but the core idea is as stated above.

The algorithm was tested on actual flights with a jetson nano or, sometimes, RasberryPi as the onboard computer. I conducted about 50 flights and the drone was able to avoid all major obstacles which could be seen by the depth map.

However, since the depth map was made using a "home-made" stereocamera, the depth map wasn't too good. I would suggest to use Zed Cam or any other such depth map making sterecams if you could afford it.

