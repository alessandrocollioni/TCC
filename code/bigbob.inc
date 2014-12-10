# bigbob.inc
# model for the robot "Bigbob"
# Author: Jennifer Owen
# Date: 16/04/2010
# Updated to Stage4 syntax 1 Aug 13 Kevin Nickels


define bigbob_one_laser sensor
(
	# define the size of each transducer [xsize ysize zsize] in meters
	size [0.05 0.05 0.07 ]
	# define the range bounds [min max]
	range [0.02 5.6]
	# define the angular field of view in degrees
	fov 240
	# define the color that ranges are drawn in the gui
	color_rgba [ 0 0 1 0.5] 
	# range of sensor
	samples 666
)

define bigbob_laser ranger
( 
	# one line for each sonar [xpos ypos zpos heading]
	bigbob_one_laser( pose [ -0.45 0 0 0])
)


define bigbob position
(
	# actual size
	size [1.25 1 1]
	# centre of rotation offset
	origin [0.125 0 0 0]
	block 

	(	 

		points 6 

		point[0] [0.75 0] 

		point[1] [1 0.25] 

		point[2] [1 0.75] 

		point[3] [0.75 1] 

		point[4] [0 1] 

		point[5] [0 0] 

		z [0 1] 

	) 

	drive "diff"
	bigbob_laser( ) 
)
