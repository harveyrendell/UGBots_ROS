define workerlaser ranger
(
  sensor(
  range [ 0.0 30.0 ] # range
  fov 180       # field of vision
  samples 180   # samples 
  )
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
  
)

define worker position
(
  size [0.5 0.5 1.8]
  drive "diff"
  velocity_bounds [ -10 10 -10 10 -10 10 -90 90 ]

  #body
  block(
	color "red30"
  	points 4
	point[0] [0.1 0.3]
	point[1] [0.9 0.3]
	point[2] [0.9 0.7]
	point[3] [0.1 0.7]
	z [0.8 1.4]
  )

  #neck
  block(
	color "SandyBrown"
	points 4
	point[0] [0.4 0.4]
	point[1] [0.6 0.4]
	point[2] [0.6 0.6]
	point[3] [0.4 0.6]
	z [1.4 1.5]
  )

  #head
  block(
	color "SandyBrown"
	points 4
	point[0] [0.3 0.35]
	point[1] [0.7 0.35]
	point[2] [0.7 0.65]
	point[3] [0.3 0.65]
	z [1.5 1.7]
  )

  #legs
  block(
	color "blue"
	points 4
	point[0] [0.25 0.4]
	point[1] [0.45 0.4]
	point[2] [0.45 0.6]
	point[3] [0.25 0.6]
	z [0 0.8]
  )

  block(
	color "blue"
	points 4
	point[0] [0.55 0.4]
	point[1] [0.75 0.4]
	point[2] [0.75 0.6]
	point[3] [0.55 0.6]
	z [0 0.8]
  )

  #arms
  block(
	color "SandyBrown"
	points 4
	point[0] [0 0.45]
	point[1] [0.1 0.45]
	point[2] [0.1 0.55]
	point[3] [0 0.55]
	z [0.7 1.4]
  )
  block(
	color "SandyBrown"
	points 4
	point[0] [1 0.45]
	point[1] [0.9 0.45]
	point[2] [0.9 0.55]
	point[3] [1 0.55]
	z [0.7 1.4]
  )

  #hat
  block(
	color "brown"
	points 4
	point[0] [0.1 0.3]
	point[1] [0.9 0.3]
	point[2] [0.9 0.7]
	point[3] [0.1 0.7]
	z [1.7 1.72]
  )

  block(
	color "brown"
	points 4
	point[0] [0.3 0.4]
	point[1] [0.7 0.4]
	point[2] [0.7 0.6]
	point[3] [0.3 0.6]
	z [1.72 1.8]
  )
  workerlaser(pose [ 0.050 0.000 -1 0.000 ])
)


