define visitorlaser ranger
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

define visitor position
(
  size [0.5 0.5 1.6]
  drive "diff"

  #body
  block(
    color "pink"
  	points 4
	point[0] [0.1 0.3]
	point[1] [0.9 0.3]
	point[2] [0.9 0.7]
	point[3] [0.1 0.7]
	z [0.8 1.4]
  )

  #neck
  block(
    color "white"
	points 4
	point[0] [0.4 0.4]
	point[1] [0.6 0.4]
	point[2] [0.6 0.6]
	point[3] [0.4 0.6]
	z [1.4 1.5]
  )

  #head
  block(
    color "white"
	points 4
	point[0] [0.3 0.35]
	point[1] [0.7 0.35]
	point[2] [0.7 0.65]
	point[3] [0.3 0.65]
	z [1.5 1.7]
  )

  #legs
  block(
    color "black"
	points 4
	point[0] [0.25 0.4]
	point[1] [0.45 0.4]
	point[2] [0.45 0.6]
	point[3] [0.25 0.6]
	z [0 0.8]
  )

  block(
    color "black"
	points 4
	point[0] [0.55 0.4]
	point[1] [0.75 0.4]
	point[2] [0.75 0.6]
	point[3] [0.55 0.6]
	z [0 0.8]
  )

  #arms
  block(
    color "pink"
	points 4
	point[0] [0 0.45]
	point[1] [0.1 0.45]
	point[2] [0.1 0.55]
	point[3] [0 0.55]
	z [0.7 1.4]
  )
  block(
    color "pink"
	points 4
	point[0] [1 0.45]
	point[1] [0.9 0.45]
	point[2] [0.9 0.55]
	point[3] [1 0.55]
	z [0.7 1.4]
  )
  visitorlaser(pose [ 0.050 0.000 -1 0.000 ])
)

