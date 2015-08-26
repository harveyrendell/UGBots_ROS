define catlaser ranger
(
  sensor(
  range [ 0.0 3.0 ] # range
  fov 30       # field of vision
  samples 30   # samples 
  )
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
  
)

define cats position
(
  size [0.3 0.65 0.4]
  drive "diff"
  
  #body
  block(
    color "white"
	points 4
	point[0] [0 0.2]
	point[1] [1 0.2]
	point[2] [1 0.65]
	point[3] [0 0.65]
	z [0.1 0.3]
  )

  #head
  block(
    color "white"
	points 4
	point[0] [0.1 0.65]
	point[1] [0.9 0.65]
	point[2] [0.9 0.9]
	point[3] [0.1 0.9] 
	z [0.15 0.35]
  )
  #snout
  block(
    color "grey"
	points 4
	point[0] [0.28 0.9]
	point[1] [0.72 0.9]
	point[2] [0.72 1]
	point[3] [0.28 1] 
	z [0.17 0.25]
  )
  #ears
  block(
    color "grey"
	points 4
	point[0] [0.35 0.65]
	point[1] [0.45 0.65]
	point[2] [0.45 0.75]
	point[3] [0.35 0.75] 
	z [0.35 0.4]
  )
  block(
    color "grey"
	points 4
	point[0] [0.55 0.65]
	point[1] [0.65 0.65]
	point[2] [0.65 0.75]
	point[3] [0.55 0.75] 
	z [0.35 0.4]
  ) 
  #legs
   block(
    color "grey"
	points 4
	point[0] [0 0.2]
	point[1] [0.2 0.2]
	point[2] [0.2 0.3]
	point[3] [0 0.3]
	z [0 0.1]
  )
   block(
    color "grey"
	points 4
	point[0] [0.8 0.2]
	point[1] [1 0.2]
	point[2] [1 0.3]
	point[3] [0.8 0.3]
	z [0 0.1]
  )
   block(
    color "grey"
	points 4
	point[0] [0 0.55]
	point[1] [0.2 0.55]
	point[2] [0.2 0.65]
	point[3] [0 0.65]
	z [0 0.1]
  )
   block(
    color "grey"
	points 4
	point[0] [0.8 0.55]
	point[1] [1 0.55]
	point[2] [1 0.65]
	point[3] [0.8 0.65]
	z [0 0.1]
  )
  #tail
  block(
    color "white"
	points 4
	point[0] [0.4 0]
	point[1] [0.6 0]
	point[2] [0.6 0.2]
	point[3] [0.4 0.2]
	z [0.25 0.3]
  )
  block(
    color "grey"
	points 4
	point[0] [0.4 0]
	point[1] [0.6 0]
	point[2] [0.6 0.1]
	point[3] [0.4 0.1]
	z [0.3 0.4]
  )

  catlaser(pose [ 0.050 0.000 0 0.000 ])
)

