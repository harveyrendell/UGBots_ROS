define anilaser ranger
(
  sensor(
  range [ 0.0 7.0 ] # range
  fov 180       # field of vision
  samples 180   # samples 
  )
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
  
)

define dog position
(
  size [0.3 0.6 0.5]
  drive "diff"
  
  #body
  block(
	points 4
	point[0] [0.05 0]
	point[1] [0.95 0]
	point[2] [0.95 0.6]
	point[3] [0.05 0.6]
	z [0.1 0.4]
  )

  #head
  block(
	points 4
	point[0] [0.1 0.6]
	point[1] [0.9 0.6]
	point[2] [0.9 0.9]
	point[3] [0.1 0.9] 
	z [0.2 0.5]
  )
  #snout
  block(
	points 4
	point[0] [0.28 0.9]
	point[1] [0.72 0.9]
	point[2] [0.69 1]
	point[3] [0.25 1] 
	z [0.22 0.35]
  )
  #ears
  block(
	points 4
	point[0] [0 0.6]
	point[1] [0.1 0.6]
	point[2] [0.1 0.9]
	point[3] [0 0.9] 
	z [0.3 0.5]
  )
  block(
	points 4
	point[0] [0.9 0.6]
	point[1] [1 0.6]
	point[2] [1 0.9]
	point[3] [0.9 0.9] 
	z [0.3 0.5]
  ) 
  #legs
   block(
	points 4
	point[0] [0.05 0]
	point[1] [0.25 0]
	point[2] [0.25 0.1]
	point[3] [0.05 0.1]
	z [0 0.1]
  )
   block(
	points 4
	point[0] [0.05 0.5]
	point[1] [0.25 0.5]
	point[2] [0.25 0.6]
	point[3] [0.05 0.6]
	z [0 0.1]
  )
   block(
	points 4
	point[0] [0.75 0]
	point[1] [0.95 0]
	point[2] [0.95 0.1]
	point[3] [0.75 0.1]
	z [0 0.1]
  )
   block(
	points 4
	point[0] [0.75 0.5]
	point[1] [0.95 0.5]
	point[2] [0.95 0.6]
	point[3] [0.75 0.6]
	z [0 0.1]
  )
  #tail
  block(
	points 4
	point[0] [0.4 0]
	point[1] [0.6 0]
	point[2] [0.6 0.1]
	point[3] [0.4 0.1]
	z [0.4 0.5]
  )
  anilaser(pose [ 0.050 0.000 0 90.000 ])
)

