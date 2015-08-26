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
  size [0.4 0.8 0.7]
  drive "diff"
  
  #body
  block(
    color "orange"
	points 4
	point[0] [0.05 0]
	point[1] [0.95 0]
	point[2] [0.95 0.6]
	point[3] [0.05 0.6]
	z [0.2 0.6]
  )

  #head
  block(
    color "orange"
	points 4
	point[0] [0.1 0.6]
	point[1] [0.9 0.6]
	point[2] [0.9 0.9]
	point[3] [0.1 0.9] 
	z [0.3 0.7]
  )
  #snout
  block(
    color "brown"
	points 4
	point[0] [0.28 0.9]
	point[1] [0.72 0.9]
	point[2] [0.69 1]
	point[3] [0.25 1] 
	z [0.35 0.45]
  )
  #ears
  block(
    color "brown"
	points 4
	point[0] [0 0.6]
	point[1] [0.1 0.6]
	point[2] [0.1 0.9]
	point[3] [0 0.9] 
	z [0.4 0.7]
  )
  block(
    color "brown"
	points 4
	point[0] [0.9 0.6]
	point[1] [1 0.6]
	point[2] [1 0.9]
	point[3] [0.9 0.9] 
	z [0.4 0.7]
  ) 
  #legs
   block(
    color "orange"
	points 4
	point[0] [0.05 0]
	point[1] [0.25 0]
	point[2] [0.25 0.1]
	point[3] [0.05 0.1]
	z [0 0.2]
  )
   block(
    color "orange"
	points 4
	point[0] [0.05 0.5]
	point[1] [0.25 0.5]
	point[2] [0.25 0.6]
	point[3] [0.05 0.6]
	z [0 0.2]
  )
   block(
    color "orange"
	points 4
	point[0] [0.75 0]
	point[1] [0.95 0]
	point[2] [0.95 0.1]
	point[3] [0.75 0.1]
	z [0 0.2]
  )
   block(
    color "orange"
	points 4
	point[0] [0.75 0.5]
	point[1] [0.95 0.5]
	point[2] [0.95 0.6]
	point[3] [0.75 0.6]
	z [0 0.2]
  )
  #tail
  block(
    color "brown"
	points 4
	point[0] [0.4 0]
	point[1] [0.6 0]
	point[2] [0.6 0.1]
	point[3] [0.4 0.1]
	z [0.6 0.7]
  )
  anilaser(pose [ 0.050 0.000 0 0.000 ])
)

