define mylaser ranger
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

define animal position
(
  size [0.3 0.5 0.5]
  drive "diff"
  
  #body
  block(
	points 4
	point[0] [0 0]
	point[1] [1 0]
	point[2] [1 0.65]
	point[3] [0 0.65]
	z [0.15 0.4]
  )
________________________________
  #head
  block(
	points 4
	point[0] [0.3 0.65]
	point[1] [0.7 0.65]
	point[2] [0.7 0.95]
	point[3] [0.3 0.95] 
	z [0.35 0.5]
  )
 ________________________________ 
  #snout
  block(
	points 4
	point[0] [0.42 0.95]
	point[1] [0.58 0.95]
	point[2] [0.58 1]
	point[3] [0.42 1] 
	z [0.38 0.47]
  )
_________________________________ 
  #ears
  block(
	points 4
	point[0] [0.4 0.75]
	point[1] [0.42 0.75]
	point[2] [0.42 0.95]
	point[3] [0.3 0.95] 
	z [0.35 0.5]
  )
  block(
	points 4
	point[0] [0.3 0.65]
	point[1] [0.7 0.65]
	point[2] [0.7 0.95]
	point[3] [0.3 0.95] 
	z [0.35 0.5]
  )
_________________________________ 
  #legs
   block(
	points 4
	point[0] [0.1 0.1]
	point[1] [0.15 0.1]
	point[2] [0.15 0.15]
	point[3] [0.1 0.15]
	z [0 0.25]
  )
   block(
	points 4
	point[0] [0.85 0.1]
	point[1] [0.9 0.1]
	point[2] [0.9 0.15]
	point[3] [0.85 0.15]
	z [0 0.25]
  )
   block(
	points 4
	point[0] [0.1 0.55]
	point[1] [0.15 0.55]
	point[2] [0.15 0.6]
	point[3] [0.1 0.6]
	z [0 0.25]
  )
   block(
	points 4
	point[0] [0.85 0.55]
	point[1] [0.9 0.55]
	point[2] [0.9 0.6]
	point[3] [0.85 0.6]
	z [0 0.25]
  )


  mylaser(pose [ 0.050 0.000 0 0.000 ])
)

