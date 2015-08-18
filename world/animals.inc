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

define animal position
(
  size [0.25 0.5 0.5]
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

  #head
  block(
	points 4
	point[0] [0.2 0.65]
	point[1] [0.8 0.65]
	point[2] [0.8 0.95]
	point[3] [0.2 0.95] 
	z [0.35 0.5]
  )
  #snout
  block(
	points 4
	point[0] [0.32 0.95]
	point[1] [0.68 0.95]
	point[2] [0.68 1]
	point[3] [0.32 1] 
	z [0.35 0.45]
  )
  #ears
  block(
	points 4
	point[0] [0.15 0.8]
	point[1] [0.2 0.8]
	point[2] [0.2 0.9]
	point[3] [0.15 0.9] 
	z [0.4 0.5]
  )
  block(
	points 4
	point[0] [0.8 0.8]
	point[1] [0.85 0.8]
	point[2] [0.85 0.9]
	point[3] [0.8 0.9] 
	z [0.4 0.5]
  ) 
  #legs
   block(
	points 4
	point[0] [0.2 0.1]
	point[1] [0.3 0.1]
	point[2] [0.3 0.2]
	point[3] [0.2 0.2]
	z [0 0.15]
  )
   block(
	points 4
	point[0] [0.2 0.45]
	point[1] [0.3 0.45]
	point[2] [0.3 0.55]
	point[3] [0.2 0.55]
	z [0 0.15]
  )
   block(
	points 4
	point[0] [0.7 0.45]
	point[1] [0.8 0.45]
	point[2] [0.8 0.55]
	point[3] [0.7 0.55]
	z [0 0.15]
  )
   block(
	points 4
	point[0] [0.7 0.1]
	point[1] [0.8 0.1]
	point[2] [0.8 0.2]
	point[3] [0.7 0.2]
	z [0 0.15]
  )

  anilaser(pose [ 0.050 0.000 0 90.000 ])
)

