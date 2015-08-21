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
  size [0.25 0.5 0.5]
  drive "diff"
  
  #tail
  block(
	points 4
	point[0] [0 0.4]
	point[1] [0 0.6]
	point[2] [0.15 0.6]
	point[3] [0.15 0.4]
	z [0.6 0.75]
  )

  #body
  block(
	points 4
	point[0] [0.15 0.15]
	point[1] [0.15 0.85]
	point[2] [0.55 0.15]
	point[3] [0.55 0.85] 
	z [0.35 0.85]
  )
  #mane
  block(
	points 4
	point[0] [0.55 0]
	point[1] [0.55 1]
	point[2] [0.80 1]
	point[3] [0.80 0] 
	z [0.35 1]
  )
  #face
  block(
	points 4
	point[0] [0.8 0.15]
	point[1] [0.8 0.85]
	point[2] [0.86 0.15]
	point[3] [0.86 0.85] 
	z [0.5 0.9]
  )

  #nose
  block(
	points 4
	point[0] [0.86 0.35]
	point[1] [0.86.0.65]
	point[2] [1 0.35]
	point[3] [1 0.65] 
	z [0.5 0.75]
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

  anilaser(pose [ 0.050 0.000 0 0.000 ])
)

