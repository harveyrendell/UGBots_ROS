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
  size [0.75 0.25 0.35]
  drive "diff"
  
  #tail
  block(
	points 4
	point[0] [0 0.4]
	point[1] [0 0.6]
	point[2] [0.15 0.6]
	point[3] [0.15 0.4]
	z [0.75 0.85]
  )

  #face
  block(
	points 4
	point[0] [0.8 0.15]
	point[1] [0.8 0.85]
	point[2] [0.86 0.85]
	point[3] [0.86 0.15] 
	z [0.5 0.9]
  )

  #ears
  block(
	points 4
	point[0] [0.82 0.2]
	point[1] [0.82 0.35]
	point[2] [0.84 0.35]
	point[3] [0.84 0.2] 
	z [0.9 1]
  )

  block(
	points 4
	point[0] [0.82 0.8]
	point[1] [0.82 0.65]
	point[2] [0.84 0.65]
	point[3] [0.84 0.8] 
	z [0.9 1]
  )

  #snout
  block(
	points 4
	point[0] [0.86 0.35]
	point[1] [0.86 0.65]
	point[2] [0.995 0.65]
	point[3] [0.995 0.35] 
	z [0.55 0.75]
  ) 

  #nose
  block(
	points 4
	point[0] [0.96 0.44]
	point[1] [0.96 0.56]
	point[2] [1 0.56]
	point[3] [1 0.44] 
	z [0.72 0.76]
  ) 

  #body
  block(
	points 4
	point[0] [0.15 0.1]
	point[1] [0.15 0.9]
	point[2] [0.55 0.9]
	point[3] [0.55 0.1] 
	z [0.35 0.95]
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

  #legs
   block(
	points 4
	point[0] [0.18 0.15]
	point[1] [0.18 0.25]
	point[2] [0.25 0.25]
	point[3] [0.25 0.15]
	z [0 0.35]
  )
   block(
	points 4
	point[0] [0.18 0.75]
	point[1] [0.18 0.85]
	point[2] [0.25 0.85]
	point[3] [0.25 0.75]
	z [0 0.35]
  )
   block(
	points 4
	point[0] [0.7 0.15]
	point[1] [0.7 0.25]
	point[2] [0.76 0.25]
	point[3] [0.76 0.15]
	z [0 0.35]
  )
   block(
	points 4
	point[0] [0.7 0.75]
	point[1] [0.7 0.85]
	point[2] [0.76 0.85]
	point[3] [0.76 0.75]
	z [0 0.35]
  )

  anilaser(pose [ 0.050 0.000 0 0.000 ])
)

