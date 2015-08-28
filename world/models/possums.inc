#model for possums
define poslaser ranger
(
  sensor(
  range [ 0.0 3.5 ] # range
  fov 140       # field of vision
  samples 140   # samples 
  )
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
  
)

define possum position
(
  size [0.25 0.5 0.5]
  drive "diff"
  velocity_bounds [ -10 10 -10 10 -10 10 -90 90 ]
  #body
  block(
	points 4
	point[0] [0 0]
	point[1] [1 0]
	point[2] [1 0.7]
	point[3] [0 0.7]
	z [0.1 0.27]
  )

  #head
  block(
	points 4
	point[0] [0.15 0.7]
	point[1] [0.85 0.7]
	point[2] [0.85 0.95]
	point[3] [0.15 0.95] 
	z [0.1 0.25]
  )
  #snout
  block(
	points 4
	point[0] [0.2 0.95]
	point[1] [0.8 0.95]
	point[2] [0.8 1]
	point[3] [0.2 1] 
	z [0.1 0.15]
  )
  #ears
  block(
	points 4
	point[0] [0.15 0.7]
	point[1] [0.4 0.7]
	point[2] [0.4 0.75]
	point[3] [0.15 0.75] 
	z [0.25 0.32]
  )
  block(
	points 4
	point[0] [0.6 0.7]
	point[1] [0.85 0.7]
	point[2] [0.85 0.75]
	point[3] [0.6 0.75] 
	z [0.25 0.32]
  ) 

  #legs
   block(
	points 4
	point[0] [0 0]
	point[1] [0.15 0]
	point[2] [0.15 0.15]
	point[3] [0 0.15]
	z [0 0.1]
  )
   block(
	points 4
	point[0] [0.85 0]
	point[1] [1 0]
	point[2] [1 0.15]
	point[3] [0.85 0.15]
	z [0 0.1]
  )
   block(
	points 4
	point[0] [0 0.55]
	point[1] [0.15 0.55]
	point[2] [0.15 0.7]
	point[3] [0 0.7]
	z [0 0.1]
  )
   block(
  points 4
	point[0] [0.85 0.55]
	point[1] [1 0.55]
	point[2] [1 0.7]
	point[3] [0.85 0.7]
	z [0 0.1]
  )

  #tail
   block(
	points 4
	point[0] [0.4 0]
	point[1] [0.6 0]
	point[2] [0.6 0.1]
	point[3] [0.4 0.1]
	z [0.27 0.4]
  )
  poslaser(pose [ 0.050 0.000 0 0.000 ])
)

