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

define carrierRobot position
(
  size [1.8 1.5 1.0]
  drive "diff"
  block(
	points 4
	point[0] [0 0]
	point[1] [1 0]
	point[2] [1 1]
	point[3] [0 1]
	z [0 0.5]
  )
  block(
	points 4
	point[0] [0.25 0.25]
	point[1] [0.75 0.25]
	point[2] [0.75 0.75]
	point[3] [0.25 0.75]
	z [0.5 1.0]
  )
  mylaser(pose [ 0.050 0.000 0 0.000 ])
)

define pickerRobot position
(
  size [2.5 1.5 1.0]
  drive "diff"
  mylaser(pose [ 0.050 0.000 0 0.000 ])
)


