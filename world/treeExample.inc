define treeExample model
(
  size [1 1 2.6]
  #trunk
  block(
	color "brown"
	points 4
	point[0] [0.35 0.35]
	point[1] [0.65 0.35]
	point[2] [0.65 0.65]
	point[3] [0.35 0.65]
	z [0 1.8]
  )

  #leaves
  block(
	color "green"
	points 4
	point[0] [0.2 0.2]
	point[1] [0.8 0.2]
	point[2] [0.8 0.8]
	point[3] [0.2 0.8]
	z [1.8 2.0]
  )

  block(
	color "green"
	points 4
	point[0] [0 0]
	point[1] [1 0]
	point[2] [1 1]
	point[3] [0 1]
	z [2.0 2.2]
  )

  block(
	color "green"
	points 4
	point[0] [0.2 0.2]
	point[1] [0.8 0.2]
	point[2] [0.8 0.8]
	point[3] [0.2 0.8]
	z [2.2 2.4]
  )

  block(
	color "green"
	points 4
	point[0] [0.35 0.35]
	point[1] [0.65 0.35]
	point[2] [0.65 0.65]
	point[3] [0.35 0.65]
	z [2.4 2.6]
  )
)