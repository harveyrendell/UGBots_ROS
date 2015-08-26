define tree model
(
  size [2 5 2.7]

  #trunk
  block(
	color "brown"
	points 4
	point[0] [0.4 0.45]
	point[1] [0.6 0.45]
	point[2] [0.6 0.55]
	point[3] [0.4 0.55]
	z [0 1.8]
  )

  #leaves
  block(
	color "green"
	points 4
	point[0] [0.9 0.9]
	point[1] [1 0.9]
	point[2] [1 1]
	point[3] [0.9 1]
	z [1.8 1.9]
  )

  block(
	color "green"
	points 4
	point[0] [0.1 0]
	point[1] [0.2 0]
	point[2] [0.2 0.1]
	point[3] [0.1 0.1]
	z [1.8 1.9]

  )
)
