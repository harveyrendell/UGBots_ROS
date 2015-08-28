#model for kiwifruit trees
define tree model
(
  size [2 5 2.7]

  #trunk
  block(
	color "SaddleBrown"
	points 4
	point[0] [0.4 0.45]
	point[1] [0.6 0.45]
	point[2] [0.6 0.55]
	point[3] [0.4 0.55]
	z [0 1.8]
  )

  #kiwifruits with various colors
  block(
	color "DarkGreen"
	points 4
	point[0] [0.9 0.9]
	point[1] [1 0.9]
	point[2] [1 1]
	point[3] [0.9 1]
	z [1.8 1.9]
  )

  block(
	color "dark olive green"
	points 4
	point[0] [0.1 0]
	point[1] [0.2 0]
	point[2] [0.2 0.1]
	point[3] [0.1 0.1]
	z [1.9 2]

  )

  block(
	color "DarkGreen"
	points 4
	point[0] [0.2 0.4]
	point[1] [0.3 0.4]
	point[2] [0.3 0.5]
	point[3] [0.2 0.5]
	z [1.8 1.9]

  )
  block(
	color "dark olive green"
	points 4
	point[0] [0.5 0.4]
	point[1] [0.6 0.4]
	point[2] [0.6 0.5]
	point[3] [0.5 0.5]
	z [1.8 1.9]

  )  block(
	color "green"
	points 4
	point[0] [0.8 0]
	point[1] [0.9 0]
	point[2] [0.9 0.1]
	point[3] [0.8 0.1]
	z [1.8 1.9]

  )  block(
	color "ForestGreen"
	points 4
	point[0] [0.7 0.7]
	point[1] [0.8 0.7]
	point[2] [0.8 0.8]
	point[3] [0.7 0.8]
	z [1.9 2]

  )  block(
	color "Forestgreen"
	points 4
	point[0] [0.6 0.2]
	point[1] [0.7 0.2]
	point[2] [0.7 0.3]
	point[3] [0.6 0.3]
	z [1.8 1.9]

  )  block(
	color "green"
	points 4
	point[0] [0.3 0.8]
	point[1] [0.4 0.8]
	point[2] [0.4 0.9]
	point[3] [0.3 0.9]
	z [1.9 2]

  )
)
