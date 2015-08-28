define post model
(
  size [2 0.2 2]
  color "grey"

  #vertical pole
  block(
	points 4
	point[0] [0.45 0.4]
	point[1] [0.55 0.4]
	point[2] [0.55 0.6]
	point[3] [0.45 0.6]
	z [0 1.8]
  )

  #horizontal pole
  block(
	points 4
	point[0] [0 0.4]
	point[1] [1 0.4]
	point[2] [1 0.6]
	point[3] [0 0.6]
	z [1.8 2.0]
  )
)
