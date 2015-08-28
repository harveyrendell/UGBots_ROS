define tractor model
(
  size [2 2 2]
  drive "diff"
  velocity_bounds [ -10 10 -10 10 -10 10 -90 90 ]

  #body
  block(
    color "yellow"
  	points 4
	point[0] [0.15 0]
	point[1] [0.85 0]
    point[2] [0.85 1]
	point[3] [0.15 1]
	z [0.5 1.1]
  ) 
  #head
  block(
    color "green"
  	points 4	
    point[0] [0.2 0.05]
	point[1] [0.8 0.05]
	point[2] [0.8 0.55]
	point[3] [0.2 0.55]
	z [1.9 2]
  )
  
#head support
 block(
    color "green"
  	points 4	
    point[0] [0.2 0.05]
	point[1] [0.8 0.05]
	point[2] [0.8 0.1]
	point[3] [0.2 0.1]
	z [1.1 1.9]
  ) block(
    color "green"
  	points 4	
    point[0] [0.2 0.5]
	point[1] [0.25 0.5]
	point[2] [0.25 0.55]
	point[3] [0.2 0.55]
	z [1.1 2]
  ) block(
    color "green"
  	points 4	
    point[0] [0.75 0.5]
	point[1] [0.8 0.5]
	point[2] [0.8 0.55]
	point[3] [0.75 0.55]
	z [1.1 2]
  )

#back wheel
block(
    color "black"
  	points 4	
    point[0] [0 0.05]
	point[1] [0.15 0.05]
	point[2] [0.15 0.55]
	point[3] [0 0.55]
	z [0.4 0.6]
  )block(
    color "black"
  	points 4	
    point[0] [0 0.07]
	point[1] [0.15 0.07]
	point[2] [0.15 0.53]
	point[3] [0 0.53]
	z [0.2 0.4]
  )block(
    color "black"
  	points 4	
    point[0] [0 0.07]
	point[1] [0.15 0.07]
	point[2] [0.15 0.53]
	point[3] [0 0.53]
	z [0.6 0.8]
  )block(
    color "black"
  	points 4	
    point[0] [0 0.11]
	point[1] [0.15 0.11]
	point[2] [0.15 0.49]
	point[3] [0 0.49]
	z [0 0.2]
  )block(
    color "black"
  	points 4	
    point[0] [0 0.11]
	point[1] [0.15 0.11]
	point[2] [0.15 0.49]
	point[3] [0 0.49]
	z [0.8 1]
  )

block(
    color "black"
  	points 4	
    point[0] [0.85 0.05]
	point[1] [1 0.05]
	point[2] [1 0.55]
	point[3] [0.85 0.55]
	z [0.4 0.6]
  )block(
    color "black"
  	points 4	
    point[0] [0.85 0.07]
	point[1] [1 0.07]
	point[2] [1 0.53]
	point[3] [0.85 0.53]
	z [0.2 0.4]
  )block(
    color "black"
  	points 4	
    point[0] [0.85 0.07]
	point[1] [1 0.07]
	point[2] [1 0.53]
	point[3] [0.85 0.53]
	z [0.6 0.8]
  )block(
    color "black"
  	points 4	
    point[0] [0.85 0.11]
	point[1] [1 0.11]
	point[2] [1 0.49]
	point[3] [0.85 0.49]
	z [0.8 1]
  )block(
    color "black"
  	points 4	
    point[0] [0.85 0.11]
	point[1] [1 0.11]
	point[2] [1 0.49]
	point[3] [0.85 0.49]
	z [0 0.2]
  )





#front wheel
 block(
    color "black"
  	points 4	
    point[0] [0.2 0.7]
	point[1] [0.3 0.7]
	point[2] [0.3 0.95]
	point[3] [0.2 0.95]
	z [0.2 0.3]
  ) block(
    color "black"
  	points 4	
    point[0] [0.2 0.71]
	point[1] [0.3 0.71]
	point[2] [0.3 0.94]
	point[3] [0.2 0.94]
	z [0.1 0.2]
  ) block(
    color "black"
  	points 4	
    point[0] [0.2 0.72]
	point[1] [0.3 0.72]
	point[2] [0.3 0.93]
	point[3] [0.2 0.93]
	z [0 0.1]
  ) block(
    color "black"
  	points 4	
    point[0] [0.2 0.71]
	point[1] [0.3 0.71]
	point[2] [0.3 0.94]
	point[3] [0.2 0.94]
	z [0.3 0.4]
  ) block(
    color "black"
  	points 4	
    point[0] [0.2 0.72]
	point[1] [0.3 0.72]
	point[2] [0.3 0.93]
	point[3] [0.2 0.93]
	z [0.4 0.5]
  )


 block(
    color "black"
  	points 4	
    point[0] [0.7 0.7]
	point[1] [0.8 0.7]
	point[2] [0.8 0.95]
	point[3] [0.7 0.95]
	z [0.2 0.3]
  ) block(
    color "black"
  	points 4	
    point[0] [0.7 0.71]
	point[1] [0.8 0.71]
	point[2] [0.8 0.94]
	point[3] [0.7 0.94]
	z [0.1 0.2]
  ) block(
    color "black"
  	points 4	
    point[0] [0.7 0.72]
	point[1] [0.8 0.72]
	point[2] [0.8 0.93]
	point[3] [0.7 0.93]
	z [0 0.1]
  ) block(
    color "black"
  	points 4	
    point[0] [0.7 0.71]
	point[1] [0.8 0.71]
	point[2] [0.8 0.94]
	point[3] [0.7 0.94]
	z [0.3 0.4]
  ) block(
    color "black"
  	points 4	
    point[0] [0.7 0.72]
	point[1] [0.8 0.72]
	point[2] [0.8 0.93]
	point[3] [0.7 0.93]
	z [0.4 0.5]
  )

#pipe
 block(
    color "orange"
  	points 4	
    point[0] [0.75 0.85]
	point[1] [0.8 0.85]
	point[2] [0.8 0.9]
	point[3] [0.75 0.9]
	z [1.1 1.5]
  )

  
)
