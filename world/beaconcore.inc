define core ranger
(
  sensor(
  range [ 0.0 0.0 ] # range
  fov 180       # field of vision
  samples 180   # samples 
  )
  # generic model properties
  color "black"
  size [ 0.00 0.00 0.0 ]
)

define point position
(
  size [1 1 -1]
  drive "diff"
  core(pose [ 0.00 0.000 0 0.000 ])
)

point(pose [ 0 49 0 0 ] name "V0" color "purple")
point(pose [ -10.5 38 0 0 ] name "V0" color "yellow")
point(pose [ -7 38 0 0 ] name "V0" color "yellow")
point(pose [ -3.5 38 0 0 ] name "V0" color "yellow")
point(pose [ 0 38 0 0 ] name "V0" color "yellow")
point(pose [ 3.5 38 0 0 ] name "V0" color "yellow")
point(pose [ 7 38 0 0 ] name "V0" color "yellow")
point(pose [ 10.5 38 0 0 ] name "V0" color "yellow")
point(pose [ -10.5 -38 0 0 ] name "V0" color "yellow")
point(pose [ -7 -38 0 0 ] name "V0" color "yellow")
point(pose [ -3.5 -38 0 0 ] name "V0" color "yellow")
point(pose [ 0 -38 0 0 ] name "V0" color "yellow")
point(pose [ 3.5 -38 0 0 ] name "V0" color "yellow")
point(pose [ 7 -38 0 0 ] name "V0" color "yellow")
point(pose [ 10.5 -38 0 0 ] name "V0" color "yellow")
