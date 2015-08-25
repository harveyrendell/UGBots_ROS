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

