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

define worker position
(
  size [0.5 0.5 1.8]
  drive "diff"
  mylaser(pose [ 0.050 0.000 0 0.000 ])
)

