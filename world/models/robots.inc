define robolaser ranger
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
  drive "omni"
  robolaser(pose [ 0.050 0.000 -1 0.000 ])
  velocity_bounds [ 0 3 0 0 0 0 -90 90 ]
)

define pickerRobot position
(
  size [2.5 1.5 1.0]
  drive "omni"
  robolaser(pose [ 0.050 0.000 -1 0.000 ])
)


