include "robots.inc"

define etc position
(
  size [0.5 0.5 0.5]
  drive "diff"
  robolaser(pose [ 0.050 0.000 0 0.000 ])
)


pickerRobot(pose [ -10 -40 0 0 ] name "R0" color "red")
carrierRobot(pose [ -10 -48 0 0 ] name "R1" color "blue")
etc(pose [ -16 42.5 0 0 ] name "A1" color "brown" )
etc(pose [ -40 -44 0 0 ] name "W1" color "black")
