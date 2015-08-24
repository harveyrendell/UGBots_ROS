include "kiwirow.inc" 
include "boundary.inc"
include "shrub.inc"
include "cmdCenter.inc"

shrub (pose [ 0 -40 0 0 ] color "green")

cmdCenter (pose [ 30 -55 0 0 ] color "gray")

rows (pose [ -12.25 -35 -1.002 0 ])
rows (pose [ -8.75 -35 -1.002 0 ])
rows (pose [ -5.25 -35 -1.002 0 ])
rows (pose [ -1.75 -35 -1.002 0 ])
rows (pose [ 1.75 -35 -1.002 0 ])
rows (pose [ 5.25 -35 -1.002 0 ])
rows (pose [ 8.75 -35 -1.002 0 ])
rows (pose [ 12.25 -35 -1.002 0 ])

boundary (pose [ -50 25 0 90 ])
boundary (pose [ 50 25 0 90 ])
boundary (pose [ -50 -25 0 90 ])
boundary (pose [ 50 -25 0 90 ])

boundary (pose [ -25 50 0 0 ])
boundary (pose [ 25 50 0 0 ])
boundary (pose [ -25 -50 0 0 ])
boundary (pose [ 25 -50 0 0 ])
