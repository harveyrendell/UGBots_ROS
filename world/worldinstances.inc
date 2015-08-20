include "kiwirow.inc" 
include "boundary.inc"

rows (pose [ -10.5 -35 -1.002 0 ])
rows (pose [ -7 -35 -1.002 0 ])
rows (pose [ -3.5 -35 -1.002 0 ])
rows (pose [ 0 -35 -1.002 0 ])
rows (pose [ 3.5 -35 -1.002 0 ])
rows (pose [ 7 -35 -1.002 0 ])
rows (pose [ 10.5 -35 -1.002 0 ])

boundary (pose [ -50 25 0 90 ])
boundary (pose [ 50 25 0 90 ])
boundary (pose [ -50 -25 0 90 ])
boundary (pose [ 50 -25 0 90 ])

boundary (pose [ -25 50 0 0 ])
boundary (pose [ 25 50 0 0 ])
boundary (pose [ -25 -50 0 0 ])
boundary (pose [ 25 -50 0 0 ])

