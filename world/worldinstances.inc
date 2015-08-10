include "kiwirow.inc" 
include "boundary.inc"
include "robots.inc"

rows (pose [ -2.25 0 0 0 ])
rows (pose [ 2.25 0 0 0 ])

boundary (pose [ -50 25 0 90 ])
boundary (pose [ 50 25 0 90 ])
boundary (pose [ -50 -25 0 90 ])
boundary (pose [ 50 -25 0 90 ])

boundary (pose [ -25 50 0 0 ])
boundary (pose [ 25 50 0 0 ])
boundary (pose [ -25 -50 0 0 ])
boundary (pose [ 25 -50 0 0 ])


