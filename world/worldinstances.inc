include "models/boundary.inc"
include "models/shrub.inc"
include "models/binloc.inc"

#randomly locating the shrubs across the stage
shrub (pose [ 0 -49 0 0 ])
shrub (pose [ -45 -30 0 0 ])
shrub (pose [ -35 16 0 0 ])
shrub (pose [ -20 35 0 0 ])
shrub (pose [ 6 46 0 0 ])
shrub (pose [ 20 -30 0 0 ])
shrub (pose [ 19 26 0 0 ])
shrub (pose [ -30 -23 0 0 ])
shrub (pose [ 37 -2 0 0 ])
shrub (pose [ 40 -15 0 0 ])
shrub (pose [ 21 45 0 0 ])
shrub (pose [ 29 40 0 0 ])

#boundary made around the orchard farm
boundary (pose [ -50 25 0 90 ])
boundary (pose [ 50 25 0 90 ])
boundary (pose [ -50 -25 0 90 ])
#this fence is shorter than the others, to leave a break for the driveway
boundary (pose [ 50 -20 0 90 ])

boundary (pose [ -25 50 0 0 ])
boundary (pose [ 25 50 0 0 ])
boundary (pose [ -25 -50 0 0 ])
boundary (pose [ 25 -50 0 0 ])

#bin location positioned
binloc (pose [ 32.5 -12.5 0 0 ])
