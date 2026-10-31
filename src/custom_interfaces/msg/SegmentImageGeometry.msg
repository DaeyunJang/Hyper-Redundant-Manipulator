# GUI-only geometry in CROP IMAGE PIXELS, not metric camera/base coordinates.
# Matches estimated_segment_skeleton_image header, dimensions, and source frame.
# This message must never be used as a motion/force command or a TF transform.
std_msgs/Header header
uint32 width
uint32 height

# Contact IDs 1..18 exclude the fixed proximal segment (os).
# pixels contains 18 consecutive polylines, each with samples_per_segment points:
# ID i uses pixels[(i-1)*samples_per_segment : i*samples_per_segment].
# x = crop column u, y = crop row v, z = 0; no ROI offset remains to subtract.
# Samples follow the original image skeleton at the fitted 3D segment parameters.
uint16 samples_per_segment
geometry_msgs/Point32[] pixels

# The image location at each segment's arc-length centre, indexed by ID-1.
geometry_msgs/Point32[18] centers
