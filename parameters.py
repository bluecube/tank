# Approximate scale taken from Merkava tank:
# 7.60 m x 3.72 m x 2.66 m, 65e3 kg
# 0.45 m ground clearance
# 1/18 scaled:
# 422 mm x 206 mm x 147 mm, 9.6 kg
# 25mm ground clearance

# Printer settings:
extrusion_width = 0.45
layer_height = 0.2
thin_wall = 2 * extrusion_width
thick_wall = 4 * extrusion_width
overhang_angle = 45 # [deg] How steep overhanging walls we are willing to print (90 is ceiling)

# Dimensions for common vitamins:
small_screw_diameter = 3
small_screw_head_diameter = 5.5
small_screw_head_height = 3
small_screw_nut_diameter = 6 # Outer diameter of the hex
small_screw_nut_height = 4 # Lock nut
small_screw_nut_s = 5.5 # Nut size across the flats

large_screw_nut_height = 5
large_screw_nut_s = 8

# All shoulder screws should be M5 6x20
shoulder_screw_diameter = 5
shoulder_screw_diameter2 = 6
shoulder_screw_length = 29.5
shoulder_screw_screw_length = 9.5
shoulder_screw_head_diameter = 10
shoulder_screw_head_height = 4.5
shoulder_screw_nut_height = large_screw_nut_height
shoulder_screw_nut_s = large_screw_nut_s

small_bearing_id = 3
small_bearing_od = 7
small_bearing_thickness = 3
small_bearing_shoulder_size = 2 * extrusion_width

design_weight = 4 # [kg]

drive_sprocket_tooth_count = 8
