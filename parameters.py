# Approximate scale taken from Merkava tank:
# 7.60 m x 3.72 m x 2.66 m, 65e3 kg
# 1/18 scaled:
# 422 mm x 206 mm x 147 mm, 9.6 kg

# Printer settings:
extrusion_width = 0.48
layer_height = 0.2
thin_wall = 2 * extrusion_width
thick_wall = 6 * extrusion_width

# Dimensions for vitamins:
small_screw_diameter = 3
small_screw_head_diameter = 5.5
small_screw_head_height = 3
small_screw_long = 35
small_bearing_id = 3
small_bearing_od = 7
small_bearing_thickness = 3
small_bearing_shoulder_size = 0.6

spring_length = 62 # Hole to hole
spring_travel = 11
spring_diameter = 17.5
spring_top_mount_diameter = 5
spring_bottom_mount_diameter = 3
spring_top_mount_thickness = 3.8
spring_bottom_mount_thickness = 6.5
spring_compression_leverage = 4 / (4 * 3 / 8)
    # Spring full compression force [kg]
    # tank weight [kg]
    # 1 / loaded sag ratio
    # number of road wheels

# Tread parameters
tread_thickness = small_screw_diameter + 2 * 3 * extrusion_width
tread_segment_length = 15
tread_width = small_screw_long + 2 * small_screw_head_height
tread_guide_height = 4
tread_guide_length = 7
tread_guide_width = 12
tread_guide_side_angle = 45
tread_guide_clearance = 0.5
tread_negative_bend_angle = 10
tread_groove_depth = 1
tread_clearance = 1

# Suspension parameters
road_wheel_diameter = 40
road_wheel_width = 30
road_wheel_outer_inset = 11
road_wheel_inner_inset = 3
road_wheel_half_thickness = 12
road_wheel_o_ring_minor_diameter = 2
road_wheel_base_spacing = 58
road_wheel_arm_clearance = 1
suspension_arm_dx = 30
suspension_arm_dy = 18
suspension_arm_wheel_clearance = 1
suspension_arm_thickness = 7
suspension_arm_height = small_screw_diameter + 2 * thick_wall
suspension_spring_angle = 15

drive_sprocket_tooth_count = 8
