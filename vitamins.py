import parameters
import tools

small_screw = tools.name_only_part("M3x25_socket_head_screw")
small_screw.diameter = 3
small_screw.length = 25
small_screw.head_diameter = 5.5
small_screw.head_height = 3

small_screw.lock_nut = tools.name_only_part("M3_lock_nut")
small_screw.lock_nut.height = 4
small_screw.lock_nut.s = 5.5 # Nut size across the flats

large_screw = tools.name_only_part("M5x20_socket_head_screw")
large_screw.diameter = 5
large_screw.lock_nut = tools.name_only_part("M5_lock_nut")
large_screw.lock_nut.height = 5
large_screw.lock_nut.s = 8 # Nut size across the flats

shoulder_screw = tools.name_only_part("M5_6x20_socket_head_shoulder_screw")
shoulder_screw.diameter = 5
shoulder_screw.diameter2 = 6
shoulder_screw.length = 29.5
shoulder_screw.screw_length = 9.5
shoulder_screw.head_diameter = 10
shoulder_screw.head_height = 4.5
shoulder_screw.lock_nut = large_screw.lock_nut

#TODO: Fill in bearing number
small_bearing = tools.name_only_part("SMALL_bearing")
small_bearing.id = 3
small_bearing.od = 7
small_bearing.thickness = 3
small_bearing.shoulder_size = 2 * parameters.extrusion_width

#TODO: Fill in bearing number
large_bearing = tools.name_only_part("LARGE_bearing")
large_bearing.id = 10
large_bearing.od = 7
large_bearing.thickness = 6
large_bearing.shoulder_size = 2 * parameters.extrusion_width

nail = tools.name_only_part("1.25x16_nail")
nail.diameter = 1.25
nail.length = 16
