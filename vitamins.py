from codecad.shapes import *

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

long_screw = tools.name_only_part("M3x35_socket_head_screw")
long_screw.diameter = 3
long_screw.length = 35
long_screw.head_diameter = 5.5
long_screw.head_height = 3
long_screw.lock_nut = small_screw.lock_nut

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

small_bearing = tools.name_only_part("683_ZZ_bearing")
small_bearing.id = 3
small_bearing.od = 7
small_bearing.thickness = 3
small_bearing.shoulder_size = 2 * parameters.extrusion_width

large_bearing = tools.name_only_part("6900_2RS_bearing")
large_bearing.id = 10
large_bearing.od = 22
large_bearing.thickness = 6
large_bearing.shoulder_size = 4 * parameters.extrusion_width

nail = tools.name_only_part("1.25x16_nail")
nail.diameter = 1.25
nail.length = 16

o_ring = tools.name_only_part("30x2_o_ring")
o_ring.minor_diameter = 2

def spring(length):
    """ Return a model representing a spring extended to `length` (hole to hole) """

    assert length <= spring.length
    assert length >= spring.length - spring.travel

    o = (capsule(0, 0, 0, length / 2, spring.top_mount_od) - circle(d=spring.top_mount_id)) \
        .extruded(spring.top_mount_thickness)
    o += (capsule(0, length / 2, 0, length, spring.bottom_mount_od) - circle(d=spring.bottom_mount_id).translated_y(length)) \
        .extruded(spring.bottom_mount_thickness)
    o = o.rotated_x(90)
    o += cylinder(d=spring.diameter, h=length * 2 / 3).translated_z(length * 0.45)

    return o.make_part("60mm_shock_absorber")
spring.length = 62 # Center to center, relaxed
spring.travel = 12
spring.diameter = 17.1
spring.top_mount_id = 5
spring.top_mount_od = 8.5
spring.top_mount_thickness = 3.8
spring.bottom_mount_id = 3
spring.bottom_mount_od = 9
spring.bottom_mount_thickness = 6.5
spring.preload_force = 0.95 # [kg]
spring.full_compression_force = 4.5 # [kg]
spring.force = lambda length: spring.preload_force + (spring.full_compression_force - spring.preload_force) * (spring.length - length) / spring.travel
