import codecad

import suspension
import track
import hull

import parameters

test_print = codecad.Assembly([hull.test_side \
                                    .rotated_x(90),
                               suspension.suspension_assembly_left \
                                    .translated(0,
                                                -hull.side_thickness - hull.side_pivot_height,
                                                hull.suspension_pivot_z),
                               track.track_row(2) \
                                    .translated(suspension.track_offset.x,
                                                -suspension.track_offset.y-hull.side_thickness - hull.side_pivot_height,
                                                suspension.track_offset.z + hull.suspension_pivot_z - track.thickness),
                               ])

if __name__ == "__main__":
    codecad.commandline_render(test_print, 0.1)
#.shape().rotated_z(90) & codecad.shapes.half_space()
