import codecad

import suspension
import tread

import parameters

complete_side_suspension = codecad.Assembly([suspension.left_wheel_suspension.translated_x(-1.5 * parameters.road_wheel_base_spacing),
                                             suspension.left_wheel_suspension.translated_x(-0.5 * parameters.road_wheel_base_spacing),
                                             suspension.right_wheel_suspension.translated_x(0.5 * parameters.road_wheel_base_spacing),
                                             suspension.right_wheel_suspension.translated_x(1.5 * parameters.road_wheel_base_spacing),
                                             ]).make_part("complete_side_suspension")
complete_side = codecad.Assembly([complete_side_suspension.translated_z(parameters.road_wheel_diameter / 2 + parameters.tread_thickness),
                                  tread.tread_row(10).translated_z(parameters.tread_thickness / 2)
                                  ]).make_part("complete_side")

all_suspension = codecad.Assembly([complete_side.translated_y(-80),
                                   complete_side.rotated_z(180).translated_y(80)])

if __name__ == "__main__":
    codecad.commandline_render(all_suspension, 0.1)
