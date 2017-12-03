import codecad

import suspension

import parameters

complete_side_suspension = codecad.Assembly([suspension.left_wheel_suspension.translated_x(-1.5 * parameters.road_wheel_base_spacing),
                                             suspension.left_wheel_suspension.translated_x(-0.5 * parameters.road_wheel_base_spacing),
                                             suspension.right_wheel_suspension.translated_x(0.5 * parameters.road_wheel_base_spacing),
                                             suspension.right_wheel_suspension.translated_x(1.5 * parameters.road_wheel_base_spacing)]
                                             ).make_part("complete_side_suspension")

all_suspension = codecad.Assembly([complete_side_suspension.translated_y(-80),
                                   complete_side_suspension.rotated_z(180).translated_y(80)])

if __name__ == "__main__":
    codecad.commandline_render(all_suspension, 0.1)
