import codecad

import suspension
import track

import parameters

test_print = codecad.Assembly([suspension.bogie_assembly.translated_z(track.thickness),
                               track.track_row(2).translated_x(20)])

if __name__ == "__main__":
    codecad.commandline_render(test_print.shape().rotated_z(-90), 0.2)
