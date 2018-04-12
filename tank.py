import codecad

import suspension
import drive_sprocket

layout = codecad.assembly("tank_layout_test_assembly",
    [
    suspension.suspension_assembly_left.hidden(),
    suspension.suspension_assembly_left.translated_x(suspension.suspension_spacing).hidden(),
    suspension.suspension_assembly_left.translated_x(2 * suspension.suspension_spacing),
    drive_sprocket.drive_sprocket_assembly.rotated_x(-90).translated_x(2 * suspension.suspension_spacing + 145)
    ])

if __name__ == "__main__":
    codecad.commandline_render(layout)

