import codecad

import suspension
import transmission

layout = codecad.assembly("tank_layout_test_assembly",
    [
    suspension.suspension_assembly_left.hidden(),
    suspension.suspension_assembly_left.translated_x(suspension.suspension_spacing).hidden(),
    suspension.suspension_assembly_left.translated_x(2 * suspension.suspension_spacing),
    transmission.drive_sprocket_assembly.rotated_x(-90).translated_x(2 * suspension.suspension_spacing + 145)
    ])

if __name__ == "__main__":
    codecad.commandline_render(layout, 0.1)

