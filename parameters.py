# Approximate scale taken from Merkava tank:
# 7.60 m x 3.72 m x 2.66 m, 65e3 kg, 64km/h
# 0.45 m ground clearance
# 1/18 scaled:
# 422 mm x 206 mm x 147 mm, 9.6 kg, 3.5km/h
# 25mm ground clearance

# Printer settings:
extrusion_width = 0.45
layer_height = 0.2
thin_wall = 2 * extrusion_width
thick_wall = 4 * extrusion_width

wheel_clearance = 2 # Distance between wheel and other parts

overhang_hole_blinding = layer_height
overhang_spokes_width = 2 * extrusion_width
overhang_spokes_height = overhang_hole_blinding

design_weight = 4 # [kg]
