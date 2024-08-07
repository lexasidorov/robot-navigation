; generated by Slic3r 1.3.0 on 2023-04-22 at 16:45:33

; external perimeters extrusion width = 0.11mm (0.27mm^3/s)
; perimeters extrusion width = 0.11mm (0.50mm^3/s)
; infill extrusion width = 0.11mm (0.67mm^3/s)
; solid infill extrusion width = 0.11mm (0.17mm^3/s)
; top infill extrusion width = 0.11mm (0.13mm^3/s)

M107
M104 S205 ; set temperature
G28 ; home all axes
G1 Z5 F5000 ; lift nozzle

; Filament gcode

M109 S205 ; set temperature and wait for it to be reached
G21 ; set units to millimeters
G90 ; use absolute coordinates
M82 ; use absolute distances for extrusion
G92 E0
G1 Z0.500 F7800.000
G1 X93.499 Y95.498 F7800.000
G1 F1800
G1 X100.501 Y95.498 E0.44215
G1 X100.501 Y106.501 E1.13696
G1 X93.499 Y106.501 E1.57910
G1 X93.499 Y95.513 E2.27297
G1 X93.480 Y95.399 F7800.000
G92 E0
G1 X92.391 Y93.805 F7800.000
G1 F1800
G1 X89.393 Y93.805 E0.18931
G1 X89.393 Y92.389 E0.27872
G1 X93.607 Y92.389 E0.54482
G1 X93.607 Y94.605 E0.68476
G1 X92.391 Y94.605 E0.76154
G1 X92.391 Y93.820 E0.81113
G92 E0
G1 X94.499 Y94.097 F7800.000
G1 F1800
G1 X94.499 Y90.096 E0.25265
G1 X99.701 Y90.096 E0.58109
G1 X99.701 Y94.097 E0.83374
G1 X94.514 Y94.097 E1.16123
G1 X94.402 Y94.119 F7800.000
G92 E0
G1 X101.394 Y94.605 F7800.000
G1 F1800
G1 X100.593 Y94.605 E0.05053
G1 X100.593 Y89.393 E0.37966
G1 X108.212 Y89.393 E0.86071
G1 X108.212 Y97.190 E1.35310
G1 X110.607 Y97.190 E1.50438
G1 X110.607 Y104.608 E1.97279
G1 X106.411 Y104.608 E2.23778
G1 X106.411 Y110.607 E2.61659
G1 X92.391 Y110.607 E3.50190
G1 X92.391 Y107.394 E3.70482
G1 X101.394 Y107.394 E4.27332
G1 X101.394 Y94.620 E5.07993
G92 E0
G1 X109.104 Y96.298 F7800.000
G1 F1800
G1 X111.500 Y96.298 E0.15128
G1 X111.500 Y105.501 E0.73244
G1 X107.304 Y105.501 E0.99742
G1 X107.304 Y111.500 E1.37623
G1 X88.500 Y111.500 E2.56360
G1 X88.500 Y109.702 E2.67713
G1 X91.498 Y109.702 E2.86644
G1 X91.498 Y94.697 E3.81392
G1 X88.500 Y94.697 E4.00323
G1 X88.500 Y91.496 E4.20538
G1 X93.899 Y91.496 E4.54629
G1 X93.899 Y88.500 E4.73547
G1 X109.104 Y88.500 E5.69563
G1 X109.104 Y96.283 E6.18706
G1 X109.104 Y96.398 F7800.000
G92 E0
G1 X104.112 Y92.111 F7800.000
G1 F1800
G1 X105.493 Y92.111 E0.08918
G1 X105.493 Y99.909 E0.59267
G1 X107.889 Y99.909 E0.74737
G1 X107.889 Y101.890 E0.87530
G1 X104.112 Y101.890 E1.11918
G1 X104.112 Y92.126 E1.74963
G1 X103.199 Y92.799 F7800.000
G1 F1800
G1 X102.399 Y92.799 E1.80131
G1 X102.399 Y91.198 E1.90470
G1 X106.406 Y91.198 E2.16344
G1 X106.406 Y98.996 E2.66693
G1 X108.802 Y98.996 E2.82163
G1 X108.802 Y102.803 E3.06745
G1 X104.605 Y102.803 E3.33841
G1 X104.605 Y108.802 E3.72577
G1 X103.199 Y108.802 E3.81657
G1 X103.199 Y92.814 E4.84888
G1 X102.286 Y93.712 F7800.000
G1 F1800
G1 X101.486 Y93.712 E4.90055
G1 X101.486 Y90.285 E5.12183
G1 X107.319 Y90.285 E5.49845
G1 X107.319 Y98.083 E6.00195
G1 X109.715 Y98.083 E6.15665
G1 X109.715 Y103.716 E6.52035
G1 X105.518 Y103.716 E6.79132
G1 X105.518 Y109.715 E7.17868
G1 X93.283 Y109.715 E7.96869
G1 X93.283 Y108.286 E8.06091
G1 X102.286 Y108.286 E8.64224
G1 X102.286 Y93.727 E9.58233
G92 E0
; Filament-specific end gcode 
;END gcode for filament

M104 S0 ; turn off temperature
G28 X0  ; home X axis
M84     ; disable motors

M140 S0 ; set bed temperature
; filament used = 25.1mm (0.2cm3)
; total filament cost = 0.0

; avoid_crossing_perimeters = 0
; bed_shape = 0x0,200x0,200x200,0x200
; bed_temperature = 0
; before_layer_gcode = 
; between_objects_gcode = 
; bridge_acceleration = 0
; bridge_fan_speed = 100
; brim_connections_width = 0
; brim_width = 0
; complete_objects = 0
; cooling = 1
; default_acceleration = 0
; disable_fan_first_layers = 3
; duplicate_distance = 6
; end_filament_gcode = "; Filament-specific end gcode \n;END gcode for filament\n"
; end_gcode = M104 S0 ; turn off temperature\nG28 X0  ; home X axis\nM84     ; disable motors\n
; extruder_clearance_height = 20
; extruder_clearance_radius = 20
; extruder_offset = 0x0
; extrusion_axis = E
; extrusion_multiplier = 1
; fan_always_on = 0
; fan_below_layer_time = 60
; filament_colour = #FFFFFF
; filament_cost = 0
; filament_density = 0
; filament_diameter = 3
; filament_max_volumetric_speed = 0
; filament_notes = ""
; first_layer_acceleration = 0
; first_layer_bed_temperature = 0
; first_layer_extrusion_width = 200%
; first_layer_speed = 30
; first_layer_temperature = 205
; gcode_arcs = 0
; gcode_comments = 0
; gcode_flavor = reprap
; has_heatbed = 1
; infill_acceleration = 0
; infill_first = 0
; interior_brim_width = 0
; layer_gcode = 
; max_fan_speed = 100
; max_layer_height = 0.3
; max_print_speed = 80
; max_volumetric_speed = 0
; min_fan_speed = 35
; min_layer_height = 0.15
; min_print_speed = 10
; min_skirt_length = 0
; notes = 
; nozzle_diameter = 0.1
; only_retract_when_crossing_perimeters = 1
; ooze_prevention = 0
; output_filename_format = [input_filename_base].gcode
; perimeter_acceleration = 0
; post_process = 
; pressure_advance = 0
; printer_notes = 
; resolution = 0
; retract_before_travel = 2
; retract_layer_change = 0
; retract_length = 0
; retract_length_toolchange = 10
; retract_lift = 0
; retract_lift_above = 0
; retract_lift_below = 0
; retract_restart_extra = 0
; retract_restart_extra_toolchange = 0
; retract_speed = 40
; skirt_distance = 6
; skirt_height = 1
; skirts = 0
; slowdown_below_layer_time = 5
; spiral_vase = 0
; standby_temperature_delta = -5
; start_filament_gcode = "; Filament gcode\n"
; start_gcode = G28 ; home all axes\nG1 Z5 F5000 ; lift nozzle\n
; temperature = 200
; threads = 4
; toolchange_gcode = 
; travel_speed = 130
; use_firmware_retraction = 0
; use_relative_e_distances = 0
; use_set_and_wait_bed = 0
; use_set_and_wait_extruder = 0
; use_volumetric_e = 0
; vibration_limit = 0
; wipe = 0
; z_offset = 0
; z_steps_per_mm = 0
; adaptive_slicing = 0
; adaptive_slicing_quality = 10%
; dont_support_bridges = 1
; extrusion_width = 0
; first_layer_height = 0.5
; infill_only_where_needed = 0
; interface_shells = 0
; layer_height = 0.5
; match_horizontal_surfaces = 0
; raft_layers = 0
; regions_overlap = 0
; seam_position = aligned
; sequential_print_priority = 0
; support_material = 0
; support_material_angle = 0
; support_material_buildplate_only = 0
; support_material_contact_distance = 0.2
; support_material_enforce_layers = 0
; support_material_extruder = 1
; support_material_extrusion_width = 0
; support_material_interface_extruder = 1
; support_material_interface_extrusion_width = 0
; support_material_interface_layers = 3
; support_material_interface_spacing = 0
; support_material_interface_speed = 100%
; support_material_max_layers = 0
; support_material_pattern = pillars
; support_material_spacing = 2.5
; support_material_speed = 60
; support_material_threshold = 60%
; xy_size_compensation = 0
; bottom_infill_pattern = concentric
; bottom_solid_layers = 3
; bridge_flow_ratio = 1
; bridge_speed = 60
; external_perimeter_extrusion_width = 0
; external_perimeter_speed = 50%
; external_perimeters_first = 0
; extra_perimeters = 1
; fill_angle = 45
; fill_density = 0%
; fill_gaps = 1
; fill_pattern = stars
; gap_fill_speed = 20
; infill_every_layers = 1
; infill_extruder = 1
; infill_extrusion_width = 0
; infill_overlap = 55%
; infill_speed = 80
; overhangs = 1
; perimeter_extruder = 1
; perimeter_extrusion_width = 0
; perimeter_speed = 60
; perimeters = 2
; small_perimeter_speed = 15
; solid_infill_below_area = 70
; solid_infill_every_layers = 0
; solid_infill_extruder = 1
; solid_infill_extrusion_width = 0
; solid_infill_speed = 20
; thin_walls = 1
; top_infill_extrusion_width = 0
; top_infill_pattern = rectilinear
; top_solid_infill_speed = 15
; top_solid_layers = 0
