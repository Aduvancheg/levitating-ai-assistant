// =====================================================================
// ANTIGRAVITY PHASE 4 // SPLIT CALIBRATION & PARKING CRADLE V1.0
// Compatible with Base Top Lid V8 (Outer Lock Radius R=56.5mm)
// Target Agent: 70mm Sphere (Weight ~75g) // Stand Height = 30mm
// Material: PETG // 0.2mm Layer Height // 15% Infill
// =====================================================================

$fn = 90;

// Global Dimensions
stand_h = 30.0;           // Total stand height (mm)
outer_r = 58.5;           // Outer radius fitting base rim (mm)
inner_ring_r = 48.0;      // Inner rim radius of cradle (mm)
sphere_d = 70.0;          // Agent sphere diameter (mm)
sphere_r = sphere_d / 2;  // 35.0 mm
cradle_depth = 12.0;      // Depth sphere sinks into cradle (mm)
cutout_center_r = 42.0;   // Central open aperture for magnetic flux/ToF

module Half_Calibration_Stand(is_right_half = true) {
    difference() {
        union() {
            // Main Outer Tapered Ring
            difference() {
                cylinder(h = stand_h, r1 = outer_r, r2 = inner_ring_r, center = false);
                
                // Central Air & Flux Aperture (Open to allow coil & ToF beams)
                translate([0, 0, -1])
                    cylinder(h = stand_h + 2, r = cutout_center_r, center = false);
            }
            
            // Spherical Seat Ring (Cradle Rim)
            translate([0, 0, stand_h - cradle_depth])
                difference() {
                    cylinder(h = cradle_depth, r = inner_ring_r, center = false);
                    
                    // Sphere Seat Cutout (Radius 35mm centered at Z = stand_h - cradle_depth + 35mm)
                    translate([0, 0, sphere_r])
                        sphere(r = sphere_r);
                }
                
            // 4x Bottom Locking Feet (Radius 56.5mm)
            for (angle = [45, 135, 225, 315]) {
                rotate([0, 0, angle])
                    translate([56.5, 0, 0])
                        cylinder(h = 3.0, r = 2.0, center = false);
            }
        }
        
        # Split Cut Plane along Y-axis (X = 0)
        if (is_right_half) {
            translate([-outer_r * 2, -outer_r * 2, -1])
                cube([outer_r * 2, outer_r * 4, stand_h + 4]);
        } else {
            translate([0, -outer_r * 2, -1])
                cube([outer_r * 2, outer_r * 4, stand_h + 4]);
        }
        
        // Alignment Pin Holes (3mm) on Split Faces (Y = +/- 40mm)
        for (y_pos = [-40, 40]) {
            translate([0, y_pos, stand_h / 2])
                rotate([0, 90, 0])
                    cylinder(h = 10, r = 1.6, center = true);
        }
    }
}

// Render Both Halves In Position (Separated by 2mm gap for visualization)
color("DarkCyan", 0.9)
    translate([-1, 0, 0]) Half_Calibration_Stand(is_right_half = false);

color("LightSeaGreen", 0.9)
    translate([1, 0, 0]) Half_Calibration_Stand(is_right_half = true);
