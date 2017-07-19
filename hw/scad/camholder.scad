thickness = 7;
cam_border = 3;
wide_cam_width = 25.15;
wide_cam_height = 24.15;
wide_cam_mount = [
  [-wide_cam_width/2 + 2.1, wide_cam_height/2 - 2.1],
  [wide_cam_width/2 - 2.1, wide_cam_height/2 - 2.1],
  [-wide_cam_width/2 + 2.1, wide_cam_height/2 - 14.6 - 1.1],
  [wide_cam_width/2 - 2.1, wide_cam_height/2 - 14.6 - 1.1],
];
wide_cam_lens_diam = 16.4;
screw_diam = 2.1;
screw_wall_thickness = 0.801;  // must be a multiple of nozzle size 0.4
support_offset = 4;

module CameraMount() {
  difference() {
    union() {
      translate([-(cam_border + wide_cam_width)/2, -(cam_border + wide_cam_height)/2, 0])
        cube([cam_border + wide_cam_width, cam_border + wide_cam_height, thickness]);
      for (h = wide_cam_mount) {
        translate([h[0], h[1], thickness - 0.1]) {
          cylinder(d=screw_diam + 2*screw_wall_thickness, h=support_offset + 0.1, $fn=20);
          cylinder(d=screw_diam + 4*screw_wall_thickness, h=support_offset/2 + 0.1, $fn=20);
        }
      }
    }
    for (h = wide_cam_mount) {
      translate([h[0], h[1], -0.1]) {
        cylinder(d=screw_diam, h=thickness + 10, $fn=10);
      }
    }
    // hole for lens
    translate([0, wide_cam_height/2 - 8.9, -0.1])
      cylinder(d=wide_cam_lens_diam, h=thickness + 3);
  }
}

module BumperSpar(sparlen) {
  rotate([90, 0, 0]) rotate([0, 90, 0]) linear_extrude(height = 2)
    polygon(points=[
      [-1.72, 3.0],
      [sparlen, 3], [sparlen, -0.1], [0, -0.1], [-1.72, 3.0]]);
}

bb_len = 105;
module BumperBracket() {
  // 30.75 inner to inner
  // 3.5mm diameter holes (we will probably have to drill out)
  // 60 deg angle from vertical
  difference() {
    union() {
      rotate([-60, 0, 0]) {
        translate([-25, -20, 0])
          cube([50, 20, thickness]);
      }
      linear_extrude(height = thickness)
        polygon(points=[
            [-25, 0],
            [-(cam_border + wide_cam_width)/2, bb_len],
            [(cam_border + wide_cam_width)/2, bb_len],
            [25, 0], [-25, 0]]);
    }
    // translate([-12, 0, 1.0]) BumperSpar(bb_len - 5);
    // translate([10, 0, 1.0]) BumperSpar(bb_len - 5);
    rotate([-60, 0, 0]) {
      translate([-30.75/2 - 3.5/2, -6, -0.1])
        cylinder(d=4.5, h=thickness + 1, $fn=10);
      translate([-30.75/2 - 3.5/2, -6, thickness-0.1])
        cylinder(d=9, h=thickness + 1, $fn=20);
      translate([30.75/2 + 3.5/2, -6, -0.1])
        cylinder(d=4.5, h=thickness + 1, $fn=10);
      translate([30.75/2 + 3.5/2, -6, thickness-0.1])
        cylinder(d=9, h=thickness + 1, $fn=20);
    }
  }
}

BumperBracket();
translate([0, bb_len+10, 0]) CameraMount();
