$fs = 0.05;
thickness = 2;

screw_diam = 2.5;
screw_drill = 2.5;
screw_clearance = 3.4;
screw_wall_thickness = 1.6;  // must be a multiple of nozzle size 0.4
beam_radius = screw_wall_thickness + screw_drill/2;

dispW = 61.189;
dispL = 34.087;
disp_mount = [
  [-dispW/2, 0],
  [dispW/2, 0],
  [dispW/2, dispL],
  [-dispW/2, dispL]
];

dispOffY = 13-27;
dispOffZ = 22.7 - 5;

RPi3MountSizX = 58;
RPi3MountSizY = 49;
RPi3MountOffsetX = 5;

module Beam(p0, p1) {
  difference() {
    union() {
      hull() {
        translate(p0) cylinder(r=beam_radius, h=4);
        translate(p1) translate([0, 0, -6]) cylinder(r=beam_radius, h=2);
      }
      translate(p1) translate([0, 0, -4]) cylinder(r=beam_radius, h=4);
    }
    translate(p1) translate([0, 0, -20]) cylinder(h=20.1, d=screw_drill);
  }
}

module OffsetMount(p0, p1, r) {
  difference() {
    linear_extrude(height=2) {
      hull() {
        translate(p0) circle(r=r);
        translate(p1) circle(r=r);
      }
    }
    translate([p0[0], p0[1], -0.1]) cylinder(d=screw_clearance, h=2+0.2);
  }
}

module mountpoint(x, y) {
  translate([x, y, -2]) cylinder(d=screw_drill-0.2, h=2);
}

module DispMount() {
  translate([0, dispOffY, dispOffZ]) for(m = disp_mount) mountpoint(m[0], m[1]);
  translate([-dispW/2-3.26, dispOffY+5.6, dispOffZ-8.8]) cube([3.26, 23, 8.8]);
}

module Thing() {
  mounts = [
    [-24, -6, 0],
    [40, -4, 0],
    [-24, RPi3MountSizY-6, 0],
    [40, RPi3MountSizY+6, 0],
  ];

  OffsetMount([RPi3MountOffsetX - RPi3MountSizX/2, 0, 0], mounts[0], beam_radius);
  OffsetMount([RPi3MountOffsetX + RPi3MountSizX/2, 0, 0], mounts[1], beam_radius);
  OffsetMount([RPi3MountOffsetX - RPi3MountSizX/2, RPi3MountSizY, 0], mounts[2], beam_radius);
  OffsetMount([RPi3MountOffsetX + RPi3MountSizX/2, RPi3MountSizY, 0], mounts[3], beam_radius);

  mounttable = [
    [0, [0, ], 0],
    [2, [0, 3], 0],
    [1, [1], 0],
    [1, [2], 2.5],
    [3, [2], 5],
  ];

  difference() {
    for (m = mounttable) {
      for (i = m[1]) {
        zoff = m[2];
        am = disp_mount[i];
        om = mounts[m[0]];
        om1 = [om[0], om[1], zoff];
        if (zoff > 0) {
          translate(om) cylinder(r=beam_radius, h=zoff);
        }
        Beam(om1, [am[0], am[1] + dispOffY, dispOffZ]);
      }
    }
    translate([-dispW/2-3.26, dispOffY+5.1, dispOffZ-10]) cube([4, 24, 10]);
  }
}

%DispMount();
Thing();
