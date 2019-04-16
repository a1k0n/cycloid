// exceed magnet mounting frame

$fs = 0.05;

screw_drill = 2.5;
screw_clearance = 3.4;
screw_wall_thickness = 1.6;  // must be a multiple of nozzle size 0.4

beam_radius = screw_wall_thickness + screw_drill/2;

module mountpoint(x, y) {
  translate([x, y, -2]) cylinder(d=screw_drill-0.2, h=2);
}

mounts = [  // x, y, diam
  [-17.75/2, 0],
  [17.75/2, 0],
  [11.75, 13.25],
  [19.5, 63.5],
  [19.5, 63.5+47],
  [15.5/2, 63.5+47+15],
  [-15.5/2, 63.5+47+15],
];

offsetmounts = [  // x, y, diam
  [-17.75/2 - 7, 0],
  [17.75/2 + 7, 0],
  //[11-4, 11.7+8],
  [5, 35],

  [20, 63.5-10],
  [-25, 63.5-30],
  [5, 35]
];

RPi3MountSizX = 58;
RPi3MountSizY = 49;
RPi3MountOffsetX = 5;
RPi3MountOffsetY = 10;
RPi3MountOffsetZ = 15;

RPi3Mounts = [
  [-RPi3MountSizX/2 + RPi3MountOffsetX, RPi3MountOffsetY, RPi3MountOffsetZ],
  [ RPi3MountSizX/2 + RPi3MountOffsetX, RPi3MountOffsetY, RPi3MountOffsetZ],
  [-RPi3MountSizX/2 + RPi3MountOffsetX, RPi3MountOffsetY+RPi3MountSizY, RPi3MountOffsetZ],
  [ RPi3MountSizX/2 + RPi3MountOffsetX, RPi3MountOffsetY+RPi3MountSizY, RPi3MountOffsetZ],
];

arducam_mount = [
  [-28.5/2, 28.5/2],
  [-28.5/2, -28.5/2],
  [28.5/2, -28.5/2],
  [28.5/2, 28.5/2],
];

CamMountOffsetY = 118;
CamMountOffsetZ = 50;

module BaseplateMounts() {
  for (m = mounts) {
    mountpoint(m[0], m[1]);
  }
  // cutout for spur gear
  translate([-25/2, 3, -3]) cube([25, 7, 4]);
  translate([0, 4, -30/2]) rotate([-90, 0, 0]) cylinder(h=5, d=35);
  // cutout for motor heat sink
  translate([18, 22, -16]) rotate([-90, 0, 0]) cylinder(h=22, d=38);
  // ESC
  translate([-19, 73.5, 0]) cube([25, 39, 17]);
  translate([-19, 80, 0]) cube([35, 20, 17]);
}

module RpiMounts() {
  for (m = RPi3Mounts) {
    translate([0, 0, RPi3MountOffsetZ])
      mountpoint(m[0], m[1]);
  }
}

module Beam(p0, p1) {
  difference() {
    union() {
      hull() {
        translate(p0) cylinder(r=beam_radius, h=4);
        translate(p1) translate([0, 0, -6]) cylinder(r=beam_radius, h=2);
      }
      translate(p1) translate([0, 0, -4]) cylinder(r=beam_radius, h=4);
    }
    translate(p1) translate([0, 0, -7]) cylinder(h=7.1, d=screw_drill);
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

module Template() {
  OffsetMount(mounts[0], [0, 0], beam_radius);
  OffsetMount(mounts[1], [0, 0], beam_radius);
  OffsetMount(mounts[1], [11, 5], beam_radius);
  OffsetMount(mounts[2], [11, 5], beam_radius);
  OffsetMount(mounts[2], [5, 35], beam_radius);
  OffsetMount(mounts[3], [5, 35], beam_radius);
  OffsetMount(mounts[3], [15, 63+23], beam_radius);
  OffsetMount(mounts[4], [15, 63+23], beam_radius);
  OffsetMount(mounts[4], [0, 63+47+10], beam_radius);
  OffsetMount(mounts[5], [0, 63+47+10], beam_radius);
  OffsetMount(mounts[6], [0, 63+47+10], beam_radius);
}

module CrapMount() {
  for (i = [0 : 4]) {
    if (i < 4) {
      OffsetMount(mounts[i], offsetmounts[i], beam_radius);
    }
    for (x = [-RPi3MountSizX/2, RPi3MountSizX/2]) {
      for (y = [0, RPi3MountSizY]) {
        Beam(offsetmounts[i], [x, y+RPi3MountOffsetY, RPi3MountOffsetZ]);
      }
    }
  }
}

module PiMount() {
  OffsetMount(mounts[0], [0, 0], beam_radius);
  OffsetMount(mounts[1], [0, 0], beam_radius);
  OffsetMount(mounts[1], [11, 5], beam_radius);
  OffsetMount(mounts[2], [11, 5], beam_radius);
  OffsetMount(offsetmounts[3], offsetmounts[4], beam_radius);
  OffsetMount(mounts[2], offsetmounts[5], beam_radius);
  OffsetMount(mounts[3], offsetmounts[5], beam_radius);

  OffsetMount(mounts[0], offsetmounts[0], beam_radius);
  OffsetMount(mounts[1], offsetmounts[1], beam_radius);
  OffsetMount(mounts[2], offsetmounts[2], beam_radius);
  OffsetMount(mounts[3], offsetmounts[3], beam_radius);

  mounttable = [
    [0, [0]],
    [1, [1]],
    [2, [0, 1]],
    [3, [3]],
    [4, [0, 2]],
    [5, [1, 3]],
  ];
  for (m = mounttable) {
    for (i = m[1]) {
      Beam(offsetmounts[m[0]], RPi3Mounts[i]);
    }
  }
}

module CamMount() {
  OffsetMount(mounts[5], [0, 125.5], beam_radius);
  OffsetMount(mounts[6], [0, 125.5], beam_radius);

  offsetmounts2 = [
    [20, 104],
    [12, 119.5],
    [-12, 119.5],
  ];

  OffsetMount(mounts[4], offsetmounts2[0], beam_radius);
  OffsetMount(mounts[4], offsetmounts2[1], beam_radius);
  OffsetMount(mounts[5], offsetmounts2[1], beam_radius);
  OffsetMount(mounts[6], offsetmounts2[2], beam_radius);

  mounttable = [
    [0, [2], 0],
    [1, [2, 3], 0],
    [2, [0, 3], 0],
    [2, [1, 2], 20],
  ];

  for (m = mounttable) {
    for (i = m[1]) {
      zoff = m[2];
      am = arducam_mount[i];
      om = offsetmounts2[m[0]];
      om1 = [om[0], om[1], zoff];
      if (zoff > 0) {
        translate(om) cylinder(r=beam_radius, h=zoff);
      }
      Beam(om1, [am[0], am[1] + CamMountOffsetY, CamMountOffsetZ]);
    }
  }
}

%BaseplateMounts();
%RpiMounts();
*Template();
PiMount();
CamMount();
