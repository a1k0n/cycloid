$fs=0.1; 

screw_diam = 2.5;

camera_angle = [0, 22, 0];
// i messed up the hole and it's 2mm towards the left side of the car
camera_pos = [29.5+3, 2, 25-5];
camboard_mounting = [
  [-21/2, -13.5/2, 0],
  [-21/2, 13.5/2,  0],
  [ 21/2, -13.5/2, 0],
  [ 21/2, 13.5/2,  0],

//  [-13.5/2, -21/2, 0],
//  [13.5/2, -21/2, 0],
//  [-13.5/2, 21/2, 0],
//  [13.5/2, 21/2, 0],
];

// we're going to attach the imu to a hat
// imu_mounting = [10, 0, 5];

rpiz_pos = [-12/2-11-7.5, 0, 4];
rpiz_mounting = [
  [-58/2, -23/2, 0],
  [58/2, -23/2, 0],
  [-58/2, 23/2, 0],
  [58/2, 23/2, 0],
];

module reference_parts() {
  // two 2mm diameter slots, 18mm wide, 12mm apart
  difference() {
    translate([-85/2, -13/2, -1.6]) cube([85, 13, 1.6]);
    translate([0, 0, -1.7]) linear_extrude(2) hull() {
      translate([-12/2-18, 0, -0.0]) circle(d=2);
      translate([-12/2, 0, -0.0]) circle(d=2);
    }
    translate([0, 0, -1.7]) linear_extrude(2) hull() {
      translate([12/2+18, 0]) circle(d=2);
      translate([12/2, 0]) circle(d=2);
    }
    translate([-12/2-18-7.25, 0, -1.7]) cylinder(d=3.5, h=3);
    translate([-12/2-18-7.25-8.75, -7.75/2, -1.7]) cylinder(d=2.0, h=3);
    translate([-12/2-18-7.25-8.75, 7.75/2, -1.7]) cylinder(d=2.0, h=3);
  }
  // camera center
  translate(camera_pos) {
    rotate(camera_angle) {
      sphere(d=15);
      difference() {
        translate([-18/2 - (25-18), -25/2, -8]) cube([25, 25, 1]);
        translate([0, 0, -8.1]) for (h = camboard_mounting) {
          translate(h) cylinder(d=2.25, h=10);
        }
      }
      translate([0, 0, -7]) cylinder(d=15, h=11);
    }
  }

  /*
  translate(imu_mounting) {
    difference() {
      translate([-16/2, -21/2, 0]) cube([16, 21, 1.6]);
      translate([8-2.5, -15/2, -0.1]) cylinder(d=3, h=2);
      translate([8-2.5, 15/2, -0.1]) cylinder(d=3, h=2);
    }
  }
  */

  translate(rpiz_pos) {
    difference() {
      union() {
        translate([-65/2, -30/2, 0]) cube([65, 30, 1.6]);
        translate([-65/2, -30/2, 8.5+1.6]) cube([65, 30, 1.6]);
        translate([65/2, -6, 2.2]) cube([9, 12, 0.5]);
      }
      translate([0, 0, -0.1]) for (h = rpiz_mounting) {
        translate(h) cylinder(d=2.75, h=20);
      }
    }
  }
}

module floorbeam(p0, d0, p1, d1, ang1) {
  hull() {
    translate(p0) intersection() { sphere(d=d0); translate([-d0/2, -d0/2, 0]) cube([d0, d0, d0/2]); }
    translate([p1[0], p1[1], p0[2]]) intersection() { sphere(d=d0); translate([-d0/2, -d0/2, 0]) cube([d0, d0, d0/2]); }
    translate(p1) intersection() { sphere(d=d1); translate([-d1/2, -d1/2, -d1/2]) cube([d1, d1, d1/2]); }
  }
}

module anglebeam(p0, d0, p1plane, ang1, p1, d1) {
  hull() {
    translate(p0) intersection() { sphere(d=d0); translate([-d0/2, -d0/2, 0]) cube([d0, d0, d0/2]); }
    translate(p1plane) rotate(ang1) translate(p1) intersection() { sphere(d=d1); translate([-d1/2, -d1/2, -d1/2]) cube([d1, d1, d1/2]); }
  }
}

module mounting() {
  // first, make a plate with holes in it which just goes across the existing one
  basethick = 1.5;
  difference() {
    union() {
      linear_extrude(basethick) {
        hull() {
          translate([-34, 0, 0]) circle(d=8);
          translate([65/2, 0, 0]) circle(d=8);
        }
      }
      hull() {
        translate([-12/2-18-7.25, -3, 0]) cylinder(d=13, h=basethick);
        translate([-12/2-18-7.25, 3, 0]) cylinder(d=13, h=basethick);
      }
      hull() {
        translate([rpiz_mounting[0][0] + rpiz_pos[0], 0, 0]) cylinder(d=2, h=basethick);
        translate([-34, 0, 0]) cylinder(d=2, h=basethick);
      }
      translate([rpiz_mounting[0][0] + rpiz_pos[0] + 4, 0, 0]) cylinder(r=5, h=basethick);
      floorbeam([rpiz_mounting[0][0] + rpiz_pos[0], 0, 0], 7, rpiz_mounting[0]+rpiz_pos, 6);
      floorbeam([rpiz_mounting[2][0] + rpiz_pos[0], 0, 0], 7, rpiz_mounting[2]+rpiz_pos, 6);
      floorbeam([58/2-12/2-10, -1, 0], 7, rpiz_mounting[1]+rpiz_pos, 6);
      floorbeam([58/2-12/2-10, 1, 0], 7, rpiz_mounting[3]+rpiz_pos, 6);

      anglebeam([65/2, -1, basethick], 6, camera_pos, camera_angle, camboard_mounting[1] - [0,, 0, 8], 5);
      anglebeam([65/2, 1, basethick], 6, camera_pos, camera_angle, camboard_mounting[3] - [0,, 0, 8], 5);
      anglebeam([65/2, -1, basethick], 6, camera_pos, camera_angle, camboard_mounting[0] - [0,, 0, 8], 5);
      anglebeam([65/2, 1, basethick], 6, camera_pos, camera_angle, camboard_mounting[2] - [0,, 0, 8], 5);
    }
    for (hx = [-12/2-18, -12/2, 12/2, 12/2+18]) {
      translate([hx, 0, -0.1]) cylinder(d=2.5, h=10);
    }
    translate(rpiz_pos - [0, 0, 6.1]) for (h = rpiz_mounting) {
      translate(h) cylinder(d=screw_diam, h=20);
      translate(h) cylinder(d=5.9, h=4, $fn=6);
    }

    translate(camera_pos) rotate(camera_angle) translate([0, 0, -20]) for (h = camboard_mounting) {
      translate(h) cylinder(d=screw_diam, h=30);
      translate(h) cylinder(d=4.5, h=10, $fn=6);
    }
    translate([-12/2-18-7.25, 0, -0.1]) cylinder(d=6, h=basethick+2);
  }
}

%reference_parts();
mounting();
