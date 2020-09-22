#!/bin/bash

rosservice call /base/undercarriage_drive_mode "mode: $1
params:
  velocity_point_x: 0.0
  velocity_point_y: 0.0
  icr: {x: $2, y: $3}
  virtual_axle: {x: $4, y: $5, a: $6}
  r_min: $7
  p: 0.0
  castor: {alpha: 0.0, beta: 0.0, d: 0.0, l: 0.0, r: 0.0}
  a_and_b: {a11: 0.0, a12: 0.0, a13: 0.0, a21: 0.0, a22: 0.0, a23: 0.0, a31: 0.0,
    a32: 0.0, a33: 0.0, b1: 0.0, b2: 0.0, b3: 0.0}" 
