# How To get rotation matrix from PRY angles

## Summary

- (1) The resulted matrix `R_A_B` is (new {B} expressed in old {A} frame), which means it may need **transpose** `R_A_B` first in order to calculate the position xyz in new frame (e.g., `P_B = R_B_A * P_A`).
- (2) Roll-X, Pitch-Y, Yaw-Z, always true.
- (3) Positive/negative rotation angles are based on right-hand rule, correct.
- (4) ZYX-euler angles are first rotate around Z, then Y, then X. (Remember, ZYX each time always rotate around the updated frame.)')
- (5) ZYX-euler angles are SAME as the RPY where Z(yaw)Y(pitch)X(roll).

## Code (MATLAB)

The following MATLAB code help you calculate rotation matrix (3x3) from RPY angles.

```
%teng4, this file calculate robot rotation matrix (3x3) based on the
%ZYX-Euler angles.
% from ECE464 ppt 4.2

% ZYX Euler angles are defined as
% (1) First,rotate reference frame {A.old} 'alpha' around Z-axis;
% (2) then, rotate the rotated frame 'beta' around its Y-axis;
% (3) then, rotate the latest frame 'gamma' around its X-axis, results is new frame {B};
% frame{A} is old, and frame{B} is new.
%R_A_B = Rzprime(alpha) * Ryprime(beta) * Rxprime(gamma).
% = [ca, -sa, 0;
%    sa,  ca, 0;
%     0,   0, 1] *
% [cb, 0, sb;
%   0, 1,  0;
% -sb, 0, cb] *
% [1,  0,   0;
%  0, cg, -sg;
%  0, sg,  cg]
%= [ ca.cb, ca.sb.sg-sa.cg, ca.sb.cg+sa.sg;
%    sa.cb, sa.sb.sg+ca.cg, sa.sb.cg-ca.sg;
%      -sb,          cb.sg,          cb.cg]

% Note that, this ZYX-Euler transformation is exactly same as that obtained
% for the same three rotations in the opposite order about the fixed
% angles (i.e., XYZ fixed angles)!
% Rzprime*yprime*xprime(alpha,beta,gamma) = Rxyz(gamma, beta, alpha)
% = Rzprime(alpha) * Ryprime(beta) * Rxprime(gamma)
% = Rz(alpha) * Ry(beta) * Rx(gamma)

roll_x  = -pi/2; %angles around x-axis.
pitch_y = 0;     %angles around y-axis.
yaw_z   = 0;     %angles around z-axis.

%based on ZYX-Euler angles above, we have,
gamma = roll_x;
beta  = pitch_y;
alpha = yaw_z;

g = gamma;
b = beta;
a = alpha;

R_A_B = [ cos(a)*cos(b), cos(a)*sin(b)*sin(g)-sin(a)*cos(g), cos(a)*sin(b)*cos(g)+sin(a)*sin(g);
          sin(a)*cos(b), sin(a)*sin(b)*sin(g)+cos(a)*cos(g), sin(a)*sin(b)*cos(g)-cos(a)*sin(g);
                -sin(b),                      cos(b)*sin(g),                     cos(b)*cos(g)];
%

disp('ZYX-Euler angles -   yaw(z),pitch(y); roll(x)')
disp('ZYX-Euler angles - alpha(z), beta(y);gamma(x)')
disp([alpha, beta, gamma])
disp('R_A_B =: ')
disp(R_A_B)

disp('----------')
disp('teng4 important note: (2024-09-12)')
disp('(1) The resulted matrix R_A_B is (new {B} expressed in old {A} frame),')
disp('      means it may need transpose R_A_B to calculate xyz in new frame.')
disp('(2) Roll-X, Pitch-Y, Yaw-Z, always true.')
disp('(3) Positive/negative rotation angles are based on right-hand rule, correct.')
disp('(4) ZYX-euler angles are first rotate around Z, then Y, then X.')
disp('      (Remember, ZYX each time always rotate around the updated frame.)')
disp('(5) ZYX-euler angles are SAME as the RPY where Z(yaw)Y(pitch)X(roll).')
```

------
Created on 2024-09-12.
