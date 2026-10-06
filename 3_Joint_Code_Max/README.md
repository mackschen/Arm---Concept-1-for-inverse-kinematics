# 3 DOF Arm MATLAB Simulation

Kinematics, dynamics and reach simulation for the URC rover's 3-joint arm.
Everything runs from a single script, `robot_arm_kinematics.m`.

From the parameters, the script generates:

- A forward/inverse kinematics check (first figure)
- A motor torque graph for a sample move, plus holding torques
- The reach envelope of the arm on the rover
- A test move between two points, with a torque graph and an animation

**Requirements:** MATLAB R2016b or newer. No toolboxes are needed.

**To run:** open `robot_arm_kinematics.m`, edit the parameters described below, and run it.

> Line numbers below refer to `robot_arm_kinematics.m`. They shift if lines are added or removed, so each one also lists the variable name to search for.

## 1. Arm parameters

| Line | Variable | Description |
|---|---|---|
| 12 | `arm.L` | Length of each link [m], `[L1 L2 L3]` |
| 13 | `arm.qmin` | Minimum angle of each joint, `[Joint 1, Joint 2, Joint 3]` (entered in degrees inside `deg2rad(...)`) |
| 14 | `arm.qmax` | Maximum angle of each joint, same order |
| 17 | `arm.m` | Mass of each link [kg], `[link 1, link 2, link 3]` |

### How the joint angles are measured

![Joint angle definitions](3_Joint_Code_Max/joint_angle_definitions.png)

- **Joint 1 (θ1)** is measured from horizontal (forward along the ground).
- **Joint 2 (θ2)** and **joint 3 (θ3)** are measured relative to the link before them (the dashed lines). 0° means the link is in line with the previous link.
- Positive angles rotate counterclockwise, which is upward when the arm reaches forward.

## 2. Load and mount parameters

| Line | Variable | Description |
|---|---|---|
| 22 | `arm.mp` | Mass of the load held by the gripper [kg]. `0` = no load. |
| 23 | `arm.lp` | Distance from the wrist joint (joint 3) to the load's center of mass [m]. See the note below. |
| 24 | `arm.Ip` | Inertia of the load about its own center of mass [kg·m²]. For a solid cube of side `s`: `arm.mp*(s^2 + s^2)/12`. |
| 34 | `mount.h` | Height of the arm base (joint 1) above the ground [m]. |
| 35 | `mount.clear` | Minimum clearance any part of the arm keeps above the ground [m]. |
| 36 | `mount.box` | Rover body keep-out rectangle `[xmin xmax ymin ymax]` [m]. Set to `[]` to ignore the rover body. |

**Note on `arm.lp`:** it's currently set to `arm.L(3)`, which puts the load's center of mass at the gripper tip. For a real load it's larger than the gripper length. For example, a 40 cm box gripped from the top has its center of mass about 0.20 m past the tip:

```matlab
arm.lp = arm.L(3) + 0.20;
```

### Coordinate system

All positions you enter are relative to the ground:

- **x = 0** is directly below the arm's base joint (joint 1), positive forward.
- **y = 0** is the ground, positive up.

## 3. Testing a move

You can test the arm going from one point to another. The output is a torque graph and an animation of the arm's solution.

| Line | Variable | Description |
|---|---|---|
| 133 | `testTarget` | Coordinates of the final position, `[x, height]` [m] |
| 134 | `testPhi` | Final angle of the gripper* |
| 137 | `testStartMode` | Type of starting point: `'position'` or `'angles'` |
| 138 | `testStartPos` | Coordinates of the starting position, `[x, height]` [m] |
| 139 | `testStartPhi` | Starting angle of the gripper* |
| 140 | `testStartElbow` | Preferred starting elbow, `'up'` or `'down'` |
| 141 | `qStartAngles` | Joint angles of the starting position |

Whether the code uses line 138 (position) or line 141 (angles) depends on the value of line 137.

\* **Why the gripper angle must be set:** with 3 motors and a position with only 2 values (x and height), the problem is undefined, because infinitely many arm poses reach the same point. The gripper angle must therefore be defined beforehand. It's measured from horizontal:

| Gripper angle | Meaning |
|---|---|
| `0°` | Pointing straight forward |
| `-45°` | Angled down |
| `-90°` | Pointing straight down (e.g. picking up off the ground) |

### If a move fails

The Command Window says why:

- The target is out of reach.
- Every solution violates the joint limits at that gripper angle.
- The solution collides with the ground or the rover body.
