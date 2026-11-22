#!/usr/bin/env python3
"""Generate the Gazebo Harmonic world (table, basket, objects, UR5, camera)
from app/ur5vla/{kinematics,scene}.py.   usage: gen_world.py OUT.sdf"""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "app"))
from ur5vla import kinematics as K, scene as S

WORLD = "demo"
MODEL = "ur5"
RADII = [0.075, 0.062, 0.050, 0.042, 0.038, 0.036, 0.034]
MASSES = [4.0, 3.7, 8.4, 2.3, 1.2, 1.2, 0.2]
EFFORT = [150, 150, 150, 28, 28, 28]
KP = 40.0


def f(v):
    return f"{v:.6f}"


def pose(xyz, rpy=(0, 0, 0), rel=None):
    r = f" relative_to='{rel}'" if rel else ""
    return f"<pose{r}>{f(xyz[0])} {f(xyz[1])} {f(xyz[2])} {f(rpy[0])} {f(rpy[1])} {f(rpy[2])}</pose>"


def mat(c, a=1.0):
    return (f"<material><ambient>{c[0]} {c[1]} {c[2]} {a}</ambient><diffuse>{c[0]} {c[1]} {c[2]} {a}</diffuse>"
            f"<specular>0.2 0.2 0.2 1</specular></material>")


def segment_visual(name, p0, p1, radius, color):
    p0, p1 = np.array(p0, float), np.array(p1, float)
    v = p1 - p0
    L = float(np.linalg.norm(v))
    if L < 1e-6:
        return ""
    z = v / L
    a = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(a, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    rpy = K.R_to_rpy(R)
    mid = (p0 + p1) / 2
    return (f"<visual name='{name}'>{pose(mid, rpy)}<geometry><cylinder><radius>{f(radius)}</radius>"
            f"<length>{f(L)}</length></cylinder></geometry>{mat(color)}</visual>")


def sphere_visual(name, p, radius, color):
    return (f"<visual name='{name}'>{pose(p)}<geometry><sphere><radius>{f(radius)}</radius></sphere></geometry>"
            f"{mat(color)}</visual>")


def inertial(m):
    i = max(0.004, m * 0.01)
    return (f"<inertial><mass>{m}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>"
            f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")


def robot_sdf():
    blue, grey, dark = (0.20, 0.45, 0.80), (0.70, 0.72, 0.75), (0.12, 0.12, 0.12)
    col = [dark, blue, blue, blue, grey, grey, grey]
    out = [f"<model name='{MODEL}'>", pose(S.BASE_WORLD)]
    # child origins expressed in each link's own frame
    child = [j[1] for j in K.JOINTS] + [K.TOOL0_XYZ]
    for i, ln in enumerate(K.LINK_NAMES):
        rel = None if i == 0 else K.LINK_NAMES[i - 1]
        if i == 0:
            p = pose((0, 0, 0))
        else:
            p = pose(K.JOINTS[i - 1][1], K.JOINTS[i - 1][2], rel=rel)
        vis = segment_visual(f"seg", (0, 0, 0), child[i], RADII[i], col[i])
        vis += sphere_visual("joint", (0, 0, 0), RADII[i] * 1.12, col[i])
        if ln == "wrist_3_link":    # flange + suction cup
            tip = np.array(K.TOOL0_XYZ) + np.array([0, K.CUP_LENGTH, 0])
            vis += segment_visual("cup", K.TOOL0_XYZ, tip, 0.022, dark)
            vis += sphere_visual("cup_tip", tip, 0.026, dark)
        out.append(f"<link name='{ln}'>{p}<gravity>false</gravity>{inertial(MASSES[i])}{vis}</link>")
    out.append(f"<joint name='world_fix' type='fixed'><parent>world</parent><child>base_link</child></joint>")
    for i, (jn, xyz, rpy, axis) in enumerate(K.JOINTS):
        out.append(
            f"<joint name='{jn}' type='revolute'><parent>{K.LINK_NAMES[i]}</parent><child>{K.LINK_NAMES[i+1]}</child>"
            f"<axis><xyz>{axis[0]} {axis[1]} {axis[2]}</xyz><limit><lower>-6.2832</lower><upper>6.2832</upper>"
            f"<effort>{EFFORT[i]}</effort><velocity>3.14</velocity></limit><dynamics><damping>1.0</damping></dynamics></axis></joint>")
        out.append(
            f"<plugin filename='gz-sim-joint-position-controller-system' name='gz::sim::systems::JointPositionController'>"
            f"<joint_name>{jn}</joint_name><topic>/{MODEL}/cmd/{jn}</topic><p_gain>{KP}</p_gain><i_gain>0.0</i_gain>"
            f"<d_gain>0.0</d_gain><cmd_max>3.0</cmd_max><cmd_min>-3.0</cmd_min></plugin>")
    out.append(
        f"<plugin filename='gz-sim-joint-state-publisher-system' name='gz::sim::systems::JointStatePublisher'>"
        f"<topic>/{MODEL}/joint_states</topic></plugin>")
    for o in S.OBJECTS:   # suction grasp = detachable fixed joint to the cup link
        out.append(
            f"<plugin filename='gz-sim-detachable-joint-system' name='gz::sim::systems::DetachableJoint'>"
            f"<parent_link>wrist_3_link</parent_link><child_model>{o.name}</child_model><child_link>link</child_link>"
            f"<attach_topic>/{MODEL}/attach/{o.name}</attach_topic><detach_topic>/{MODEL}/detach/{o.name}</detach_topic>"
            f"<output_topic>/{MODEL}/attached/{o.name}</output_topic></plugin>")
    out.append("</model>")
    return "\n".join(out)


def box_geom(sx, sy, sz):
    return f"<geometry><box><size>{f(sx)} {f(sy)} {f(sz)}</size></box></geometry>"


def static_box(name, center_world, size, color):
    return (f"<link name='{name}'>{pose(center_world)}"
            f"<collision name='c'>{box_geom(*size)}</collision>"
            f"<visual name='v'>{box_geom(*size)}{mat(color)}</visual></link>")


def basket_sdf():
    bx, by = S.BASKET_CENTER
    zw = S.TABLE_TOP_Z
    i_, t, h, ft = S.BASKET_INNER, S.BASKET_WALL_T, S.BASKET_WALL_H, S.BASKET_FLOOR_T
    outer = i_ + 2 * t
    c = (0.55, 0.38, 0.20)
    parts = ["<model name='basket'><static>true</static>"]
    parts.append(static_box("floor", (bx, by, zw + ft / 2), (outer, outer, ft), c))
    zc = zw + ft + h / 2
    parts.append(static_box("wall_n", (bx, by + (i_ + t) / 2, zc), (outer, t, h), c))
    parts.append(static_box("wall_s", (bx, by - (i_ + t) / 2, zc), (outer, t, h), c))
    parts.append(static_box("wall_e", (bx + (i_ + t) / 2, by, zc), (t, i_, h), c))
    parts.append(static_box("wall_w", (bx - (i_ + t) / 2, by, zc), (t, i_, h), c))
    parts.append("</model>")
    return "".join(parts)


def object_sdf(o):
    if o.shape == "box":
        g = box_geom(*o.size)
        m = o.mass
        i = m * (o.size[0] ** 2 + o.size[1] ** 2) / 12
        iz = i
    else:
        r, hh = o.size[0], o.size[1]
        g = f"<geometry><cylinder><radius>{f(r)}</radius><length>{f(hh)}</length></cylinder></geometry>"
        m = o.mass
        i = m * (3 * r * r + hh * hh) / 12
        iz = m * r * r / 2
    x, y = o.pos
    z = S.TABLE_TOP_Z + S.initial_z(o)
    fr = "<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>"
    return (f"<model name='{o.name}'>{pose((x, y, z))}<link name='link'>"
            f"<inertial><mass>{m}</mass><inertia><ixx>{f(i)}</ixx><iyy>{f(i)}</iyy><izz>{f(iz)}</izz>"
            f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
            f"<collision name='c'>{g}{fr}</collision><visual name='v'>{g}{mat(o.color)}</visual></link></model>")


def camera_sdf():
    p = (S.CAM_POS[0], S.CAM_POS[1], S.CAM_POS[2] + S.TABLE_TOP_Z)
    return (f"<model name='overview_cam'><static>true</static>{pose(p, (0, S.CAM_PITCH, S.CAM_YAW))}"
            f"<link name='link'><sensor name='camera' type='camera'><always_on>1</always_on><update_rate>8</update_rate>"
            f"<visualize>false</visualize><topic>/camera/image</topic><camera><horizontal_fov>{S.CAM_FOV}</horizontal_fov>"
            f"<image><width>{S.CAM_W}</width><height>{S.CAM_H}</height><format>R8G8B8</format></image>"
            f"<clip><near>0.1</near><far>10</far></clip></camera></sensor></link></model>")


def world_sdf():
    tx, ty, th = S.TABLE_SIZE
    cx, cy = S.TABLE_CENTER
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
<world name="{WORLD}">
  <physics name="fast" type="dart"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <gravity>0 0 -9.81</gravity>
  <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.78 0.80 0.84 1</background><shadows>true</shadows></scene>
  <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>1 -1 4 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.3 -1</direction></light>
  <model name="floor"><static>true</static><link name="link"><pose>0 0 -0.05 0 0 0</pose>
    <collision name="c"><geometry><box><size>12 12 0.1</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>12 12 0.1</size></box></geometry>{mat((0.62, 0.64, 0.67))}</visual></link></model>
  <model name="table"><static>true</static>
    {static_box("top", (cx, cy, th - 0.02), (tx, ty, 0.04), (0.55, 0.40, 0.27))}
    {static_box("leg_a", (cx - tx/2 + 0.05, cy - ty/2 + 0.05, (th - 0.04) / 2), (0.06, 0.06, th - 0.04), (0.35, 0.25, 0.17))}
    {static_box("leg_b", (cx + tx/2 - 0.05, cy - ty/2 + 0.05, (th - 0.04) / 2), (0.06, 0.06, th - 0.04), (0.35, 0.25, 0.17))}
    {static_box("leg_c", (cx - tx/2 + 0.05, cy + ty/2 - 0.05, (th - 0.04) / 2), (0.06, 0.06, th - 0.04), (0.35, 0.25, 0.17))}
    {static_box("leg_d", (cx + tx/2 - 0.05, cy + ty/2 - 0.05, (th - 0.04) / 2), (0.06, 0.06, th - 0.04), (0.35, 0.25, 0.17))}
  </model>
  {basket_sdf()}
  {chr(10).join(object_sdf(o) for o in S.OBJECTS)}
  {robot_sdf()}
  {camera_sdf()}
</world>
</sdf>
"""


def gui_camera_cmd():
    """gz service call that moves the GUI camera to a nice overview."""
    eye = np.array([1.7, -1.5, S.TABLE_TOP_Z + 1.1])
    tgt = np.array([0.25, -0.05, S.TABLE_TOP_Z + 0.1])
    d = tgt - eye
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))     # gz: positive pitch looks down
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    q = (-sp * sy, sp * cy, cp * sy, cp * cy)             # quaternion x,y,z,w for rpy=(0,pitch,yaw)
    return ("gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 "
            f"--req 'pose: {{position: {{x: {eye[0]:.3f}, y: {eye[1]:.3f}, z: {eye[2]:.3f}}}, "
            f"orientation: {{x: {q[0]:.4f}, y: {q[1]:.4f}, z: {q[2]:.4f}, w: {q[3]:.4f}}}}}'")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "demo.sdf"
    with open(out, "w") as fh:
        fh.write(world_sdf())
    with open(os.path.splitext(out)[0] + "_guicam.sh", "w") as fh:
        fh.write("#!/bin/sh\n" + gui_camera_cmd() + "\n")
    print("wrote", out)
