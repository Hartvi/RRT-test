import bpy
import os
import math
import csv


#create a scene
# scene = bpy.data.scenes.new("Scene")
# camera_data = bpy.data.cameras.new("Camera")


# constr = camera.constraints.new(type="TRACK_TO")
"""
 bpy.data.objects["Camera"].constraints.new("LOL")
Traceback (most recent call last):
  File "/usr/lib/python3.8/code.py", line 90, in runcode
    exec(code, self.locals)
  File "<blender_console>", line 1, in <module>
TypeError: ObjectConstraints.new(): error with argument 1, "type" -  enum "LOL" not found in (, 'CAMERA_SOLVER', 'FOLLOW_TRACK', 'OBJECT_SOLVER', 'COPY_LOCATION', 'COPY_ROTATION', 'COPY_SCALE', 'COPY_TRANSFORMS', 'LIMIT_DISTANCE', 'LIMIT_LOCATION', 'LIMIT_ROTATION', 'LIMIT_SCALE', 'MAINTAIN_VOLUME', 'TRANSFORM', 'TRANSFORM_CACHE', 'CLAMP_TO', 'DAMPED_TRACK', 'IK', 'LOCKED_TRACK', 'SPLINE_IK', 'STRETCH_TO', 'TRACK_TO', 'ACTION', 'ARMATURE', 'CHILD_OF', 'FLOOR', 'FOLLOW_PATH', 'PIVOT', 'SHRINKWRAP')
"""

def cylinder_between(x1, y1, z1, x2, y2, z2, r, mat=None, reuse_cyl=None):
    """
    https://blender.stackexchange.com/questions/5898/how-can-i-create-a-cylinder-linking-two-points-with-python

    reuse_cyl : cylinder object to reuse (instead of creating a new one)
    """
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    dist = math.sqrt(dx**2 + dy**2 + dz**2)

    if dist == 0:
        return

    phi = math.atan2(dy, dx)
    theta = math.acos(dz / dist)

    if reuse_cyl is None:
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=8, radius=r, depth=1, location=(dx / 2 + x1, dy / 2 + y1, dz / 2 + z1)
        )

        cyl = bpy.context.object
        if mat is not None:
            cyl.data.materials.append(mat)

    else:
        cyl = reuse_cyl
        cyl.location = (dx / 2 + x1, dy / 2 + y1, dz / 2 + z1)

    cyl.rotation_euler[1] = theta
    cyl.rotation_euler[2] = phi
    cyl.scale = (1.0, 1.0, dist)

    return cyl


def look_at_object(obj_name="Cube"):
    track_to_constraint = bpy.data.objects["Camera"].constraints.new("TRACK_TO")
    track_to_constraint.target = bpy.data.objects[obj_name]
    track_to_constraint.up_axis = 'UP_Y'
    track_to_constraint.track_axis = "TRACK_NEGATIVE_Z"


def save_still(file_path="/home/hartvi/Pictures/img.png"):
    bpy.context.scene.render.filepath = file_path
    bpy.context.scene.render.resolution_x = 800
    bpy.context.scene.render.resolution_y = 600
    bpy.ops.render.render(write_still=True)




def add_and_set_color(obj, color, mat=None):
    # Add new material
    if mat is None:
        mat = bpy.data.materials.new(name=f"Material")
    obj.data.materials.append(mat)

    # Enable "Use nodes" and add Vertex Color Node
    mat.use_nodes = True
    nodes = mat.node_tree.nodes

    # Set the color and alpha parameters
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs[0].default_value = color  # color
    bsdf.inputs[21].default_value = color[3]  # alpha


def create_point(location, color, mat=None):
    # Create uv_sphere at `location` and assing a new material with `color` to it
    bpy.ops.mesh.primitive_uv_sphere_add(location=location, radius=0.2)
    obj = bpy.context.active_object
    obj.location = location
    add_and_set_color(obj, color, mat)



"""
track_to_constraint = bpy.data.objects["Camera"].constraints.new("TRACK_TO")
track_to_constraint.target = bpy.data.objects["Cube"]
track_to_constraint.up_axis = 'UP_Y'
track_to_constraint.track_axis = "TRACK_NEGATIVE_Z"


bpy.context.scene.render.filepath = "/home/hartvi/Pictures/img.png"
bpy.context.scene.render.resolution_x = 800
bpy.context.scene.render.resolution_y = 600
bpy.ops.render.render(write_still=True)


"""