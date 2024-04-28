import bpy
import math
import random
import mathutils
import csv
import sys
import os.path
time = []

# Total number of parts of the rover
num_body = 15

# Radius of the particle 
radius_particle = 0.005

# Resolution of the output image
res_x = 1920
res_y = 1080

#============ specify the directory of csv, obj, image, and such
data_sim = "./"
file_obj = "./obj_for_render/"
image_dir = "./"

#============ find the position that the camera points at
def point_at(obj, target, roll=0):
    """
    Rotate obj to look at target
    :arg obj: the object to be rotated. Usually the camera
    :arg target: the location (3-tuple or Vector) to be looked at
    :arg roll: The angle of rotation about the axis from obj to target in radians. 
    """
    if not isinstance(target, mathutils.Vector):
        target = mathutils.Vector(target)
    loc = obj.location
    # direction points from the object to the target
    direction = target - loc

    quat = direction.to_track_quat('-Z', 'Y')
    
    # /usr/share/blender/scripts/addons/add_advanced_objects_menu/arrange_on_curve.py
    quat = quat.to_matrix().to_4x4()
    rollMatrix = mathutils.Matrix.Rotation(roll, 4, 'Z')

    # remember the current location, since assigning to obj.matrix_world changes it
    loc = loc.to_tuple()
    #obj.matrix_world = quat * rollMatrix
    # in blender 2.8 and above @ is used to multiply matrices
    # using * still works but results in unexpected behaviour!
    obj.matrix_world = quat @ rollMatrix
    obj.location = loc

#============ find the position and rotation of the camera
def rot_pos(center, radius, angle=0):
    """
    return (x,y,z) points on a circle centered at (center) with a radius of (radius) with an angle of (angle)
    """
    # convert to radian
    angle_pi = angle/180*math.pi
    # generate point on circle
    return (center[0]+radius*math.sin(angle_pi),center[1]+radius*math.cos(angle_pi),center[2])

#============ load position, velocity, rotation of each body at different time
dis = []
rot = []
for i in range(num_body):
    with open(data_sim + "/rover/body_pos_rot_vel" + str(i+1) + ".csv", 'r') as file:
        dis_temp = []
        rot_temp = []
        reader = csv.reader(file)
        i = 0
        for row in reader:
            i=i+1
            if i!=1:
                time.append(float(row[0]))
                dis_buff = []
                dis_buff.append(float(row[1]))
                dis_buff.append(float(row[2]))
                dis_buff.append(float(row[3]))
                rot_buff = []
                rot_buff.append(float(row[4]))
                rot_buff.append(float(row[5]))
                rot_buff.append(float(row[6]))
                rot_buff.append(float(row[7]))
                dis_temp.append(dis_buff)
                rot_temp.append(rot_buff)
        dis.append(dis_temp)
        rot.append(rot_temp)

#===========================================
#============================ Start the loop
#===========================================
jobid = int(sys.argv[4])
start_frame = jobid*1 + 0
end_frame = jobid*1 + 1
for i in range(start_frame, end_frame, 1):
    #===========================================
    #======== check if the png file exits or not
    #===========================================
    # image_path = image_dir + str(i) + ".png"
    # file_exists = os.path.exists(image_path)
    # if file_exists:
    #     sys.exit()
        
    #===========================================
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.objects.keys()

    #===========================================
    #===================== load vehicle obj file
    #===========================================
    for n_r in range(1):
        obj_name = ""
        obj_name_spe = ""
        for n in range(num_body):
            if n==0:
                obj_name = "body"
                obj_name_spe = "body"
            if n==1:
                # continue
                obj_name = "wheel"
                obj_name_spe = "wheel"
            if n==2:
                # continue
                obj_name = "wheel"
                obj_name_spe = "wheel." + str(1).zfill(3)
            if n==3:
                # continue
                obj_name = "wheel"
                obj_name_spe = "wheel." + str(2).zfill(3)
            if n==4:
                # continue
                obj_name = "wheel"
                obj_name_spe = "wheel." + str(3).zfill(3)
            if n==5:
                # continue
                obj_name = "wheel"
                obj_name_spe = "wheel." + str(4).zfill(3)
            if n==6:
                # continue
                obj_name = "wheel"
                obj_name_spe = "wheel." + str(5).zfill(3)
            if n==7:
                obj_name = "F_L"
                obj_name_spe = "F_L"
            if n==8:
                obj_name = "F_R"
                obj_name_spe = "F_R"
            if n==9:
                obj_name = "B_L"
                obj_name_spe = "B_L"
            if n==10:
                obj_name = "B_R"
                obj_name_spe = "B_R"
            if n==11:
                obj_name = "ster_front"
                obj_name_spe = "ster_front"
            if n==12:
                obj_name = "ster_front"
                obj_name_spe = "ster_front." + str(1).zfill(3)
            if n==13:
                obj_name = "ster_back"
                obj_name_spe = "ster_back" 
            if n==14:
                obj_name = "ster_back"
                obj_name_spe = "ster_back." + str(1).zfill(3)

            file_loc = file_obj + obj_name + ".obj"
            imported_object = bpy.ops.import_scene.obj(filepath = file_loc)
            obj_object = bpy.context.object
            body_id = n + n_r * num_body
            bpy.data.objects[obj_name_spe].location.x += dis[body_id][i][0]
            bpy.data.objects[obj_name_spe].location.y += dis[body_id][i][1]
            bpy.data.objects[obj_name_spe].location.z += dis[body_id][i][2]  
            bpy.data.objects[obj_name_spe].rotation_mode = 'QUATERNION'
            q = (rot[body_id][i][0],rot[body_id][i][1],rot[body_id][i][2],rot[body_id][i][3])
            bpy.data.objects[obj_name_spe].rotation_quaternion = q

    bpy.context.view_layer.update()


    #===========================================
    #==================== Load SPH particle file
    #===========================================
    positions = []
    dir_fluid = data_sim + "particles/fluid" + str(i) + ".csv"
    dir_BCE = data_sim + "particles/BCE_Rigid" + str(i) + ".csv"
    count = 0
    for line in open(dir_fluid):
        if count == 0:
            count = count + 1
            continue
        else:
            # you have to parse "x", "y", "z" and "r" from the variable "line"
            line_seg = line.split(",")
            x, y, z = line_seg[0], line_seg[1], line_seg[2]
            # if float(y) > 1.0 or float(y) < -1.0 or float(z) > 0.9:
            #     continue
            position_buff = (float(x), float(y), float(z))
            positions.append(position_buff)
            count = count + 1

    """ -------------- PARTICLE SYSTEM START-------------- """
    context = bpy.context
    # instance object
    bpy.ops.mesh.primitive_ico_sphere_add(radius=1, location=(50,50,50))
    pippo = radius_particle
    ico = context.object

    # cube with ps
    bpy.ops.mesh.primitive_cube_add(size=0.0001)
    cube = context.object

    # ps
    ps = cube.modifiers.new("SomeName", 'PARTICLE_SYSTEM').particle_system
    psname = ps.name
    ps.settings.count = count-1
    ps.settings.lifetime = 1000
    ps.settings.frame_start = ps.settings.frame_end = 1
    ps.settings.render_type = "OBJECT"
    ps.settings.instance_object = ico

    def particle_handler(scene, depsgraph):
        ob = depsgraph.objects.get(cube.name)
        if ob:
            ps = ob.particle_systems[psname]
            f = scene.frame_current
            for m, particle in enumerate(ps.particles):
                setattr(particle, "location", positions[m])
                setattr(particle, "size", radius_particle)

    # Clear the post frame handler
    bpy.app.handlers.frame_change_post.clear()

    # Register our particleSetter with the post frame handler
    bpy.app.handlers.frame_change_post.append(particle_handler)

    # Trigger frame update
    bpy.context.scene.frame_current = 2
    """ -------------- PARTICLE SYSTEM END -------------- """

    bpy.context.view_layer.update()
    #===========================================
    #===========================================
    #===========================================


    #===========================================
    #========================= Rendering setting
    #===========================================
    bpy.ops.transform.rotate(value=(-math.pi * 0.5), orient_axis='X')  # value = Angle
    bpy.ops.mesh.primitive_plane_add(size=200.0, calc_uvs=True, enter_editmode=False, 
                                     align='WORLD', location=(0.0, 0.0, -0.1))

    #======== create a camera and settings
    bpy.ops.object.camera_add(enter_editmode=False, align='WORLD', scale=(5.0, 5.0, 5.0))
    scene.camera = bpy.context.object
    # Set up rotational camera
    cam = bpy.data.objects["Camera"]
    # add rotational angle by 1
    ini_rad = 135
    cur_rad = ini_rad #+ i * 0.5
    # cam.location = rot_pos((dis[0][i][0],dis[0][i][1],dis[0][i][2]+4),10,cur_rad)
    cam.location = rot_pos((0,0,4),15.0,cur_rad)
    # point_at(cam, (dis[0][i][0],dis[0][i][1],dis[0][i][2]+0.5), roll=math.radians(0))
    point_at(cam, (0,0,0.5), roll=math.radians(0))

    scene.cycles.device = 'GPU'

    prefs = bpy.context.preferences
    cprefs = prefs.addons['cycles'].preferences

    # Attempt to set GPU device types if available
    for compute_device_type in ('CUDA', 'OPENCL', 'NONE'):
        try:
            cprefs.compute_device_type = compute_device_type
            break
        except TypeError:
            pass

    # Enable all CPU and GPU devices
    cprefs.get_devices()
    for device in cprefs.devices:
        device.use = True

    #======== create light datablock, set attributes
    light_data = bpy.data.lights.new(name="light_2.80", type='POINT')
    light_data.energy = 12000
    # create new object with our light datablock
    light_object = bpy.data.objects.new(name="light_2.80", object_data=light_data)
    # link light object
    bpy.context.collection.objects.link(light_object)
    # make it active
    bpy.context.view_layer.objects.active = light_object
    # change location
    light_object.location = (10, 10, 15)

    #======== create another light datablock, set attributes
    light_data1 = bpy.data.lights.new(name="light_top", type='POINT')
    light_data1.energy = 1500
    # create new object with our light datablock
    light_object1 = bpy.data.objects.new(name="light_top", object_data=light_data1)
    # link light object
    bpy.context.collection.objects.link(light_object1)
    # make it active
    bpy.context.view_layer.objects.active = light_object1
    # change location
    light_object1.location = ( dis[0][i][0],  dis[0][i][1], 15 )

    bpy.context.scene.render.engine = 'CYCLES'
    bpy.context.scene.cycles.device = 'GPU'
    bpy.context.scene.render.resolution_percentage = 100
    bpy.context.scene.cycles.samples = 256
    bpy.context.scene.render.resolution_x = res_x
    bpy.context.scene.render.resolution_y = res_y
    bpy.context.scene.render.filepath = image_dir + str(i) + ".png"
    #bpy.context.scene.render.image_settings.compression = 50
    bpy.context.scene.render.image_settings.color_mode = 'RGBA'
    bpy.context.scene.render.image_settings.file_format = 'PNG'
    bpy.ops.render.render(write_still=True)
