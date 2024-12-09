# blender file my_script.py
# file generated on 2024-12-09 12:42:39 GMT
# by lorenz
#
# to load this file, on the blender python console type the following
# for linux:
# myfile = '/the/full/path/to/my_script.py'
# for windows
# myfile = 'C:\\users\\username\\somefolder\\my_script.py'
# then type:
# exec(compile(open(myfile).read(), myfile, 'exec'))
#
# blender commands below
#
# link transform for pose index 0
#
ob = bpy.context.scene.objects["L00.Base"]
bpy.ops.object.select_all(action='DESELECT')
bpy.context.view_layer.objects.active = ob
ob.select_set(True)
bpy.context.object.location[0] = 0
bpy.context.object.location[1] = 0
bpy.context.object.location[2] = 0
bpy.context.object.rotation_euler[0] = 0 #  0
bpy.context.object.rotation_euler[1] = 0 #  0
bpy.context.object.rotation_euler[2] = 0 #  0
