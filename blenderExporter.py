import bpy
import csv
from itertools import chain

with open(r"C:\Users\jashu\git\Swerve-2022\src\main\deploy\trajectories\R1.csv",'w', newline='', encoding='utf-8') as file:
    csv_writer = csv.writer(file, dialect='excel')

    scene = bpy.context.scene
    frame_current = scene.frame_current

    for frame in range(scene.frame_start, scene.frame_end + 1):
        scene.frame_set(frame)
        root = bpy.data.objects['Root']
        root_r = root.rotation_euler[2]
        for ob in scene.objects:
            if "Empty" in ob.name:

                
                x = ob.matrix_world[0][3]
                
                y = ob.matrix_world[1][3]
                
                r = ob.rotation_euler[2] + root_r
                csv_writer.writerow(tuple((x, y, r)))

    scene.frame_set(frame_current)