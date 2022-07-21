import bpy
import csv
from itertools import chain

def accel(curve, t):
    h = 1
    return (curve.evaluate(t + h) - curve.evaluate(t) * 2 + curve.evaluate(t - h)) / (h * h)

def vel(curve, t):
    h = 1
    return (curve.evaluate(t + h) - curve.evaluate(t - h)) / (2*h)

with open(r"C:\Users\jashu\git\Swerve-2022\src\main\deploy\trajectories\R1.csv",'w', newline='', encoding='utf-8') as file:
    csv_writer = csv.writer(file, dialect='excel')

    scene = bpy.context.scene
    frame_current = scene.frame_current

    for frame in range(scene.frame_start, scene.frame_end + 1):
        scene.frame_set(frame)
        for ob in scene.objects:
            if "Cube" in ob.name:
                x_curve = ob.animation_data.action.fcurves[0]
                y_curve = ob.animation_data.action.fcurves[1]
                rot_curve = ob.animation_data.action.fcurves[5]
                
                x = x_curve.evaluate(frame)
                vx = vel(x_curve, frame)
                y = y_curve.evaluate(frame)
                vy = vel(y_curve, frame)
                r = rot_curve.evaluate(frame)
                vr = vel(rot_curve, frame)
                csv_writer.writerow(tuple((x, vx, y, vy, r, vr)))

    scene.frame_set(frame_current)