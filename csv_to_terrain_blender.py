import bpy
import bmesh
import numpy as np
import csv
from collections import defaultdict

def new_plane(mylocation, mysize, myname):
    
    
    current_name = bpy.context.selected_objects[0].name
    plane = bpy.data.objects[current_name]
#    
    me = plane.data
                    
    #Get Height Data based on Gd Value:
    columns = height_data('/home/admin/aditya_ws/adi_eddie/Terrain_csv/RoadMat_n5_d0.01_A.csv')
#    print(columns['Gd = 377.865 vel = 1.9775'][3])
#    print(columns['Gd = 377.865 vel = 1.9775'][4])
#    print(len(me.vertices))
    
    for j,vert in enumerate(me.vertices):
        vert.co.z = float(columns['Gd = 160.9281 vel = 1.3224'][j])
    return

def height_data(path):
    file = open(path)
    data = csv.DictReader(file)
#    header = next(data)
    i = 0
    columns = defaultdict(list) # each value in each column is appended to a lis
    for row in data:
        for (k,v) in row.items():
            columns[k].append(v)
            columns[k].append(v)
            
    return columns  


new_plane((0,0,0), 1, "NewPlane")
#obj=bpy.context.object
#if obj.mode == 'EDIT':
#    bm=bmesh.from_edit_mesh(obj.data)
#    bm.verts.sort(key=lambda v: v.co.x)
#    for v in bm.verts:
#        if v.select:
#            print(v.co.z)