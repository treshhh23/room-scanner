import serial
import open3d as o3d
import numpy as np
import math

depth = 20
# example output
pcd = o3d.io.read_point_cloud("render.xyz", format="xyz")

print("The PCD array:")
print(np.asarray(pcd.points))

print("Lets visualize the PCD: (spawns seperate interactive window)")
o3d.visualization.draw_geometries([pcd])

lines = []
points = 64
for layers in range(depth):
    for point in range(points):
        if (point == points - 1):
            lines.append([point + layers * points, layers * points])
        else:
            lines.append([point + layers * points, point + layers * points + 1])

for layers in range(depth - 1):
    for point in range(points):
        lines.append([point + layers * points, point + layers * points + points])


#Lets see what our point cloud data with lines looks like graphically     
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([line_set])