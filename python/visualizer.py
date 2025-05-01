import serial
import open3d as o3d
import numpy as np
import math

data = []
depth = 3
z = [0]

s = serial.Serial('COM5', 115200, timeout = 10)

print("Opening" + s.name)

s.reset_output_buffer()
s.reset_input_buffer()

input("Press Enter to start communication...")

s.write('s'.encode())
for j in range(depth):
    if(j >= 1):
        print("How much displacement (mm)")
        displacement = input()
        z.append(int(displacement))

    for i in range(64):
        while True:
            x = s.readline()
            if not x: #Bad data
                print("No data received on iteration. Trying again.")
                continue

            try:
                x_str = x.decode('utf-8').strip()
                splitString = x_str.split(", ")
                value = int(splitString[1])
                if value == 0: #Bad data
                    print("Detected value 0. Trying again.")
                    continue
                if(int(splitString[0]) == 0):
                    data.append(value)
                    print(value) # Testing purposes
                else:
                    data.append(4000)
                    print("error", value, "line ", i)
                break  #Correct data gathered
            except (IndexError, ValueError) as e:
                print("Error processing line, trying again:", x_str, e)
                continue  #Bad data


print("Closing: " + s.name)
s.close()


coordinates = []
degree = 0
for j in range(0, depth):
    degree = 0 
    for i in range(0,64):
        index = j * 64 + i
        x = math.sin(math.radians(degree)) * float(data[index])
        y = math.cos(math.radians(degree)) * float(data[index])
        degree += 5.625
        coordinates.append([x, y, z[j]])

for coords in coordinates:
    print(coords)

f = open("test.xyz", "w", encoding="utf-8")
for i in range(depth*64):
    f.write(str(coordinates[i][0]))
    f.write(" ")
    f.write(str(coordinates[i][1]))
    f.write(" ")
    f.write(str(coordinates[i][2]))
    f.write("\n")
f.close()

pcd = o3d.io.read_point_cloud("test.xyz", format="xyz")

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
   
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([line_set])