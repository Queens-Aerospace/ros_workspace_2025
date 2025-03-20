from array import *
import math

points = [[76, 16, 68],
 [81, 7, 82],
 [16, 58, 28],
 [31, 73, 20],
 [52, 57, 22],
 [3, 26, 90],
 [38, 24, 20],
 [44, 80, 96],
 [25, 2, 90],
 [39, 20, 25],
 [4, 76, 72],
 [36, 16, 24],
 [25, 16, 3],
 [4, 67, 81],
 [21, 73, 68],
 [3, 17, 98],
 [93, 63, 88],
 [62, 90, 64],
 [73, 21, 7],
 [81, 70, 69],
 [44, 67, 9],
 [25, 51, 10],
 [91, 47, 34],
 [27, 90, 83],
 [43, 25, 100],
 [65, 94, 95],
 [15, 25, 15],
 [23, 83, 69],
 [97, 91, 94],
 [15, 85, 37]]

sizeOfArray=len(points)
distanceMatrix = [[0] * sizeOfArray for _ in range(sizeOfArray)]
distance=0

iteration = 0
for i in range(sizeOfArray-1):
    for x in range(i, sizeOfArray):
        if i!=x:
            distanceMatrix[i][x] = math.sqrt((points[i][0]-points[x][0])**2+(points[i][1]-points[x][1])**2+ (points[i][2]-points[x][2])**2)
        
for i in range(sizeOfArray-1):
    minimumDistance=100000
    for x in range(i, sizeOfArray):
        if distanceMatrix[i][x]!= 0 and distanceMatrix[i][x]<minimumDistance:
            minimumDistance = distanceMatrix[i][x]
            

visited = [False] * sizeOfArray  # To track visited points
path = []  # To store the order of the path
total_distance = 0  # Total distance of the path

# Start at the first point (index 0)
current_point = 0
visited[current_point] = True
path.append(current_point)

# Loop until all points are visited
while len(path) < sizeOfArray:
    nearest_distance = float('inf')
    nearest_point = None
    
    # Find the nearest unvisited point
    for i in range(sizeOfArray):
        if not visited[i] and distanceMatrix[current_point][i] < nearest_distance:
            nearest_distance = distanceMatrix[current_point][i]
            nearest_point = i
    
    # Move to the nearest point
    visited[nearest_point] = True
    path.append(nearest_point)
    total_distance += nearest_distance
    current_point = nearest_point

# Optionally, return to the starting point to complete the circuit
total_distance += distanceMatrix[path[-1]][path[0]]

# Print the path and total distance
print("Path:", path)
print("Total distance:", total_distance)