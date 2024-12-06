import numpy as np
import heapq
from scipy import ndimage

# initializae the thresholded grid 
grid_size = (10, 10)
room_grid = np.zeros(grid_size)
room_grid[(1, 3)] = 1


robot_start = (1, 1)


# if we use height and width to find spots 
robot_height = 3
robot_width = 5

# THIS IS JUST FOR TESTING
room_grid = np.array([
    [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
    [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
    [0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1]
])

# find the center of mass for the brightest spot, whites are therolded to be 1s
# we are going to assume that there are no obstacles

def find_target(room_grid, robot_start = robot_start, robot_size = (robot_width, robot_height)):

    labeled_array, num_spots = ndimage.label(room_grid) # finds all the light spots and label them
    centers = ndimage.center_of_mass(room_grid, labeled_array, range(1, num_spots + 1)) 
   
    '''
    # find the size of these spots to make sure they can fit the pupper
    sizes = ndimage.sum(room_grid, labeled_array, range(1, num_spots + 1))
    '''
    # if we want to find the largest 
    largest_spot_index = np.argmax(sizes)
    largest_spot_center = centers[largest_spot_index]
   
    # print(centers, "centers", largest_spot_center, "largest")
    return largest_spot_center

    '''
    nearest_centers_idx = find_nearest(centers, robot_start)
    
    for idx in nearest_centers_idx:
        # idx is a tuple with (index, distance value)
        
        # if sizes.tolist()[idx[0]] > robot_size:
            # return centers[idx[0]]

        spot_size = find_height_width(labeled_array, idx[0])
      
        if all(a > b for a, b in zip(spot_size, robot_size)):
            return centers[idx[0]]


    # rethink edge cases where all spots are smaller than the robot
    raise ValueError ("No proper target") 
    '''

def find_height_width(labeled_array, i):
   
    rows, cols = np.where(labeled_array == i)
    min_row, max_row = rows.min(), rows.max()
    min_col, max_col = cols.min(), cols.max()
    
    height = max_row - min_row + 1
    width = max_col - min_col + 1
    return [height, width]


# this finds the nearest spot 
def find_nearest(centers, robot_start):
    distances = [] # a list of how far each spot is from the robot
    for point in centers:
        distance = np.linalg.norm(np.array(point)-np.array(robot_start)) # find the euclidean distance
        distances.append(distance)

    sorted_indices = heapq.nsmallest(len(distances), enumerate(distances)) # key=lambda x: x[0]
    # print("nearest:", sorted_indices)
    return sorted_indices


# only if we're doing gradient
def find_target_gradient(room_grid):
    brightest_spot = ndimage.maximum_position(room_grid)
    return brightest_spot


if __name__ == "__main__":
    find_target(room_grid)
    find_target_gradient(room_grid)





