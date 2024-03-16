#!/usr/bin/env python3

import cv2
import numpy as np
from math import atan2
from geometry_msgs.msg import Pose
import os

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from math import sin, cos #,radians

'''
User Instruction
1. Include the full image path in "find_object" function or you will get an error

2. For the first run, within the "build_castle" function, set the "pixel_to_m" ratio to 1. Also, set the x, y and z offsets to zero.
    This way you will find out what the coordinate of the x axis is in pixel coordinate. 
    Then, next time set the "pixel_to_m" ratio  = actual width(m) / 2 * x coordinate of CoM (pixel)
    Make sure to change the x, y and z coordinates back to desired values.

3. 

'''

def find_object(image_path, display=True, publish=False, print_res=False):
    '''
    Scans an image to find colored object and returns their coordinates.

    Args:
        image_path: Path to the input image (JPG/PNG format)
        display: True to draw squares around
        publish: True to publish result image to a topic
        print_res: True to print coordinates of each block found in terminal

    Returns:
        found_objects: dictionary where each key is a color, and the value is a list of dictionaries for each object found
            e.g. found_object['red'] = [list of dictionaries corresponding to red objects]
    '''

    # Fixed variables
    area_ratio = .001  # Defines which contours to keep based on contour area

    ## Color Definitions
    colors = color_def()

    # Reading the image
    img = cv2.imread(image_path)
    img=cv2.rotate(img, cv2.ROTATE_180) #Image rotated by 180 degrees to get proper coordinate (origin=top left, +x=right, +z=downward)
    img=cv2.flip(img,1) #flip image horizontally

    # Define kernel size for noise removal
    kernel = np.ones((7, 7), np.uint8)

    # Convert to hsv colorspace
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    result = None
    if display or publish:
        # Copy the image to not alter it when tracing the bounding boxes
        result = img.copy()

    found_objects = dict()

    for color in colors:

        found_objects[color['name']] = []

        # Lower bound and upper bound for color
        lower_bound = color['hsv'][0]
        upper_bound = color['hsv'][1]

        if color['name'] == 'red':  # Red has 2 ranges (on both ends of hue spectrum)
            lower_bound = color['hsv'][0][0]
            upper_bound = color['hsv'][0][1]

        # Find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        if color['name'] == 'red':  # Taking the union of red's 2 ranges
            lower_bound2 = color['hsv'][1][0]
            upper_bound2 = color['hsv'][1][1]
            mask2 = cv2.inRange(hsv, lower_bound2, upper_bound2)
            mask = cv2.bitwise_or(mask, mask2)

        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours from the mask
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)  # Order by decreasing area

        if len(contours) > 0:
            min_area = area_ratio * cv2.contourArea(contours[0])

        for cntr in contours:

            if cv2.contourArea(cntr) >= min_area:
                found_object = dict()

                # Determine if the object geometry is rectangular or crcular

                epsilon = .018 * cv2.arcLength(cntr, True)
                approx = cv2.approxPolyDP(cntr, epsilon, True) #approx is a list of vertices of detected object
                #length=len(approx)
                #print(length)

                if len(approx) == 4: #4 vertices = quadrilateral
                    found_object['geometry']='quad'
                    xc, zc, w, h = cv2.boundingRect(cntr)
                    xc = xc + w // 2
                    zc = zc + h // 2

                    found_object['center'] = (xc, zc)
                    found_object['width'] = w
                    found_object['height'] = h

                    #if print_res:
                        #print('Found {} object at ({}, {})'.format(color['name'], xc, zc))

                    rect = cv2.minAreaRect(cntr)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    angle = get_orientation(box, result) #don't really need this- just for visual

                
                elif len(approx)== 3: #3 vertices = triangle
                    #print(approx)
                    found_object['geometry']='triad'

                    #calculating centroid, base and height of triangle
                    (xc,zc), _, _ = find_triangle_details(approx)
                    _, _, w, h = cv2.boundingRect(cntr)

                    found_object['center'] = (xc, zc)
                    found_object['width'] = w
                    found_object['height'] = h

                    rect = cv2.minAreaRect(cntr)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    angle = get_orientation(box, result)
                

                if display or publish:
                    # Traces a rectangle around each "object":
                    #cv2.drawContours(result, [box], 0, color['bgr'], 5)
                    #cv2.drawContours(result, [box], 0, (0,0,0), 5)
                    cv2.drawContours(result, [box], 0, (0, 0, 0), 5)
                    #cv2.putText(result, str(angle * 180 / np.pi), box[1], cv2.FONT_HERSHEY_SIMPLEX, 0.5, color['bgr'], 1,
                                #cv2.LINE_AA)
                    
                    cv2.putText(result, str(angle * 180 / np.pi), tuple(box[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)


                found_objects[color['name']].append(found_object)
                #print(found_objects)

    if display:  # Displays the frame with rectangles on it
        cv2.imshow("Result", result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return found_objects

def find_triangle_details(approx): #This functions takes the vertices of a traingle and calculates centroid, base and height
    # Extract vertices and treat the coordinates as (x, z)
    x1, z1 = approx[0][0]
    x2, z2 = approx[1][0]
    x3, z3 = approx[2][0]
    
    # Calculate the centroid (CoM)
    G_x = (x1 + x2 + x3) / 3
    G_z = (z1 + z2 + z3) / 3
    
    # Use np.linalg.norm to calculate the distances (sides of the triangle)
    side1 = np.linalg.norm(np.array([x1, z1]) - np.array([x2, z2]))
    side2 = np.linalg.norm(np.array([x2, z2]) - np.array([x3, z3]))
    side3 = np.linalg.norm(np.array([x3, z3]) - np.array([x1, z1]))
    
    # Base is the longest of the three sides
    base = max(side1, side2, side3)
    
    # Use Heron's formula to find the area of the triangle
    s = (side1 + side2 + side3) / 2  # semi-perimeter
    area = np.sqrt(s * (s - side1) * (s - side2) * (s - side3))
    
    # Height can be found from the area formula: Area = 0.5 * base * height
    height = (2 * area) / base
    
    return (G_x, G_z), base, height



def get_orientation(box, img=None): #Not really needed for this program
    '''
    box: 4x2 array containing the coordinates of a box's 4 corners
    img: the img on which to draw
    '''

    p1, p2, p3, _ = box
    center = np.mean(box, 0)

    c1 = np.linalg.norm(p1 - p2)
    c2 = np.linalg.norm(p2 - p3)

    if c1 > c2:
        major_ax = p1 - p2
        minor_ax = p2 - p3
    else:
        major_ax = p2 - p3
        minor_ax = p1 - p2
    
    #major_ax = p1 - p2
    #minor_ax = p2 - p3

    if img is not None:
        draw_axis(img, center, center + major_ax, (255, 255, 0), 10)
        draw_axis(img, center, center + minor_ax, (0, 0, 255), 10)

    angle = (-atan2(major_ax[1], major_ax[0]) - np.pi / 2) % np.pi
    if angle >= 6 / 10 * np.pi:
        angle -= np.pi

    return angle


def draw_axis(img, p_, q_, color, scale):
    p = list(p_)
    q = list(q_)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 1, cv2.LINE_AA)


def color_def():
    '''
    Each color is represented by dictionary with 3 keys:
        'name': name of color (string)
        'bgr': BGR value for that color (tuple)
        'hsv': HSV range for that color (list of 2 arrays)

    Returns:
        list of dictionaries
    '''

    yellow = {'name': 'yellow', 'bgr': (107, 183, 189), 'hsv': [np.array([20, 80, 80]), np.array([30, 255, 255])]}
    blue = {'name': 'blue', 'bgr': (255, 0, 0), 'hsv': [np.array([90, 70, 100]), np.array([128, 255, 255])]}
    green = {'name': 'green', 'bgr': (0, 255, 0), 'hsv': [np.array([36, 60, 100]), np.array([85, 255, 255])]}
    white = {'name': 'white', 'bgr': (255, 255, 255), 'hsv': [np.array([0, 0, 200]), np.array([255, 20, 255])]}
    red = {'name': 'red', 'bgr': (0, 0, 255),
           'hsv': [[np.array([180, 150, 100]), np.array([255, 255, 255])], [np.array([0, 150, 100]), np.array([10, 255, 255])]]}

    colors = [yellow, blue, green, red]
    #colors = [green]

    return colors

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler Angles to Quaternion.

    Args:
        roll, pitch, yaw: Euler angles in radians.

    Returns:
        tuple: Quaternion (w, x, y, z).
    """
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (w, x, y, z)

def determine_shape(width, height, depth):
        """
        Determines the shape of an object based on its dimensions within a tolerance.
        
        Args:
            width (float): The width of the object in m.
            height (float): The height of the object in m.
            depth (float): The depth of the object in m.
        
        Returns:
            str: The determined shape of the object.
        """
        shapes = {
            'cube': [.029, .029, .029],
            'rect': [.059, .029, .014],
            'long': [.089, .029, .014],
            'rect': [.059, .029, .029]
        }
        tolerance = .004

        # Sort the input dimensions for comparison with shape dimensions
        dimensions = sorted([width, height, depth])

        for shape, shape_dims in shapes.items():
            sorted_shape_dims = sorted(shape_dims)
            # Check if dimensions are within tolerance for this shape
            if all(abs(dim - s_dim) <= tolerance for dim, s_dim in zip(dimensions, sorted_shape_dims)):
                return shape
        
        return "unknown"

def build_castle(found_objects, pixel_to_m, y_offset, x_offset=0.155,z_offset=0, z_tolerance=0.005, depth=.029):
    """
    Simulates the building process of a castle based on the detected objects, grouping blocks into layers.
    Additionally, calculates the shape of each block based on its dimensions.

    Args:
        found_objects (dict): Output of the find_object function containing detected objects categorized by color.
        z_tolerance (float): Tolerance for grouping blocks into layers based on their z-coordinate. Defaults to 0.05 cm.
        pixel_to_m (float): Conversion factor from pixels to meters. Defaults to .029 m / 190 pixels.
        depth (float): Default depth of each block, used for shape determination.
        x_offset, y_offset, z_offset: Adding this realistic world coordinate to calculated coordinates values from the image.

    Returns:
        dict: A dictionary containing layers of blocks, where each layer contains sequential blocks with their color,
              center of mass (x, y, z coordinates), gripper_roll, gripper_pitch, gripper_yaw angles, width, height, depth,
              and shape.
    """

    # Flatten the detected objects dictionary to a list of tuples (color, object)
    blocks = [(color, obj) for color, objects in found_objects.items() for obj in objects]

    for color, obj in blocks:
        # Convert pixel values to meters
        geometry=obj['geometry']
        obj['center'] = (obj['center'][0] * pixel_to_m + x_offset, y_offset + (depth/2), obj['center'][1] * pixel_to_m + z_offset)
        obj['width'] *= pixel_to_m
        obj['height'] *= pixel_to_m
        obj['depth'] = depth  # Assuming depth is a constant for simplicity

        # Calculate the shape of the block - For triangular it returns NA
        obj_shape = determine_shape(obj['width'], obj['height'], obj['depth']) #this function outputs "NA" for triangles
        obj['shape'] = obj_shape
        if geometry == 'triad':
            obj['shape'] = 'triangle'


        # Calculating the horizontal vs vertical configuration for rectangular and vertical objectobject
        # Configuration determines how the gripper's pose while dropping an object at the desired location
        if geometry == 'quad':
            if obj_shape == 'cube': #for cube, it does not matter so we will keep the horizontal configuration
                config = 'horizontal'
            elif obj_shape == 'rect':
                if obj['width']>obj['height']:
                    config = 'horizontal'
                else:
                    config = 'vertical'
        else: #for triangular object, we will leave it as horizontal
            config = 'horizontal'
        
        if config == 'horizontal':
            roll, pitch, yaw = (0,0,0)
        else:
            roll, pitch, yaw = (0,0,0) ## Need to fix this value
            
        # Convert Euler angles to quaternion (roll, pitch, yaw are provided by obj)
        #roll, pitch, yaw = (obj['roll']), (obj['pitch']), (obj['yaw'])
        quaternion = euler_to_quaternion(roll, pitch, yaw)
        obj['quaternion1'] = quaternion[0]
        obj['quaternion2'] = quaternion[1]
        obj['quaternion3'] = quaternion[2]
        obj['quaternion4'] = quaternion[3]

    # Sort the blocks based on their z-coordinate in ascending order
    sorted_blocks = sorted(blocks, key=lambda x: x[1]['center'][2])

    # Dictionary to store layers of blocks
    layers = {}
    current_layer_index = 0  # Initialize current_layer_index as 0
    current_z = None

    # Group blocks based on z-coordinate with tolerance
    for color, obj in sorted_blocks:
        z_coord = obj['center'][2]

        # If current_z is None or the block's z-coordinate is outside the tolerance of the current z,
        # start a new group
        if current_z is None or abs(z_coord - current_z) > z_tolerance:
            current_z = z_coord
            current_layer_index += 1
            layers[f"Layer {current_layer_index}"] = []

        # Add block to the current layer
        layers[f"Layer {current_layer_index}"].append({
            'color': color,
            'center': obj['center'],
            'quaternion1': obj['quaternion1'],
            'quaternion2': obj['quaternion2'],
            'quaternion3': obj['quaternion3'],
            'quaternion4': obj['quaternion4'],
            'width': obj['width'],
            'height': obj['height'],
            'depth': obj['depth'],
            'shape': obj['shape'],
            'geometry': obj['geometry']
        })

    # Sort blocks within each layer based on their x-coordinate in ascending order
    for layer, blocks in layers.items():
        layers[layer] = sorted(blocks, key=lambda x: x['center'][0])

    return layers


def evaluate_stability(layer_2_blocks, layer_1_blocks):
    """
    Evaluate the stability of blocks in Layer 2 based on their placement on supporting blocks from Layer 1.

    Args:
        layer_2_blocks (list): List of dictionaries representing blocks in Layer 2.
        layer_1_blocks (list): List of dictionaries representing blocks in Layer 1.

    Returns:
        dict: A dictionary indicating whether each block from Layer 2 is stable or unstable.
              Keys are block indices (starting from 1), and values are booleans (True for stable, False for unstable).
    """

    # Dictionary to store stability of each block from Layer 2
    stability_results = {}

    for idx, block_2 in enumerate(layer_2_blocks, start=1):
        # Get the x-coordinate of the current block from Layer 2
        block_2_x = block_2['center'][0]

        # Calculate valid distances based on the block widths from both layers
        valid_distances = [abs(block_2_x - block_1['center'][0])
                           for block_1 in layer_1_blocks
                           if abs(block_2_x - block_1['center'][0]) <= 0.5 * block_2['width'] + 0.5 * block_1['width']]

        if len(valid_distances) >= 2:
            # Find the two supporting blocks from Layer 1 based on the valid distances
            supporting_blocks = [block for block in layer_1_blocks if abs(block_2_x - block['center'][0]) in valid_distances]

            # Sort the supporting blocks in ascending order of their x-coordinates
            supporting_blocks = sorted(supporting_blocks, key=lambda x: x['center'][0])

            # Calculate the minimum and maximum x-coordinates within which the block from Layer 2 is stable
            min_x = supporting_blocks[0]['center'][0] - supporting_blocks[0]['width'] / 2
            max_x = supporting_blocks[1]['center'][0] + supporting_blocks[1]['width'] / 2

            # Print the center of mass of Block 2 coordinate
            print(f"Layer 2 Block {idx} coordinates: {block_2['center']}")
            # Print the coordinates of the supporting blocks from Layer 1
            print(f"Supporting Block 1 coordinates: {supporting_blocks[0]['center']}")
            print(f"Supporting Block 2 coordinates: {supporting_blocks[1]['center']}")
            # Print the projection range
            print(f"Projection Range for Block {idx}: [{min_x}, {max_x}]")
            
            # Check if the block from Layer 2 is stable based on its x-coordinate
            if min_x <= block_2_x <= max_x:
                stability_results[idx] = True
                print(f"Block {idx} is stable.")
            else:
                stability_results[idx] = False
                print(f"Block {idx} is unstable.")
        elif len(valid_distances) == 1:
            # Only one supporting block found
            supporting_block = [block for block in layer_1_blocks if abs(block_2_x - block['center'][0]) in valid_distances][0]

            # Calculate the minimum and maximum x-coordinates within which the block from Layer 2 is stable
            min_x = supporting_block['center'][0] - supporting_block['width'] / 2
            max_x = supporting_block['center'][0] + supporting_block['width'] / 2

            # Print the center of mass of Block 2 coordinate
            print(f"Layer 2 Block {idx} coordinates: {block_2['center']}")
            # Print the coordinates of the supporting block from Layer 1
            print(f"Supporting Block coordinates: {supporting_block['center']}")
            # Print the projection range
            print(f"Projection Range for Block {idx}: [{min_x}, {max_x}]")
            
            # Check if the block from Layer 2 is stable based on its x-coordinate
            if min_x <= block_2_x <= max_x:
                stability_results[idx] = True
                print(f"Block {idx} is stable.")
            else:
                stability_results[idx] = False
                print(f"Block {idx} is unstable.")
        else:
            # No supporting blocks found or not enough supporting blocks, mark the block as unstable
            stability_results[idx] = False
            print(f"No supporting blocks found for Layer 2 Block {idx}")

    return stability_results

def flatten_layers(layers_dict):
    """
    Flattens the layers of blocks into a single list of blocks.

    Args:
        layers_dict (dict): A dictionary containing layers of blocks, where each layer is a list of block dictionaries.

    Returns:
        list: A list of dictionaries, each representing a block with its properties.
    """
    flattened_blocks = []
    for layer in layers_dict.values():
        flattened_blocks.extend(layer)
    return flattened_blocks

def plot_castle_structure(flattened_blocks):
    """
    Plots a 3D representation of the castle structure with triangular prisms having their triangular faces on the xz plane.
    Rectangular faces connect these triangles along the y-axis.

    Args:
        flattened_blocks (list): The list containing blocks and their properties.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    color_map = {
        'yellow': 'yellow',
        'blue': 'blue',
        'green': 'green',
        'red': 'red',
        'white': 'white'
    }

    for block in flattened_blocks:
        geometry = block['geometry']
        color = block['color']
        x, y, z = block['center']
        width = block['width']
        height = block['height']
        depth = block['depth']
        plot_color = color_map.get(color, 'grey')

        if geometry == 'quad':
            # Plot rectangular blocks
            ax.bar3d(x - width / 2, y - depth / 2, z - height / 2,
                     width, depth, height, color=plot_color)
        elif geometry == 'triad':
            # Triangular faces on the xz plane
            # Base triangle vertices
            base = np.array([
                [x - width / 2, y - depth / 2, z - height / 2],  # Left corner
                [x + width / 2, y - depth / 2, z - height / 2],  # Right corner
                [x, y - depth / 2, z + height / 2]               # Top middle
            ])
            # Top triangle vertices (shifted along the y-axis)
            top = base + np.array([[0, depth, 0]])  # Shift all vertices along the y-axis

            # Combine base and top for easier indexing
            vertices = np.vstack([base, top])

            # Sides: Connect vertices to form rectangular faces
            faces = [
                [vertices[i] for i in [0, 3, 4, 1]],  # Side 1
                [vertices[i] for i in [1, 4, 5, 2]],  # Side 2
                [vertices[i] for i in [2, 5, 3, 0]],  # Side 3
                [vertices[i] for i in [0, 1, 2]],     # Base triangle
                [vertices[i] for i in [3, 4, 5]]      # Top triangle
            ]

            poly3d = [list(map(tuple, face)) for face in faces]
            ax.add_collection3d(Poly3DCollection(poly3d, facecolors=plot_color, edgecolors=plot_color, linewidths=1, alpha=1))

    ax.set_xlabel('X axis (m)')
    ax.set_ylabel('Y axis (m)')
    ax.set_zlabel('Z axis (m)')
    ax.set_title('3D Castle Plot')

    ax.set_xlim(0.150,0.350)
    ax.set_ylim(0.0,0.20)
    ax.set_zlim(0.0,0.20)
    plt.show()



def create_poses_colors_shape_list(final_list):
    poses_colors_shape = []
    
    for obj in final_list:
        place_pose = Pose()

        place_pose.position.x = obj['center'][0]
        place_pose.position.y = obj['center'][1]
        place_pose.position.z = obj['center'][2]

        place_pose.orientation.x= obj['quaternion1']
        place_pose.orientation.y= obj['quaternion2']
        place_pose.orientation.z= obj['quaternion3']
        place_pose.orientation.w= obj['quaternion4']

        shape = obj['shape']
        color = obj['color']
        
        # Combine place_pose, shape, and color into a tuple and add it to the poses list
        info = (place_pose, shape, color)
        poses_colors_shape.append(info)
    
    return poses_colors_shape

#Execution of all functions
if __name__ == '__main__':

    #Block Detection- provide image(s) on after another
    # Define the relative path to the blueprint image
    blueprint_filename = "blueprint_cube_triangle.png"
    # Get the directory where the script is located
    script_dir = os.path.dirname(__file__)
    # Construct the full path to the blueprint image
    blueprint_path = os.path.join(script_dir, blueprint_filename)

    # Use the constructed path in the find_object function
    found_object_results = find_object(blueprint_path)
    #print(found_object_results)
    
    #Srting the blocks based on x and z coordinates- also adding y-coordinate
    sequential_blocks = build_castle(found_object_results,pixel_to_m=.029/146, y_offset=0.155)
    #sequential_blocks = build_castle(found_object_results,pixel_to_m=.029/144, y_offset=0.155)

    
    #You can perform stability analysis any two sequential layers (e.g. Layer 2 on Layer 1, Layer 3 on Layer 2, Layer 4 on Layer 3 etc.)
    #layer_2_blocks = sequential_blocks['Layer 2']
    #layer_1_blocks = sequential_blocks['Layer 1']

    #stability_results = evaluate_stability(layer_2_blocks, layer_1_blocks)
    

    #Flatten the blocks to provide a sequential list (gets rid of "Layer" differentiation)
    flattened_blocks = flatten_layers(sequential_blocks)

    #Final List
    #final_list=flattened_blocks + flattened_blocks2 + flattened_blocks3
    final_list=flattened_blocks
    #print(final_list)

    #plot the multi-layered castle structure
    plot_castle_structure(final_list)

    #Print poses
    transfer = create_poses_colors_shape_list(final_list)
    print(transfer)