import math

objects = []
springs = []


def add_object(x, halfsize, rotation=0):
    objects.append([x, halfsize, rotation])
    return len(objects) - 1


# actuation 0.0 will be translated into default actuation
def add_spring(a, b, offset_a, offset_b, length, stiffness, actuation=0.0):
    springs.append([a, b, offset_a, offset_b, length, stiffness, actuation])


def robotA():
    add_object(x=[0.3, 0.25], halfsize=[0.15, 0.03])
    add_object(x=[0.2, 0.15], halfsize=[0.03, 0.02])
    add_object(x=[0.3, 0.15], halfsize=[0.03, 0.02])
    add_object(x=[0.4, 0.15], halfsize=[0.03, 0.02])
    add_object(x=[0.4, 0.3], halfsize=[0.005, 0.03])

    l = 0.12
    s = 15
    add_spring(0, 1, [-0.03, 0.00], [0.0, 0.0], l, s)
    add_spring(0, 1, [-0.1, 0.00], [0.0, 0.0], l, s)
    add_spring(0, 2, [-0.03, 0.00], [0.0, 0.0], l, s)
    add_spring(0, 2, [0.03, 0.00], [0.0, 0.0], l, s)
    add_spring(0, 3, [0.03, 0.00], [0.0, 0.0], l, s)
    add_spring(0, 3, [0.1, 0.00], [0.0, 0.0], l, s)
    # -1 means the spring is a joint
    add_spring(0, 4, [0.1, 0], [0, -0.05], -1, s)

    return objects, springs, 0


def robotC():
    add_object(x=[0.3, 0.25], halfsize=[0.15, 0.03])
    add_object(x=[0.2, 0.15], halfsize=[0.03, 0.02])
    add_object(x=[0.3, 0.15], halfsize=[0.03, 0.02])
    add_object(x=[0.4, 0.15], halfsize=[0.03, 0.02])

    l = 0.12
    s = 15
    add_spring(0, 1, [-0.03, 0.00], [0.0, 0.0], l, s)
    add_spring(0, 1, [-0.1, 0.00], [0.0, 0.0], l, s)
    add_spring(0, 2, [-0.03, 0.00], [0.0, 0.0], l, s)
    add_spring(0, 2, [0.03, 0.00], [0.0, 0.0], l, s)
    add_spring(0, 3, [0.03, 0.00], [0.0, 0.0], l, s)
    add_spring(0, 3, [0.1, 0.00], [0.0, 0.0], l, s)

    return objects, springs, 3


l_thigh_init_ang = 10
l_calf_init_ang = -10
r_thigh_init_ang = 10
r_calf_init_ang = -10
initHeight = 0.15

hip_pos = [0.3, 0.5 + initHeight]
hip_pos1 = [0.1, 0.5 + initHeight]
thigh_half_length = 0.11
calf_half_length = 0.11

foot_half_length = 0.1


def rotAlong(half_length, deg, center):
    ang = math.radians(deg)
    return [
        half_length * math.sin(ang) + center[0],
        -half_length * math.cos(ang) + center[1]
    ]


half_hip_length = 0.08


def robotLeg():
    #hip
    add_object(hip_pos, halfsize=[0.08, half_hip_length])
    hip_end = [hip_pos[0], hip_pos[1] - (0.08 - 0.01)]

    #left
    l_thigh_center = rotAlong(thigh_half_length, l_thigh_init_ang, hip_end)
    l_thigh_end = rotAlong(thigh_half_length * 2.0, l_thigh_init_ang, hip_end)
    add_object(l_thigh_center,
               halfsize=[0.02, thigh_half_length],
               rotation=math.radians(l_thigh_init_ang))
    add_object(rotAlong(calf_half_length, l_calf_init_ang, l_thigh_end),
               halfsize=[0.02, calf_half_length],
               rotation=math.radians(l_calf_init_ang))
    l_calf_end = rotAlong(2.0 * calf_half_length, l_calf_init_ang, l_thigh_end)
    add_object([l_calf_end[0] + foot_half_length, l_calf_end[1]],
               halfsize=[foot_half_length, 0.02])

    #right
    add_object(rotAlong(thigh_half_length, r_thigh_init_ang, hip_end),
               halfsize=[0.02, thigh_half_length],
               rotation=math.radians(r_thigh_init_ang))
    r_thigh_end = rotAlong(thigh_half_length * 2.0, r_thigh_init_ang, hip_end)
    add_object(rotAlong(calf_half_length, r_calf_init_ang, r_thigh_end),
               halfsize=[0.02, calf_half_length],
               rotation=math.radians(r_calf_init_ang))
    r_calf_end = rotAlong(2.0 * calf_half_length, r_calf_init_ang, r_thigh_end)
    add_object([r_calf_end[0] + foot_half_length, r_calf_end[1]],
               halfsize=[foot_half_length, 0.02])
    
    # Upper arm
    shoulder_pos = [0.3, 0.4]
    add_object(shoulder_pos, halfsize=[0.02, half_hip_length+0.04])

    # Forearm
    arm = [0.3+0.04, 0.4-0.12-0.01]
    add_object([arm[0], arm[1]],
               halfsize=[0.08, 0.02])

    s = 200

    thigh_relax = 0.90
    leg_relax = 0.89
    foot_relax = 0.71

    thigh_stiff = 5
    leg_stiff = 20
    foot_stiff = 26

    #left springs
    add_spring(0, 1, [0, (half_hip_length - 0.01) * 0.4],
               [0, -thigh_half_length],
               thigh_relax * (2.0 * thigh_half_length + 0.22), thigh_stiff)
    add_spring(1, 2, [-0.07, thigh_half_length+0.01], [0, -thigh_half_length],
               leg_relax * 4.0 * thigh_half_length, leg_stiff, 0.167)
    add_spring(
        2, 3, [0, thigh_half_length], [foot_half_length, 0],
        foot_relax *
        math.sqrt(pow(2*thigh_half_length, 2) + pow(2.0 * foot_half_length, 2)),
        foot_stiff, 0.04)

    add_spring(0, 1, [0, -(half_hip_length - 0.01)], [0.0, thigh_half_length],
               -1, s)
    add_spring(1, 2, [0, -thigh_half_length], [0.0, thigh_half_length], -1, s)
    add_spring(2, 3, [0, -thigh_half_length], [-foot_half_length, 0], -1, s)

    #right springs
    add_spring(0, 4, [0, (half_hip_length - 0.01) * 0.4],
               [0, -thigh_half_length],
               thigh_relax * (2.0 * thigh_half_length + 0.22), thigh_stiff)
    add_spring(4, 5, [-0.07, thigh_half_length+0.01], [0, -thigh_half_length],
               leg_relax * 4.0 * thigh_half_length, leg_stiff, 0.16)
    add_spring(
        5, 6, [0, thigh_half_length], [foot_half_length, 0],
        foot_relax *
        math.sqrt(pow(2*thigh_half_length, 2) + pow(2.0 * foot_half_length, 2)),
        foot_stiff, 0.04)

    add_spring(0, 4, [0, -(half_hip_length - 0.01)], [0.0, thigh_half_length],
               -1, s)
    add_spring(4, 5, [0, -thigh_half_length], [0.0, thigh_half_length], -1, s)
    add_spring(5, 6, [0, -thigh_half_length], [-foot_half_length, 0], -1, s)

    # arm..
    add_spring(0,7, [0, 0], [0, 0.08], -1, s)
    add_spring(0,7, [0.07, -0.07], [0, 0.01], 0.07, 1, 0.08)

    add_spring(7,8, [0, -0.09], [-0.05, 0.0], -1, s)
    add_spring(7,8, [0.0, 0.08], [0.05, 0.0], 0.20, 5, 0.08)

    return objects, springs, 3


def robotB():
    body = add_object([0.15, 0.25], [0.1, 0.03])
    back = add_object([0.08, 0.22], [0.03, 0.10])
    front = add_object([0.22, 0.22], [0.03, 0.10])

    rest_length = 0.22
    stiffness = 50
    act = 0.03
    add_spring(body,
               back, [0.08, 0.02], [0.0, -0.08],
               rest_length,
               stiffness,
               actuation=act)
    add_spring(body,
               front, [-0.08, 0.02], [0.0, -0.08],
               rest_length,
               stiffness,
               actuation=act)

    add_spring(body, back, [-0.08, 0.0], [0.0, 0.02], -1, stiffness)
    add_spring(body, front, [0.08, 0.0], [0.0, 0.05], -1, stiffness)

    return objects, springs, body

y_max = 1.0 
y_min = 0.1
x_min = 0.01
x_max = 0.5

center = [0.25, 0.5]

import random as rand

def build_robot_skeleton(shape: str = "wheel", stiffness: int = 75, actuation: float = 0.115, rest_to_spring: float = 1, num_boxes: int = 6, radius: float = 0.15, random=False, spokes=False):
    """
    Generate a rigid body skeleton based on the given user parameters.
    
    Parameters:
        shape (str): Shape of the robot skeleton. Can be "circle" or "wheel".
        stiffness (int): Stiffness of the springs.
        actuation (float): Actuation of the springs.
        rest_to_spring (float): Rest length of the springs compared to actual, i.e 1.25 = springs compressed.
        num_boxes (int): Number of boxes in the skeleton.
        radius (float): Radius of underlying circle.
        random (bool): If True, randomize the parameters.
        spokes (bool): If True, add spokes to the wheel shape. Not very interesting.
    """
    global objects, springs
    objects = []
    springs = []
    # Randomize the parameters.
    if random:
        shape = rand.choice(["circle", "wheel"])
        stiffness = rand.randint(50, 100)
        actuation = rand.uniform(0.05, 0.35)
        rest_to_spring = rand.uniform(0.75, 1.25)
        num_boxes = rand.randint(3, 8)
        radius = rand.uniform(0.1, 0.20)

    # print("PARAMETERS:")
    # print("shape: ", shape)
    # print("stiffness: ", stiffness)
    # print("actuation: ", actuation)
    # print("rest_to_spring ratio: ", rest_to_spring)
    # print("num_boxes: ", num_boxes)
    # print("radius: ", radius)

    # Angle between each box.
    if shape == "circle":
        angle_size = 2 * math.pi / num_boxes
    else:
        angle_size = 2 * math.pi / (num_boxes - 1)

    # Halfsize of the boxes.
    if shape == "wheel" or shape == "circle":
        halfsize = [0.02,radius*math.tan(angle_size/2)] # Halfsize now determined by radius and angle size.

    # c = []

    # If the shape isn't a circle, add named center node.
    if shape != "circle":
        center_node = add_object(center, halfsize=[0.05, 0.05])

    # Decrease box count for center node.
    if shape == "star" or shape == "wheel":
        num_boxes -= 1

    # Add boxes (objects).
    for i in range(num_boxes):
        angle = (angle_size * i)                                    # Iteratively calculate the angle of each box.
        x = center[0] + radius * math.cos(angle)      # Box center position
        y = center[1] + radius * math.sin(angle)
        add_object([x, y], halfsize=[halfsize[0], halfsize[1]*1.1], rotation=angle)                                # Add box.
        # c.append([x,y])
    
    # Option for spokes (boxes connecting center/outer nodes). Doesn't yield very interesting results, laggy.
    if (shape == "star" or shape == "wheel") and spokes:
        for i in range(num_boxes):
            angle = (angle_size * i)                                    # Iteratively calculate the angle of each box.
            x = 0.5 * center[0] + radius * math.cos(angle)              # Box center position
            y = 0.5 * center[1] + radius * math.sin(angle)
            add_object([x, y], halfsize=[0.6*radius, halfsize[0]], rotation=angle) 

    # # Rest length for circle shape calculated as distance between first and second box.
    # rest_length = math.sqrt(pow(c[0][0] - c[1][0], 2) + pow(c[0][1] - c[1][1], 2))

    if shape == "wheel" or shape == "circle":
        rest_length = 2*halfsize[1]

    # Add springs.
    for i in range(num_boxes):
        # If circle, add springs between neighboring boxes.
        if shape == "circle":
            # actuation = math.sin(i)/10
            if i == num_boxes-1:
                add_spring(i, 0, [0, 0], [0, 0], rest_to_spring*rest_length, stiffness, actuation)
                add_spring(i, 0, [0, halfsize[1]], [0, -halfsize[1]], -1, stiffness, 0)
                continue
            add_spring(i, i+1, [0, 0], [0, 0], rest_to_spring*rest_length, stiffness, actuation)
            add_spring(i, i+1, [0, halfsize[1]], [0, -halfsize[1]], -1, stiffness, 0)

        # DEPRECATED
        # If star, add springs between center node and all boxes.
        # elif shape == "star":
        #     add_spring(0, i+1, [0, 0], [0, 0], rest_to_spring*radius, stiffness, actuation)
        #     add_spring(0,i+1+num_boxes, [0, 0], [-halfsize[1],0], -1, stiffness, 0)
        #     add_spring(i+1,i+1+num_boxes, [0, 0], [halfsize[1],0], -1, stiffness, 0)

        elif shape == "wheel":
            # actuation = math.sin(i)/10
            add_spring(0, i+1, [0, 0], [0, halfsize[1]], (radius/math.cos(angle_size/2)), 0.75*stiffness, actuation)
            if spokes:
                add_spring(0,i+1+num_boxes, [0, 0], [-0.6*radius,0], -1, 10, 0)
                add_spring(i+1,i+1+num_boxes, [0, 0], [0.6*radius,0], -1, 10, 0)
            if i == num_boxes-1:
                add_spring(i+1, 1, [0, halfsize[1]], [0, halfsize[1]], rest_to_spring*rest_length, 10, actuation)
                add_spring(i+1, 1, [0, halfsize[1]], [0, -halfsize[1]], -1, stiffness, 0)
                continue
            if i == num_boxes:
                continue
            add_spring(i+1, i+2, [0, halfsize[1]], [0, halfsize[1]], rest_to_spring*rest_length, 10, actuation)
            add_spring(i+1, i+2, [0, halfsize[1]], [0, -halfsize[1]], -1, stiffness, 0)

    return objects, springs, 0

def test_robot():
    return build_robot_skeleton(shape="wheel", num_boxes=9)
    #return build_robot_skeleton(shape="circle", stiffness=80, actuation=0.04, rest_to_spring=1.03, num_boxes=8, radius=0.12)

def robotC():
    body = add_object([0.15, 0.16], [0.05, 0.05])
    back = add_object([0.35, 0.16], [0.05, 0.05])

    rest_length = 0.10
    stiffness = 50
    act = 0.1
    add_spring(body,
               back, [0.00, 0.00], [0.0, 0.0],
               rest_length,
               stiffness,
               actuation=act)

    # add_spring(body, back, [0.0, 0.0], [0.0, 0.0], -1, stiffness)
    # add_spring(body, front, [0.0, 0.0], [0.0, 0.0], -1, stiffness)

    return objects, springs, body




robots = [robotA, robotB, robotLeg, robotC, test_robot]
