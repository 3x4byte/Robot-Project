from top_camera import TopCamera
from information_extraction import InformationExtraction
from robot import Robot
from vector import Vector
from mqtt_client import MqttClient

import operator
import math
import cv2
import numpy as np
import time
import keyboard
import threading

# values for removing fisheye distortion
DIM = (1024, 768)
K = np.array([[26864.957112648033, 0.0, 462.391611030626], [0.0, 26100.130628757528, 355.3302327869351], [0.0, 0.0, 1.0]])
D = np.array([[-267.22399997112325], [21930.119543790075], [3445785.4755827268], [772191813.4918289]])

debug = True
condition = threading.Condition()
pause_condition = threading.Condition()
extract = None
robot = None
vector = None
img_clear = None
paused = False
station_pos = [70, 550]

def debug_camera():
    while debug:
        image = cam.get_img_undistorted(DIM, K, D)

        if img_clear is not None:
            debug_diff = extract.get_diff_as_bitmap(img_clear, image)
            debug_diff = extract.erode_dilate(debug_diff)
            debug_contours, _ = extract.get_contours(debug_diff)

            debug_robot = extract.get_robot(debug_contours)
            if debug_robot is not None:
                pt1 = (int(debug_robot[0]), int(debug_robot[1]))
                pt2 = (int(debug_robot[0] + debug_robot[2]), int(debug_robot[1] + debug_robot[3]))
                image = cv2.rectangle(image, pt1, pt2, (255, 0, 0), 2, cv2.LINE_AA)
                # image = cv2.putText(image, "robot", pt1, cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 1, cv2.LINE_AA)

            debug_bricks = extract.get_bricks(debug_contours)
            if debug_bricks is not None and debug_robot is not None:
                for brick in debug_bricks:
                    if brick[0] < debug_robot[0] or brick[1] < debug_robot[1] or brick[0] > (debug_robot[0] + debug_robot[2]) or brick[1] > (debug_robot[1] + debug_robot[3]):
                        pt1 = (int(brick[0]), int(brick[1]))
                        pt2 = (int(brick[0] + brick[2]), int(brick[1] + brick[3]))
                        image = cv2.rectangle(image, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)
                        # image = cv2.putText(image, "brick", pt1, cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)

        if (robot.position is not None and robot.rotation is not None):
            pt1 = (int(robot.position[0]), int(robot.position[1]))
            pos2 = robot.position + 50 * vector.unit(robot.rotation)
            pt2 = (int(pos2[0]), int(pos2[1]))
            image = cv2.line(image, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)

        image = cv2.circle(image, (station_pos[0], station_pos[1]), 2, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow("image", image)
        cv2.waitKey(10)

def get_robot_position():
    print(f"--------------- function: get_robot_position ---------------")
    img = cam.get_img_undistorted(DIM, K, D)
    cv2.waitKey(10)
    diff = extract.get_diff_as_bitmap(img_clear, img)
    diff = extract.erode_dilate(diff)
    contours, _ = extract.get_contours(diff)
    robot_center = extract.get_robot_center(contours)
    return robot_center

def go_to_brick(nearest_brick):
    print(f"--------------- function: go_to_brick ---------------")
    print(f"nearest brick: {nearest_brick}")

    # get steps to brick and move
    print(f"(2) pixels: {nearest_brick[2]}")
    print(f"(2) steps: {nearest_brick[2] / robot.pixel_per_step}")
    brick_vec = [nearest_brick[0] - robot.position[0], nearest_brick[1] - robot.position[1]]
    move_dist = vector.length(brick_vec) / robot.pixel_per_step

    while move_dist > 900:
        print(f"__________MOVE_DIST__________{move_dist}")
        # get steps to rotate and rotate
        brick_vec = [nearest_brick[0] - robot.position[0], nearest_brick[1] - robot.position[1]]
        angle = vector.angle(robot.rotation, brick_vec)

        steps = angle / robot.degrees_per_step
        print(f"(1) robot vec: {robot.rotation} || vec to brick: {[nearest_brick[0], nearest_brick[1]]}")
        print(f"(1) angle: {angle}")
        print(f"(1) steps: {steps}")
        robot.rotate(steps)
        with condition:
            condition.wait()

        # move halfway to brick
        pos1 = get_robot_position()

        robot.move((move_dist/2))
        with condition:
            condition.wait()

        pos2 = get_robot_position()

        print(f"position_calc: {pos2}")
        print(f"position_robo: {robot.position}")

        robot.position = pos2

        rot = [pos2[0] - pos1[0], pos2[1] - pos1[1]]
        print(f"rotation_calc: {rot}")
        print(f"rotation_robo: {robot.rotation}")
        robot.rotation = rot

        print(f"movedist: {move_dist}")
        brick_vec = [nearest_brick[0] - robot.position[0], nearest_brick[1] - robot.position[1]]
        move_dist = vector.length(brick_vec) / robot.pixel_per_step
        print(f"updated movedist: {move_dist}")

    else:
        lower_grabber()
        # robot.move(500, 100)
        # with condition:
        #     condition.wait()

        brick_vec = [nearest_brick[0] - robot.position[0], nearest_brick[1] - robot.position[1]]
        angle = vector.angle(robot.rotation, brick_vec)
        robot.rotate(angle / robot.degrees_per_step)
        with condition:
            condition.wait()

        move_dist = vector.length(brick_vec) / robot.pixel_per_step
        robot.move(move_dist, 200)
        with condition:
            condition.wait()

def lower_grabber():
    print(f"--------------- function: lower_grabber ---------------")

    robot.down(360)
    with condition:
        condition.wait()
    robot.release(300)
    with condition:
        condition.wait()

def pickup_brick():
    print(f"--------------- function: pickup_brick ---------------")

    robot.grab(600)
    with condition:
        condition.wait()
    robot.up(360)
    with condition:
        condition.wait()

def go_near_station():
    print(f"--------------- function: go_near_station ---------------")

    vec_to_station = [station_pos[0] - robot.position[0], station_pos[1] - robot.position[1]]
    move_dist = vector.length(vec_to_station) / robot.pixel_per_step

    while move_dist > 900:
        vec_to_station = [station_pos[0] - robot.position[0], station_pos[1] - robot.position[1]]
        angle = vector.angle(robot.rotation, vec_to_station)
        robot.rotate(angle / robot.degrees_per_step)
        with condition:
            condition.wait()

        pos1 = get_robot_position()

        move_dist = vector.length(vec_to_station) / robot.pixel_per_step
        robot.move(move_dist/2)
        with condition:
            condition.wait()

        pos2 = get_robot_position()
        robot.position = pos2

        rot = [pos2[0] - pos1[0], pos2[1] - pos1[1]]
        robot.rotation = rot

    else:
        vec_to_station = [station_pos[0] - robot.position[0], station_pos[1] - robot.position[1]]
        angle = vector.angle(robot.rotation, vec_to_station)
        robot.rotate(angle / robot.degrees_per_step)
        with condition:
            condition.wait()

        move_dist = vector.length(vec_to_station) / robot.pixel_per_step
        robot.move(move_dist)
        with condition:
            condition.wait()

def go_to_station():
    print(f"--------------- function: go_to_station ---------------")

    go_near_station()
    vec_down = [0, 1]
    angle = vector.angle(robot.rotation, vec_down)
    robot.rotate(angle/robot.degrees_per_step)
    with condition:
        condition.wait()

def deliver_brick_to_station():
    print(f"--------------- function: deliver_brick_to_station ---------------")

    robot.up(720)
    with condition:
        condition.wait()

    robot.move(110/robot.pixel_per_step)
    with condition:
        condition.wait()

    release_brick()
    robot.brick_delivered()
    reset_position()

def release_brick():
    print(f"--------------- function: release_brick ---------------")

    robot.release(300)
    with condition:
        condition.wait()

def reset_position():
    print(f"--------------- function: reset_position ---------------")

    steps = -300
    robot.move(steps)
    with condition:
        condition.wait()

    rotation = 180
    robot.rotate(rotation/robot.degrees_per_step)
    with condition:
        condition.wait()

    robot.down(720)
    with condition:
        condition.wait()

def calculate_dist(image_1, image_2):
    print(f"--------------- function: calculate_dist ---------------")

    cont1, _ = extract.get_contours(image_1)
    cont2, _ = extract.get_contours(image_2)
    pos1 = extract.get_robot_center(cont1)
    pos2 = extract.get_robot_center(cont2)
    print(f"pos1: {pos1}")
    print(f"pos2: {pos2}")

    robot.position = pos2
    rot = [pos2[0] - (pos1[0]), pos2[1] - (pos1[1])]
    print(f"rotation: {rot}")
    robot.rotation = rot

    if pos1 is not None and pos2 is not None:
        a = pos2[0] - pos1[0]
        b = pos2[1] - pos1[1]

        dist = math.sqrt(a*a + b*b)
        return dist
    return None

def reference_trip(camera, clear_image):
    print(f"--------------- function: reference_trip ---------------")

    steps = 1000
    img1 = camera.get_img_undistorted(DIM, K, D)
    cv2.waitKey(10)
    pos1 = extract.get_diff_as_bitmap(clear_image, img1)
    pos1 = extract.erode_dilate(pos1)

    robot.reference_move(steps, 300)
    with condition:
        condition.wait()

    img2 = camera.get_img_undistorted(DIM, K, D)
    cv2.waitKey(10)
    pos2 = extract.get_diff_as_bitmap(clear_image, img2)
    pos2 = extract.erode_dilate(pos2)

    dist = calculate_dist(pos1, pos2)
    print(f"distance: {dist}")

    robot.pixel_per_step = dist/steps

def initialization():
    print(f"--------------- function: initialization ---------------")

    mqtt.send_pause_msg("false")

    robot.init()
    with condition:
        condition.wait()

def message_received(_, __, msg_obj):
    print(f"--------------- function: message_received ---------------")
    global paused

    msg = mqtt.parse_message(msg_obj)
    print(msg)

    if msg["type"] == "paused":
        if msg["value"] == "true":
            paused = True
        elif msg["value"] == "false":
            paused = False
            with pause_condition:
                pause_condition.notify_all()

    elif msg["type"] == "information" and msg["message"] == "completed":
        with condition:
            condition.notify_all()

if __name__ == "__main__":

    robot = Robot()
    vector = Vector()

    mqtt = MqttClient("image processing")
    mqtt.subscribe_to("laptop/navigator")
    mqtt.on_message(message_received)
    extract = InformationExtraction()

    # connecting to webcam
    cam = TopCamera("rtsp://141.46.137.93:8554/mystream")
    cam.start()

    time.sleep(5)

    threading.Thread(target=debug_camera).start()

    # creating a reference image of the empty table
    key = None
    print("empty the table for a reference image, then press 1")
    while key != "1":
        key = keyboard.read_key()

    img_clear = cam.get_img_undistorted(DIM, K, D)
    cv2.waitKey(10)

    key = None
    print("place robot on the table, then press 2")
    while key != "2":
        key = keyboard.read_key()
    initialization()
    reference_trip(cam, img_clear)
    go_to_station()
    robot.rotate(180/robot.degrees_per_step)
    with condition:
        condition.wait()
    print("reference trip done")

    key = None
    print("place bricks on the table, then press 3")
    while key != "3":
        key = keyboard.read_key()

    while True:
        key = None
        key = keyboard.read_key()
        if key == "q":
            print(f"stopping...")
            cam.stop()
            debug = False
        elif key == "d":
            debug = not debug
            print(f"changing debug mode to: {debug}")
            if debug:
                threading.Thread(target=debug_camera).start()

        # getting current img
        img = cam.get_img_undistorted(DIM, K, D)
        cv2.waitKey(10)

        # get diff and convert to bitmap
        diff = extract.get_diff_as_bitmap(img_clear, img)
        diff = extract.erode_dilate(diff)
        contours, _ = extract.get_contours(diff)

        # getting bot and bricks
        trys = 3
        while trys > 0:
            bot = extract.get_robot(contours)
            bot_center = extract.get_robot_center(contours)
            bricks = extract.get_bricks_center(contours)

            # get vectors from robot to every brick
            pos_dist = []
            for brick in bricks:
                vec = [brick[0] - bot_center[0], brick[1] - bot_center[1]]
                print(f"bot x: {bot_center[0]}| brick y: {bot_center[1]}")
                print(f"robot pos: {robot.position}")
                print(f"brick x: {brick[0]}| brick y: {brick[1]}")
                if brick[0] < bot[0] or brick[1] < bot[1] or brick[0] > (bot[0]+bot[2]) or brick[1] > (bot[1]+bot[3]):
                    # img = cv2.rectangle(img, (int(brick[0]) - 5, int(brick[1]) - 5), (int(brick[0]) + 5, int(brick[1]) + 5), (255, 0, 0), 2)
                    pos_dist_t = [brick[0], brick[1], vector.length(vec)]
                    pos_dist.append(pos_dist_t)
            # img = cv2.rectangle(img, (bot[0], bot[1]), (bot[0]+bot[2], bot[1]+bot[3]), (255, 0, 0), 2)
            # img = cv2.rectangle(img, (int(bot_center[0])-1, int(bot_center[1])-1), (int(bot_center[0])+1, int(bot_center[1])+1), (255, 0, 0), 2)
            print(f"brick positions: {pos_dist}")

            list.sort(pos_dist, key=operator.itemgetter(2))

            print(f"sorted: {pos_dist}")
            if not pos_dist:
                trys -= 1
            else:
                break
        else:
            print("No bricks where found...")
            continue

        go_to_brick(pos_dist[0])
        pickup_brick()
        go_to_station()

        if paused:
            with pause_condition:
                pause_condition.wait()

        deliver_brick_to_station()




