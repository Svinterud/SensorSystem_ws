#Import modules
import rclpy
from rclpy.node import Node
import std_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
import time
import numpy as np
import cv2
import math
import operator
import io
import picamera
#from pyzbar.pyzbar import decode
#import pyzbar.pyzbar as pyzbar
import modern_robotics as mr

#Utilities, from github.com/czcbankai/CameraPoseEstimation

def get_center(points):
    x = 0
    y = 0
    for i in range(4):
        x += points[i][0]
        y += points[i][1]
    x_c = round(x / 4)
    y_c = round(y / 4)
    return np.c_[[x_c, y_c, 1]]

def distanceP2P(P, Q):
    P = P.astype(np.float32, copy=False)
    Q = Q.astype(np.float32, copy=False)
    return np.linalg.norm(P - Q)

def distanceP2L(M, N, P):
    M = M.astype(np.float32, copy=False)
    N = N.astype(np.float32, copy=False)
    P = P.astype(np.float32, copy=False)

    # General form for the eqation of a line: Ax + By + C = 0
    A, B, C = 0.0, 0.0, 0.0

    if N[0] - M[0] == 0:
        # Case 0: the line is vertical, x = -C
        A = 1.0
        C = -M[0]
    else:
        # Otherwise: Ax + y + C = 0
        A = -(N[1] - M[1]) / (N[0] - M[0])
        B = 1.0  # Fix B to 1.0
        C = -(A * M[0] + M[1])

    # Perpendicular distance: (Ax' + By' + C) / sqrt(A ^ 2 + B ^ 2)
    # If distance is positive, P is above / at the right side of line MN.
    # If distance is zero, P is on line MN.
    # If distance is negative, P is below / at the left side of line MN.
    return (A * P[0] + B * P[1] + C) / math.sqrt(A ** 2 + B ** 2)


def slope(M, N):
    M = M.astype(np.float32, copy=False)
    N = N.astype(np.float32, copy=False)

    if N[0] - M[0] == 0:
        # Case 0: Line MN is vertical, slope = infinity
        return None
    else:
        # Otherwise: slope = (y2 - y1) / (x2 - x1)
        return (N[1] - M[1]) / (N[0] - M[0])

def cross(v1, v2):
    v1 = v1.astype(np.float32, copy=False)
    v2 = v2.astype(np.float32, copy=False)

    # Cross product = x1 * y2 - x2 * y1
    return v1[0] * v2[1] - v1[1] * v2[0]

def intersection(M, N, P, Q):
    M = M.astype(np.float32, copy=False)
    N = N.astype(np.float32, copy=False)
    P = P.astype(np.float32, copy=False)
    Q = Q.astype(np.float32, copy=False)

    v1 = N - M
    v2 = Q - P
    crossProd = cross(v1, v2)

    if (crossProd == 0):
        # Case 0 / Case 1: Two lines overlap or are parallel
        return None
    else:
        # Otherwise: find intersection point
        t1 = cross(P - M, v2) / crossProd
        return np.array(np.int0(M + t1 * v1))

def getVertices(contour, marker_center):
    minRect = cv2.minAreaRect(contour)

    rectVertices = cv2.boxPoints(minRect)

    dists_to_marker_center = np.zeros((4, 1))
    vertices = np.array([None] * 4)

    for P in contour:
        P = P[0]

        dists_to_rect_vertices = []
        for rectVertex in rectVertices:
            rectVertex = np.array(rectVertex)
            dists_to_rect_vertices.append(distanceP2P(P, rectVertex))

        # Determine which quadrant that P locates in
        section_idx = np.argmin(dists_to_rect_vertices)

        dist_to_marker_center = distanceP2P(P, marker_center)

        if dist_to_marker_center > dists_to_marker_center[section_idx]:
            dists_to_marker_center[section_idx] = dist_to_marker_center
            vertices[section_idx] = P

    return vertices

def updateVerticesOrder(vertices, marker_center, pattern_center):
    dists = []
    for i in range(len(vertices)):
        dists.append((i, abs(distanceP2L(marker_center, pattern_center, vertices[i]))))

    dists = sorted(dists, key=operator.itemgetter(1))

    corner_idx = dists[0][0] if distanceP2P(vertices[dists[0][0]], pattern_center) \
        > distanceP2P(vertices[dists[1][0]], pattern_center) else dists[1][0]

    return np.append(vertices[corner_idx:], vertices[:corner_idx])

#####################################

thresh = 80
thresh_max_value = 90
canny_thresh1 = 100
canny_thresh2 = 200

def poseQR(img):
    #Modified code from github.com/czcbankai/CameraPoseEstimation
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Threshold
    th, thresh_img = cv2.threshold(gray_img, 80, 90, 0)

    # Canny to extract edges
    canny_img = cv2.Canny(thresh_img, canny_thresh1, canny_thresh2)

    # Find contours
    n, contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0, 255, 0), 1)
    hierarchy = hierarchy[0]

    marker_candidate = []
    for i in range(len(hierarchy)):
        j, count = i, 0
        # 'hierarchy[i]' format: [Next, Previous, First_Child, Parent]
        while hierarchy[j][2] != -1:
            # Keep searching for child in next level
            j = hierarchy[j][2]
            count += 1
        # Markers have nested contours with a total of six levels
        if count == 2:
            marker_candidate.append(i)
            marker_candidate = marker_candidate[-3:]

    #Center of each contour
    mass_centers = []
    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] == 0:
            mass_centers.append((0, 0))
        else:
            mass_centers.append(np.array((int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))))

    if (len(marker_candidate) == 3):
        A = marker_candidate[0]
        B = marker_candidate[1]
        C = marker_candidate[2]
    else:
        return None, None

    AB = distanceP2P(mass_centers[A], mass_centers[B])
    BC = distanceP2P(mass_centers[B], mass_centers[C])
    AC = distanceP2P(mass_centers[A], mass_centers[C])


    top_left, top_right, bot_left = None, None, None
    P, Q = None, None

    # In triangle ABC, the vertex not involved in the longest side is the mass center of 'top_left' contour.
    if AB > BC and AB > AC:
        top_left, P, Q = C, A, B
    elif BC > AB and BC > AC:
        top_left, P, Q  = A, B, C
    elif AC > AB and AC > BC:
        top_left, P, Q = B, A, C

    d = distanceP2L(mass_centers[P], mass_centers[Q], mass_centers[top_left])
    slope_line = slope(mass_centers[P], mass_centers[Q])

    if slope_line is None:
        bot_left, top_right = P, Q
    elif slope_line < 0 and d < 0:
        bot_left, top_right = P, Q
    elif slope_line > 0 and d < 0:
        top_right, bot_left = P, Q
    elif slope_line < 0 and d > 0:
        top_right, bot_left = P, Q
    elif slope_line > 0 and d > 0:
        bot_left, top_right = P, Q

    pattern_center = None
    top_left_vertices, top_right_vertices, bot_left_vertices = None, None, None

    # Get the pattern center
    pattern_center = np.array(((mass_centers[P][0] + mass_centers[Q][0]) // 2, \
                               (mass_centers[P][1] + mass_centers[Q][1]) // 2))

    # Get markers vertices
    top_left_vertices = getVertices(contours[top_left], mass_centers[top_left])
    top_right_vertices = getVertices(contours[top_right], mass_centers[top_right])
    bot_left_vertices = getVertices(contours[bot_left], mass_centers[bot_left])

    top_left_vertices = updateVerticesOrder(top_left_vertices, mass_centers[top_left], pattern_center)
    top_right_vertices = updateVerticesOrder(top_right_vertices, mass_centers[top_right], pattern_center)
    bot_left_vertices = updateVerticesOrder(bot_left_vertices, mass_centers[bot_left], pattern_center)

    '''
    	Find the fourth Bottom Right corner of the pattern.
    	'''
    M1, M2 = None, None
    bot_right_corner = None

    M1 = top_right_vertices[1] if distanceP2P(top_right_vertices[1], mass_centers[top_left]) \
                                  > distanceP2P(top_right_vertices[-1], mass_centers[top_left]) else \
    top_right_vertices[-1]
    M2 = bot_left_vertices[1] if distanceP2P(bot_left_vertices[1], mass_centers[top_left]) \
                                 > distanceP2P(bot_left_vertices[-1], mass_centers[top_left]) else bot_left_vertices[
        -1]

    bot_right_corner = intersection(top_right_vertices[0], M1, bot_left_vertices[0], M2)

    #Coordinate of the center of the markers
    tl_x, tl_y = top_left_vertices[0][0], top_left_vertices[0][1]
    tr_x, tr_y = top_right_vertices[0][0], top_right_vertices[0][1]
    bl_x, bl_y = bot_left_vertices[0][0], bot_left_vertices[0][1]
    br_x, br_y = bot_right_corner[0], bot_right_corner[1]

    #cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
    cv2.circle(img, (tl_x, tl_y), 10, (255, 0, 0), 3)
    cv2.circle(img, (tr_x, tr_y), 10, (0, 255, 0), 3)
    cv2.circle(img, (bl_x, bl_y), 10, (0, 0, 255), 3)
    cv2.circle(img, (br_x, br_y), 10, (100, 100, 100), 3)

    cv2.line(img, (tl_x, tl_y), (tl_x + 400, tl_y), (0, 255, 0), 3)
    cv2.line(img, (tl_x, tl_y), (tr_x, tr_y), (255, 0, 0), 3)


    #find rotation around z-axis
    a = np.array([tl_x + 400, tl_y])
    b = np.array([tl_x, tl_y])
    c = np.array([tr_x, tr_y])
    ba = a - b
    bc = c - b

    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)
    if tr_y > tl_y:
        angle = -angle


    center = get_center([[tl_x, tl_y],[tr_x, tr_y],[bl_x, bl_y],[br_x, br_y]])

    center_x = -top_left_vertices[0][0]+bot_left_vertices[0][0]
    center_y = -top_left_vertices[0][1]+top_right_vertices[0][1]
    return angle, center



def PosQR(img):
       # Position of QR-code in homogenous coordinates
    angle, center = poseQR(img)
    if (angle == None and center == None):
        return np.array([[0],
                         [0],
                         [0]]), 0
    else:
        # Camera matrix 1640 1232
        K = np.array([[2714.286, 0, 1640],
                      [0, 2714.286, 1232],
                      [0, 0, 1]])
        k1 = np.array([[2714.286, 0, 1296],
                      [0, 2714.286, 972],
                      [0, 0, 1]])
        # Normalized image coordinates
        s = np.linalg.solve(k1, center)
        # Position of QR.code with respect to camera
        p_c_QR = s * (2.538)
        p_c_QR[0] = -p_c_QR[0]
        p_c_QR[1] = -p_c_QR[1]
        return p_c_QR, angle

def PosRotCamera(img):
    posQR, Z_rot = PosQR(img)
    R_cam_QR = mr.MatrixExp3(mr.VecToso3([0, 0, 1]) * Z_rot)
    r_cam_QR = np.array([[posQR[0][0]],
                         [posQR[1][0]],
                         [posQR[2][0]]])
    T_cam_QR = mr.RpToTrans(R_cam_QR, r_cam_QR)
    T_QR_cam = mr.TransInv(T_cam_QR)
    [R, P] = mr.TransToRp(T_QR_cam)
    return R, P
#End utilities

class PosCamera(Node):
    def __init__(self):
        super().__init__('pos_camera')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.scan)
        #Default message
        message = PoseStamped()
        self.point = Point()
        self.quaternion = Quaternion()
        self.pose = Pose()
        self.msg = message
        self.pub = self.create_publisher(PoseStamped, "agv/landmark_pose", 5)
    
    def scan(self):
        stream = io.BytesIO()
        with picamera.PiCamera() as camera: #Initialize camera
            camera.capture(stream, format='jpeg') #Capture image
        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        data = np.array(data)
        im = cv2.imdecode(data, 1) #Convert to usable format for OpenCV
        r, p = PosRotCamera(im)
        p_x, p_y, p_z = p[0], p[1], p[2]
        r_w = math.sqrt((1 + r[0][0] + r[1][1] + r[2][2])/2)
        r_x = (r[2][1] - r[1][2])/(4*r_w)
        r_y = (r[0][2] - r[2][0])/(4*r_w)
        r_z = (r[1][0] - r[0][1])/(4*r_w)
        #Position x,y,z
        self.point.x = p_x
        self.point.y = p_y
        self.point.z = p_z
        #Rotation x,y,z,w
        self.quaternion.x = r_x
        self.quaternion.y = r_y
        self.quaternion.z = r_z
        self.quaternion.w = r_w
        #Pose message
        self.pose.position = self.point
        self.pose.orientation = self.quaternion
        #PoseStamped message
        self.msg.header = std_msgs.msg.Header()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = 'base_link'
        self.msg.pose = self.pose
        self.pub.publish(self.msg)
        
        
        

def main(args=None):
    time.sleep(1)
    rclpy.init(args=args)
    pos_camera = PosCamera()
    rclpy.spin(pos_camera)
    pos_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
