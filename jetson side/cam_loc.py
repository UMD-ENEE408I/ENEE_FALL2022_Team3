# import the opencv library
import cv2
import apriltag
import argparse
import numpy as np

# define a video capture object

marker_size_m = 0.2
cameraMatrix = np.array([(245.38031229, 0, 344.34838665), (0, 250.64969581, 225.25133835), (0,  0, 1.)])
distCoeffs = np.array([ 0.27784059, -0.31474221,  0.00308828, -0.00318653,  0.08205258])
#tag locations in real world

# Transform from o' to o
# o' is when the x-axis comes out of the tags face
# o is when the z axis comes out of the tags face
Rop_o = np.array([[0, 0, 1], [1,0,0], [0,1,0]]).astype(np.double)

Rid4_w_op = cv2.Rodrigues(np.array([0, 0, 0.0]))[0]
Rid4_wo   = Rid4_w_op @ Rop_o
tid4 = np.array([-1.5, 0, 0]).reshape((3,1))

Rid9_w_op = cv2.Rodrigues(np.array([0, 0, np.pi/2]))[0] # Check
Rid9_wo   = Rid9_w_op @ Rop_o
tid9 = np.array([0, -1.5, 0]).reshape((3,1))

Rid10_w_op = cv2.Rodrigues(np.array([0, 0, -np.pi/2]))[0] # Check
Rid10_wo   = Rid10_w_op @ Rop_o
tid10 = np.array([0, 1.5, 0]).reshape((3,1))

Rid8_w_op = cv2.Rodrigues(np.array([0, 0, np.pi]))[0] # Check
Rid8_wo   = Rid8_w_op @ Rop_o
tid8 = np.array([1.5, 0, 0]).reshape((3,1))

#dict of tags by id
tag_locs = {4:  [Rid4_wo,  tid4],
            9:  [Rid9_wo,  tid9],
            10: [Rid10_wo, tid10],
            8:  [Rid8_wo,  tid8]}

def read_tag(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    x = []
    y = []
    # loop over the AprilTag detection results
    for r in results:
	# extract the bounding box (x, y)-coordinates for the AprilTag
	# and convert each of the (x, y)-coordinate pairs to integers
            #point 0 top left then clockwise, need object 
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            
            m_half_size = marker_size_m / 2
            marker_center = np.array((0, 0, 0))
            marker_points = []
            marker_points.append(marker_center + (-m_half_size, m_half_size, 0))
            marker_points.append(marker_center + ( m_half_size, m_half_size, 0))
            marker_points.append(marker_center + ( m_half_size, -m_half_size, 0))
            marker_points.append(marker_center + (-m_half_size, -m_half_size, 0))
            marker_points = np.array(marker_points)
            objectPoints = marker_points
            imagePoints = r.corners
        # draw the bounding box of the AprilTag detection
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)

	# draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
	# draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            #print("[INFO] tag family: {}".format(tagFamily))
            
            (retval, rvec, tvec) = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, 0, 0, False, cv2.SOLVEPNP_IPPE_SQUARE)
            #print(retval) #true or false if it worked or not
            #print('rvec ')
            #print(rvec) #angle
            #print('t ')
            #print(tvec) #3rd is z-distance, t in the phone pics is translation vector
            Rco = cv2.Rodrigues(rvec)
            #print('R ')
            #print(R[0])
            Tco = np.hstack([Rco[0], tvec.reshape((3,1))])
            Tco = np.vstack([Tco, np.array([0,0,0,1]).reshape((1,4))])
            #print('Tco ')
            #print(Tco)
            Tco_inv = np.linalg.inv(Tco)

            #print('Tco inverse ')
            #print(Tco_inv)
            #block to get Two by r.ID
            if r.tag_id in tag_locs:
                Rwo = tag_locs[r.tag_id][0]
                Two = np.hstack([Rwo, tag_locs[r.tag_id][1]])
                Two = np.vstack([Two, np.array([0,0,0,1]).reshape((1,4))])
                #print('Two ')
                #print(Two)
                Twc = np.matmul(Two, Tco_inv) # Twc = Two Toc
                #print('Twc ')
                #print(Twc)
                x = Twc[0][3]
                y = Twc[1][3]
                Rc = np.hstack([Twc[0][0], Twc[0][1], Twc[0][2]])
                Rc = np.vstack([Rc, np.array([Twc[1][0], Twc[1][1], Twc[1][2]]).reshape((1,3))])
                Rc = np.vstack([Rc, np.array([Twc[2][0], Twc[2][1], Twc[2][2]]).reshape((1,3))])
                Rc = Rc.reshape((3,3))
                # heading = cv2.Rodrigues(Rc)[0]
                #print("RC ")
                #print(Rc)
                #print("Heading ")
                #print(heading)
                #print(x)
                #print(y)

                # It turns out we can not use rodriguez trick anymore
                # because the rotation between our camera and the world
                # is pretty complicated. So instead lets find the angle
                # between the camera's Z axis projected onto the floor
                # plane and the worlds X axis that is the heading
                v_c = [0, 0, 1]
                v_w = Rc @ v_c
                proj_v_w = v_w[0:2] / np.linalg.norm(v_w[0:2])
                theta_w = np.arctan2(proj_v_w[1], proj_v_w[0])
                # print(theta_w)
                # print(Rc)

                coords = [x, y, theta_w]
                return coords
            else:
                continue
    # show the output image after AprilTag detection
    # cv2.imshow("Image", image)
    

# Destroy all the windows
cv2.destroyAllWindows()
