import rospy

import numpy as np

import threading

import tf
import tf.transformations as tr

import threading


def transl(t):
    # TRANSL	Translational transform
    #
    #	T= TRANSL(X, Y, Z)
    #	T= TRANSL( [X Y Z] )
    #
    #	[X Y Z]' = TRANSL(T)
    #
    #	[X Y Z] = TRANSL(TG)
    #
    #	Returns a homogeneous transformation representing a
    #	translation of X, Y and Z.
    #
    #	The third form returns the translational part of a
    #	homogenous transform as a 3-element column vector.
    #
    #	The fourth form returns a  matrix of the X, Y and Z elements
    #	extracted from a Cartesian trajectory matrix TG.
    r = np.array([[1, 0, 0, t[0]], [0, 1, 0, t[1]], [0, 0, 1, t[2]], [0, 0, 0, 1]])
    return r


def skew(V):
    # skew - returns skew matrix of a 3x1 vector.
    #        cross(V,U) = skew(V)*U
    #
    #    S = skew(V)
    #
    #          0  -Vz  Vy
    #    S =   Vz  0  -Vx
    #         -Vy  Vx  0
    #
    S = np.array([[0, -V[2], V[1]], [V[2], 0, -V[0]], [-V[1], V[0], 0]])
    return S


def rot2quat(R):
    # rot2quat - converts a rotation matrix (3x3) to a unit quaternion(3x1)
    #
    #    q = rot2quat(R)
    #
    #    R - 3x3 rotation matrix, or 4x4 homogeneous matrix
    #    q - 3x1 unit quaternion
    #        q = sin(theta/2) * v
    #        teta - rotation angle
    #        v    - unit rotation axis, |v| = 1
    #

    w4 = 2 * np.sqrt(1 + np.trace(R[0:3, 0:3]))  # can this be imaginary?
    q = np.array([[(R[2, 1] - R[1, 2]) / w4], [(R[0, 2] - R[2, 0]) / w4], [(R[1, 0] - R[0, 1]) / w4]])
    return q


def quat2rot(q):
    # quat2rot - a unit quaternion(3x1) to converts a rotation matrix (3x3)
    #
    #    R = quat2rot(q)
    #
    #    q - 3x1 unit quaternion
    #    R - 4x4 homogeneous rotation matrix (translation component is zero)
    #        q = sin(theta/2) * v
    #        teta - rotation angle
    #        v    - unit rotation axis, |v| = 1
    #
    p = np.dot(q, q)
    if p > 1:
        print('Warning: quat2rot: quaternion greater than 1')

    w = np.sqrt(1 - p)  # w = cos(theta/2)

    R = np.eye(4)
    R[0:3, 0:3] = 2 * np.outer(q, q) + 2 * w * skew(q) + np.eye(3) - 2 * np.diag([p, p, p])
    return R


def chessboard(bHg, camHc):

    # OptCam --perform the transformation between OptiTrack and realsense camera
    #   bHg   - the center point1(g) in chessboard relative to the Optitrack Frame(base)
    #           Matrix dimensions are 4*4*M, where M is the number of points positions

    #   camHc - the center point2(c) in chessboard relative to the realsense camera Frame(cam)
    #            Dimension: size(CamHc) = size(bHg)
    #   gHc - 4*4 homogeneous tranformation from point1 to point2, that is point2(c) position relative
    #          to the point1(g)

    #   notation: point1 and point2 are the same point in the chessboard, but have different coordination
    #   relative to different Frame

    M = camHc.shape[2]

    K = (M*M-M)/2  # Number of unique position between points1 and points2
    A = np.zeros((3*K, 3))  # will store: skew(Pgij+Pcij)
    B = np.zeros((3*K, 1))  # will store: Pcij-pgij
    k = 0
    # Hc = cHcam = inv(camHc); Transformation from camera frame to point2(c)
    Hg = bHg
    Hc = np.zeros((4,4,M))
    for i in range(M):
        Hc[:,:,i] = np.linalg.inv(camHc[:,:,i])

    for i in range(M):
        for j in range(i+1,M):
            Hgij = np.dot(np.linalg.inv(Hg[:,:,j]),Hg[:,:,i]) # Transformation from i-th to j-th point1(base)
            Pgij = 2*rot2quat(Hgij)  # ... and the corresponding quaternion

            Hcij = np.dot(Hc[:,:,j], np.linalg.inv(Hc[:,:,i])) # Transformation from i-th to j-th point2(cam)
            Pcij = 2*rot2quat(Hcij)  # ... and the corresponding quaternion

            # Form linear system of equations
            A[k:(k+3), 0:3] = skew(Pgij+Pcij)  # left side
            B[k:(k+3), :] = Pcij - Pgij        # right side
            k += 3

    #    Rotation from point2 to point1 is obtained from the set of equations:
    #    skew(Pgij+Pcij) * Pcg_ = Pcij - Pgij

    Pcg_, residuals, rank, singulars = np.linalg.lstsq(A, B)
    Pcg_ = Pcg_[:, 0]

    #Pcg = A\B;         # Solve the equation A*Pcg_=B
    Pcg = 2*Pcg_/np.sqrt(1+np.dot(Pcg_,Pcg_))

    Rcg = quat2rot(Pcg/2) # Rotation matrix

    # calculate translational component
    k=0;
    for i in range(M):
        for j in range(i+1,M):
            Hgij = np.dot(np.linalg.inv(Hg[:,:,j]),Hg[:,:,i])
            Hcij = np.dot(Hc[:,:,j], np.linalg.inv[:,:,i])

            # Form linear system of equations
            A[k:(k + 3), 0:3] = Hgij[0:3, 0:3] - np.eye(3)  # left-hand side
            B[k:(k + 3), 0] = np.dot(Rcg[0:3, 0:3], Hcij[0:3, 3]) - Hgij[0:3, 3]  # right-hand side
            k += 3

    Tcg, residuals, rank, singulars = np.linalg.lstsq(A, B)
    # Tcg = A \ B;
    gHc = np.dot(transl(Tcg), Rcg)  # incorporate translation with rotation

    return gHc
