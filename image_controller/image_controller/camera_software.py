import numpy as np

# relative position to base_link: xyz(1, 0, 1)
# roll pitch yaw (radial): rpy(-1.5707963267948966, 0, -1.5707963267948966)

# homography matrix: K(R|t)

def project_polyline(polyline_world, trafo_world_to_cam, K):
    x,y,z = polyline_world[:,0], polyline_world[:,1], polyline_world[:,2]
    homvec = np.stack((x,y,z,np.ones_like(x)))
    proj_mat = K @ trafo_world_to_cam[:3,:]
    pl_uv_cam = (proj_mat @ homvec).T
    u = pl_uv_cam[:,0] / pl_uv_cam[:,2]
    v = pl_uv_cam[:,1] / pl_uv_cam[:,2]
    return np.stack((u,v)).T


class CameraGeometry(object):
    def __init__(self, height=1.0, roll_deg=0, pitch_deg=0, yaw_deg=0, K_matrix=None):
        # scalar constants
        self.height = height
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.yaw_deg = yaw_deg
        
        # Camera intrinsics
        self.intrinsic_matrix = K_matrix
        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)

        ### Perspective projection matrix ###
        # Create extrinsic matrix for the perspective of the camera 
        self.rotation_perspective_cam = self.create_rotation_matrix(roll_deg, pitch_deg, yaw_deg)
        self.trafo_perspective_cam = np.eye(4)
        self.trafo_perspective_cam[0:3,0:3] = self.rotation_perspective_cam
        self.trafo_perspective_cam[0:3, 3] = np.array([0, 0, self.height]) # translation vector

        # Create extrinsic matrix for the perspective of the camera
        self.rotation_perspective_bird = self.create_rotation_matrix(roll_deg=0, pitch_deg=0, yaw_deg=0)
        self.trafo_perspective_bird = np.eye(4)
        self.trafo_perspective_bird[0:3,0:3] = self.rotation_perspective_bird
        self.trafo_perspective_bird[0:3, 3] = np.array([0, 0, self.height]) # translation vector

        ### Transformation matrix camera coordinates to world coordinates ###
        # Note: "rotation_world_to_cam" : R_{wc} 
        self.rotation_cam_to_world = self.create_rotation_matrix(roll_deg=-90, pitch_deg=0, yaw_deg=-90)
        self.rotation_world_to_cam = self.rotation_cam_to_world.T # for rotation matrices, taking the transpose is the same as inversion (R_{wc}^T = R_{cw})

        self.translation_cam_to_world = np.array([0, 0, self.height]) 
        self.trafo_cam_to_world = np.eye(4)
        self.trafo_cam_to_world[0:3, 0:3] = self.rotation_cam_to_world
        self.trafo_cam_to_world[0:3, 3] = self.translation_cam_to_world
        
        # compute vector nc
        self.world_normal_camframe = self.rotation_world_to_cam @ np.array([0, 0, -1])

    def create_rotation_matrix(self, roll_deg, pitch_deg, yaw_deg):
        roll = np.deg2rad(roll_deg)
        pitch = np.deg2rad(pitch_deg)
        yaw = np.deg2rad(yaw_deg)

        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        # Create rotation matrices in vehicle coordinate system ISO8855 (x: up, y: left, z: forward)
        # vehicle coordinate system ISO8855 is also world system
        # R_x
        R_roll = np.array([[1, 0, 0],
                            [0, cr, -sr],
                            [0, sr, cr]])
        # R_y
        R_pitch = np.array([[cp, 0, sp],
                            [0, 1, 0],
                            [-sp, 0, cp]])
        # R_z
        R_yaw = np.array([[cy, -sy, 0],
                            [sy, cy, 0],
                            [0, 0, 1]]) 
        
        # Combine rotations

        return np.array([[0, 0, 1],
                        [-1, 0, 0],
                        [0, -1, 0]]) 
        """
        return R_yaw @ R_pitch @ R_roll"""

    def create_perspective_homography(self, K1, K2, R_t1, R_t2):
        """
        Create a homography matrix that transforms points from the perspective of the camera to the perspective of the bird's eye view.

        Arguments:
            K1(np.array): The intrinsic matrix of the first view.
            K2(np.array): The intrinsic matrix of the second view.
            R_t1(np.array): The rotation and translation matrix of the first view.
            R_t2(np.array): The rotation and translation matrix of the second view. (example: bird's eye view)
        Returns:
            H(np.array): The homography matrix that transforms points from the first view to the second view.
        """

        H1 = K1 @ R_t1[0:3, 0:4]
        H2 = K2 @ R_t2[0:3, 0:4]

        H1[0:3, 2] = H1[0:3, 3]
        H1 = H1[0:3, 0:3]
        H2[0:3, 2] = H2[0:3, 3] 
        H2 = H2[0:3, 0:3]

        H_1to2 = H2 @ np.linalg.inv(H1)

        return H_1to2

    def camframe_to_worldframe(self, vec_in_cam_frame):
        return self.rotation_cam_to_world @ vec_in_cam_frame + self.translation_cam_to_world

    def uv_to_XYZ_camframe(self,u,v):
        print("u v:")
        print(u, v)
        uv_hom = np.array([u,v,1])
        Kinv_uv_hom = self.inverse_intrinsic_matrix @ uv_hom
        denominator = self.world_normal_camframe.dot(Kinv_uv_hom)
        return self.height*Kinv_uv_hom/denominator
    
    def uv_to_XYZ_worldframe(self,u,v):
        r_camframe = self.uv_to_XYZ_camframe(u,v)
        return self.camframe_to_worldframe(r_camframe)

    def compute_dist(self, u, v):
        """
        Compute the distance from the camera to the point on the road.
        The distance is computed as the length of the vector from the vehicle front (ISO8855 [1, 0, 0]) to the point on the road.
        """
        vec_camframe = self.uv_to_XYZ_camframe(u,v)
        print("vec_camframe:")
        print(vec_camframe)
        len_vehicle_shadow = 2.24
        len_vehicle_front = 1
        dist_hypo = np.linalg.norm(vec_camframe)

        dist = np.sqrt(dist_hypo**2 - self.height**2) - len_vehicle_front - len_vehicle_shadow
        
        return dist

    def get_min_carless_pixel(self, shadow_point=None):
        """
        Get the minimum pixel (u,v) for which the car is not shown in the image.
        """

        u, v, _ = self.intrinsic_matrix @ shadow_point / shadow_point[2] # (u, v , 1).T = 1/Zc * K * (Xc, Yc, Zc).T

        return u, (v-1)

    def test_camera_geometry(self, test_vec_camframe, test_vec_worldframe):
        # Test the camera geometry
        print("Camera Geometry:")
        print("Intrinsic Matrix:")
        print(self.intrinsic_matrix)
        print("Extrinsic Matrix (Camera to World):")
        print(self.trafo_cam_to_world)
        print("Rotation Matrix (World to Camera):")
        print(self.rotation_world_to_cam)
        print("Translation Vector (Camera to World):")
        print(self.translation_cam_to_world)

        print("\n### Test the transformation from camera frame to world frame ###")
        print("vec_camframe:")
        vec_camframe = test_vec_camframe
        print(vec_camframe)

        print("vec_worldframe:")
        vec_worldframe = self.rotation_cam_to_world @ vec_camframe + self.translation_cam_to_world
        print(vec_worldframe)

        print("vec_camframe:")
        vec_camframe = self.rotation_world_to_cam @ (vec_worldframe - self.translation_cam_to_world)
        print(vec_camframe)

        print("\n### Test the transformation from world frame to cam frame ###")
        print("vec_worldframe:")
        vec_worldframe = test_vec_worldframe
        print(vec_worldframe)

        print("vec_camframe:")
        vec_camframe = self.rotation_world_to_cam @ (vec_worldframe - self.translation_cam_to_world)
        print(vec_camframe)

        print("vec_worldframe:")
        vec_worldframe = self.rotation_cam_to_world @ vec_camframe + self.translation_cam_to_world  
        print(vec_worldframe)