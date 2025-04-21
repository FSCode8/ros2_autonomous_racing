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
    def __init__(self, height=1, roll_deg=0, pitch_deg=0, yaw_deg=0, K_matrix=None):
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
        self.rotation_perspective_bird = self.create_rotation_matrix(roll_deg=10, pitch_deg=0, yaw_deg=0)
        self.trafo_perspective_bird = np.eye(4)
        self.trafo_perspective_bird[0:3,0:3] = self.rotation_perspective_bird
        self.trafo_perspective_bird[0:3, 3] = np.array([0, 0, self.height]) # translation vector

        ### Transformation matrix camera coordinates to world coordinates ###
        # Note: "rotation_world_to_cam" : R_{wc} 
        rotation_world_to_cam = self.create_rotation_matrix(roll_deg=-90, pitch_deg=0, yaw_deg=-90)
        self.rotation_cam_to_world = rotation_world_to_cam.T # for rotation matrices, taking the transpose is the same as inversion (R_{wc}^T = R_{cw})

        self.translation_cam_to_world = np.array([0, 0, self.height]) 
        self.trafo_cam_to_world = np.eye(4)
        self.trafo_cam_to_world[0:3, 0:3] = self.rotation_cam_to_world
        self.trafo_cam_to_world[0:3, 3] = self.translation_cam_to_world
        
        # compute vector nc
        self.world_normal_camframe = rotation_world_to_cam @ np.array([0,0,1])

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
        return R_yaw @ R_pitch @ R_roll

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

    def camframe_to_roadframe(self,vec_in_cam_frame):
        return self.rotation_cam_to_world @ vec_in_cam_frame + self.translation_cam_to_world

    def uv_to_roadXYZ_camframe(self,u,v):
        # NOTE: The results depend very much on the pitch angle (0.5 degree error yields bad result)
        # Here is a paper on vehicle pitch estimation:
        # https://refubium.fu-berlin.de/handle/fub188/26792
        uv_hom = np.array([u,v,1])
        Kinv_uv_hom = self.inverse_intrinsic_matrix @ uv_hom
        denominator = self.road_normal_camframe.dot(Kinv_uv_hom)
        return self.height*Kinv_uv_hom/denominator
    
    def uv_to_roadXYZ_roadframe(self,u,v):
        r_camframe = self.uv_to_roadXYZ_camframe(u,v)
        return self.camframe_to_roadframe(r_camframe)

    def uv_to_roadXYZ_roadframe_iso8855(self,u,v):
        X,Y,Z = self.uv_to_roadXYZ_roadframe(u,v)
        return np.array([Z,-X,-Y]) # read book section on coordinate systems to understand this

    """
    def precompute_grid(self,dist=60):
        cut_v = int(self.compute_minimum_v(dist=dist)+1)
        xy = []
        for v in range(cut_v, self.image_height):
            for u in range(self.image_width):
                X,Y,Z= self.uv_to_roadXYZ_roadframe_iso8855(u,v)
                xy.append(np.array([X,Y]))
        xy = np.array(xy)
        return cut_v, xy
    """

    def compute_minimum_v(self, dist):
        """
        Find cut_v such that pixels with v < cut_v are irrelevant for polynomial fitting.
        Everything that is further than `dist` along the road is considered irrelevant.
        """        
        trafo_world_to_cam = np.linalg.inv(self.trafo_cam_to_world)
        point_far_away_on_world = trafo_world_to_cam @ np.array([0,0,dist,1])
        uv_vec = self.intrinsic_matrix @ point_far_away_on_world[:3]
        uv_vec /= uv_vec[2]
        cut_v = uv_vec[1]
        return cut_v

    

    """
    def create_homography_camera_birdseye(self):
        roll = np.deg2rad(0)
        pitch = np.deg2rad(0)
        yaw = np.deg2rad(0)

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
        R_bird = R_yaw @ R_pitch @ R_roll

        H1 = self.intrinsic_matrix @ self.rotation_cam_to_world
        H2 = self.intrinsic_matrix @ R_bird
        #print(self.rotation_cam_to_world)

        H_1to2 = H2 @ np.linalg.inv(H1)

        
        H_1to2[0:3, 2] = np.array([0, 0, 0])
        print(H_1to2)
        return H_1to2
    """

    

###############################################################################

def get_intrinsic_matrix_func(field_of_view_deg, image_width, image_height):
    # For our Carla camera alpha_u = alpha_v = alpha
    # alpha can be computed given the cameras field of view via
    field_of_view_rad = field_of_view_deg * np.pi/180
    alpha = (image_width / 2.0) / np.tan(field_of_view_rad / 2.)
    Cu = image_width / 2.0
    Cv = image_height / 2.0
    return np.array([[alpha, 0, Cu],
                     [0, alpha, Cv],
                     [0, 0, 1.0]])

class CameraGeometry2(object):
    def __init__(self, height=1, yaw_deg=-90, pitch_deg=0, roll_deg=-90, image_width=800, image_height=800, field_of_view_deg=90):
        # scalar constants
        self.height = height
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.yaw_deg = yaw_deg
        self.image_width = image_width
        self.image_height = image_height
        self.field_of_view_deg = field_of_view_deg
        # camera intriniscs and extrinsics
        self.intrinsic_matrix = get_intrinsic_matrix()
        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)
        ## Note that "rotation_cam_to_road" has the math symbol R_{rc} in the book
        yaw = np.deg2rad(yaw_deg)
        pitch = np.deg2rad(pitch_deg)
        roll = np.deg2rad(roll_deg)
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        rotation_road_to_cam = np.array([[cr*cy+sp*sr+sy, cr*sp*sy-cy*sr, -cp*sy],
                                            [cp*sr, cp*cr, sp],
                                            [cr*sy-cy*sp*sr, -cr*cy*sp -sr*sy, cp*cy]])
        self.rotation_cam_to_road = rotation_road_to_cam.T # for rotation matrices, taking the transpose is the same as inversion
        self.translation_cam_to_road = np.array([0,-self.height,0])
        self.trafo_cam_to_road = np.eye(4)
        self.trafo_cam_to_road[0:3,0:3] = self.rotation_cam_to_road
        self.trafo_cam_to_road[0:3,3] = self.translation_cam_to_road
        # compute vector nc. Note that R_{rc}^T = R_{cr}
        self.road_normal_camframe = self.rotation_cam_to_road.T @ np.array([0,1,0])

    def create_homography_camera_birdseye(self):
        roll = np.deg2rad(10)
        pitch = np.deg2rad(10)
        yaw = np.deg2rad(0)

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
        R_bird = R_yaw @ R_pitch @ R_roll

        H1 = self.intrinsic_matrix @ self.rotation_cam_to_road
        H2 = self.intrinsic_matrix @ R_bird

        H_1to2 = H2 @ np.linalg.inv(H1)
        print(self.rotation_cam_to_road)

        return H_1to2

    def camframe_to_roadframe(self,vec_in_cam_frame):
        return self.rotation_cam_to_road @ vec_in_cam_frame + self.translation_cam_to_road

    def uv_to_roadXYZ_camframe(self,u,v):
        # NOTE: The results depend very much on the pitch angle (0.5 degree error yields bad result)
        # Here is a paper on vehicle pitch estimation:
        # https://refubium.fu-berlin.de/handle/fub188/26792
        uv_hom = np.array([u,v,1])
        Kinv_uv_hom = self.inverse_intrinsic_matrix @ uv_hom
        denominator = self.road_normal_camframe.dot(Kinv_uv_hom)
        return self.height*Kinv_uv_hom/denominator
    
    def uv_to_roadXYZ_roadframe(self,u,v):
        r_camframe = self.uv_to_roadXYZ_camframe(u,v)
        return self.camframe_to_roadframe(r_camframe)

    def uv_to_roadXYZ_roadframe_iso8855(self,u,v):
        X,Y,Z = self.uv_to_roadXYZ_roadframe(u,v)
        return np.array([Z,-X,-Y]) # read book section on coordinate systems to understand this

    def precompute_grid(self,dist=60):
        cut_v = int(self.compute_minimum_v(dist=dist)+1)
        xy = []
        for v in range(cut_v, self.image_height):
            for u in range(self.image_width):
                X,Y,Z= self.uv_to_roadXYZ_roadframe_iso8855(u,v)
                xy.append(np.array([X,Y]))
        xy = np.array(xy)
        return cut_v, xy

    def compute_minimum_v(self, dist):
        """
        Find cut_v such that pixels with v<cut_v are irrelevant for polynomial fitting.
        Everything that is further than `dist` along the road is considered irrelevant.
        """        
        trafo_road_to_cam = np.linalg.inv(self.trafo_cam_to_road)
        point_far_away_on_road = trafo_road_to_cam @ np.array([0,0,dist,1])
        uv_vec = self.intrinsic_matrix @ point_far_away_on_road[:3]
        uv_vec /= uv_vec[2]
        cut_v = uv_vec[1]
        return cut_v