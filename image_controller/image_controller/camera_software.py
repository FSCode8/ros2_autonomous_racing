import numpy as np

# relative position to base_link: xyz(1, 0, 1)
# roll pitch yaw (radial): rpy(-1.5707963267948966, 0, -1.5707963267948966)

# homography matrix: K(R|t)

def get_intrinsic_matrix():
    alpha = 476.7014503479004
    Cu = 400.0
    Cv = 0.0
    return np.array([[alpha, 0, Cu],
                     [0, alpha, Cv],
                     [0, 0, 1.0]])

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

def project_polyline(polyline_world, trafo_world_to_cam, K):
    x,y,z = polyline_world[:,0], polyline_world[:,1], polyline_world[:,2]
    homvec = np.stack((x,y,z,np.ones_like(x)))
    proj_mat = K @ trafo_world_to_cam[:3,:]
    pl_uv_cam = (proj_mat @ homvec).T
    u = pl_uv_cam[:,0] / pl_uv_cam[:,2]
    v = pl_uv_cam[:,1] / pl_uv_cam[:,2]
    return np.stack((u,v)).T

# def __init__(self, height=1.3, yaw_deg=0, pitch_deg=-5, roll_deg=0, image_width=800, image_height=800, field_of_view_deg=45):

class CameraGeometry(object):
    def __init__(self, height=1, roll_deg=0, pitch_deg=0, yaw_deg=0):
        # scalar constants
        self.height = height
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.yaw_deg = yaw_deg
        #self.image_width = image_width
        #self.image_height = image_height
        #self.field_of_view_deg = field_of_view_deg
        
        # camera intriniscs and extrinsics
        self.intrinsic_matrix = get_intrinsic_matrix()
        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)

        ## Note that "rotation_cam_to_road" has the math symbol R_{rc} in the book
        roll = np.deg2rad(roll_deg)
        pitch = np.deg2rad(pitch_deg)
        yaw = np.deg2rad(yaw_deg)

        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        # Create rotation matrices for standard axes (x: up, y: left, z: forward)
        R_roll = np.array([[1, 0, 0],
                            [0, cr, -sr],
                            [0, sr, cr]])
        R_pitch = np.array([[cp, 0, sp],
                            [0, 1, 0],
                            [-sp, 0, cp]])
        R_yaw = np.array([[cy, -sy, 0],
                            [sy, cy, 0],
                            [0, 0, 1]]) 
        # Combine rotations
        rotation_road_to_cam = R_yaw @ R_pitch @ R_roll
        self.rotation_cam_to_road = rotation_road_to_cam.T # for rotation matrices, taking the transpose is the same as inversion

        self.translation_cam_to_road = np.array([0,self.height,0])
        self.trafo_cam_to_road = np.eye(4)
        self.trafo_cam_to_road[0:3,0:3] = self.rotation_cam_to_road
        self.trafo_cam_to_road[0:3,3] = self.translation_cam_to_road
        # compute vector nc. Note that R_{rc}^T = R_{cr}
        self.road_normal_camframe = self.rotation_cam_to_road.T @ np.array([0,1,0])


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
        Find cut_v such that pixels with v<cut_v are irrelevant for polynomial fitting.
        Everything that is further than `dist` along the road is considered irrelevant.
        """        
        trafo_road_to_cam = np.linalg.inv(self.trafo_cam_to_road)
        point_far_away_on_road = trafo_road_to_cam @ np.array([0,0,dist,1])
        uv_vec = self.intrinsic_matrix @ point_far_away_on_road[:3]
        uv_vec /= uv_vec[2]
        cut_v = uv_vec[1]
        return cut_v

    def create_homography(self):
        # Reshape translation if needed
        t = self.translation_cam_to_road.reshape(3, 1)
        
        # Create [R|t] matrix
        Rt = np.hstack((self.rotation_cam_to_road, t))
        
        # Create homography as K[R|t]
        H = self.intrinsic_matrix @ Rt
        
        # For planar homography, normalize
        H = H[:3, :3]  # Take first 3 columns
        #H = H / H[2, 2]  # Normalize
        
        return H

    def create_homography_for_iso8855(self):
        """
        Create a homography matrix to transform from camera coordinates to ISO 8855 directly
        """
        # First, define the rotation from road to ISO 8855 based on your conversion
        # Road [X,Y,Z] → ISO [Z,-X,-Y]
        road_to_iso8855 = np.array([
            [0, 0, 1],  # ISO X = Road Z
            [-1, 0, 0], # ISO Y = -Road X
            [0, -1, 0]  # ISO Z = -Road Y
        ])
        
        # Combined rotation from camera to ISO 8855
        rotation_cam_to_iso8855 = road_to_iso8855 @ self.rotation_cam_to_road
        
        # Ensure translation is a column vector
        t_cam = self.translation_cam_to_road.reshape(3, 1)
        
        # Transform the translation to ISO 8855
        t_iso8855 = road_to_iso8855 @ t_cam
        
        # Create [R|t] matrix
        Rt = np.hstack((rotation_cam_to_iso8855, t_iso8855))
        
        # Create homography as K[R|t]
        H = self.intrinsic_matrix @ Rt
        
        # For planar homography, take first 3 columns and normalize
        H = H[:, :3]
        H = H / H[2, 2]
        
        return H