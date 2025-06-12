import numpy as np

class Camera:
    def __init__(self, K_matrix=None, image_height=800, image_width=800):
        self.image_height = image_height
        self.image_width = image_width
        
        if K_matrix is None:
            raise ValueError("K_matrix must be provided")
        self.K_matrix = K_matrix 

class VehicleGeometry:
    def __init__(self, cam_height=1, len_vehicle_shadow=2.24, len_vehicle_front=1.0):
        self.cam_height = cam_height
        self.len_vehicle_shadow = len_vehicle_shadow
        self.len_vehicle_front = len_vehicle_front
        self.shadow_point = np.array([0, cam_height, len_vehicle_shadow+len_vehicle_front]) # (Xc, Yc, Zc).T


class VisionCalculation:
    def __init__(self, camera_object=None, vehicle_object=None, rotation_cam_to_world=None, translation_cam_to_world=None):
        if not isinstance(camera_object, Camera):
            raise TypeError("camera_object must be an instance of Camera")
        if not isinstance(vehicle_object, VehicleGeometry):
            raise TypeError("vehicle_object must be an instance of VehicleGeometry")
        if rotation_cam_to_world is None:
            raise ValueError("rotation_cam_to_world must be provided")

        if translation_cam_to_world is None:
            raise ValueError("translation_cam_to_world must be provided")

        self.camera = camera_object
        self.cam_intrinsic_matrix = camera_object.K_matrix
        self.inverse_cam_intrinsic_matrix = np.linalg.inv(self.cam_intrinsic_matrix)

        self.vehicle = vehicle_object
        
        self.rotation_cam_to_world = rotation_cam_to_world 
        self.translation_cam_to_world = translation_cam_to_world 

        # compute vector nc
        self.world_normal_camframe = self.rotation_cam_to_world.T @ np.array([0, 0, -1])

        self.cut_v, self.grid_coordinates = self.precompute_grid(dist=2.5)

        self.grid_coordinates_old = self.precompute_grid_old()


    def camframe_to_worldframe(self, vec_in_cam_frame):
        return self.rotation_cam_to_world @ vec_in_cam_frame + self.translation_cam_to_world

    def uv_to_XYZ_camframe(self,u,v):
        uv_hom = np.array([u,v,1])
        Kinv_uv_hom = self.inverse_cam_intrinsic_matrix @ uv_hom
        denominator = self.world_normal_camframe.dot(Kinv_uv_hom)
        return self.vehicle.cam_height*Kinv_uv_hom/denominator
    
    def uv_to_XYZ_worldframe(self,u,v):
        r_camframe = self.uv_to_XYZ_camframe(u,v)
        return self.camframe_to_worldframe(r_camframe)

    def precompute_grid(self,dist=1.5):
        cut_v = int(self.compute_minimum_v(dist=dist)+1)
        xy = []
        print("cut_v: ", cut_v)
        for v in range(cut_v, self.camera.image_height):
            for u in range(self.camera.image_width):
                X,Y,Z= self.uv_to_XYZ_worldframe(u,v)
                xy.append(np.array([X,Y]))
        xy = np.array(xy)

        print(xy.shape)
        return cut_v, xy

    def precompute_grid_old(self):
        min_pixel = int(self.camera.image_height/2)+1

        coordinates_grid = np.full((self.camera.image_height, self.camera.image_width, 2), -1, dtype=float)
        for v in range(min_pixel, self.camera.image_height):
            for u in range(self.camera.image_width):
                X,Y,Z= self.uv_to_XYZ_worldframe(u,v)
                coordinates_grid[v,u] = np.array([X,Y])
        print(coordinates_grid.shape)
        return coordinates_grid

    def compute_minimum_v(self, dist):
        """
        Find cut_v such that pixels with v<cut_v are irrelevant for polynomial fitting.
        Everything that is further than `dist` along the road is considered irrelevant.
        """        
        point_far_away_on_road = self.rotation_cam_to_world.T @ (np.array([dist,0,0])-self.translation_cam_to_world)
        uv_vec = self.cam_intrinsic_matrix @ point_far_away_on_road
        uv_vec /= uv_vec[2]
        cut_v = uv_vec[1]
        return cut_v

    @staticmethod
    def create_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
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

    def test_camera_geometry(self, test_vec_camframe, test_vec_worldframe):
        rotation_world_to_cam = np.linalg.inv(self.rotation_cam_to_world)
        # Test the camera geometry
        print("Camera Geometry:")
        print("Intrinsic Matrix:")
        print(self.cam_intrinsic_matrix)
        print("Rotation Matrix (Camera to World):")
        print(self.rotation_cam_to_world)

        print("\n### Test the transformation from camera frame to world frame ###")
        print("vec_camframe:")
        vec_camframe = test_vec_camframe
        print(vec_camframe)

        print("vec_worldframe:")
        vec_worldframe = self.rotation_cam_to_world @ vec_camframe + self.translation_cam_to_world
        print(vec_worldframe)

        print("vec_camframe:")
        vec_camframe = rotation_world_to_cam @ (vec_worldframe - self.translation_cam_to_world)
        print(vec_camframe)

        print("\n### Test the transformation from world frame to cam frame ###")
        print("vec_worldframe:")
        vec_worldframe = test_vec_worldframe
        print(vec_worldframe)

        print("vec_camframe:")
        vec_camframe = rotation_world_to_cam @ (vec_worldframe - self.translation_cam_to_world)
        print(vec_camframe)

        print("vec_worldframe:")
        vec_worldframe = self.rotation_cam_to_world @ vec_camframe + self.translation_cam_to_world  
        print(vec_worldframe)