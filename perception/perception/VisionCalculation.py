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

    def compute_dist(self, u, v, alpha=0):
        """
        Compute the distance from the camera to the point on the road.
        The distance is computed as the length of the vector from the vehicle front (ISO8855 [1, 0, 0]) to the point on the road.
        """
        vec_camframe = self.uv_to_XYZ_camframe(u,v)

        len_vehicle_shadow = self.vehicle.len_vehicle_shadow
        len_vehicle_front = self.vehicle.len_vehicle_front
        dist_hypo = np.linalg.norm(vec_camframe)
        
        if -22.5 <= alpha <= 22.5: 
            dist = vec_camframe[2] -1
        elif -90 <= alpha <= 90:
            dist = np.sqrt(vec_camframe[0]**2 + vec_camframe[1]**2)
        else:
            raise ValueError("alpha must be in the range [-90, 90]")
        
        return dist

    def get_min_carless_pixel(self):
        """
        Get the minimum pixel (u,v) for which the car is not shown in the image.
        """
        u, v, _ = self.cam_intrinsic_matrix @ self.vehicle.shadow_point / self.vehicle.shadow_point[2] # (u, v , 1).T = 1/Zc * K * (Xc, Yc, Zc).T

        return u, (v-1)

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

    def get_bev_homography(self, K_orig, R_cw, t_cam_in_world,
                       world_roi_x_m, world_roi_y_m, bev_img_size_px,
                       ground_plane_world_z=0.0,
                       invert_bev_y_axis=False):
        """
        Calculates the homography to warp an image to a bird's-eye view (BEV).

        Args:
            K_orig (np.ndarray): 3x3 intrinsic matrix of the original camera.
            R_cw (np.ndarray): 3x3 rotation matrix from original camera frame to world frame.
            t_cam_in_world (np.ndarray): 3x1 or (3,) array for camera position in world coordinates [x,y,z]^T.
                                        z is typically the camera height above the world origin.
            world_roi_x_m (tuple): (min_x_world, max_x_world) meters for the BEV.
            world_roi_y_m (tuple): (min_y_world, max_y_world) meters for the BEV.
            bev_img_size_px (tuple): (width_bev_pixels, height_bev_pixels) for the output BEV image.
            ground_plane_world_z (float): Z-coordinate of the ground plane in world frame. Default 0.0.
            invert_bev_y_axis (bool): If True, increasing world Y maps to decreasing BEV image v
                                    (e.g., world Y North maps to image v going upwards).
                                    Default False (world Y North maps to image v going downwards).

        Returns:
            np.ndarray: 3x3 Homography matrix for cv2.warpPerspective.
                        Returns None if inputs are invalid (e.g., singular matrix).
        """
        # Homography from World Plane (Z_world=ground_plane_world_z) to Original Image (H_w2orig) ---
        R_wc = R_cw.T
        t_cam_in_world = np.asarray(t_cam_in_world).reshape(3,1)
        t_wc = -R_wc @ t_cam_in_world

        # Effective translation considering the plane's Z offset
        t_wc_eff_for_plane = R_wc[:, 2:3] * ground_plane_world_z + t_wc

        # H_w2orig maps [X_w, Y_w, 1]^T (on the Z_w=ground_plane_world_z plane) to original image pixels
        # The columns of R_wc are r_wc1, r_wc2, r_wc3
        # The matrix for plane projection: K_orig @ [r_wc1 | r_wc2 | r_wc3*Z_plane_offset + t_wc]
        H_w2orig_3x3 = K_orig @ np.hstack((R_wc[:, 1:2], R_wc[:, 0:1], t_wc_eff_for_plane))
        
        try:
            H_orig2w_3x3 = np.linalg.inv(H_w2orig_3x3)
        except np.linalg.LinAlgError:
            print("Error: H_w2orig matrix is singular. Cannot compute inverse.")
            return None

        # Homography from World Plane to BEV image (H_w2bev) ---
        W_bev_px, H_bev_px = bev_img_size_px
        X_world_min, X_world_max = world_roi_x_m
        Y_world_min, Y_world_max = world_roi_y_m

        if X_world_max == X_world_min or Y_world_max == Y_world_min:
            print("Error: World ROI dimensions cannot be zero.")
            return None

        scale_x = (W_bev_px -1e-6) / (X_world_max - X_world_min) # Add epsilon for W_bev_px=0 case safety
        offset_x = -scale_x * X_world_min

        if not invert_bev_y_axis:
            # Increasing Y_world maps to increasing v_bev (downwards in image)
            scale_y = (H_bev_px -1e-6) / (Y_world_max - Y_world_min)
            offset_y = -scale_y * Y_world_min
        else:
            # Increasing Y_world maps to decreasing v_bev (upwards in image)
            # Y_world_min maps to H_bev_px-1
            # Y_world_max maps to 0
            scale_y = (0 - (H_bev_px-1e-6)) / (Y_world_max - Y_world_min) # scale_y will be negative
            offset_y = -scale_y * Y_world_max # or (H_bev_px-1) - scale_y * Y_world_min

        H_w2bev = np.array([[scale_x,       0, offset_x],
                            [      0, scale_y, offset_y],
                            [      0,       0,        1]], dtype=np.float32)

        # H_final maps p_orig to p_bev
        H_final = H_w2bev @ H_orig2w_3x3
        
        return H_final

    @staticmethod
    def quaternion_to_rotation_matrix(x, y, z, w):
        """
        Convert a quaternion into a 3x3 rotation matrix.
        Quaternion should be in the form [x, y, z, w]
        """

        # Normalize the quaternion
        norm = np.linalg.norm(np.array([x, y, z, w]))
        if norm == 0:
            raise ValueError("Zero norm quaternion is invalid.")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

        # Compute the rotation matrix elements
        R = np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])

        return R

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