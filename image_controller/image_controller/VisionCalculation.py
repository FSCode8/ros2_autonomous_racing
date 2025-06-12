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
        self.world_normal_camframe = np.linalg.inv(self.rotation_cam_to_world) @ np.array([0, 0, -1])

        self.grid_coordinates = self.precompute_grid(pitch_angle=0)

        self.bev_homography = self.get_bev_homography(  # bird's-eye view homography
            K_orig=self.cam_intrinsic_matrix,
            R_cw=self.rotation_cam_to_world,
            t_cam_in_world=self.translation_cam_to_world,
            world_roi_x_m=(-1, 1),  
            world_roi_y_m=(0, 2),  
            bev_img_size_px=(self.camera.image_height, self.camera.image_height),  # Example BEV image size in pixels
            ground_plane_world_z=0.0,  # Assuming ground plane at Z=0
            invert_bev_y_axis=False)


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

    def precompute_grid(self, pitch_angle):
        if pitch_angle == 0:
            min_pixel = int(self.camera.image_height/2) + 1 
        else:
            return ValueError("Logic for pitch_angle unequal to zero has to be implemented in the future.")
        
        coordinates_grid = np.full((self.camera.image_height, self.camera.image_width, 2), -1, dtype=float) # 

        for v in range(min_pixel, self.camera.image_height):
            for u in range(self.camera.image_width):
                X,Y,Z= self.uv_to_XYZ_worldframe(u,v)
                coordinates_grid[v, u] = np.array([X,Y])
        return coordinates_grid

    def compute_dist(self, u, v, alpha=0):
        """
        Compute the distance from the camera to the point on the road.
        The distance is computed as the length of the vector from the vehicle front (ISO8855 [1, 0, 0]) to the point on the road.
        """
        vec_camframe = self.uv_to_XYZ_camframe(u,v)
        print("vec_camframe:")
        print(vec_camframe)
        len_vehicle_shadow = self.vehicle.len_vehicle_shadow
        len_vehicle_front = self.vehicle.len_vehicle_front
        dist_hypo = np.linalg.norm(vec_camframe)
        
        if -22.5 <= alpha <= 22.5: 
            dist = vec_camframe[2] -1
        elif -90 <= alpha <= 90:
            dist = np.sqrt(vec_camframe[0]**2 + vac_camframe[1]**2)
        else:
            raise ValueError("alpha must be in the range [-90, 90]")
        #dist = np.sqrt(dist_hypo**2 - self.vehicle.cam_height**2) - len_vehicle_front - len_vehicle_shadow
        
        return dist

    def get_min_carless_pixel(self):
        """
        Get the minimum pixel (u,v) for which the car is not shown in the image.
        """
        u, v, _ = self.cam_intrinsic_matrix @ self.vehicle.shadow_point / self.vehicle.shadow_point[2] # (u, v , 1).T = 1/Zc * K * (Xc, Yc, Zc).T

        return u, (v-1)

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