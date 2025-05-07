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
        self.shadow_point = np.array([0, 1, 3.22]) # (Xc, Yc, Zc).T


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

    def precompute_grid(self,dist=60):
        cut_v = int(self.compute_minimum_v(dist=dist)+1)
        xy = []
        for v in range(cut_v, self.camera.image_height):
            for u in range(self.camera.image_width):
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

    def compute_dist(self, u, v):
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

        dist = np.sqrt(dist_hypo**2 - self.vehicle.cam_height**2) - len_vehicle_front - len_vehicle_shadow
        
        return dist

    def get_min_carless_pixel(self):
        """
        Get the minimum pixel (u,v) for which the car is not shown in the image.
        """
        u, v, _ = self.cam_intrinsic_matrix @ self.vehicle.shadow_point / self.vehicle.shadow_point[2] # (u, v , 1).T = 1/Zc * K * (Xc, Yc, Zc).T

        return u, (v-1)

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