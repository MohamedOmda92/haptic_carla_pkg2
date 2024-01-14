# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import os
import sys
import csv
import signal
import random
import carla
import cv2
import math
import numpy as np
from agents.navigation.basic_agent import BasicAgent

pkg_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), os.pardir, os.pardir))
img_dir = os.path.join(pkg_dir, 'dataset/images')
lbl_dir = os.path.join(pkg_dir, 'dataset')
lbl_filename = 'labels.csv'



# ==============================================================================
# -- CarlaClient Class ---------------------------------------------------------
# ==============================================================================

class CarlaClient():

    show_preview = False
    form_dataset = False
    im_height = 480
    im_width = 640

    def __init__(self, min_map=False):
        # Parameters
        self.img_num = 0
        self.steering_hist = []
        self.img_hist = []

        # Connect to Carla server        
        print("\nConneting To Simulator ...\n")
        signal.signal(signal.SIGINT, self._signal_handler)
        i = 1
        while True:
            try:
                client = carla.Client('localhost', 2000)
                client.set_timeout(5.0)
                self.world = client.get_world()
                print("\n" + u'\u2713'*3 + " Connected Successfully")
                break
            except RuntimeError:
                print(u'\u2715' + f" Connection failed, trying again: {i}")
                i += 1

        # Set World's Map (Town / Track)
        if min_map:
            current_town = self.world.get_map().name            
            # Sets Minimal-version Town
            if not "_Opt" in current_town:
                print(f"\nLoading minimum version of current map ({current_town})")
                print("This may take few seconds ...")
                client.set_timeout(60.0)
                self.world = client.load_world(current_town+"_Opt")
                self.world.unload_map_layer(carla.MapLayer.Buildings) 
                self.world.unload_map_layer(carla.MapLayer.Foliage)
                self.world.unload_map_layer(carla.MapLayer.StreetLights)   
                print("\n" + u'\u2713'*3 + " Loaded Successfully!")          

        # Set World's Settings 
        settings = self.world.get_settings()
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

        # Set World's Weather
        self.world.set_weather(carla.WeatherParameters.ClearNoon)

        # Actos List
        self.actor_list = []
        blueprint_library = self.world.get_blueprint_library()

        # Ego Vehicle
        self.ego_bp = blueprint_library.filter("model3")[0]
        self.ego_trans = random.choice(self.world.get_map().get_spawn_points())

        # RGB Camera Sensor
        img_height = CarlaClient.im_height
        img_width = CarlaClient.im_width
        self.cam_img = np.zeros((img_height, img_width, 3), dtype=np.uint8)
        self.cam_bp = blueprint_library.find('sensor.camera.rgb')
        self.cam_trans = carla.Transform() # will be changed after ego spawn
        # Configure RGB Camera Attributes
        self.cam_bp.set_attribute("image_size_x", f"{img_width}")
        self.cam_bp.set_attribute("image_size_y", f"{img_height}")
        self.cam_bp.set_attribute("fov", "110")
        self.cam_bp.set_attribute('sensor_tick', '0.1')

        # Collision Sensor  
        self.collision_hist = []
        self.collision_hist_l = 1 # collision histroy length
        self.collision_bp = blueprint_library.find('sensor.other.collision')
        self.collision_trans = carla.Transform(carla.Location(x=2.5, z=0.7))

        # Lane Invasion
        self.laneInv_hist = []
        self.laneInv_bp = blueprint_library.find('sensor.other.lane_invasion')
        self.laneInv_trans = carla.Transform(carla.Location(x=2.5, z=0.7))

    

    def reset(self):
        """
        This method intialize world for new epoch"""
        # Clear Actor Objects  
        self.ego_vehicle = None
        self.cam_sensor = None
        self.collision_sensor = None
        self.laneInv_sensor = None

        # Spawn Ego Vehicle 
        self.ego_vehicle = self.world.spawn_actor(self.ego_bp, self.ego_trans)
        self.ego_vehicle.role_name = "ego_vehicle"
        self.actor_list.append(self.ego_vehicle)
        # Turn Ego-vehicle into Basic Agent
        self.agent = BasicAgent(self.ego_vehicle)
        self.agent.ignore_traffic_lights(active=True)

        # Spawn RGB Camera        
        self.cam_trans = carla.Transform(carla.Location(z=2))          
        self.cam_sensor = self.world.spawn_actor(self.cam_bp, self.cam_trans, attach_to=self.ego_vehicle)
        self.cam_sensor.listen(lambda data: get_cam_img(data))
        self.actor_list.append(self.cam_sensor)
        
        # RGB Camera Callback
        def get_cam_img(data):
            """
            Process RGB camera data"""        
            i = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
            i2 = np.reshape(i, (data.height, data.width, 4))
            i3 = i2[:, :, :3]
            # i3 = i3[:, :, ::-1] # reverse sequence
            self.cam_img = i3
            if CarlaClient.form_dataset == True:
                self._get_images()
                self._get_steers()
            if CarlaClient.show_preview:
                cv2.imshow("", self.cam_img)
                cv2.waitKey(1)

        # Spawn Collision Sensor        
        self.col_sensor = self.world.spawn_actor(self.collision_bp, self.collision_trans, attach_to=self.ego_vehicle)
        self.col_sensor.listen(lambda event: get_col_hist(event))
        self.actor_list.append(self.col_sensor)
        
        # Collision Sensor
        def get_col_hist(event):
            """
            Add impulse to collision history"""
            impulseX = event.normal_impulse.x
            impulseY = event.normal_impulse.y
            impulseZ = event.normal_impulse.z
            intensity = math.sqrt((impulseX**2) + (impulseY**2) + (impulseZ**2))
            self.collision_hist.append(intensity)

        # Spawn Lane Invasion Detector
        self.laneInv_sensor = self.world.spawn_actor(self.laneInv_bp, self.laneInv_trans, attach_to=self.ego_vehicle)
        self.actor_list.append(self.laneInv_sensor)
        self.laneInv_sensor.listen(lambda event: get_lane_hist(event))
        
        # Lane Invasion Detector Callback
        def get_lane_hist(event):
            """
            Add timestamp of lane invasion event"""
            self.laneInv_hist.append(event.timestamp)

        # Set Spectator Navigation (Location)
        self._set_spectator()
        

        
    def destroy_actors(self) -> None:
        """
        Destroy all actors in the actor_list"""
        for actor in self.actor_list:
            actor.destroy()
        print("\nAll Actors Destroyed")



    @staticmethod
    def _signal_handler(signal, frame):
        """
        Handles interrupt from Keyboard (Ctrl + C)"""
        sys.exit()



    def _set_spectator(self, transform=None) -> None:
        """
        Focuses world spectator at ego vehilce"""
        spect = self.world.get_spectator()
        if transform == None:
            transform = self.ego_vehicle.get_transform()
            transform.location.z += 3 # ABOVE vehicle
            transform.rotation.pitch -= 30 # LOOK down
        spect.set_transform(transform)



    def _get_images(self):
        """
        Write RGB camera images to png files"""
        filename = os.path.join(img_dir, str(self.img_num)+'.jpg')
        cv2.imwrite(filename,self.cam_img)
        self.img_hist.append(str(self.img_num) + '.jpg')
        self.img_num += 1


    
    def _get_steers(self):
        """
        Store steering angles"""
        steer = self.ego_vehicle.get_control().steer
        steerDeg = steer * 540 # converted to deg
        self.steering_hist.append("%.5f" % steerDeg)


    def generate_dataset_labels(self):
        """
        Create CSV file of dataset labels (Steering Angles in Deg)"""
        self.img_hist = np.array(self.img_hist)
        self.steering_hist = np.array(self.steering_hist)

        data = np.stack((self.img_hist, self.steering_hist), axis=1)

        filepath = os.path.join(lbl_dir, lbl_filename)
        with open(filepath, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(data)



# ==============================================================================
# -- Main ----------------------------------------------------------------------
# ==============================================================================
          
if __name__ == "__main__":
    client = CarlaClient()
