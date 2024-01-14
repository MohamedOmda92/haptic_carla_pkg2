# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import sys
sys.path.append('/home/mohamed/CARLA_0.9.11/PythonAPI/carla') # TO BE ELIMINATED
import os
import carla
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Int16, String
from .submodules.carla_client import CarlaClient

pkg_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), os.pardir))
dataset_dir = os.path.join(pkg_dir, 'dataset/images')



# ==============================================================================
# -- Global Parameters ---------------------------------------------------------
# ==============================================================================

ros_mode = True
autopilot_mode = True
dataset_size = 100


# ==============================================================================
# -- Global Functions ----------------------------------------------------------
# ==============================================================================

class HapticController(Node):
    
    def __init__(self):
        super().__init__('haptice_controller_node')
        self.steerSub = self.create_subscription(Int16, '/steer_cmd', self.steerCb, 10)        
        self.modeSub = self.create_subscription(String, '/control_mode', self.modeCb, 10)
    


    def steerCb(msg):
        """
        Apply steering control to ego-vehicle based on ROS messages"""
        global env
        try:
            angle = (msg.data / 511.5) - 1
            angle = angle / 5
            env.ego_vehicle.apply_control(carla.VehicleControl(throttle=0.2, steer=angle))
        except:
            pass



    def modeCb(msg) -> None:
        """
        Switch ego-vehicle control mode between CARLA BasicAgent and User Manual Contorl"""
        global autopilot_mode
        
        try: 
            if msg.data == 'auto':
                print("\n>>> Autopilot Mode")
                autopilot_mode = True
            
            elif msg.data == 'manual':
                print("\n>>> Manual Mode")
                autopilot_mode = False

        except AttributeError:
            pass
        




# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main(args=None):

    global env

    try:
        rclpy.init(args=args)

        haptic_controller = HapticController()

        env = CarlaClient()
        env.reset()
        
        while rclpy.ok():
            if autopilot_mode:
                Agent_ctrl = env.agent.run_step()
                env.ego_vehicle.apply_control(Agent_ctrl)        
            else:
                man_ctrl = carla.VehicleControl(throttle=0, steer=0, brake=1.0, hand_brake=True)
                env.ego_vehicle.apply_control(man_ctrl)

        rclpy.spin(haptic_controller)

    except KeyboardInterrupt:
        if env != None:   
            env.destroy_actors()
        print('\n>>> Cancelled by user. Bye!\n')

    finally:
        if env != None:
            env.destroy_actors()
        print('\n>>> Cancelled by user. Bye!\n')



if __name__ == "__main__":
    main()