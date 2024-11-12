import rclpy
from rclpy.node import Node
import math


from tkinter import *
from px4_msgs.msg import VehicleStatus,VehicleOdometry, BatteryStatus
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


dic_nav_state={0:"Manuel",
                1:"Altitude",
                2:"Position",
                3:"auto mission",
                4:"auto loiter",
                5:"autoreturn to launch",
                6:"POSITION_SLOW",
                10:"Acro",
                12:"Descend",
                13:"Termination",
                14:"OffBoard",
                15:"Stabilized",
                17:"TakeOff",
                18:"Land",
                19:"Auto Follow",
                20:"Precision Land",
                21:"Orbit"
                }
dic_arm_state={0:"Inconnu",
                1:"Disarmed",
                2:"Armed"}



def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


class supervision(Node):
    def __init__(self):
        super().__init__("supervision")
        self.get_logger().info("demarrage supervision")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_vehicle_status=self.create_subscription(
            VehicleStatus,"/fmu/out/vehicle_status",self.cb_vehicle_status,qos_profile)
        self.sub_batt=self.create_subscription(BatteryStatus,"/fmu/out/battery_status",self.cb_batt,qos_profile)
        self.sub_vehicle_odometry=self.create_subscription(
            VehicleOdometry,"/fmu/out/vehicle_odometry",self.cb_vehicle_odometry,qos_profile)

        # self.sub_Joy=self.create_subscription(Joy,"/joy",self.cb_joy,1)
        
        self.upd=self.create_timer(0.2, self.update)

        self.vehicle_status=VehicleStatus()
        self.battery_status=BatteryStatus()
        self.vehicle_odometry=VehicleOdometry()
        # self.vehicule_status_date=now()

        self.F=Tk()
        self.F.option_add("*font","Terminal 12")
        self.F.title("Supervision")
        self.F.geometry('+10+10')
        self.T_arm_state=Label(self.F,text="arm_state : ??              ")
        self.T_arm_state.grid(row=0,column=0,sticky='W')
        self.T_nav_state=Label(self.F,text="nav_state : ??              ")
        self.T_nav_state.grid(row=1,column=0,sticky='W')
        
        self.T_odometry=Label(self.F,text="odometry: \nx:???.?? y:???.?? z:???.?? \n vx:?.?? vy:?.?? vz:?.??             ")
        self.T_odometry.grid(row=2,column=0,sticky='NW')
        
        # lance le premier affichage
        self.F.update_idletasks()
        self.F.update()


    def update(self):
        # self.get_logger().info("update")
        #NAV_STATUS
        if self.vehicle_status.nav_state in dic_nav_state:
            self.T_nav_state.config(text="nav_state : "+dic_nav_state[self.vehicle_status.nav_state])
        else:
            self.get_logger().info("nav_state : INCONNU : "+str(self.vehicle_status.nav_state))
        #ARM_STATE
        self.T_arm_state.config(text="arm_state : "+dic_arm_state[self.vehicle_status.arming_state])
        #ODOMETRY
        roll, pitch, yaw=euler_from_quaternion(self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3])
        self.T_odometry.config(text="odometry: heading:"+f'{yaw:+7.2f}'+"     "+
                    "\nx:"+f'{self.vehicle_odometry.position[0]:+7.2f}'+
                    " y:"+f'{self.vehicle_odometry.position[1]:+7.2f}'+
                    " z:"+f'{self.vehicle_odometry.position[2]:+7.2f}'+" \n"+
                    "vx: "+f'{self.vehicle_odometry.velocity[0]:+5.2f}'+
                    " vy: "+f'{self.vehicle_odometry.velocity[1]:+5.2f}'+
                    " vz: "+f'{self.vehicle_odometry.velocity[2]:+5.2f}')
        # Battery

        self.F.update_idletasks()
        self.F.update()
        

    def cb_vehicle_status(self,msg):
        # self.get_logger().info("sub_vehicle_status")
        self.vehicle_status=msg    
    def cb_vehicle_odometry(self,msg):
        # self.get_logger().info("sub_vehicle_status")
        self.vehicle_odometry=msg
        
    def cb_batt(self,msg):
        # self.get_logger().info("sub_batt")
        self.battery_status=msg

    # def cb_joy(self,msg):
    #     self.get_logger().info("sub_joy")





def main(args=None):
    rclpy.init(args=args)
    node=supervision()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()