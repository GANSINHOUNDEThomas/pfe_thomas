



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import tkinter as tk
from tkinter import PhotoImage

B_CROIX = 0
B_ROND = 1
B_CARRE = 2
B_TRIANGLE = 3
B_SHARE = 4
B_PS = 5
B_OPT = 6
B_JL = 7
B_JR = 8
B_L1 = 9
B_R1 = 10
B_H = 11
B_B = 12
B_L = 13
B_R = 14
B_CLIC = 15

B_L3 = 7
B_R3 = 8

J_LH = 0  # horizontal axis of the left stick
J_LV = 1  # vertical axis of the left stick
J_L2 = 4  # left trigger
J_RH = 2  # horizontal axis of the right stick
J_RV = 3  # vertical axis of the right stick
J_R2 = 5  # right trigger

class JoyInterfaceNode(Node):
    def __init__(self, root):
        super().__init__('joy_interface_node')
        #self.get_logger().info('Joy Interface Node has been started.')

        # Initialize Joy message
        self.joy = Joy()
        self.joy.axes = [0.0] * 8
        self.joy.buttons = [0] * 16

        # Create subscriber for Joy messages
        self.subscriber = self.create_subscription(Joy, "joy", self.cb_joy, 1)

        # Create a timer to check button states
        self.timer = self.create_timer(0.05, self.timer_callback)  # Increased frequency

        # Tkinter setup
        self.root = root
        self.canvas = tk.Canvas(self.root, width=1000, height=600, bg="black")
        self.canvas.pack()

        # Top-left corner position
        self.corner_x = 0
        self.corner_y = 0

        # Scale variable
        self.scale = 0.25

        # Load and scale images
        self.resize_images()

        # Add green images to the canvas
        self.image_ids = {
            B_L1: self.create_image_on_canvas(self.l1_img, 200, 250),
            B_R1: self.create_image_on_canvas(self.r1_img, 800, 250),
            B_SHARE: self.create_image_on_canvas(self.share_img, 300, 300),
            B_OPT: self.create_image_on_canvas(self.option_img, 700, 300),
            B_H: self.create_image_on_canvas(self.up_img, 200, 400),
            B_L: self.create_image_on_canvas(self.left_img, 150, 450),
            B_R: self.create_image_on_canvas(self.right_img, 250, 450),
            B_B: self.create_image_on_canvas(self.down_img, 200, 500),
            B_CROIX: self.create_image_on_canvas(self.cross_img, 800, 520),
            B_TRIANGLE: self.create_image_on_canvas(self.triangle_img, 800, 370),
            B_ROND: self.create_image_on_canvas(self.round_img, 870, 450),
            B_CARRE: self.create_image_on_canvas(self.square_img, 730, 450),
            J_L2: self.create_image_on_canvas(self.l2_img, 200, 200),
            J_R2: self.create_image_on_canvas(self.r2_img, 800, 200),
            'L_sticks_zone': self.create_image_on_canvas(self.l_sticks_zone, 400, 500),
            'R_sticks_zone': self.create_image_on_canvas(self.r_sticks_zone, 600, 500),
            'L_stick': self.create_image_on_canvas(self.l_stick_img, 400, 500),
            'R_stick': self.create_image_on_canvas(self.r_stick_img, 600, 500),
        }

    def cb_joy(self, msg):
        """Callback function for Joy messages."""
        self.joy = msg
        #self.get_logger().info(f"Joy message received: {self.joy.buttons}")

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.highlight_buttons()

    def load_image(self, file_path):
        image = PhotoImage(file=f"src/px4-ros2-interface-lib/scripts/{file_path}")
        width = int(image.width() * self.scale)
        height = int(image.height() * self.scale)
        #self.get_logger().info(f"Loaded {file_path}: width={width}, height={height}")
        return image.subsample(int(1/self.scale)), width, height

    def resize_images(self):
        # for green images
        self.l1_img, self.l1_width, self.l1_height = self.load_image("green/L1.png")
        self.l2_img, self.l2_width, self.l2_height = self.load_image("green/L2.png")
        self.r1_img, self.r1_width, self.r1_height = self.load_image("green/R1.png")
        self.r2_img, self.r2_width, self.r2_height = self.load_image("green/R2.png")
        self.share_img, self.share_width, self.share_height = self.load_image("green/share.png")
        self.option_img, self.option_width, self.option_height = self.load_image("green/option.png")
        self.up_img, self.up_width, self.up_height = self.load_image("green/up.png")
        self.left_img, self.left_width, self.left_height = self.load_image("green/left.png")
        self.right_img, self.right_width, self.right_height = self.load_image("green/right.png")
        self.down_img, self.down_width, self.down_height = self.load_image("green/down.png")
        self.cross_img, self.cross_width, self.cross_height = self.load_image("green/cross.png")
        self.triangle_img, self.triangle_width, self.triangle_height = self.load_image("green/triangle.png")
        self.round_img, self.round_width, self.round_height = self.load_image("green/round.png")
        self.square_img, self.square_width, self.square_height = self.load_image("green/square.png")
        self.l_stick_img, self.l_stick_width, self.l_stick_height = self.load_image("green/L_sticks_level.png")
        self.r_stick_img, self.r_stick_width, self.r_stick_height = self.load_image("green/R_sticks_level.png")
        self.l_sticks_zone, self.l_sticks_zone_width, self.l_sticks_zone_height = self.load_image("green/L_sticks_zone.png")
        self.r_sticks_zone, self.r_sticks_zone_width, self.r_sticks_zone_height = self.load_image("green/R_sticks_zone.png")

        # for red images
        self.l1_img_red, self.l1_width_red, self.l1_height_red = self.load_image("red/L1.png")
        self.l2_img_red, self.l2_width_red, self.l2_height_red = self.load_image("red/L2.png")
        self.r1_img_red, self.r1_width_red, self.r1_height_red = self.load_image("red/R1.png")
        self.r2_img_red, self.r2_width_red, self.r2_height_red = self.load_image("red/R2.png")
        self.share_img_red, self.share_width_red, self.share_height_red = self.load_image("red/share.png")
        self.option_img_red, self.option_width_red, self.option_height_red = self.load_image("red/option.png")
        self.up_img_red, self.up_width_red, self.up_height_red = self.load_image("red/up.png")
        self.left_img_red, self.left_width_red, self.left_height_red = self.load_image("red/left.png")
        self.right_img_red, self.right_width_red, self.right_height_red = self.load_image("red/right.png")
        self.down_img_red, self.down_width_red, self.down_height_red = self.load_image("red/down.png")
        self.cross_img_red, self.cross_width_red, self.cross_height_red = self.load_image("red/cross.png")
        self.triangle_img_red, self.triangle_width_red, self.triangle_height_red = self.load_image("red/triangle.png")
        self.round_img_red, self.round_width_red, self.round_height_red = self.load_image("red/round.png")
        self.square_img_red, self.square_width_red, self.square_height_red = self.load_image("red/square.png")
        self.l_stick_img_red, self.l_stick_width_red, self.l_stick_height_red = self.load_image("red/L3.png")
        self.r_stick_img_red, self.r_stick_width_red, self.r_stick_height_red = self.load_image("red/R3.png")
        self.l_sticks_zone_red, self.l_sticks_zone_width_red, self.l_sticks_zone_height_red = self.load_image("red/L_sticks_zone.png")
        self.r_sticks_zone_red, self.r_sticks_zone_width_red, self.r_sticks_zone_height_red = self.load_image("red/R_sticks_zone.png")

    def create_image_on_canvas(self, image, x, y):
        return self.canvas.create_image(self.corner_x + x, self.corner_y + y, image=image, anchor=tk.CENTER)

    def highlight_buttons(self):
        button_images = {
            B_L1: (self.l1_img, self.l1_img_red),
            B_R1: (self.r1_img, self.r1_img_red),
            B_SHARE: (self.share_img, self.share_img_red),
            B_OPT: (self.option_img, self.option_img_red),
            B_H: (self.up_img, self.up_img_red),
            B_L: (self.left_img, self.left_img_red),
            B_R: (self.right_img, self.right_img_red),
            B_B: (self.down_img, self.down_img_red),
            B_CROIX: (self.cross_img, self.cross_img_red),
            B_TRIANGLE: (self.triangle_img, self.triangle_img_red),
            B_ROND: (self.round_img, self.round_img_red),
            B_CARRE: (self.square_img, self.square_img_red),
            
        } 

        for button, (green_img, red_img) in button_images.items():

            # if (not(button ==B_SHARE)):
                if self.joy.buttons[button] == 1:
                    self.canvas.itemconfig(self.image_ids[button], image=red_img)
                    #self.get_logger().info(f"Button {button} pressed, switching to red image.")
                else:
                    self.canvas.itemconfig(self.image_ids[button], image=green_img)
                    #self.get_logger().info(f"Button {button} released, switching to green image.")

        # Handle axes for L2 and R2
        if self.joy.axes[J_L2] < 1:
            self.canvas.itemconfig(self.image_ids[J_L2], image=self.l2_img_red)
            #self.get_logger().info("L2 pressed, switching to red image.")
        else:
            self.canvas.itemconfig(self.image_ids[J_L2], image=self.l2_img)
            #self.get_logger().info("L2 released, switching to green image.")

        if self.joy.axes[J_R2] < 1:
            self.canvas.itemconfig(self.image_ids[J_R2], image=self.r2_img_red)
            #self.get_logger().info("R2 pressed, switching to red image.")
        else:
            self.canvas.itemconfig(self.image_ids[J_R2], image=self.r2_img)
            #self.get_logger().info("R2 released, switching to green image.")

        # Handle stick zones and levels
        self.canvas.coords(self.image_ids['L_stick'], self.corner_x + 400 - self.joy.axes[J_LH] * 50, self.corner_y + 500 - self.joy.axes[J_LV] * 50)
        self.canvas.coords(self.image_ids['R_stick'], self.corner_x + 600 - self.joy.axes[J_RH] * 50, self.corner_y + 500 - self.joy.axes[J_RV] * 50)

        # Handle L3 and R3 buttons
        if self.joy.buttons[B_L3] == 1:
            self.canvas.itemconfig(self.image_ids['L_stick'], image=self.l_stick_img_red)
            #self.get_logger().info("L3 pressed, switching to red image.")
        else:
            self.canvas.itemconfig(self.image_ids['L_stick'], image=self.l_stick_img)
            #self.get_logger().info("L3 released, switching to green image.")

        if self.joy.buttons[B_R3] == 1:
            self.canvas.itemconfig(self.image_ids['R_stick'], image=self.r_stick_img_red)
            #self.get_logger().info("R3 pressed, switching to red image.")
        else:
            self.canvas.itemconfig(self.image_ids['R_stick'], image=self.r_stick_img)
            #self.get_logger().info("R3 released, switching to green image.")

        # # handle share button   
        # if self.joy.buttons[B_SHARE] == 1:
        #    self.canvas.itemconfig(self.image_ids[B_SHARE], image=self.share_img_red)
        # #   self.get_logger().info("Share pressed, switching to red image.")
        # else:
        #   self.canvas.itemconfig(self.image_ids[B_SHARE], image=self.share_img)
        # # self.get_logger().info("Share released, switching to green image.") 


def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    root.title("PS4 Controller Interface")
    joy_interface = JoyInterfaceNode(root)
    
    def ros_spin():
        rclpy.spin_once(joy_interface, timeout_sec=0.01)  # Increased frequency
        root.after(10, ros_spin)  # Increased frequency
    
    root.after(10, ros_spin)
    root.mainloop()
    
    joy_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()












# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Joy
# import tkinter as tk
# from tkinter import PhotoImage

# B_CROIX = 0
# B_ROND = 1
# B_CARRE = 2
# B_TRIANGLE = 3
# B_SHARE = 4
# B_PS = 5
# B_OPT = 6
# B_JL = 7
# B_JR = 8
# B_L1 = 9
# B_R1 = 10
# B_H = 11
# B_B = 12
# B_L = 13
# B_R = 14
# B_CLIC = 15

# J_LH = 0
# J_LV = 1
# J_L2 = 4
# J_RH = 2
# J_RV = 3
# J_R2 = 5

# class JoyInterfaceNode(Node):
#     def __init__(self, root):
#         super().__init__('joy_interface_node')
#         self.get_logger().info('Joy Interface Node has been started.')

#         # Initialize Joy message
#         self.joy = Joy()
#         self.joy.axes = [0.0] * 8
#         self.joy.buttons = [0] * 16

#         # Create subscriber for Joy messages
#         self.subscriber = self.create_subscription(Joy, "joy", self.cb_joy, 1)

#         # Create a timer to check button states
#         self.timer = self.create_timer(0.1, self.timer_callback)

#         # Tkinter setup
#         self.root = root
#         self.canvas = tk.Canvas(self.root, width=1000, height=600, bg="black")
#         self.canvas.pack()

#         # Top-left corner position
#         self.corner_x = 0
#         self.corner_y = 0

#         # Scale variable
#         self.scale = 0.25

#         # Load and scale images
#         self.resize_images()

#         # Add green images to the canvas
#         self.image_ids = {
#             B_L1: self.create_image_on_canvas(self.l1_img, 200, 250),
#             B_R1: self.create_image_on_canvas(self.r1_img, 800, 250),
#             B_SHARE: self.create_image_on_canvas(self.share_img, 300, 300),
#             B_OPT: self.create_image_on_canvas(self.option_img, 700, 300),
#             B_H: self.create_image_on_canvas(self.up_img, 200, 400),
#             B_L: self.create_image_on_canvas(self.left_img, 150, 450),
#             B_R: self.create_image_on_canvas(self.right_img, 250, 450),
#             B_B: self.create_image_on_canvas(self.down_img, 200, 500),
#             B_CROIX: self.create_image_on_canvas(self.cross_img, 800, 520),
#             B_TRIANGLE: self.create_image_on_canvas(self.triangle_img, 800, 370),
#             B_ROND: self.create_image_on_canvas(self.round_img, 870, 450),
#             B_CARRE: self.create_image_on_canvas(self.square_img, 730, 450),
#         }

#     def cb_joy(self, msg):
#         """Callback function for Joy messages."""
#         self.joy = msg
#         self.get_logger().info(f"Joy message received: {self.joy.buttons}")

#     def timer_callback(self) -> None:
#         """Callback function for the timer."""
#         self.highlight_buttons()

#     def load_image(self, file_path):
#         image = PhotoImage(file=f"src/px4-ros2-interface-lib/scripts/{file_path}")
#         width = int(image.width() * self.scale)
#         height = int(image.height() * self.scale)
#         self.get_logger().info(f"Loaded {file_path}: width={width}, height={height}")
#         return image.subsample(int(1/self.scale)), width, height

#     def resize_images(self):
#         # for green images
#         self.l1_img, self.l1_width, self.l1_height = self.load_image("green/L1.png")
#         self.l2_img, self.l2_width, self.l2_height = self.load_image("green/L2.png")
#         self.r1_img, self.r1_width, self.r1_height = self.load_image("green/R1.png")
#         self.r2_img, self.r2_width, self.r2_height = self.load_image("green/R2.png")
#         self.share_img, self.share_width, self.share_height = self.load_image("green/share.png")
#         self.option_img, self.option_width, self.option_height = self.load_image("green/option.png")
#         self.up_img, self.up_width, self.up_height = self.load_image("green/up.png")
#         self.left_img, self.left_width, self.left_height = self.load_image("green/left.png")
#         self.right_img, self.right_width, self.right_height = self.load_image("green/right.png")
#         self.down_img, self.down_width, self.down_height = self.load_image("green/down.png")
#         self.cross_img, self.cross_width, self.cross_height = self.load_image("green/cross.png")
#         self.triangle_img, self.triangle_width, self.triangle_height = self.load_image("green/triangle.png")
#         self.round_img, self.round_width, self.round_height = self.load_image("green/round.png")
#         self.square_img, self.square_width, self.square_height = self.load_image("green/square.png")
#         self.l_stick_img, self.l_stick_width, self.l_stick_height = self.load_image("green/L_sticks_level.png")
#         self.r_stick_img, self.r_stick_width, self.r_stick_height = self.load_image("green/R_sticks_level.png")
#         self.l_sticks_zone, self.l_sticks_zone_width, self.l_sticks_zone_height = self.load_image("green/L_sticks_zone.png")
#         self.r_sticks_zone, self.r_sticks_zone_width, self.r_sticks_zone_height = self.load_image("green/R_sticks_zone.png")

#         # for red images
#         self.l1_img_red, self.l1_width_red, self.l1_height_red = self.load_image("red/L1.png")
#         self.l2_img_red, self.l2_width_red, self.l2_height_red = self.load_image("red/L2.png")
#         self.r1_img_red, self.r1_width_red, self.r1_height_red = self.load_image("red/R1.png")
#         self.r2_img_red, self.r2_width_red, self.r2_height_red = self.load_image("red/R2.png")
#         self.share_img_red, self.share_width_red, self.share_height_red = self.load_image("red/share.png")
#         self.option_img_red, self.option_width_red, self.option_height_red = self.load_image("red/option.png")
#         self.up_img_red, self.up_width_red, self.up_height_red = self.load_image("red/up.png")
#         self.left_img_red, self.left_width_red, self.left_height_red = self.load_image("red/left.png")
#         self.right_img_red, self.right_width_red, self.right_height_red = self.load_image("red/right.png")
#         self.down_img_red, self.down_width_red, self.down_height_red = self.load_image("red/down.png")
#         self.cross_img_red, self.cross_width_red, self.cross_height_red = self.load_image("red/cross.png")
#         self.triangle_img_red, self.triangle_width_red, self.triangle_height_red = self.load_image("red/triangle.png")
#         self.round_img_red, self.round_width_red, self.round_height_red = self.load_image("red/round.png")
#         self.square_img_red, self.square_width_red, self.square_height_red = self.load_image("red/square.png")
#         self.l_sticks_zone_red, self.l_sticks_zone_width_red, self.l_sticks_zone_height_red = self.load_image("red/L_sticks_zone.png")
#         self.r_sticks_zone_red, self.r_sticks_zone_width_red, self.r_sticks_zone_height_red = self.load_image("red/R_sticks_zone.png")

#     def create_image_on_canvas(self, image, x, y):
#         return self.canvas.create_image(self.corner_x + x, self.corner_y + y, image=image, anchor=tk.CENTER)

#     def highlight_buttons(self):
#         button_images = {
#             B_L1: (self.l1_img, self.l1_img_red),
#             B_R1: (self.r1_img, self.r1_img_red),
#             B_SHARE: (self.share_img, self.share_img_red),
#             B_OPT: (self.option_img, self.option_img_red),
#             B_H: (self.up_img, self.up_img_red),
#             B_L: (self.left_img, self.left_img_red),
#             B_R: (self.right_img, self.right_img_red),
#             B_B: (self.down_img, self.down_img_red),
#             B_CROIX: (self.cross_img, self.cross_img_red),
#             B_TRIANGLE: (self.triangle_img, self.triangle_img_red),
#             B_ROND: (self.round_img, self.round_img_red),
#             B_CARRE: (self.square_img, self.square_img_red),
#         }

#         for button, (green_img, red_img) in button_images.items():
#             if self.joy.buttons[button] == 1:
#                 self.canvas.itemconfig(self.image_ids[button], image=red_img)
#                 self.get_logger().info(f"Button {button} pressed, switching to red image.")
#             else:
#                 self.canvas.itemconfig(self.image_ids[button], image=green_img)
#                 self.get_logger().info(f"Button {button} released, switching to green image.")

# def main(args=None):
#     rclpy.init(args=args)
    
#     root = tk.Tk()
#     root.title("PS4 Controller Interface")
#     joy_interface = JoyInterfaceNode(root)
    
#     def ros_spin():
#         rclpy.spin_once(joy_interface, timeout_sec=0.1)
#         root.after(100, ros_spin)
    
#     root.after(100, ros_spin)
#     root.mainloop()
    
#     joy_interface.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

