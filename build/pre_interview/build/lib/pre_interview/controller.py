#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import matplotlib.pyplot as plt 


class SpeedController(Node):
    def __init__(self):
        super().__init__('speed_controller')
        
        ###### PID parameters ##############
        self.kp = 0.8
        self.ki = 0.15  
        self.kd = 0.05
        
        ###### target ####################
        self.target_speed = 60.0 
        
        ###### PID variables #####################
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time() 
        
        ###### current speed ########################
        self.current_speed = 0.0
        
        ###### data for plotting ####################
        self.time_data = []
        self.target_data = []
        self.actual_data = []
        self.start_time = time.time()
        
        ################################# publishers & subscribers ###########################
        self.cmd_publisher = self.create_publisher(Float32, '/cmd_vel', 10) #(massage_type, topic, dhistory_depth)
        self.speed_subscriber = self.create_subscription(Float32, '/current_speed', self.speed_callback, 10) #each time that subscribe a massage will call the "self.speed_callback" function
        
        ###### timer for control loop #############################################
        self.timer = self.create_timer(0.05, self.control_loop) 

        ###### to check the node when it initialized ######################
        self.get_logger().info('PID Controller initialized')

        ###### call back function #######################
        ###### this function update the current_speed value ##############
    def speed_callback(self, msg):
        self.current_speed = msg.data
        ###### if the speed != 0.0 print it ############
        if self.current_speed != 0.0:  
            self.get_logger().info(f'Speed received: {self.current_speed:.2f} km/h')
    


    def control_loop(self):
        current_time = time.time()
        dt = current_time - self.last_time  #important for I & D calculations
        
        # ignore too small values cuz the derivation will read it as a noise and it hates noise! to avoid division by zero
        if dt < 0.001:
            return
            
        # calculate error
        error = self.target_speed - self.current_speed
        
        ###### PID calculations "all of these rules are constent rules for PID logic" ########
        # for P
        p_term = self.kp * error
        
        # for I
        self.integral += error * dt
        # anti windup
        if self.integral > 10:
            self.integral = 10
        elif self.integral < -10:
            self.integral = -10

        i_term = self.ki * self.integral
        
        # for D
        d_term = self.kd * (error - self.previous_error) / dt
        
        # total PID output
        output = p_term + i_term + d_term
        
        ####### clamp output to [-1, 1] ############
        if output > 1.0:
            output = 1.0
        elif output < -1.0:
            output = -1.0
        
        # store the time
        elapsed_time = current_time - self.start_time 
        #the diff between elapsed_time and dt is the dt is used for calaulation of pid and the elapsed_time is just the relative time
        
        # Publish control command
        cmd_msg = Float32() #new msg
        cmd_msg.data = output 
        self.cmd_publisher.publish(cmd_msg) #print msg
        
        ##################################################################################
        ##if elapsed_time < 5.0:  # Log first 5 seconds only
            ##self.get_logger().info(f'Publishing cmd_vel: {output:.3f}')
        ##################################################################################

        ##### save data for plot #################
        self.time_data.append(elapsed_time)
        self.target_data.append(self.target_speed)
        self.actual_data.append(self.current_speed)
        
        # update values or next calculations
        self.previous_error = error
        self.last_time = current_time
        
        # log every 40 sample
        if len(self.time_data) % 40 == 0:  
            self.get_logger().info(f'[Time {elapsed_time:.1f}s] Target: {self.target_speed:.1f} km/h | Actual: {self.current_speed:.1f} km/h | Error: {error:.2f} | Output: {output:.3f}')
        
        # stop and plot after 60 seconds
        if elapsed_time >= 60.0:
            self.plot_results()
            self.get_logger().info('Test completed - 60 seconds reached')
            rclpy.shutdown()
    
    ################################ plot function ######################################
    def plot_results(self):
        plt.figure(figsize=(10, 6)) #screen size
        plt.plot(self.time_data, self.target_data, 'r--', label='Target Speed', linewidth=2)
        plt.plot(self.time_data, self.actual_data, 'b-', label='Actual Speed', linewidth=2)
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (km/h)')
        plt.title('PID Speed Control Performance')
        plt.legend()
        plt.grid(True)
        plt.xlim(0, 60)
        plt.ylim(0, 70)
        plt.tight_layout()
        plt.savefig('speed_control_results.png', dpi=300)
        ######### save photo ############
        plt.show()
        print("Plot saved as 'speed_control_results.png'")


def main(args=None):
    rclpy.init(args=args)
    controller = SpeedController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller stopped')
    finally:
        if len(controller.time_data) > 0:
            controller.plot_results()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()