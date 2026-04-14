#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import time
import numpy as np

class ErrorPlotter(Node):
    def __init__(self):
        super().__init__('error_plotter')
        self.subscription = self.create_subscription(
            Float32,
            '/trajectory_error',
            self.error_callback,
            10)
        self.errors = []
        self.times = []
        self.start_time = None
        
        self.get_logger().info('================================')
        self.get_logger().info('Error Plotter Initialized!')
        self.get_logger().info('Listening to /trajectory_error...')
        self.get_logger().info('Drive the car, then press Ctrl+C to stop and display the graph.')
        self.get_logger().info('================================')

    def error_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info("Received first error data point. Recording...")
        
        current_time = time.time() - self.start_time
        self.times.append(current_time)
        self.errors.append(msg.data)

def main(args=None):
    rclpy.init(args=args)
    error_plotter = ErrorPlotter()
    
    try:
        rclpy.spin(error_plotter)
    except KeyboardInterrupt:
        pass
    finally:
        if len(error_plotter.errors) > 0:
            error_plotter.get_logger().info(f'Plotting {len(error_plotter.errors)} data points...')
            
            # Matplotlib graphing
            plt.figure(figsize=(10, 6))
            plt.plot(error_plotter.times, error_plotter.errors, linewidth=2, color='r')
            
            # Calculate some statistics
            mean_error = np.mean(error_plotter.errors)
            max_error = np.max(error_plotter.errors)
            
            plt.axhline(y=mean_error, color='b', linestyle='--', label=f'Mean Error ({mean_error:.3f}m)')
            
            plt.title('Pure Pursuit Cross-Track Error Over Time')
            plt.xlabel('Time (s)')
            plt.ylabel('Cross-track Error (meters)')
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            
            # Show the graph and block until the user closes it
            plt.show()
        else:
            error_plotter.get_logger().warn('No error data was received! Did the car move?')
            
        error_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
