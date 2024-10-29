#!/usr/bin/env python

import rospy
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
import numpy as np
from sar_msgs.msg import LandingTarget, LandingTargetArray
from filterpy.stats import mahalanobis


class UKFNode:
    def __init__(self):
        rospy.init_node('ukf_node', anonymous=True)

        # State dimension and measurement dimension
        self.dim_x = 2 # State Dimension
        self.dim_z = 1 # Measurement Dimension

        self.fixed_dt = 20e-3 # Fixed time step for prediction

        # Define sigma points
        points = MerweScaledSigmaPoints(self.dim_x, alpha=0.1, beta=2., kappa=1.)

        # Initialize UKF
        self.ukf = UKF(dim_x=self.dim_x, dim_z=self.dim_z, fx=self.fx, hx=self.hx, dt=self.fixed_dt, points=points)

        # Initial State Estimate
        self.ukf.x = np.array([0., 0.,]) # Initial State

        # Initial Covariance Estimate
        self.ukf.P *= 10 # Initial Covariance

        # Process Noise Covariance
        self.ukf.Q = np.diag([0.01, 0.05])

        # Measurement Noise Covariance
        self.ukf.R = np.diag([0.1])

        

        # Subscribers and Publishers
        self.measurement_sub = rospy.Subscriber("/LandingTargets", LandingTargetArray, self.update_ukf)
        self.filtered_pub = rospy.Publisher("/LandingTargets_Filtered", LandingTargetArray, queue_size=10)



        self.predict_timer = rospy.Timer(rospy.Duration(self.fixed_dt), self.predict)

        self.is_initialized = False


    # State transition function
    def fx(self, x, dt):
        F = np.array([[1, dt],
                      [0, 1]])
        return np.dot(F, x)
        
    # Measurement function (Converts a state into a measurement)
    def hx(self,x):

        return [x[0]]
    
    def predict(self, event):

        if not self.is_initialized:
            rospy.logwarn("UKF not initialized. Skipping prediction.")
            return
        
        self.ukf.predict(dt=self.fixed_dt)

        fused_target = LandingTarget()
        fused_target.Pose_Centroid_Filtered_body.position.x = self.ukf.x[0]
        fused_target.Twist_Centroid_Filtered_body.linear.x = -self.ukf.x[1]

        tau = np.clip(self.ukf.x[0]/-self.ukf.x[1],0,2)
        fused_target.Tau = tau
        fused_target.Theta_x = 0.0
        fused_target.D_perp = self.ukf.x[0]
        fused_target.Plane_Angle = 90.0

        LandingTargets = LandingTargetArray()
        LandingTargets.LandingTargets = [fused_target]
        LandingTargets.header.stamp = event.current_real

        self.filtered_pub.publish(LandingTargets)

        pass


    def update_ukf(self, LandingSurfaces_msg):

        if not LandingSurfaces_msg.LandingTargets:
            rospy.logwarn("No Landing Targets received. Skipping this measurement.")
            return
        
        # Assuming single target for now
        target = LandingSurfaces_msg.LandingTargets[0]

        # Extract current position
        p_x_cam = target.Pose_Centroid_Cam_body.position.x

        # Extract current position from LiDAR
        p_x_lidar = target.Pose_Centroid_Lidar_body.position.x

        if not self.is_initialized:
            self.ukf.x[0] = p_x_lidar
            self.is_initialized = True
    
        z = np.array([p_x_lidar])

        # Calculate Mahalanobis Distance
        # m1 = mahalanobis(x=p_x_lidar, mean=self.ukf.x[0], cov=self.ukf.P[0,0])
        # m2 = mahalanobis(x=p_x_cam, mean=self.ukf.x[0], cov=self.ukf.P[0,0])

        # print(f"Mahalanobis Distance (LiDAR): {m1:.2f}")
        # print(f"Mahalanobis Distance (Camera): {m2:.2f}")

        # if m1 > 3:
        #     rospy.logwarn("Mahalanobis Distance too high. Skipping LiDAR measurement.")
        #     return
        # if m2 > 3:
        #     rospy.logwarn("Mahalanobis Distance too high. Skipping Camera measurement.")
        #     return
        
        self.ukf.update(z)


if __name__ == '__main__':
    try:
        ukf_node = UKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
