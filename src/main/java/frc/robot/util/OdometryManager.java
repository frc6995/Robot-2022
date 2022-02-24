// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** Add your docs here. */
public class OdometryManager implements Loggable {
    @Log(name="transformX", methodName = "getX")
    Translation2d m_transformToHub = new Translation2d(8, 4);
    @Log(name = "currX", methodName = "getX")
    Pose2d m_currPoseMeters = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    @Log(name = "lastX", methodName = "getX")
    Pose2d m_lastPoseMeters = m_currPoseMeters;

    Supplier<Pose2d> poseSupplier;

    public OdometryManager(Supplier<Pose2d> currentPose) {
        poseSupplier = currentPose;
    }

    public void periodic(){
        updateOdometry();
    }

    public Pose2d getTargetPose() {
        return new Pose2d(
            m_currPoseMeters
                .getTranslation().plus(m_transformToHub),
            Rotation2d.fromDegrees(0)
        );
    }
    
    /**
     * Read the pose from the Field2d widget and treat it as a drivetrain odometry update.
     */
    public void updateOdometry() {
        m_lastPoseMeters = m_currPoseMeters;
        m_currPoseMeters = poseSupplier.get();
        updateTargetOffset(m_currPoseMeters);
    }

    /**
     * Update the robot-relative target offset. 
     * @param newPose the new robot pose to update against.
     */
    public void updateTargetOffset(Pose2d newPose) {
        Translation2d delta = m_lastPoseMeters.getTranslation().minus(newPose.getTranslation());
        m_transformToHub = m_transformToHub.plus(delta);
    }

    /**
     * Adjusts the transform to the hub based on a vision measurement,
     * including the distance from the center of the robot to the vision tape,
     * and the radian offset between the robot heading and the target.
     * @param distanceToTapeMeters
     * @param rotationOffsetRadians
     */
    public void addVisionMeasurement(double distanceToTapeMeters, double rotationOffsetRadians) {
        m_transformToHub = new Translation2d(distanceToTapeMeters, new Rotation2d(rotationOffsetRadians).plus(m_currPoseMeters.getRotation()));
        System.out.println("added");
    }

    /**
     * Get the distance to the center of the hub.
     * @return the distance to the center of the hub.
     */
    @Log
    public double getDistanceToCenter() {
        return m_transformToHub.getNorm();
    }
    
    /**
     * Get the rotation transform that would transform the robot to be pointing at the hub.
     */
    @Log(methodName = "getRadians")
    public Rotation2d getRotationOffset() {

        return new Rotation2d(Math.atan2(m_transformToHub.getY(), m_transformToHub.getX()))
            .minus(m_lastPoseMeters.getRotation());
    }
}
