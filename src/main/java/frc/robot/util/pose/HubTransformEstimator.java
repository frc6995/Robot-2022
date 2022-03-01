// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.pose;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.Constants.*;
import static frc.robot.util.NomadMathUtil.*;

/** Add your docs here. */
class HubTransformEstimator {
    private Supplier<Pose2d> poseSupplier;
    private Pose2d m_lastPose;
    /**
     * Maps the current robot pose to a pose at the hub's translation, matching the heading of the robot;
     */
    private Transform2d m_robotToHubTranslation;

    HubTransformEstimator(Supplier<Pose2d> pose) {
        poseSupplier = pose;
        m_robotToHubTranslation = new Transform2d(pose.get(), HUB_CENTER_POSE);
        m_robotToHubTranslation = new Transform2d(
            m_robotToHubTranslation.getTranslation(), new Rotation2d());
        m_lastPose = poseSupplier.get();
    }

    void update() {
        Transform2d delta = new Transform2d(m_lastPose, poseSupplier.get()); 
        //odometry may become inaccurate in the field frame, but the delta should not go wrong.
        m_robotToHubTranslation = new Transform2d(
                m_robotToHubTranslation
                    .getTranslation()
                    .plus(
                        delta.inverse().getTranslation()
                    )
                    .rotateBy(delta.inverse().getRotation()),
                new Rotation2d());
        m_lastPose = poseSupplier.get();
    }

    Transform2d getRobotToHubTransform(){
        return m_robotToHubTranslation;
    }

    void setCorrectedRobotToHubTransform(Transform2d newTransform){
        m_robotToHubTranslation = new Transform2d(
            newTransform.getTranslation(), new Rotation2d());
    }
}
