// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.pose;

import static frc.robot.Constants.HUB_CENTER_POSE;
import static frc.robot.util.NomadMathUtil.getDistance;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.interpolation.ShooterInterpolatingTable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** Add your docs here. */

public class NavigationManager implements Loggable{
    private Supplier<Pose2d> m_poseSupplier;
    private Supplier<Rotation2d> m_drivebaseHeadingSupplier;
    private Supplier<Rotation2d> m_robotToTurretSupplier;
    

    private Rotation2d m_lastHeading;


    public NavigationManager(Supplier<Pose2d> poseSupplier,
        Supplier<Rotation2d> robotToTurretSupplier,
        Consumer<Double> turretAngularVelocityConsumer
        ) {
        m_poseSupplier = poseSupplier;
        m_drivebaseHeadingSupplier = ()->poseSupplier.get().getRotation();
        m_robotToTurretSupplier = robotToTurretSupplier;
        m_lastHeading = m_drivebaseHeadingSupplier.get();
    }

    public void update() {
    }
    
    
    public static boolean getDistanceInRange(double distance) {
        return (
            (ShooterInterpolatingTable.MIN_DISTANCE < distance)
            && (distance < ShooterInterpolatingTable.MAX_DISTANCE)
        );
    }

    public Pose2d getCurrentRobotPose() {
        return m_poseSupplier.get();
    }

    public Transform2d getRobotToCameraTransform() {
        return new Transform2d(
            new Translation2d(
                Constants.CAMERA_CENTER_OFFSET,0
            ).rotateBy(
                m_robotToTurretSupplier.get()
            ),
            m_robotToTurretSupplier.get());
    }
    
    public Transform2d getRobotToTurretTransform() {
        return new Transform2d(
            new Translation2d(),
            m_robotToTurretSupplier.get()
          );
    }

    // for logging only
    //@Log(methodName = "getRadians")
    public Rotation2d getRobotToTurretRotation() {
        return m_robotToTurretSupplier.get();
    }

}
