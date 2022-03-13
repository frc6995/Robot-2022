// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.pose;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.interpolation.ShooterInterpolatingTable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.util.NomadMathUtil.*;
import static frc.robot.Constants.*;

/** Add your docs here. */

public class NavigationManager implements Loggable{
    private Supplier<Pose2d> m_poseSupplier;
    private Supplier<Rotation2d> m_drivebaseHeadingSupplier;
    private Supplier<Rotation2d> m_robotToTurretSupplier;
    private Consumer<Double> m_turretAngularVelocityConsumer;
    

    private Rotation2d m_lastHeading;
    private HubTransformEstimator hubTransformEstimator;
    private VisionHubTransformAdjuster visionHubTransformAdjuster;


    public NavigationManager(Supplier<Pose2d> poseSupplier,
        Supplier<Rotation2d> robotToTurretSupplier,
        Consumer<Double> turretAngularVelocityConsumer
        ) {
        m_poseSupplier = poseSupplier;
        m_drivebaseHeadingSupplier = ()->poseSupplier.get().getRotation();
        m_robotToTurretSupplier = robotToTurretSupplier;
        m_lastHeading = m_drivebaseHeadingSupplier.get();
        m_turretAngularVelocityConsumer = turretAngularVelocityConsumer;

        hubTransformEstimator = new HubTransformEstimator(m_poseSupplier);
        visionHubTransformAdjuster = new VisionHubTransformAdjuster(m_robotToTurretSupplier, hubTransformEstimator::setCorrectedRobotToHubTransform);
    }

    public void update() {
        hubTransformEstimator.update();
        
        visionHubTransformAdjuster.update();
        double omega = (m_drivebaseHeadingSupplier.get().getRadians() - m_lastHeading.getRadians()) / 0.02;

        m_turretAngularVelocityConsumer.accept(omega);

        m_lastHeading = m_drivebaseHeadingSupplier.get();


    }

    public void resetPose(Pose2d pose) {
        hubTransformEstimator.resetPose(pose);
    }

    //Setters
    public void setVisionEnabledSupplier(BooleanSupplier enabled) {
        visionHubTransformAdjuster.setVisionEnabled(enabled);
    }

    public void addVisionMeasurement(Rotation2d xOffset, double distanceToCenter) {
        visionHubTransformAdjuster.addVisionMeasurement(xOffset, distanceToCenter);
    }
    //Transform and pose getters
    
    
    public static boolean getDistanceInRange(double distance) {
        return (
            (ShooterInterpolatingTable.MIN_DISTANCE < distance)
            && (distance < ShooterInterpolatingTable.MAX_DISTANCE)
        );
    }

    @Log
    public boolean getTargetDistanceInRange() {
        return getDistanceInRange(getDistance(getRobotToHubTransform()));
    } 

    public Pose2d getCurrentRobotPose() {
        return m_poseSupplier.get();
    }

    public Transform2d getRobotToHubTransform() {
        return hubTransformEstimator.getRobotToHubTransform();
    }

    @Log(methodName = "getRadians")
    public Rotation2d getRobotToHubDirection() {
        return NomadMathUtil.getDirection(getRobotToHubTransform());
    }

    @Log()
    public double getRobotToHubDistance() {
        return NomadMathUtil.getDistance(getRobotToHubTransform());
    }

    public Transform2d getRobotToCameraTransform() {
        return visionHubTransformAdjuster.getRobotToCamera();
    }
    
    public Transform2d getRobotToTurretTransform() {
        return visionHubTransformAdjuster.getRobotToTurret();
    }

    // for logging only
    //@Log(methodName = "getRadians")
    public Rotation2d getRobotToTurretRotation() {
        return getRobotToTurretTransform().getRotation();
    }

    public Pose2d getEstimatedRobotPose() {
        return HUB_CENTER_POSE
        .plus( // Turn the hub pose to match the robot
            new Transform2d(
                new Translation2d(),
                m_drivebaseHeadingSupplier.get())
            )
            .plus(
                getRobotToHubTransform().inverse());
    }

}
