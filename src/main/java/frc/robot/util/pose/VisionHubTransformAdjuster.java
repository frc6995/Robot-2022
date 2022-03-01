// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.pose;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.*;

/** Add your docs here. */
class VisionHubTransformAdjuster {

    private boolean isVisionEnabled = false;
    private Transform2d m_robotToTurretTrans;
    private Transform2d m_robotToCameraTrans;
    private Supplier<Rotation2d> m_robotToTurretSupplier;
    private Consumer<Transform2d> m_correctedRobotToHubConsumer;
    
    VisionHubTransformAdjuster(Supplier<Rotation2d> robotToTurretSupplier, Consumer<Transform2d> correctedRobotToHubConsumer) {
        m_robotToTurretSupplier = robotToTurretSupplier;
        m_correctedRobotToHubConsumer = correctedRobotToHubConsumer;
    }

    void setVisionEnabled(boolean enabled) {
        isVisionEnabled = enabled;
    }

    boolean getVisionEnabled() {
        return isVisionEnabled;
    }

    void update(){
        m_robotToTurretTrans = new Transform2d(
            new Translation2d(),
            m_robotToTurretSupplier.get()
          );
        m_robotToCameraTrans = new Transform2d(
            new Translation2d(
                CAMERA_CENTER_OFFSET,0
            ).rotateBy(
                m_robotToTurretSupplier.get()
            ),
            m_robotToTurretSupplier.get()
        );
    }

    Transform2d getRobotToTurret() {
        return m_robotToTurretTrans;
    }

    Transform2d getRobotToCamera() {
        return m_robotToCameraTrans;
    }

    void addVisionMeasurement(PhotonTrackedTarget target) {
        double xOffset = target.getYaw();
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        TARGET_HEIGHT_METERS,
        CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(target.getPitch()));
        
        Transform2d cameraToHubTrans = new Transform2d(
        PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-1 * xOffset))
        .times((distance + HUB_RADIUS_METERS)/distance),
        new Rotation2d()); //Scale the translation an extra 2 meters to account for the radius of the ring
        m_correctedRobotToHubConsumer.accept(m_robotToCameraTrans.plus(cameraToHubTrans));
    }
}
