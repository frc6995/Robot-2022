// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.pose;

import static frc.robot.Constants.CAMERA_CENTER_OFFSET;
import static frc.robot.Constants.HUB_RADIUS_METERS;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
class VisionHubTransformAdjuster {

    private BooleanSupplier isVisionEnabled = ()->false;
    private Transform2d m_robotToTurretTrans;
    private Transform2d m_robotToCameraTrans;
    private Supplier<Rotation2d> m_robotToTurretSupplier;
    private Consumer<Transform2d> m_correctedRobotToHubConsumer;
    
    VisionHubTransformAdjuster(Supplier<Rotation2d> robotToTurretSupplier, Consumer<Transform2d> correctedRobotToHubConsumer) {
        m_robotToTurretSupplier = robotToTurretSupplier;
        m_correctedRobotToHubConsumer = correctedRobotToHubConsumer;
    }

    void setVisionEnabled(BooleanSupplier enabled) {
        isVisionEnabled = enabled;
    }

    boolean getVisionEnabled() {
        return isVisionEnabled.getAsBoolean();
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

    void addVisionMeasurement(Rotation2d xOffset, double distance) {
        if(isVisionEnabled.getAsBoolean()) {
            Transform2d cameraToHubTrans = new Transform2d(
            PhotonUtils.estimateCameraToTargetTranslation(distance - HUB_RADIUS_METERS, xOffset)
            .times((distance + HUB_RADIUS_METERS)/distance),
            new Rotation2d()); //Scale the translation an extra 2 meters to account for the radius of the ring
            m_correctedRobotToHubConsumer.accept(m_robotToCameraTrans.plus(cameraToHubTrans));
        }

    }
}
