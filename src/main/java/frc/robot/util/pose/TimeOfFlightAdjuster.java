// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.pose;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.interpolation.ShooterInterpolatingTable;

/** Add your docs here. */
public class TimeOfFlightAdjuster {
    Supplier<ChassisSpeeds> m_drivebaseVelocitySupplier;
    Supplier<Transform2d> m_unAdjustedRobotToHubTranslationSupplier;
    Transform2d m_hubToAdjustedHubTranslation;

    TimeOfFlightAdjuster(Supplier<ChassisSpeeds> drivebaseVelocitySupplier, Supplier<Transform2d> unAdjustedRobotToHubTranslationSupplier) {
        m_drivebaseVelocitySupplier = drivebaseVelocitySupplier;
        m_unAdjustedRobotToHubTranslationSupplier = unAdjustedRobotToHubTranslationSupplier;
        update();
    }

    void update() {
        double distanceToTarget = NomadMathUtil.getDistance(m_unAdjustedRobotToHubTranslationSupplier.get());
        double timeOfFlight = ShooterInterpolatingTable.get(distanceToTarget).timeOfFlight;
        ChassisSpeeds velocity = m_drivebaseVelocitySupplier.get();
        Transform2d robotToFutureRobot = new Transform2d(
            new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond), new Rotation2d()
        ).times(timeOfFlight);
        m_hubToAdjustedHubTranslation = robotToFutureRobot.inverse();
    }

    Transform2d getHubTOFAdjustment() {
        return m_hubToAdjustedHubTranslation;
    }

}
