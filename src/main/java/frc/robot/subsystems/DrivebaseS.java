// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivebaseS extends SubsystemBase {
  private final CANSparkMax frontRight = new CANSparkMax(Constants.CAN_ID_FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(Constants.CAN_ID_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(Constants.CAN_ID_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backLeft = new CANSparkMax(Constants.CAN_ID_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
  public DifferentialDrive differentialDrive;

  /** Creates a new DrivebaseS. */
  public DrivebaseS() {
    frontRight.setInverted(true);
    backRight.follow(frontRight, false);
    backLeft.follow(frontLeft, false);
    differentialDrive = new DifferentialDrive(frontLeft, frontRight);

  }

  // Curvature drive method
  // Forward back is from 1 to -1, turn is from 1 to -1
  public void curvatureDrive(double fwdBack, double turn, boolean arcadeDrive) {
    differentialDrive.curvatureDrive(fwdBack, turn, arcadeDrive);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
