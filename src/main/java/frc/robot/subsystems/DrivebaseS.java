// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivebaseS extends SubsystemBase {
  private final CANSparkMax frontRight = new CANSparkMax(Constants.CAN_ID_FRONT_RIGHT_DRIVE_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(Constants.CAN_ID_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(Constants.CAN_ID_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backLeft = new CANSparkMax(Constants.CAN_ID_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
  private final SlewRateLimiter fwdBackLimiter = new SlewRateLimiter(Constants.DRIVEBASE_FWD_BACK_SLEW_LIMIT);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.DRIVEBASE_TURN_SLEW_LIMIT);
  private final AHRS navX = new AHRS(Port.kMXP);

  /** Creates a new DrivebaseS. */
  public DrivebaseS() {
    frontRight.restoreFactoryDefaults();
    frontLeft.restoreFactoryDefaults();
    backRight.restoreFactoryDefaults();
    backLeft.restoreFactoryDefaults();
    frontRight.setInverted(true);
    backRight.follow(frontRight, false);
    backLeft.follow(frontLeft, false);

  }

  public double deadbandJoysticks(double value) {
    if (Math.abs(value) < Constants.DRIVEBASE_DEADBAND) {
      value = 0;
    }
    return value;
  }

  // Curvature drive method
  // Forward back is from 1 to -1, turn is from 1 to -1
  public void curvatureDrive(double fwdBack, double turn) {
    fwdBack = deadbandJoysticks(fwdBack);
    turn = deadbandJoysticks(turn);
    fwdBack = fwdBackLimiter.calculate(fwdBack);
    turn = turnLimiter.calculate(turn);
    boolean quickTurn = false;
    SmartDashboard.putNumber("fwdBack", fwdBack);
    if (fwdBack == 0) {
      quickTurn = true;
    }
    WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(fwdBack, turn, quickTurn);
    tankDrive(speeds.left, speeds.right);
  }

  public void tankDrive(double left, double right) {
    frontLeft.set(left);
    frontRight.set(right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
