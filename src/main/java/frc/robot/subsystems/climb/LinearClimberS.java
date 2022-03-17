// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class LinearClimberS extends SubsystemBase implements Loggable {
  private CANSparkMax frontSparkMax = new CANSparkMax(Constants.CAN_ID_CLIMBER_MOTOR, MotorType.kBrushless);
  @Log(methodName = "getPosition", name = "frontPosition")
  private RelativeEncoder sparkMaxEncoder = frontSparkMax.getEncoder();
  
  /** Creates a new ClimberS. */
  public LinearClimberS() {
    
    frontSparkMax.restoreFactoryDefaults();
    frontSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    frontSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    frontSparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.CLIMBER_FRONT_SOFT_LIMIT_FORWARD);
    frontSparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.CLIMBER_FRONT_SOFT_LIMIT_BACK);
    frontSparkMax.setIdleMode(IdleMode.kBrake);
    frontSparkMax.setSmartCurrentLimit(40, 40, 0);
    frontSparkMax.burnFlash();
  }

  public double getFrontPosition() {
    return sparkMaxEncoder.getPosition();
  }
  
  public void extendFront() {
    frontSparkMax.setVoltage(10);
  }

  public void retractFront() {
    frontSparkMax.setVoltage(-10);
  }

  public void stopFront() {
    frontSparkMax.setVoltage(0);
  }

  public void holdFront() {
    frontSparkMax.setVoltage(0.5);
  }
  @Override
  public void periodic() {
    // // // This method will be called once per scheduler run
    // if(Math.abs(frontSparkMax.getAppliedOutput()) < 0.1 && frontSparkMax.getEncoder().getVelocity() > 10) {
    //   holdFront();
    // }
  }
}
