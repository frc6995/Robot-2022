// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ClimberS extends SubsystemBase implements Loggable {
  private CANSparkMax frontSparkMax = new CANSparkMax(Constants.CAN_ID_CLIMBER_MOTOR, MotorType.kBrushless);
  @Log(methodName = "getPosition", name = "frontPosition")
  private RelativeEncoder sparkMaxEncoder = frontSparkMax.getEncoder();

  private DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.DOUBLE_SOLENOID_CLIMBER_FORWARD, 
  Constants.DOUBLE_SOLENOID_CLIMBER_BACK);

  private CANSparkMax backSparkMax = new CANSparkMax(Constants.CAN_ID_BACK_CLIMB_MOTOR, MotorType.kBrushless);
  @Log(methodName = "getPosition", name = "backPosition")
  private RelativeEncoder sparkMaxEncoderTwo = backSparkMax.getEncoder();
  @Log
  public Command resetBackToExtended = new InstantCommand(()->sparkMaxEncoderTwo.setPosition(Constants.CLIMBER_BACK_SOFT_LIMIT_FORWARD)).withName("resetBackToExtended");
  @Log
  public Command resetBackToRetracted= new InstantCommand(()->sparkMaxEncoderTwo.setPosition(Constants.CLIMBER_BACK_SOFT_LIMIT_BACK)).withName("resetBackToRetracted");
  /** Creates a new ClimberS. */
  public ClimberS() {
    
    frontSparkMax.restoreFactoryDefaults();
    frontSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    frontSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    frontSparkMax.setSoftLimit(SoftLimitDirection.kForward, Constants.CLIMBER_FRONT_SOFT_LIMIT_FORWARD);
    frontSparkMax.setSoftLimit(SoftLimitDirection.kReverse, Constants.CLIMBER_FRONT_SOFT_LIMIT_BACK);
    frontSparkMax.setIdleMode(IdleMode.kBrake);
    frontSparkMax.setSmartCurrentLimit(40, 40, 0);

    backSparkMax.restoreFactoryDefaults();
    backSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    backSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    backSparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.CLIMBER_BACK_SOFT_LIMIT_FORWARD);
    backSparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.CLIMBER_BACK_SOFT_LIMIT_BACK);
    backSparkMax.setIdleMode(IdleMode.kBrake);
    backSparkMax.setSmartCurrentLimit(40, 40, 0);
  }

  public void extendBack() {
    backSparkMax.setVoltage(9);
  }

  public void retractBack() {
    backSparkMax.setVoltage(-9);
  }

  public void stopBack() {
    backSparkMax.setVoltage(0);
  }

  public void holdBack() {
    backSparkMax.setVoltage(Constants.CLIMBER_BACK_HOLDING_FF);
  }

  public void extendFront() {
    frontSparkMax.setVoltage(3);
  }

  public void retractFront() {
    frontSparkMax.setVoltage(-3);
  }

  public void stopFront() {
    frontSparkMax.setVoltage(0);
  }

  public void tiltForward() {
    doubleSolenoid.set(Value.kForward);
  }

  public void tiltBackward() {
    doubleSolenoid.set(Value.kReverse);
  }

  public void tiltStop() {
    doubleSolenoid.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // if(Math.abs(backSparkMax.getAppliedOutput()) < 0.1 && backSparkMax.getEncoder().getVelocity() > 10) {
    //   holdBack();
    // }
  }
}
