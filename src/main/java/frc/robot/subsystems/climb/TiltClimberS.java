// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;

public class TiltClimberS extends SubsystemBase implements Loggable {
  private DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.DOUBLE_SOLENOID_CLIMBER_FORWARD, 
  Constants.DOUBLE_SOLENOID_CLIMBER_BACK);

  /** Creates a new ClimberS. */
  public TiltClimberS() {
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
