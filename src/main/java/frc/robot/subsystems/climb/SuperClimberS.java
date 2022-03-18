// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;

public class SuperClimberS implements Loggable {

  public final ThriftyClimberS thriftyClimberS = new ThriftyClimberS();
  public final LinearClimberS linearClimberS = new LinearClimberS();
  public final TiltClimberS tiltClimberS = new TiltClimberS();

  private boolean locked = true;

  public final Trigger climberLockedTrigger = new Trigger(this::getIsLocked);
  
  /** Creates a new ClimberS. */
  public SuperClimberS() {
  }

  public void unlock() {
    locked = false;
  }

  public void lock() {
    if(thriftyClimberS.getBackPosition() < 10 && linearClimberS.getFrontPosition() < 10) {
      locked = true;
    }
  }

  public boolean getIsLocked() {
    return locked;
  }

  public void extendBack() {
    thriftyClimberS.extendBack();
  }

  public void retractBack() {
    thriftyClimberS.retractBack();
  }

  public void stopBack() {
    thriftyClimberS.stopBack();
  }

  public void holdBack() {
    thriftyClimberS.holdBack();
  }

  public void transferBack() {
    thriftyClimberS.driveBackTransfer();
  }

  public void transferFront() {
    linearClimberS.driveFrontTransfer();
  }
  public void extendFront() {
    linearClimberS.extendFront();
  }

  public void retractFront() {
    linearClimberS.retractFront();
  }

  public void stopFront() {
    linearClimberS.stopFront();
  }

  public void tiltForward() {
    tiltClimberS.tiltForward();
  }

  public void tiltBackward() {
    tiltClimberS.tiltBackward();
  }

  public void tiltStop() {
    tiltClimberS.tiltStop();
  }

  
}
