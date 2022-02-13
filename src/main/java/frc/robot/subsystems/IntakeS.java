// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The intake subsystem, which extends and spins to pull balls over the bumper into the midtake.
 */
public class IntakeS extends SubsystemBase {
    private final CANSparkMax intakeLeadMotor = new CANSparkMax(Constants.CAN_ID_INTAKE_LEAD_MOTOR, MotorType.kBrushless);
    private final CANSparkMax intakeFollowerMotor = new CANSparkMax(Constants.CAN_ID_INTAKE_FOLLOWER_MOTOR, MotorType.kBrushless);
    private DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        Constants.DOUBLE_SOLENOID_INTAKE_EXTEND, Constants.DOUBLE_SOLENOID_INTAKE_RETRACT);

    /**
     * Constructs a new IntakeS.
     */
    public IntakeS() {
        intakeLeadMotor.restoreFactoryDefaults();
        intakeFollowerMotor.restoreFactoryDefaults();
        intakeFollowerMotor.follow(intakeLeadMotor, true);
    }

    /**
     * Extends the intake.
     */
    public void deploy() {
        doubleSolenoid.set(Value.kForward);
    }

    /**
     * Retracts the intake.
     */
    public void retract() {
        doubleSolenoid.set(Value.kReverse);
    }

    /**
     * Toggles the intake.
     */
    public void toggle() {
        doubleSolenoid.toggle();
    }

    /**
     * Spins the intake at the given speed.
     * @param speed the speed
     */
    public void spin(double speed) {
        intakeLeadMotor.set(speed);
    }

    /**
     * Spins the intake inward at a speed set in Constants.
     */
    public void spin() {
        spin(Constants.INTAKE_SPEED);
    }

    /**
     * Spins the intake outward at a speed set in Constants to eject balls.
     */
    public void eject() {
        spin(Constants.INTAKE_EJECT_SPEED);
    }

    /**
     * Stops the intake.
     */
    public void stop() {
        spin(0);
    }

    @Override
    public void periodic() {
    }
}
