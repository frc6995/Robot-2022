// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class IntakeS extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.CAN_ID_TURRET, MotorType.kBrushless);
    private DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        Constants.DOUBLE_SOLENOID_INTAKE_EXTEND, Constants.DOUBLE_SOLENOID_INTAKE_RETRACT);
    
    public IntakeS() {
        intakeMotor.restoreFactoryDefaults();
    }

    public void intakeDeploy() {
        doubleSolenoid.set(Value.kForward);
    }

    public void intakeRetract() {
        doubleSolenoid.set(Value.kReverse);
    }

    public void intakeToggle() {
        if (doubleSolenoid.get() == Value.kReverse) {
            doubleSolenoid.set(Value.kForward);
        }
        else {
            doubleSolenoid.set(Value.kReverse);
        }
    }

    public void intakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void periodic() {
    }
}
