// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */

  private TalonFX shooterMotor = new TalonFX(AlgaeConstants.shooterMotorID);
  private TalonFX armMotor = new TalonFX(AlgaeConstants.armMotorID);

  private TalonFXConfiguration config = new TalonFXConfiguration();
  private Slot0Configs slot0Congfigs;

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  public AlgaeSubsystem() {
    slot0Congfigs = config.Slot0;
    slot0Congfigs.kP = AlgaeConstants.armP; // A position error of 2.5 rotations results in 12 V output //4.8

    // set Motion Magic settings
    MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 125; 
    motionMagicConfigs.MotionMagicAcceleration = 200;
    
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
     
    armMotor.getConfigurator().apply(config);
  }

  // Arm Moving Methods
  public Command moveArmToPoint(double position) {
    return Commands.run(() -> armMotor.setControl(m_request.withPosition(position)), this);
  } 
  public void moveArmToPointMethod(double position) {
    armMotor.setControl(m_request.withPosition(position));
  } 

  // Algae Intake
  public Command intakeAlgae() {
    return Commands.run(() -> 
      shooterMotor.set(AlgaeConstants.intakeSpeed)
    );
  }
  public void intakeAlgaeMethod() {
    shooterMotor.set(AlgaeConstants.intakeSpeed);
  }

  // Algae Outtake
  public Command outtakeAlgae() {
    return Commands.run(() -> 
      shooterMotor.set(AlgaeConstants.outtakeSpeed)
    );
  }
  public void outtakeAlgaeMethod() {
    shooterMotor.set(AlgaeConstants.outtakeSpeed);
  }

  public void stopAlgaeShooter() {
    shooterMotor.set(0);
  }

  public Command algaeDefaultCommand() {
    return Commands.run(() -> {
      shooterMotor.set(0);
      armMotor.setControl(m_request.withPosition(AlgaeConstants.HomePosition));
    });
  }

  public double getAlgaeArmPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Arm Positon", armMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Algae Arm Speed", armMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Algae Shooter Speed", shooterMotor.getVelocity().getValueAsDouble()); 
  }
}
