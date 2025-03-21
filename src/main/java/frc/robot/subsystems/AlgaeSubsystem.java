// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ErrorConstants;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */

  //private TalonFX shooterMotor = new TalonFX(AlgaeConstants.shooterMotorID);
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
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armMotor.setPosition(0);
     
    armMotor.getConfigurator().apply(config);
  }

  public Command AlgaeDefaultCommand() {
    return Commands.runOnce(() -> {
      //shooterMotor.set(0);
      armMotor.setControl(m_request.withPosition(AlgaeConstants.HomePosition));
    }, this);
  }
  public boolean isAlgaeAtHome() {
    return MathUtil.isNear(AlgaeConstants.HomePosition, GetAlgaeArmPosition(), ErrorConstants.AlgaeError);
  }

  // Arm Moving Methods
  public Command MoveArmToPointCommand(double position) {
    return Commands.run(() -> armMotor.setControl(m_request.withPosition(position)), this);
  } 

  public void MoveArmToPointMethod(double position) {
    armMotor.setControl(m_request.withPosition(position));
    
  } 

  // Manual Methods
  public void algaeJoystick(double joystick) { 
    armMotor.setControl(new DutyCycleOut(joystick * AlgaeConstants.armspeed));
    //shooterMotor.setControl(new DutyCycleOut(shooterspeed));
 }

 public boolean isArmInDealgifyPosition() {
  return (GetAlgaeArmPosition() > Constants.AlgaeConstants.DealgifyPosition - Constants.AlgaeConstants.ALGAE_ARM_ERROR && GetAlgaeArmPosition() < AlgaeConstants.DealgifyPosition + AlgaeConstants.ALGAE_ARM_ERROR);
 }

 public boolean IsArmAwayFromHome() {
  return (GetAlgaeArmPosition() > AlgaeConstants.DistanceAwayFromHome);
   
 }

  public double GetAlgaeArmPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  public Command resetAlgaePosition() {
    return Commands.runOnce(() -> armMotor.setPosition(0), this);
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Arm Positon", armMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Algae Arm Speed", armMotor.getVelocity().getValueAsDouble());
    //SmartDashboard.putNumber("Algae Shooter Speed", shooterMotor.getVelocity().getValueAsDouble()); 
  }
}
