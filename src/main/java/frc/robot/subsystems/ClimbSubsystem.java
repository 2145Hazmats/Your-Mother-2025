// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbingForWorlds. */
  private TalonFX climbMotor = new TalonFX(Constants.ClimbContants.ClimbMotorId);

  private TalonFXConfiguration ClimbConfig = new TalonFXConfiguration();

  public ClimbSubsystem() {
     Slot0Configs slot0PID = new Slot0Configs();
      slot0PID.kP = 0.05;
      slot0PID.kI = 0;
      slot0PID.kD = 0;

      ClimbConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive); //SWSP IF NEEDED
      
      
      //-------------------------------------------
      climbMotor.getConfigurator().apply(ClimbConfig);
      climbMotor.getConfigurator().apply(slot0PID); //THIS MAY BE WRONG CHECK EXAMPLE CODE ONLINE

  }

  public Command ClimbToHome() {
      return Commands.runOnce(() -> 
        climbMotor.setControl(new PositionDutyCycle(Constants.ClimbContants.ClimbHome)), this);
      }

    public Command ClimbLockIn() {
      return Commands.runOnce(() -> 
        climbMotor.setControl(new PositionDutyCycle(Constants.ClimbContants.ClimbLockIn)), this);
      }

    public Command ClimbUp() {
      return Commands.runOnce(() -> 
        climbMotor.setControl(new PositionDutyCycle(Constants.ClimbContants.ClimbUp)), this);
      }

       public Command ClimbJoystick(DoubleSupplier joystick) {
      return Commands.run(() -> climbMotor.setControl(new DutyCycleOut(joystick.getAsDouble())), this);
    }

    public Command disableClimbMotor(DoubleSupplier joystick) {
      return Commands.run(() -> climbMotor.setControl(new DutyCycleOut(0)), this);
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Position", climbMotor.getPosition().getValueAsDouble());
  }
}
