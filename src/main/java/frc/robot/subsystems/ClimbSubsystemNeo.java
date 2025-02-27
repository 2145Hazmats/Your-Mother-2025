// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbContants;

public class ClimbSubsystemNeo extends SubsystemBase{
  /** Creates a new ClimbingForWorlds. */
  private SparkMax climbMotorNeo = new SparkMax(Constants.ClimbContants.ClimbMotorId, MotorType.kBrushless);
  
  private Servo climbServo = new Servo(1);
  //private Servo climbServo = new Servo(2);

  private SparkMaxConfig ClimbConfig = new SparkMaxConfig();


  public ClimbSubsystemNeo() {
   ClimbConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40);

   ClimbConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
  }

  //Neo position to down spot then servo turn 180 then Neo spin back up to up position

    public void climbForwardCommand() {
      climbMotorNeo.set(Constants.ClimbContants.climbForwardSpeed);
      }

    public void climbBackwardCommand() {
      if (climbMotorNeo.getEncoder().getPosition() > 0) {climbMotorNeo.set(Constants.ClimbContants.ClimbBackwardSpeed);}
      else {
        climbMotorNeo.set(0);
      }
      }

    public void climbStopCommand() {
        climbMotorNeo.set(Constants.ClimbContants.ClimbStop);
      }

      @Override
      public void setDefaultCommand(Command defaultCommand) {
        // TODO Auto-generated method stub
        super.setDefaultCommand(defaultCommand);
      }
       public void ClimbJoystick(double joystick) { //MIGHT NEED TO CHANGE THIS TO SUPPLIER
        climbMotorNeo.set(joystick * .3); //NERFED SPEED CHANGE LATER
    }

    public void disableClimbMotor() {
        climbMotorNeo.set(0);
    }


    // Servo

    public Command climbUnlock() {
      return Commands.run(() -> climbServo.setPosition(Constants.ClimbContants.climbUnlockServoSpeed), this);
      }

    public Command climbLock() {
      return Commands.run(() -> climbServo.setPosition(Constants.ClimbContants.climbLockServoSpeed), this);
      }

    public void ClimbJoystickServo(DoubleSupplier joystick) { //MIGHT NEED TO CHANGE THIS TO SUPPLIER
        climbServo.setPosition(joystick.getAsDouble() * .3); //NERFED SPEED CHANGE LATER
    }

    public void disableClimbServo() {
        climbServo.setPosition(0);
    }

    public Command Keepclimbsafe() {
return Commands.run(() -> { if (climbMotorNeo.getEncoder().getPosition() < 0) {

  climbMotorNeo.set(0);
}}, this);
     
    }

   

    // public void isClimbSafe() {
    //  if (climbMotorNeo.getEncoder().getPosition() > 10) {


    //  }


   // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Position", climbMotorNeo.getEncoder().getPosition());
  
  } 
}

