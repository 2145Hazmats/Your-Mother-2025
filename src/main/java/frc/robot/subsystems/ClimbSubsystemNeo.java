// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
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
  
  private PIDController pidControllerClimbReady = new PIDController(1, 0, 0);

  private PIDController pidControllerClimbNailedIt = new PIDController(6, 0, 0);

  private Servo climbServo = new Servo(1);
  //private Servo climbServo = new Servo(2);

  private SparkMaxConfig ClimbConfig = new SparkMaxConfig();

  public Command KeepClimbSafeDefaultCommand() {
    return Commands.run(() -> { if (true) { 
      climbServo.set(Constants.ClimbContants.climbLockServoPosition);
      climbMotorNeo.set(0);
    }}, this);
  }

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
  public void climbInCommand() {//.05
    climbServo.set(Constants.ClimbContants.climbLockServoPosition);
    climbMotorNeo.set(Constants.ClimbContants.climbForwardSpeed);
  } 

  public void goodClimbInCommand() {//.05
    if (climbMotorNeo.getEncoder().getPosition() <0 ) {
    climbServo.set(Constants.ClimbContants.climbLockServoPosition);
    climbMotorNeo.set(Constants.ClimbContants.climbForwardSpeed);}
    else {
      climbMotorNeo.set(0);
    }
  } 

  public void climbOutCommandpart1() {
    climbServo.set(Constants.ClimbContants.climbUnlockServoPosition);
  }

  public void climbOutCommandpart2() {
    climbServo.set(Constants.ClimbContants.climbUnlockServoPosition);
    climbMotorNeo.set(Constants.ClimbContants.ClimbBackwardSpeed);
  }

  public void climbToSetpointPID() {
    climbServo.set(Constants.ClimbContants.climbUnlockServoPosition);
    climbMotorNeo.set(pidControllerClimbReady.calculate(climbMotorNeo.getEncoder().getPosition(), Constants.ClimbContants.ClimbReadySetpoint));
  }

  public void climbToNailItPID() {
    climbServo.set(Constants.ClimbContants.climbLockServoPosition);
    climbMotorNeo.set(pidControllerClimbNailedIt.calculate(climbMotorNeo.getEncoder().getPosition(), Constants.ClimbContants.ClimbLockedInSetpoint));
  }

  // Same Command?? cody fix it
  public void climbStopCommand() {
    climbMotorNeo.set(Constants.ClimbContants.ClimbStop);
  }
  public void disableClimbMotor() {
    climbMotorNeo.set(0);
  }

  // Untested lol prolly dont need it (STANLEY!!)
  // public void ClimbJoystick(double joystick) { //MIGHT NEED TO CHANGE THIS TO SUPPLIER
  //   climbMotorNeo.set(joystick * .3); //NERFED SPEED CHANGE LATER
  // }
  // public void ClimbJoystickServo(DoubleSupplier joystick) { //MIGHT NEED TO CHANGE THIS TO SUPPLIER
  //   climbServo.set(joystick.getAsDouble() * .3); //NERFED SPEED CHANGE LATER
  // }

  // Servo

  public Command climbUnlock() {
    return Commands.run(() -> climbServo.set(Constants.ClimbContants.climbUnlockServoPosition), this);
  }

  public Command climbLock() {
    return Commands.run(() -> climbServo.set(Constants.ClimbContants.climbLockServoPosition), this);
  }

  public void disableClimbServo() {
    climbServo.set(0);
  }

  public void resetMotorPosition() {
    climbMotorNeo.getEncoder().setPosition(0);
  }

  public void PutTheServoInTheRightSpotPlease() {

    climbServo.set(Constants.ClimbContants.climbLockServoPosition);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Position", climbMotorNeo.getEncoder().getPosition());
    SmartDashboard.putNumber("servo pos", climbServo.getPosition());
    SmartDashboard.putNumber("servo angle", climbServo.getAngle());
  } 
}

