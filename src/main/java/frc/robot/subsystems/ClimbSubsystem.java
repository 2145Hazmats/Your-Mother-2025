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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbingForWorlds. */
  private TalonFX climbMotor = new TalonFX(Constants.ClimbContants.ClimbMotorId);

  private TalonFXConfiguration ClimbConfig = new TalonFXConfiguration();

  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
  new PositionTorqueCurrentFOC(0); //MECHANICAL ADVANTAGE POSITION CONTROL WITH FF

  public ClimbSubsystem() {
    
    ClimbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ClimbConfig.Slot0 = new Slot0Configs().withKP(Constants.ClimbContants.ClimbP).withKI(Constants.ClimbContants.ClimbI).withKD(Constants.ClimbContants.ClimbD);
    ClimbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // WE DONT KNOW IF THIS IS RIGHT :)
    ClimbConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive); //SWSP IF NEEDED

    //-------------------------------------------
    climbMotor.getConfigurator().apply(ClimbConfig);
  }

  public Command ClimbToHome() {
      return Commands.runOnce(() -> 
        climbMotor.setControl(new PositionDutyCycle(Constants.ClimbContants.ClimbHomeSetpoint)), this);
      }

    public Command ClimbLockIn() {
      return Commands.runOnce(() -> 
        climbMotor.setControl(new PositionDutyCycle(Constants.ClimbContants.ClimbLockInSetPoint)), this);
      }

    public Command ClimbUp() {
      return Commands.runOnce(() -> 
        climbMotor.setControl(new PositionDutyCycle(Constants.ClimbContants.ClimbUpSetPoint)), this);
      }

       public Command ClimbJoystick(double joystick) { //MIGHT NEED TO CHANGE THIS TO SUPPLIER
      return Commands.run(() -> climbMotor.setControl(new DutyCycleOut(joystick * .3)), this); //NERFED SPEED CHANGE LATER
    }

    public Command disableClimbMotor() {
      return Commands.runOnce(() -> climbMotor.setControl(new DutyCycleOut(0)), this);
    }

    public void setClimbPID(double NewKP, double NewKI, double NewKD) {
      ClimbConfig.Slot0.kP = NewKP;
      ClimbConfig.Slot0.kI = NewKI;
      ClimbConfig.Slot0.kI = NewKD;
      climbMotor.getConfigurator().apply(ClimbConfig); // NOT SURE IF THIS LINE WORKS MIGHT NEED COMMAND :)
    }

    public void FFPosition(double positionRad, double FF) { // STOEL FROM MECHANICAL ADVANTAGE
      climbMotor.setControl(
      positionTorqueCurrentRequest.withPosition(Units.radiansToRotations(positionRad)).withFeedForward(FF));
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Position", climbMotor.getPosition().getValueAsDouble());
  }
}
