package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.elevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX motorLeader = new TalonFX(elevatorConstants.motorLeaderID);
    private TalonFX motorFollower = new TalonFX(elevatorConstants.motorFollowerID);

    private TalonFXConfiguration leadConfig = new TalonFXConfiguration();
    private TalonFXConfiguration followerConfig = new TalonFXConfiguration();

    public int levelIndex = 0;
    
      /* Constructor */
    public ElevatorSubsystem() {

      Slot0Configs leaderConfig = new Slot0Configs(); //WHAT EVEN IS A SLOT 0 CONFIGS
      leaderConfig.kP = 0.05; // An error of 1 rotation results in 2.4 V output
      leaderConfig.kI = 0; // no output for integrated error
      leaderConfig.kD = 0; // A velocity of 1 rps results in 0.1 V output

      Slot0Configs followerConfig = new Slot0Configs();
      followerConfig.kP = 0.05;
      followerConfig.kI = 0;
      followerConfig.kD = 0;
      
      
      //-------------------------------------------
      motorLeader.getConfigurator().apply(leaderConfig);
      motorFollower.getConfigurator().apply(followerConfig);

      
      
      //followerConfig.MotorOutput.Inverted // DOESN"T WORKKKKK UNLESS IM STUPID :)

      // Sets Follower to follow leader
      motorFollower.setControl(new Follower(elevatorConstants.motorLeaderID, true));
    }

    public void levelIndexSwitch(boolean down){ //IT SAID UP BEFORE BUT I THINK ITS WRONG LOL 
      if(down == true) {
          if (levelIndex == 0) { levelIndex = 3; }
          else { levelIndex--; }
      }
      else {
          if(levelIndex == 3) { levelIndex = 0; }
          else{ levelIndex++; }
      }
  }

/* 
    public void elevatorToIndex() {

      motorLeader.setControl(levelIndex.check the nuber I guess);
    }*/ //THIS IS THURSDAYS PROBLEM LOL 

    public Command elevatorToHome() {
      return Commands.runOnce(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.HomePosition)), this);
      }

    public Command elevatorToL1() {
      return Commands.runOnce(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.L1Position)), this);
      }

    public Command elevatorToL2() {
      return Commands.runOnce(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.L2Position)), this);
      }

    public Command elevatorToL3() {
      return Commands.runOnce(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.L3Position)), this);
      }

    public Command elevatorToL4() {
      return Commands.runOnce(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.L4Position)), this);
      }


    public Command elevatorJoystick(DoubleSupplier joystick) {
      return Commands.run(() -> motorLeader.setControl(new DutyCycleOut(joystick.getAsDouble())), this);
    }

    public Command disableElevator() {
      return Commands.run(() -> motorLeader.setControl(new DutyCycleOut(0)), this);
    }

    public Command defaultCommand() {
      return Commands.runOnce(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.HomePosition)), this).withTimeout(1).andThen(disableElevator());
      }
    }

    //MOVE THIS TO THE SHOOTERBOXX SUBSYSTEM BRUHHHH
    // public boolean ShooterBoxSensorTrue() { 
    //   // Query some boolean state, such as a digital sensor.
    //   return false;
    // }

    // public Command ElevatorMove(double setpoint) {
    //   return Commands.runOnce(() -> {
    //     // on start
       
    //     // create a position closed-loop request, voltage output, slot 0 configs
    //     final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  
  
    //     // set position to aetpoint rotations
    //     motorLeader.setControl(m_request.withPosition(setpoint));
   
    //   });
    // }

    // public Command setSpeed(DoubleSupplier input) {
    //   return Commands.run(() -> {
    //     // on start
       
    //     // Makes stops to ensure it doesn't overshoot
    //     if (motorLeader.getPosition().getValueAsDouble() >= -17.2 && input.getAsDouble() < 0) {
    //       motorLeader.set(input.getAsDouble()*0.35);
    //     }
       
    //     // Ensures you dont drive it into the ground super hard
    //     else if (motorLeader.getPosition().getValueAsDouble() <= -2 && input.getAsDouble() > 0) {
    //       motorLeader.set(input.getAsDouble()*0.35);
    //     }
       
    //     // Ensures zero
    //     else {
    //       motorLeader.set(0);
    //     }
  
  
    //     // Motor Fights Overshoot
    //     if (motorLeader.getPosition().getValueAsDouble() <= -17.6) {
    //       motorLeader.set(0.1);
    //     }
    //   }, this);
    // }

    // public Command elevatorDefaultCommand() {
    //   return Commands.runOnce(() -> {
    //     // on start
       
    //     // create a position closed-loop request, voltage output, slot 0 configs
    //     final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  
  
    //     // set position to aetpoint rotations
    //     motorLeader.setControl(m_request.withPosition(motorLeader.getPosition().getValueAsDouble()));
   
     
    //   });
    // }
    
    

     @Override
    public void periodic() {
      SmartDashboard.putNumber("Right Elevator Position", motorFollower.getPosition().getValueAsDouble()); //MAY BE WRONG LEFT RIGHT IDK
      SmartDashboard.putNumber("Left Elevator Position", motorLeader.getPosition().getValueAsDouble());
    }
}
