package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.elevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX motorLeader = new TalonFX(elevatorConstants.motorLeaderID);
    private TalonFX motorFollower = new TalonFX(elevatorConstants.motorFollowerID);

    private TalonFXConfiguration config = new TalonFXConfiguration();
    
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
  new PositionTorqueCurrentFOC(0); //MECHANICAL ADVANTAGE POSITION CONTROL WITH FF

    public int levelIndex = 0;
    
      /* Constructor */
    public ElevatorSubsystem() {

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.Slot0 = new Slot0Configs().withKP(Constants.elevatorConstants.ElaphantP).withKI(Constants.elevatorConstants.ElaphantI).withKD(Constants.elevatorConstants.ElaphantD);
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // WE DONT KNOW IF THIS IS RIGHT :)
      config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive); //SWSP IF NEEDED
    
      //-------------------------------------------
      motorLeader.getConfigurator().apply(config);
      //motorFollower.getConfigurator().apply(config); // MECHANICAL ADVANTAGE DID NOT USE FOLLOWER

      // Sets Follower to follow leader
      motorFollower.setControl(new Follower(motorLeader.getDeviceID(), false));
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


    public Command elevatorJoystick(double joystick) { // MAY NEED TO CHANGE THIS TO DOAGLE SUPPLIER :)()()))
      return Commands.run(() -> motorLeader.setControl(new DutyCycleOut(joystick*.4)), this);
    }

    public Command disableElevator() {
      return Commands.run(() -> motorLeader.setControl(new DutyCycleOut(0)), this);
    }

    public Command defaultCommand() {
      return Commands.runOnce(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.HomePosition)), this).withTimeout(1).andThen(disableElevator());
      }



      public void setElevatorPID(double NewKP, double NewKI, double NewKD) {
      config.Slot0.kP = NewKP;
      config.Slot0.kI = NewKI;
      config.Slot0.kI = NewKD;
      motorLeader.getConfigurator().apply(config); // NOT SURE IF THIS LINE WORKS MIGHT NEED COMMAND :)
    }

    public void FFPosition(double positionRad, double FF) { // STOEL FROM MECHANICAL ADVANTAGE
      motorLeader.setControl(
      positionTorqueCurrentRequest.withPosition(Units.radiansToRotations(positionRad)).withFeedForward(FF));
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
