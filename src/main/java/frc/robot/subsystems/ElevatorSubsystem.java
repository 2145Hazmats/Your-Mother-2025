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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
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

    private int levelIndex = 1;
  
    /* Constructor */
    public ElevatorSubsystem() {

      SmartDashboard.putNumber("TESTINPUT", 0); // THIS WORKS 

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.Slot0 = new Slot0Configs().withKP(Constants.elevatorConstants.ElaphantP).withKI(Constants.elevatorConstants.ElaphantI).withKD(Constants.elevatorConstants.ElaphantD);
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // WE DONT KNOW IF THIS IS RIGHT :)
      config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive); //SWSP IF NEEDED
    
      //-------------------------------------------
      motorLeader.getConfigurator().apply(config);
      motorFollower.getConfigurator().apply(config); // MECHANICAL ADVANTAGE DID NOT USE FOLLOWER

      // Sets Follower to follow leader
      motorFollower.setControl(new Follower(motorLeader.getDeviceID(), false));

      SmartDashboard.putNumber("Elevator Level Index", levelIndex);
    }

    public void levelIndexSwitch(boolean up){
      if(up == true) {
          if (levelIndex < 4) { levelIndex++; }
      }
      else {
          if(levelIndex > 1) { levelIndex--; }
      }
      SmartDashboard.putNumber("Elevator Level Index", levelIndex);
  }


 /*  public Command elevatorToIndex() {
    return Commands.run(() -> 
      motorLeader.setControl(new PositionDutyCycle(levelIndex.convertto constants )), this);
    }*/    public Command elevatorToHome() {
      return Commands.run(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.HomePosition)), this);
      }

     public Command elevatorToL1() {
       return Commands.run(() -> 
         motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.L1Position)), this);
       }

    public Command elevatorToL2() {
      return Commands.run(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.L2Position)), this);
      }

    public Command elevatorToL3() {
      return Commands.run(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.L3Position)), this);
      }

    public Command elevatorToL4() {
      return Commands.run(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.L4Position)), this);
      }

    // public Command elevatorToHome() {
    //   return Commands.run(() -> {
    //     if (getElevatorPosition() < Constants.elevatorConstants.HomePosition - Constants.elevatorConstants.MarginOfError) {
    //         motorLeader.set(.3);

    //     }
    //   else if (getElevatorPosition() > Constants.elevatorConstants.HomePosition + Constants.elevatorConstants.MarginOfError){
    //     motorLeader.set(-.3);
    //   } }, this);
    //   }

    /*public Command elevatorToL1() {
      return Commands.run(() -> {
        if (getElevatorPosition() < Constants.elevatorConstants.L1Position - Constants.elevatorConstants.MarginOfError) {
            motorLeader.set(.3);

        }
      else if (getElevatorPosition() > Constants.elevatorConstants.L1Position + Constants.elevatorConstants.MarginOfError){
        motorLeader.set(-.3);
      } }, this);
      }*/

      // public Command elevatorToL2() {
      //   return Commands.run(() -> {
      //     if (getElevatorPosition() < Constants.elevatorConstants.L2Position - Constants.elevatorConstants.MarginOfError) {
      //         motorLeader.set(.3);
  
      //     }
      //   else if (getElevatorPosition() > Constants.elevatorConstants.L2Position + Constants.elevatorConstants.MarginOfError){
      //     motorLeader.set(-.3);
      //   } }, this);
      //   }
        
      //   public Command elevatorToL3() {
      //     return Commands.run(() -> {
      //       if (getElevatorPosition() < Constants.elevatorConstants.L3Position - Constants.elevatorConstants.MarginOfError) {
      //           motorLeader.set(.3);
    
      //       }
      //     else if (getElevatorPosition() > Constants.elevatorConstants.L3Position + Constants.elevatorConstants.MarginOfError){
      //       motorLeader.set(-.3);
      //     } }, this);
      //     }
   

      //     public Command elevatorToL4() {
      //       return Commands.run(() -> {
      //         if (getElevatorPosition() < Constants.elevatorConstants.L4Position - Constants.elevatorConstants.MarginOfError) {
      //             motorLeader.set(.3);
      
      //         }
      //       else if (getElevatorPosition() > Constants.elevatorConstants.L4Position + Constants.elevatorConstants.MarginOfError){
      //         motorLeader.set(-.3);
      //       } }, this);
      //       }


    public Command elevatorJoystick(DoubleSupplier joystick) { // MAY NEED TO CHANGE THIS TO DOAGLE SUPPLIER :)()()))
      return Commands.run(() -> motorLeader.setControl(new DutyCycleOut(joystick.getAsDouble()*.4)), this);
    }

    public Command disableElevator() {
      return Commands.run(() -> motorLeader.setControl(new DutyCycleOut(0)), this);
    }

    public Command defaultCommand() {
      return Commands.runOnce(() -> 
        motorLeader.setControl(new PositionDutyCycle(Constants.elevatorConstants.HomePosition)), this).withTimeout(1).andThen(disableElevator());
      }



    //   public void setElevatorPID(double NewKP, double NewKI, double NewKD) {
    //   config.Slot0.kP = NewKP;
    //   config.Slot0.kI = NewKI;
    //   config.Slot0.kI = NewKD;
    //   motorLeader.getConfigurator().apply(config); // NOT SURE IF THIS LINE WORKS MIGHT NEED COMMAND :)
    // }

    // public Command setElevatorPID() {
    //   return Commands.runOnce(() -> {
    //     config.Slot0.kP = SmartDashboard.getNumber("ElevatorP", 0);
    //   config.Slot0.kI = SmartDashboard.getNumber("ElevatorI", 0);;
    //   config.Slot0.kI = SmartDashboard.getNumber("ElevatorD", 0);;
    //     motorLeader.getConfigurator().apply(config);}, this);
    //    // NOT SURE IF THIS LINE WORKS MIGHT NEED COMMAND :)
    // }

    public void test() {
      double output = SmartDashboard.getNumber("TESTINPUT", 0);
      System.out.println(output);
    }

    public void FFPosition(double positionRad, double FF) { // STOEL FROM MECHANICAL ADVANTAGE
      motorLeader.setControl(
      positionTorqueCurrentRequest.withPosition(Units.radiansToRotations(positionRad)).withFeedForward(FF));
    }

    public double getElevatorPosition() {
      return motorLeader.getPosition().getValueAsDouble();
    }

    public Command resetElevatorPosition() {
      return Commands.runOnce(() -> {motorLeader.setPosition(0);
      motorFollower.setPosition(0);}, this);

    }

    public int getLevelIndex() {
      return levelIndex;
      }

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
    
      SmartDashboard.putBoolean("Reef Level 1", false); SmartDashboard.putBoolean("Reef Level 2", false);
      SmartDashboard.putBoolean("Reef Level 3", false); SmartDashboard.putBoolean("Reef Level 4", false);

      if (levelIndex == 1) {SmartDashboard.putBoolean("Reef Level 1", true);}
      else if (levelIndex == 2) {SmartDashboard.putBoolean("Reef Level 2", true);}
      else if (levelIndex == 3) {SmartDashboard.putBoolean("Reef Level 3", true);}
      else if (levelIndex == 4) {SmartDashboard.putBoolean("Reef Level 4", true);}
      SmartDashboard.putNumber("ElevatorP", 0);
      SmartDashboard.putNumber("ElevatorI", 0);
      SmartDashboard.putNumber("ElevatorD", 0);
    }
}
