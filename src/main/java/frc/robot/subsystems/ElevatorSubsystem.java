package frc.robot.subsystems;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    private Slot0Configs slot0Congfigs;
    
  //   private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
  // new PositionTorqueCurrentFOC(0); //MECHANICAL ADVANTAGE POSITION CONTROL WITH FF

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private int player1LevelIndex = 1;
    private int player2LevelIndex = 1;

    private AlgaeSubsystem algae;

    /* Constructor */
    public ElevatorSubsystem(AlgaeSubsystem fakeAlgae) {
      motorLeader.setPosition(0);
      motorFollower.setPosition(0);

      algae = fakeAlgae;

      slot0Congfigs = config.Slot0;

      slot0Congfigs.kS = 0; // Add 0.25 V output to overcome static friction //0.25
      slot0Congfigs.kV = 0; // A velocity target of 1 rps results in 0.12 V output //0.12
      slot0Congfigs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output //0.01
      slot0Congfigs.kP = Constants.elevatorConstants.ElaphantP; // A position error of 2.5 rotations results in 12 V output //4.8
      slot0Congfigs.kI = Constants.elevatorConstants.ElaphantI; // no output for integrated error //0
      slot0Congfigs.kD = Constants.elevatorConstants.ElaphantD; // A velocity error of 1 rps results in 0.1 V output //0.1
      slot0Congfigs.kG = 0; // Defying gravity //0.1
      slot0Congfigs.GravityType = GravityTypeValue.Elevator_Static;
      // set Motion Magic settings
      MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = 125; // Target cruise velocity of 80 rps 
      motionMagicConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds)
      //motionMagicConfigs.MotionMagicJerk = 800; // Target jerk of 1600 rps/s/s (0.1 seconds)

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      //config.Slot0 = new Slot0Configs().withKP(Constants.elevatorConstants.ElaphantP).withKI(Constants.elevatorConstants.ElaphantI).withKD(Constants.elevatorConstants.ElaphantD);
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // WE DONT KNOW IF THIS IS RIGHT :)
     // config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive); //SWSP IF NEEDED why twice?
    
      //-------------------------------------------
      motorLeader.getConfigurator().apply(config);
      motorFollower.getConfigurator().apply(config);

      // Sets Follower to follow leader
      motorFollower.setControl(new Follower(motorLeader.getDeviceID(), false));

      SmartDashboard.putNumber("Elevator Level Index", player2LevelIndex);
    }

  public Command defaultCommand() {
    return Commands.run(() -> {
      //if (algae.isAlgaeAtHome()) {
        motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.HomePosition));
      //}
      if (isElevatorEvenCloserToHome()) {
        disableElevator();
      }
    });
  }

  public Command elevatorToHome() {
      return Commands.run(() -> 
        motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.HomePosition)), this);
      }

      public void elevatorToHomeMethod() {
          motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.HomePosition));
        }

     public Command elevatorToL1() {
       return Commands.run(() -> 
         motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L1Position)), this);
       }

    public Command elevatorToL2() {
      return Commands.run(() -> 
        motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L2Position)), this);
      }

    public Command elevatorToL3() {
      return Commands.run(() -> 
        motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L3Position)), this);
      }

    public Command elevatorToL4() {
      return Commands.run(() -> 
        motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L4Position)), this);
      }

    public void elevatorToLevel(double index) {
      if (index == 1) {
        motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L1Position));
      } else if (index == 2) {
        motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L2Position));
      } else if (index == 3) {
        motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L3Position));
      } else if (index == 4) {
        motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L4Position));
      } else {
        motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.HomePosition));
      }
    }

    // erm why is it here twice??
    // public Command elevatorToIndexCommand() {
    //   return Commands.run(() -> {
    //       if (player1LevelIndex == 1) {
    //         motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L1Position));
    //       } 
    //       else if (player1LevelIndex ==2) {
    //         motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L2Position));
    //       } 
    //       else if (player1LevelIndex ==3) {
    //         motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L3Position));
    //       } 
    //       else if (player1LevelIndex ==4) {
    //         motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L4Position));
    //   }},this);
    //}

    public Command elevatorToP1Index() {
      return Commands.run(() -> {
          if (player1LevelIndex == 1) {
            motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L1Position));
          } 
          else if (player1LevelIndex ==2) {
            motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L2Position));
          } 
          else if (player1LevelIndex ==3) {
            motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L3Position));
          } 
          else if (player1LevelIndex ==4) {
            motorLeader.setControl(m_request.withPosition(Constants.elevatorConstants.L4Position));

      }},this);
    }


    public void elevatorJoystick(double joystick) { 
       motorLeader.setControl(new DutyCycleOut(joystick*Constants.elevatorConstants.ElevatorJoystickSpeedNerf));
    }

    public Command disableElevator() {
      return Commands.run(() -> motorLeader.setControl(new DutyCycleOut(0)), this);
    }
    
    public boolean isDrivingSafeQuestionMark() {
       if (motorLeader.getPosition().getValueAsDouble() >= Constants.elevatorConstants.SAFETY_LEVEL) {
        return true;
       }
      else {
        return false;
      }
    }

    public boolean isElevatorHome() {
      return (motorLeader.getPosition().getValueAsDouble() >= elevatorConstants.NEAR_HOME);
    }

    public boolean isElevatorEvenCloserToHome() {
      return (motorLeader.getPosition().getValueAsDouble() >= elevatorConstants.NEARER_HOME);
    }

    // public Command setElevatorPID() {
    //   return Commands.runOnce(() -> {
    //     config.Slot0.kS = SmartDashboard.getNumber("ElevatorkS", 0);
    //     config.Slot0.kV = SmartDashboard.getNumber("ElevatorkV", 0);
    //     config.Slot0.kA = SmartDashboard.getNumber("ElevatorkA", 0);

    //     config.Slot0.kP = SmartDashboard.getNumber("ElevatorP", 0);
    //     config.Slot0.kI = SmartDashboard.getNumber("ElevatorI", 0);
    //     config.Slot0.kI = SmartDashboard.getNumber("ElevatorD", 0);
    //     config.Slot0.kG = SmartDashboard.getNumber("ElevatorG", 0);

    //     motorLeader.getConfigurator().apply(config);}, this);
    // }

    public double getElevatorPosition() {
      return motorLeader.getPosition().getValueAsDouble();
    }

    public Command resetElevatorPosition() {
      return Commands.runOnce(() -> {motorLeader.setPosition(0);
      motorFollower.setPosition(0);}, this);
    }

  // Index stuff here 
    public void setPlayer1LevelIndex(int index) {
      player1LevelIndex = index;
    }

    public int getPlayer1LevelIndex() {
      return player1LevelIndex;
    }

    public void updateP1levelIndex() { 
      player1LevelIndex = player2LevelIndex;
    }

    public int getPlayer2LevelIndex() {
      return player2LevelIndex;
    }

    public void P2LevelIndexSwitch(boolean up){
      if(up == true) {
          if (player2LevelIndex < 4) { player2LevelIndex++; }
      }
      else {
          if(player2LevelIndex > 1) { player2LevelIndex--; }
      }
      SmartDashboard.putNumber("Elevator Level Index", player2LevelIndex);
  }

  public void P2LevelIndexChoose(int level) {
   if (level == 1) {
    player2LevelIndex = 1;
    } 
    else if (level ==2) {
    player2LevelIndex = 2;
    } 
    else if (level ==3) {
    player2LevelIndex =3;
    } 
    else if (level ==4) {
    player2LevelIndex =4;
    }
  }

  // Drivetrain speed limiter
    public double getElevatorSlowSpeed() {
      // If the elevator is high enough...
      if (getElevatorPosition() < elevatorConstants.SAFETY_LEVEL) {
        return -25/(getElevatorPosition() - 25);
      } else { //Else max speed
        return 1;
      }
    }

    // for the robot to serenade us
  public final TalonFX LeftElevatorMotor() {
    return motorLeader;
  }
  public final TalonFX RightElevatorMotor() {
    return motorFollower;
  }

     @Override
    public void periodic() {
      SmartDashboard.putNumber("Right Elevator Position", motorFollower.getPosition().getValueAsDouble()); //MAY BE WRONG LEFT RIGHT IDK
      SmartDashboard.putNumber("Left Elevator Position", motorLeader.getPosition().getValueAsDouble());

      //SmartDashboard.putNumber("getElevatorSlowSpeed", getElevatorSlowSpeed());
      SmartDashboard.putBoolean("Reef Level 1", false);
      SmartDashboard.putBoolean("Reef Level 2", false);
      SmartDashboard.putBoolean("Reef Level 3", false);
      SmartDashboard.putBoolean("Reef Level 4", false);

      if (player2LevelIndex == 1) {SmartDashboard.putBoolean("Reef Level 1", true);}
      else if (player2LevelIndex == 2) {SmartDashboard.putBoolean("Reef Level 2", true);}
      else if (player2LevelIndex == 3) {SmartDashboard.putBoolean("Reef Level 3", true);}
      else if (player2LevelIndex == 4) {SmartDashboard.putBoolean("Reef Level 4", true);}
      // SmartDashboard.putNumber("ElevatorP", 0);
      // SmartDashboard.putNumber("ElevatorI", 0);
      // SmartDashboard.putNumber("ElevatorD", 0);

      SmartDashboard.putNumber("getPlayer1LevelIndex", getPlayer1LevelIndex());
      SmartDashboard.putNumber("getPlayer2LevelIndex", player2LevelIndex);
    }

}