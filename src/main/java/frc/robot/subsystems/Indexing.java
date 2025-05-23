// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexing extends SubsystemBase {
  private ElevatorSubsystem m_ElevatorSubsystem;
  private CommandSwerveDrivetrain m_drivetrain;
  
  private boolean isP1ManualModeNo = true;
  private boolean isP2ManualModeNo = true;
  public boolean pathFindingInProgress = false;

  private boolean hitAlgaeWithElevator = false;
  /** Creates a new Indexing. */
  public Indexing() {}

  public Indexing(ElevatorSubsystem elevator, CommandSwerveDrivetrain drive) {
    m_ElevatorSubsystem = elevator;
    m_drivetrain = drive;
  }
  public void firstTeleopPieceChoice(int yourChoice) {
  m_drivetrain.setP1Index(yourChoice); 
  m_drivetrain.setP2Index(yourChoice);}

  public void elevatorIndexChooser(int level) {
    m_ElevatorSubsystem.P2LevelIndexChoose(level);
  }

  public void poseIndexSwitch(boolean clockwise) {
    m_drivetrain.poseIndexSwitch(clockwise);
  }

  private int storedP2LevelIndex = -1;
  private int storedP2Index = -1;
  public void updateP1Index() {
    algaeMode = false;

    if (pathFindingInProgress) {
      storedP2LevelIndex = m_ElevatorSubsystem.getPlayer2LevelIndex();
      storedP2Index = m_drivetrain.getPlayer2ReefIndex();
    } else if (storedP2LevelIndex == -1 || storedP2Index == -1) {
      m_ElevatorSubsystem.updateP1levelIndex();
      m_drivetrain.updateP1Index();
    } else {
      m_ElevatorSubsystem.setPlayer1LevelIndex(storedP2LevelIndex);
      m_drivetrain.setP1Index(storedP2Index);
      storedP2LevelIndex = -1;
      storedP2Index = -1;
    }
  }

  public boolean algaeMode = false;
  public void updateP1IndexAlgaeEdition() {
    algaeMode = true;

    int drivetrainIndexAlgaeEdition = m_drivetrain.getPlayer2ReefIndex();
    if ((drivetrainIndexAlgaeEdition % 2) == 1) { // if drivetrainIndexAlgaeEdition is odd
      drivetrainIndexAlgaeEdition = drivetrainIndexAlgaeEdition - 1;
    }

    if (pathFindingInProgress) {
      storedP2LevelIndex = m_ElevatorSubsystem.getPlayer2LevelIndex();
      storedP2Index = m_drivetrain.getPlayer2ReefIndex();
      if ((storedP2Index % 2) == 1) { // if storedP2Index is odd
        storedP2Index = storedP2Index - 1;
      }
    } else if (storedP2LevelIndex == -1 || storedP2Index == -1) {
      m_ElevatorSubsystem.setPlayer1LevelIndex(drivetrainIndexAlgaeEdition);
      m_drivetrain.setP1Index(drivetrainIndexAlgaeEdition);
    } else {
      m_ElevatorSubsystem.setPlayer1LevelIndex(storedP2LevelIndex);
      m_drivetrain.setP1Index(storedP2Index);
      storedP2LevelIndex = -1;
      storedP2Index = -1;
    }
  }

  public Command updateP1IndexInRepeatingCommand() {
    return Commands.runOnce(() -> {
      pathFindingInProgress = false;
      updateP1Index();
      pathFindingInProgress = true;
    });
  }
  
  //P1
  public void setP1ManualModeNo() {
    isP1ManualModeNo = true;
  }

  public void setP1ManualModeYes() {
    isP1ManualModeNo = false;
  }
  public boolean isP1ManualModeFalse() {
        return isP1ManualModeNo;
    }
    //P2 
  public void setP2ManualModeNo() {
      isP2ManualModeNo = true;
    }
  
  public void setP2ManualModeYes() {
      isP2ManualModeNo = false;
    }
  public boolean isP2ManualModeFalse() {
      return isP2ManualModeNo;
  }

  public boolean getHitAlgaeWithElevator() {
    return hitAlgaeWithElevator;
  }

  public void setHitAlgaeWithElevator(boolean b) {
    hitAlgaeWithElevator = b;
  }

  public Command SettingReefIndexBasedOnController(DoubleSupplier RightX, DoubleSupplier RightY) {
    return Commands.run(() -> {
      Double Y = MathUtil.applyDeadband(RightY.getAsDouble(), 0.25);
      Double X = MathUtil.applyDeadband(RightX.getAsDouble(), 0.25);
      double thetaRad = Math.atan2(Y, X);
      int newindex = 0;

      if (X != 0 || Y != 0) {
        if (thetaRad <= (-5 * Math.PI) / 6) {newindex = 9;}
        else if (thetaRad <= (-4 * Math.PI) / 6) {newindex = 8;}
        else if (thetaRad <= (-3 * Math.PI) / 6) {newindex = 7;}
        else if (thetaRad <= (-2 * Math.PI) / 6) {newindex = 6;}
        else if (thetaRad <= (-1 * Math.PI) / 6) {newindex = 5;}
        else if (thetaRad <= (0 * Math.PI) / 6) {newindex = 4;}
        else if (thetaRad <= (1 * Math.PI) / 6) {newindex = 3;}
        else if (thetaRad <= (2 * Math.PI) / 6) {newindex = 2;}
        else if (thetaRad <= (3 * Math.PI) / 6) {newindex = 1;}
        else if (thetaRad <= (4 * Math.PI) / 6) {newindex = 0;}
        else if (thetaRad <= (5 * Math.PI) / 6) {newindex = 11;}
        else if (thetaRad <= (6 * Math.PI) / 6) {newindex = 10;}
        m_drivetrain.setP2Index(newindex);
      }

    }, this);
  }
}
