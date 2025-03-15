// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexing extends SubsystemBase {
  private ElevatorSubsystem m_ElevatorSubsystem;
  private CommandSwerveDrivetrain m_drivetrain;
  

  private boolean isP1ManualModeNo = true;
  private boolean isP2ManualModeNo = true;
  public boolean pathFindingInProgress = false;
  /** Creates a new Indexing. */
  public Indexing() {}

  public Indexing(ElevatorSubsystem elevator, CommandSwerveDrivetrain drive) {
    m_ElevatorSubsystem = elevator;
    m_drivetrain = drive;
  }

  public void elevatorIndexChooser(int level) {
    m_ElevatorSubsystem.levelIndexChoose(level);
  }

  public void poseIndexSwitch(boolean clockwise) {
    m_drivetrain.poseIndexSwitch(clockwise);
  }

  private int storedP2LevelIndex = -1;
  private int storedP2Index = -1;
  public void updateP1Index() {
    if (pathFindingInProgress) {
      storedP2LevelIndex = m_ElevatorSubsystem.getPlayer2LevelIndex();
      storedP2Index = m_drivetrain.getPlayer2ReefIndex();
    } else if (storedP2LevelIndex == -1 || storedP2Index == -1) {
      m_ElevatorSubsystem.updateP1levelIndex();
      m_drivetrain.updateP1Index();
    } else {
      m_ElevatorSubsystem.setPl1LevelIndex(storedP2LevelIndex);
      m_drivetrain.setP1Index(storedP2Index);
      storedP2LevelIndex = -1;
      storedP2Index = -1;
    }
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
}
