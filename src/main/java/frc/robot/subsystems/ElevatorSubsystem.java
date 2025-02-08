package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX motorLeader = new TalonFX(elevatorConstants.motorLeaderID);
    private TalonFX motorFollower = new TalonFX(elevatorConstants.motorFollowerID);

    private TalonFXConfiguration config = new TalonFXConfiguration();

    public int levelIndex = 0;
    
      /* Constructor */
    public ElevatorSubsystem() {
      motorLeader.getConfigurator().apply(config);
      motorFollower.getConfigurator().apply(config);
      
      // config.MotorOutput.Inverted FOUND MOTOR INVERSION!!!!

      // Sets Follower to follow leader
      motorFollower.setControl(new Follower(elevatorConstants.motorLeaderID, false));
    }

    public void levelIndexSwitch(boolean up){
      if(up == true) {
          if (levelIndex == 0) { levelIndex = 3; }
          else { levelIndex--; }
      }
      else {
          if(levelIndex == 3) { levelIndex = 0; }
          else{ levelIndex++; }
      }
  }

     @Override
    public void periodic() {
       
    }
}
