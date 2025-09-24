package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sensation;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class WaitForCoral extends Command {

    Sensation sensation;
    SwerveSubsystem swerve;
    ChassisSpeeds requiredSpeeds;

    public WaitForCoral(Sensation sensation, SwerveSubsystem swerve) {
      this.swerve = swerve;
      addRequirements(swerve);
      this.sensation = sensation;    
      requiredSpeeds = new ChassisSpeeds(0, 0, 0);
    }

    @Override
    public void execute() {
      swerve.drive(requiredSpeeds);
      Logger.recordOutput("Drive/CurrentCommand", "WaitingForCoral");
    }

  @Override
  public boolean isFinished()
  {
    return sensation.coralPresent();
  }

  @Override
  public void end(boolean interrupted)
  {
    
  }

    
}
