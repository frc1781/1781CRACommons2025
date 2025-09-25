// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// public class StopMovingToTarget extends Command {
//     SwerveSubsystem drivebase;

//     public StopMovingToTarget(SwerveSubsystem drivebase) {
//         this.drivebase = drivebase;
        
//     }
    
//     @Override
//     public void initialize() {
//             CommandScheduler.getInstance().cancel(new MoveToTarget(drivebase, -1));
//     }

//     @Override
//     public boolean isFinished() {
//         return true;
//     }
    
// }
