package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensation extends SubsystemBase
{
    //NOTE: DIGITAL INPUT RETURNS TRUE FOR BEAM NOT BROKEN AND FALSE FOR BROKEN BEAM
    //      SO IT REPORTING NEGATED VALUE FOR DETECTION OF SOMETHING 
    DigitalInput enterBeam;
    DigitalInput hopperBackBeam;
    DigitalInput hopperFrontBeam;
    DigitalInput exitBeam;

    public Sensation()
    {
        

        enterBeam = new DigitalInput(Constants.SensationConstants.enter);
        hopperBackBeam = new DigitalInput(Constants.SensationConstants.hopperBack);
        hopperFrontBeam = new DigitalInput(Constants.SensationConstants.hopperFront);
        exitBeam = new DigitalInput(Constants.SensationConstants.exit);
    }

    public void periodic() {
        Logger.recordOutput("Sensation/coralPresent", coralPresent());
        Logger.recordOutput("Sensation/coralInHopper", coralInHopper());
        Logger.recordOutput("Sensation/coralExitedHopper", coralExitedHopper());
    }

    public boolean coralPresent() {
        return !enterBeam.get() || !hopperBackBeam.get() || !hopperFrontBeam.get();
    }

    public boolean coralInHopper() {
        return !hopperBackBeam.get() || !hopperFrontBeam.get();
    }

    public boolean coralExitedHopper() {
        return !exitBeam.get() && hopperBackBeam.get() && hopperFrontBeam.get();
    }
}