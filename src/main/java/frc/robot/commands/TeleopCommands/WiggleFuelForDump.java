package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class WiggleFuelForDump extends SequentialCommandGroup {

    public WiggleFuelForDump() {
        addCommands(
            RobotContainer.intakePivot.setAngle(Degrees.of(10)).withTimeout(0.35),
            RobotContainer.intakePivot.setAngle(Degrees.of(25)).withTimeout(0.35), 
            RobotContainer.intakePivot.setAngle(Degrees.of(10)).withTimeout(0.35)
        );
    }
    
}
