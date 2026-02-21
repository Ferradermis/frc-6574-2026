package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class StowIntake extends SequentialCommandGroup {

    public StowIntake() {
        addCommands(
            RobotContainer.intakePivot.setAngle(Degrees.of(40)).withTimeout(0.5),
            RobotContainer.fuelRamp.setAngle(Degrees.of(-18)).withTimeout(0.25)
        );
    }
    
}
