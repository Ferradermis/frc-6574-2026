package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class ShakeTheFuel extends SequentialCommandGroup {

    public ShakeTheFuel() {
        addCommands(
            RobotContainer.intakePivot.setAngle(Degrees.of(27)).withTimeout(0.35),
            RobotContainer.intakePivot.setAngle(Degrees.of(90)).withTimeout(0.35),
            RobotContainer.intakePivot.setAngle(Degrees.of(27)).withTimeout(0.35)
        );
    }
    
}
