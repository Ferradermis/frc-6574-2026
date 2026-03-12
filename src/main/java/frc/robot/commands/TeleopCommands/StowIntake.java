package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class StowIntake extends SequentialCommandGroup {

    public StowIntake() {
        addCommands(
            RobotContainer.intakePivot.setAngle(Degrees.of(45)).withTimeout(0.5),
            RobotContainer.intakeMainRoller.setVelocity(RPM.of(0))
        );
    }
    
}
