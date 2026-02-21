package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class GetBestAngle extends Command {
    
    Drive driveSubsystem;
    Pose2d botPose;
    Pose3d hubCenter = new Pose3d();

    public GetBestAngle(Drive drive) {
        driveSubsystem = drive;
        addRequirements(RobotContainer.shooterPivot);
    }

    @Override
    public void initialize() {
        Alliance alliance = DriverStation.getAlliance().get();
        if (alliance == Alliance.Blue) {
        hubCenter = Constants.HUB_CENTER_BLUE;
        }
        else if (alliance == Alliance.Red){
        hubCenter = Constants.HUB_CENTER_RED;
        }
    }

    @Override
    public void execute() {
        // Get pose of bot and hub and get distance between the two
        botPose = driveSubsystem.getPose();
        double robotX = botPose.getX();
        double robotY = botPose.getY();
        double hubX = hubCenter.getX();
        double hubY = hubCenter.getY();
        double xDifference = hubX - robotX;
        double yDifference = hubY - robotY;
        double distance = Math.sqrt(Math.pow(xDifference, 2) + Math.pow(yDifference, 2));

        // Find the angle based on target velocity and the distance from hub and bot using math I don't want to explain here
        double g = 9.8; // Acceleration due to gravity
        double heightDifference = 1.3338; // Difference (in meters) between hub and bot heights
        double velocity = 45; // Target velocity (m/s)
        double thetaRads = Math.atan((Math.pow(velocity, 2) - Math.sqrt(Math.pow(velocity, 4) - (g * ((g * Math.pow(distance, 2)) + (2 * heightDifference * Math.pow(velocity, 2))))) / (g * distance)));
        RobotContainer.shooterPivot.setAngle(Degrees.of(Degrees.convertFrom(thetaRads, Radians)));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
