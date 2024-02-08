package frc.robot;

//import java.util.List;
import java.util.OptionalInt;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.pegartrajeto;

public class RobotContainer {

        private pegartrajeto pegartrajeto;

        //coloque seus subsystems aqui
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
        static XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort);
        
    public RobotContainer() {
        OptionalInt station = DriverStation.getLocation();
        pegartrajeto = new pegartrajeto(station);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(//essa linha de código seta o command do swerve para sempre atuar
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),// driverJoystick.getLeftY(),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),//driverJoystick.getLeftX(),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),//driverJoystick.getRightX(),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx) /* driverJoystick.getAButton()*/));
                
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, 2).onTrue(swerveSubsystem.runOnce(swerveSubsystem::zeroHeading));
        
       
    }
       
    

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
       /* TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);*/

        // 2. Generate trajectory
        Trajectory trajectory = pegartrajeto.getTrajectory();/*TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),//ponto inicial
                List.of(
                        new Translation2d(1, 0),//pontos intermediarios
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),//ponto final
                trajectoryConfig);*/

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
