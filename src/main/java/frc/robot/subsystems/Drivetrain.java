package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DrivetrainConstants;

import java.util.HashMap;
import java.util.List;

import static frc.robot.Constants.DrivetrainConstants.*;


public class Drivetrain extends SubsystemBase {
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;
        private SwerveDriveOdometry odometry;
        private SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];
        private List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("NewPath", PATH_CONSTRAINTS);
        private Field2d field2d = new Field2d();
        SwerveModule[] module = new SwerveModule[4];
        private final AHRS m_navx = new AHRS(Port.kMXP);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Front right
                new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back left
                new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back right
                new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));
        public final HashMap<String, Command> eventMap = new HashMap<>();

        //Object to build autos
        public final SwerveAutoBuilder autoBuilder;

        // Creates our swerve kinematics using the robots track width and wheel base
        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public Drivetrain() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
                // The module has two NEOs on it. One for steering and one for driving.
                m_frontLeftModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET);
                m_frontRightModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);
                m_backLeftModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Left Modul", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET);
                m_backRightModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

                // Adding modules to array so we can get positions easier
                module[0] = m_frontLeftModule;
                module[1] = m_frontRightModule;
                module[2] = m_backLeftModule;
                module[3] = m_backRightModule;

                zeroGyroscope();
                zeroPositions();

                odometry = new SwerveDriveOdometry(m_kinematics,
                                Rotation2d.fromDegrees(m_navx.getFusedHeading()), modulePosition);
                eventMap.put("wait",new WaitCommand(2));

                autoBuilder = new SwerveAutoBuilder(
                        this::getPose,
                        this::ResetPose,
                        m_kinematics,
                        XY_CONTROLLER,
                        THEATA_CONTROLLER,
                        this::setStates,
                        eventMap,
                        false,
                        this);
        }

        // Positions
        // Get distance the bot has traveled in meters relative to the starting position

        double driveDistience[] = {0,0,0,0};
        public SwerveModulePosition getPosition(int moduleNumber) {
                //FIXME
//                return new SwerveModulePosition(
//                                (module[moduleNumber].getDriveEncoder().getPosition() *
//                                                (DrivetrainConstants.WHEEL_DIAMETER
//                                                                * Math.PI / (DrivetrainConstants.GEAR_RATIO * 2048.0))),
//                                new Rotation2d(module[moduleNumber].getSteerAngle()));
                driveDistience[moduleNumber] = driveDistience[moduleNumber] + loggingStates[moduleNumber].speedMetersPerSecond * .020;
                SmartDashboard.putNumberArray("Traveled Distence", driveDistience);
                return new SwerveModulePosition(
                        driveDistience[moduleNumber],
                        loggingStates[moduleNumber].angle
                );

        }

        public void zeroPositions() {
                modulePosition[0] = new SwerveModulePosition();
                modulePosition[1] = new SwerveModulePosition();
                modulePosition[2] = new SwerveModulePosition();
                modulePosition[3] = new SwerveModulePosition();
        }

        // Gyroscope
        public void zeroGyroscope() {
                m_navx.zeroYaw();
        }

        public Rotation2d getGyroscopeRotation() {
                return m_navx.getRotation2d();
        }

        // Pose
        public void ResetPose(Pose2d pos) {
                System.out.println(pos.getX());
                System.out.println(pos.getY());
                System.out.println(pos.getRotation().getDegrees());
                odometry.resetPosition(getGyroscopeRotation(), modulePosition, pos);
        }

        private void updatePose() {
                odometry.update(getGyroscopeRotation(), modulePosition);
        }

        public Pose2d getPose() {

                Pose2d tmp = odometry.getPoseMeters();
                return tmp;
        }

        // Literal Speeds
        private void setSpeeds(ChassisSpeeds chassisSpeeds) {
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

                // Update virtual states
                setStates(states);
        }

        SwerveModuleState[] loggingStates = { //FIXME
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };


        // Virtual Speeds
        public void setStates(SwerveModuleState[] state) {
                loggingStates = state;


                m_frontLeftModule.set(
                                (state[0].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                                                * DrivetrainConstants.MAX_VOLTAGE)
                                                * DrivetrainConstants.SPEED_LIMIT,
                                state[0].angle.getRadians());
                m_frontRightModule.set(
                                (state[1].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                                                * DrivetrainConstants.MAX_VOLTAGE)
                                                * DrivetrainConstants.SPEED_LIMIT,
                                state[1].angle.getRadians());
                m_backLeftModule.set(
                                (state[2].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                                                * DrivetrainConstants.MAX_VOLTAGE)
                                                * DrivetrainConstants.SPEED_LIMIT,
                                state[2].angle.getRadians());
                m_backRightModule.set(
                                (state[3].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                                                * DrivetrainConstants.MAX_VOLTAGE)
                                                * DrivetrainConstants.SPEED_LIMIT,
                                state[3].angle.getRadians());

                modulePosition[0] = getPosition(0);
                modulePosition[1] = getPosition(1);
                modulePosition[2] = getPosition(2);
                modulePosition[3] = getPosition(3);
                updatePose();

        }

        public SwerveDriveKinematics getKinematics() {
                return this.m_kinematics;
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        @Override
        public void periodic() {
                if (!DriverStation.isAutonomous())
                {
                        setSpeeds(m_chassisSpeeds);
                }

                SmartDashboard.putNumber("Front Left Speed", loggingStates[0].speedMetersPerSecond);
                SmartDashboard.putNumber("Front Left Angle", loggingStates[0].angle.getDegrees());

                SmartDashboard.putNumber("Front Right Speed", loggingStates[1].speedMetersPerSecond);
                SmartDashboard.putNumber("Front Right Angle", loggingStates[1].angle.getDegrees());

                SmartDashboard.putNumber("Back Left Speed", loggingStates[2].speedMetersPerSecond);
                SmartDashboard.putNumber("Back Left Angle", loggingStates[2].angle.getDegrees());

                SmartDashboard.putNumber("Back Right Speed", loggingStates[3].speedMetersPerSecond);
                SmartDashboard.putNumber("Back Right Angle", loggingStates[3].angle.getDegrees());
                SmartDashboard.putData("field", field2d);
                SmartDashboard.putNumberArray("Robot Location", new double[]{
                        odometry.getPoseMeters().getX(),
                        odometry.getPoseMeters().getY(),
                        odometry.getPoseMeters().getRotation().getDegrees()
                });

                field2d.setRobotPose(getPose());
        }

        public Command forward() {
                driveDistience = new double[]{0,0,0,0};

                return autoBuilder.fullAuto(path);
        }
}
