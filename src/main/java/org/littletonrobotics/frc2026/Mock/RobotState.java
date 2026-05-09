package org.littletonrobotics.frc2026.Mock;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import lombok.Getter;
import lombok.Setter;

public class RobotState {
    public static RobotState mInstance = new RobotState();

    public static RobotState getInstance() {
        return mInstance;
    }

    public Pose2d getEstimatedPose() {
        return Drive.mInstance.getPose();
    }

    public ChassisSpeeds getRobotVelocity() {
        return Drive.mInstance.getState().Speeds;
    }

    public ChassisSpeeds getFieldVelocity() {
        return Drive.mInstance.getFieldRelativeSpeeds();
    }

      @Getter @Setter private ChassisSpeeds robotSetpointVelocity = new ChassisSpeeds();

    public ChassisSpeeds getFieldSetpointVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotSetpointVelocity, Drive.mInstance.getHeading());
    }
}
