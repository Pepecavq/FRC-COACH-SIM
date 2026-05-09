package org.littletonrobotics.frc2026.Mock;

import java.lang.reflect.Field;
import java.util.HashMap;

//import org.littletonrobotics.frc2026.subsystems.launcher.hood.HoodIO.HoodIOInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger {

    private static final HashMap<String, DoublePublisher> doublePublishers = new HashMap<>();
    public static void recordOutput(String key, double value) {
        if(!doublePublishers.containsKey(key)) {
            DoublePublisher publisher = NetworkTableInstance.getDefault().getDoubleTopic(key).publish();
            doublePublishers.put(key, publisher);
        }
        doublePublishers.get(key).set(value);
    }

    private static final HashMap<String, BooleanPublisher> booleanPublishers = new HashMap<>();
    public static void recordOutput(String key, boolean value) {
        if(!booleanPublishers.containsKey(key)) {
            BooleanPublisher publisher = NetworkTableInstance.getDefault().getBooleanTopic(key).publish();
            booleanPublishers.put(key, publisher);
        }
        booleanPublishers.get(key).set(value);
    }

    private static final HashMap<String, StructPublisher<Pose2d>> posePublishers = new HashMap<>();
    public static void recordOutput(String key, Pose2d pose) {
        if(!posePublishers.containsKey(key)) {
            StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic(key, Pose2d.struct).publish();
            posePublishers.put(key, publisher);
        }
        posePublishers.get(key).set(pose);
    }

    private static final HashMap<String, StructPublisher<Translation2d>> translation2dPublishers = new HashMap<>();
    public static void recordOutput(String key, Translation2d translation) {
        if(!translation2dPublishers.containsKey(key)) {
            StructPublisher<Translation2d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic(key, Translation2d.struct).publish();
            translation2dPublishers.put(key, publisher);
        }
        translation2dPublishers.get(key).set(translation);
    }

    private static final HashMap<String, StructPublisher<Rotation2d>> rotationPublishers = new HashMap<>();
    public static void recordOutput(String key, Rotation2d rot) {
        if(!rotationPublishers.containsKey(key)) {
            StructPublisher<Rotation2d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic(key, Rotation2d.struct).publish();
            rotationPublishers.put(key, publisher);
        }
        rotationPublishers.get(key).set(rot);
    }

    private static final HashMap<String, StructPublisher<Rotation3d>> rotation3dPublishers = new HashMap<>();
    public static void recordOutput(String key, Rotation3d rot) {
        if(!rotation3dPublishers.containsKey(key)) {
            StructPublisher<Rotation3d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic(key, Rotation3d.struct).publish();
            rotation3dPublishers.put(key, publisher);
        }
        rotation3dPublishers.get(key).set(rot);
    }
    
    private static final HashMap<String, StructPublisher<Pose3d>> pose3dPublishers = new HashMap<>();
    public static void recordOutput(String key, Pose3d pose) {
        if(!pose3dPublishers.containsKey(key)) {
            StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic(key, Pose3d.struct).publish();
            pose3dPublishers.put(key, publisher);
        }
        pose3dPublishers.get(key).set(pose);
    }

    private static final HashMap<String, StructArrayPublisher<Pose2d>> arrayPose2dPublishers = new HashMap<>();
    public static void recordOutput(String key, Pose2d[] poseArray) {
        if(!arrayPose2dPublishers.containsKey(key)) {
            StructArrayPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
			.getStructArrayTopic(key, Pose2d.struct).publish();
            arrayPose2dPublishers.put(key, publisher);
        }
        arrayPose2dPublishers.get(key).set(poseArray);
    }

    private static final HashMap<String, StructArrayPublisher<Pose3d>> arrayPose3dPublishers = new HashMap<>();
    public static void recordOutput(String key, Pose3d[] poseArray) {
        if(!arrayPose3dPublishers.containsKey(key)) {
            StructArrayPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
			.getStructArrayTopic(key, Pose3d.struct).publish();
            arrayPose3dPublishers.put(key, publisher);
        }
        arrayPose3dPublishers.get(key).set(poseArray);
    }

    private static final HashMap<String, StringPublisher> stringPublishers = new HashMap<>();
    public static void recordOutput(String key, String value) {
        if(!stringPublishers.containsKey(key)) {
            StringPublisher publisher = NetworkTableInstance.getDefault().getStringTopic(key).publish();
            stringPublishers.put(key, publisher);
        }
        stringPublishers.get(key).set(value);
    }
    
    public static void processInputs(String key, Object inputs) {
        Field[] fields = inputs.getClass().getDeclaredFields();
        for (Field field : fields) {
            try {
                field.setAccessible(true);
                String name = field.getName();
                Object value = field.get(inputs); 
                if(value instanceof Double) {
                    recordOutput(key + "/" + name, (Double) value);
                } else if (value instanceof Boolean) {
                    recordOutput(key + "/" + name, (Boolean) value);
                }

            } catch (IllegalAccessException e) {
                System.err.println("Could not access field: " + field.getName());
            }
        }
    }
         
}
