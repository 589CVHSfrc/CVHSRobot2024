// package frc.utils;

// import edu.wpi.first.math.util.Units;

// import java.util.Map;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.util.Units;

// public class FieldLayout2024 {
//     public static final double kFieldLength = Units.inchesToMeters(651.25);
//     public static final double kFieldWidth = Units.inchesToMeters(315.5);
//     public static final double kTapeWidth = Units.inchesToMeters(2.0);

//     public static double degreesToRadians(double degrees){
//         return (degrees*Math.PI)/180.0;
//     }
    
//     public static final Map<Integer, Pose3d> aprilTags = Map.of(


//         3,
//         new Pose3d(
//             Units.inchesToMeters(652.73),
//             Units.inchesToMeters(196.17),
//             Units.inchesToMeters(57.13),
//             new Rotation3d(0.0, 0.0, degreesToRadians(180.0))
//         ),

//         4,
//         new Pose3d(
//             Units.inchesToMeters(652.73),
//             Units.inchesToMeters(218.42),
//             Units.inchesToMeters(57.13),
//             new Rotation3d(0.0, 0.0, degreesToRadians(180.0))
//         ),

//         5,
//         new Pose3d(
//             Units.inchesToMeters(578.77),
//             Units.inchesToMeters(323.0),
//             Units.inchesToMeters(53.38),
//             new Rotation3d(0.0, 0.0, degreesToRadians(270.0))
//         ),

//         6,
//         new Pose3d(
//             Units.inchesToMeters(72.5),
//             Units.inchesToMeters(323.0),
//             Units.inchesToMeters(53.38),
//             new Rotation3d(0.0, 0.0, degreesToRadians(270.0))
//             ),

//         7,
//         new Pose3d(
//             Units.inchesToMeters(-1.5),
//             Units.inchesToMeters(218.42),
//             Units.inchesToMeters(57.13),
//             new Rotation3d(0.0, 0.0, degreesToRadians(0.0))
//         ),

//         8,
//         new Pose3d(
//             Units.inchesToMeters(-1.5),
//             Units.inchesToMeters(196.17),
//             Units.inchesToMeters(57.13),
//             new Rotation3d(0.0, 0.0, degreesToRadians(0.0))
//         )
//     );

// }
