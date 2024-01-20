// package frc.utils;

// import java.util.ArrayList;
// import java.util.Comparator;
// import java.util.List;

// import org.opencv.core.Mat;
// import org.opencv.core.MatOfPoint;
// import org.opencv.imgproc.Imgproc;

// import edu.wpi.first.vision.VisionPipeline;

// public class CameraVisionPipeline implements VisionPipeline {

//     private MatOfPoint largestContour;

//     @Override
//     public void process(Mat source) {
//         Mat gray = new Mat();
//         Imgproc.cvtColor(source, gray, Imgproc.COLOR_BGR2GRAY);

//         Mat binary = new Mat();
//         Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY);

//         List<MatOfPoint> contours = new ArrayList<>();
//         Mat hierarchy = new Mat();
//         Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

//         if (!contours.isEmpty()) {
//             largestContour = contours.stream()
//                 .max(Comparator.comparingDouble(Imgproc::contourArea))
//                 .orElse(null);
//         }
//     }

//     public MatOfPoint filterContoursOutput() {
//         return largestContour;
//     }
// }
