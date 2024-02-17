package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision {
    class SamplePipeline extends OpenCvPipeline {
int current = 0;
        public int[] pointsX = new int[]{250, 340, 50, 175};
        public int[] pointsY = new int[]{30, 100, 20, 75};
        Mat ycrcbMat = new Mat();
        Mat right = new Mat();
        Mat center = new Mat();
        public Scalar bluescalarlow = new Scalar(0, 0, 130);

        public Scalar redscalarlow = new Scalar(0, 147, 0);
        public Scalar bluescalarhigh = new Scalar(255, 255, 255);
        public Scalar redscalarhigh = new Scalar(255, 255, 255);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(input, new Point(pointsX[0], pointsY[0]), new Point(pointsX[1], pointsY[1]), new Scalar(255, 0, 0));
            Imgproc.rectangle(input, new Point(pointsX[2], pointsY[2]), new Point(pointsX[3], pointsY[3]), new Scalar(255, 0, 0));
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
            center = ycrcbMat.submat(new Rect(pointsX[0], pointsY[0], pointsX[1] - pointsX[0], pointsY[1] - pointsY[0]));
            right = ycrcbMat.submat(new Rect(pointsX[2], pointsY[2], pointsX[3] - pointsX[2], pointsY[3] - pointsY[2]));
            Core.mean(right);
            Core.mean(center);
            double[] rightmean = Core.mean(right).val;
            double[] centermean = Core.mean(center).val;
            if (rightmean[0]> redscalarlow.val[0] && rightmean[0]<redscalarhigh.val[0]){
                if (rightmean[1] > redscalarlow.val[1]&& rightmean[1] < redscalarhigh.val[1]){
                    if (rightmean[2] > redscalarlow.val[2]&& rightmean[2] < redscalarhigh.val[2]){
                        Imgproc.rectangle(input, new Point(pointsX[2], pointsY[2]), new Point(pointsX[3], pointsY[3]), new Scalar(0,255,255),1);
                        Imgproc.putText(input, "right", new Point(input.width()/2, input.height()/2),0,5, new Scalar(0,255,0));
                        vars.Autorandom = AutoRandom.right;
                        current = 1;
                    }
                }
            }

if (current != 1){
    if (centermean[0] > redscalarlow.val[0] && centermean[0] < redscalarhigh.val[0]){
        if (centermean[1] > redscalarlow.val[1] && centermean[1] < redscalarhigh.val[1]){
            if (centermean[2] > redscalarlow.val[2] && centermean[2] < redscalarhigh.val[2]){
                Imgproc.rectangle(input, new Point(pointsX[0], pointsY[0]), new Point(pointsX[1], pointsY[1]), new Scalar(0,255,255),1);
                Imgproc.putText(input, "center", new Point(input.width()/2, input.height()/2),0,5, new Scalar(0,255,0));
                vars.Autorandom = AutoRandom.center;
                current = 1;

            }
        }
    }
}
if (current == 0){
    Imgproc.putText(input, "left", new Point(input.width()/2, input.height()/2),0,5, new Scalar(0,255,0));
    vars.Autorandom = AutoRandom.left;
}
current =0;
ycrcbMat.release();
right.release();
center.release();
            return input;

        }
    }
}
