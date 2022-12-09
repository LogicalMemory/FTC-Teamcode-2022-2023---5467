package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

public class CompVision {
    private OpenCvCamera[] cams;
    private int camNum = 0;
    private int path = -1;
    private ArrayList<Cluster> cubes = new ArrayList<>();

    public CompVision(HardwareMap map, int n) {
        camNum = n;
        cams = new OpenCvCamera[camNum];
        int monId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());

        if (camNum > 1) {
            int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                    .splitLayoutForMultipleViewports(
                            monId, //The container we're splitting
                            camNum, //The number of sub-containers to create
                            OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY
                    ); //Whether to split the container vertically or horizontally
            for (int i = 0; i < camNum; i++) {
                cams[i] = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam " + (i + 1)), viewportContainerIds[i]);
            }

        } else {
            cams[0] = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 1"));
        }

        if (camNum > 0) {
            cams[0].openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    cams[0].setPipeline(new SignalPipeLine());
                    cams[0].startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
            if (camNum > 1) {
                cams[1].openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        cams[1].setPipeline(new TestPipeLine());
                        cams[1].startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {

                    }
                });
                if (camNum > 2) {
                    cams[2].openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                        @Override
                        public void onOpened() {
                            cams[2].setPipeline(new TestPipeLine());
                            cams[2].startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                        }

                        @Override
                        public void onError(int errorCode) {
                        }
                    });

                    if (camNum > 3) {
                        cams[3].openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                            @Override
                            public void onOpened() {
                                cams[3].setPipeline(new TestPipeLine());
                                cams[3].startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                            }

                            @Override
                            public void onError(int errorCode) {
                            }
                        });
                    }
                }
            }
        }
    }

    public void printStats(Telemetry t) {
        for (int i = 0; i < camNum; i++) {
            try {
                t.addLine("Cam" + (i + 1));
                t.addData("Frame Count", cams[i].getFrameCount());
                t.addData("FPS", String.format("%.2f", cams[i].getFps()));
                t.addData("Total frame time ms", cams[i].getTotalFrameTimeMs());
                t.addData("Pipeline time ms", cams[i].getPipelineTimeMs());
            } catch (Exception e) {
                t.addLine("Cam" + (i + 1) + " not available");
            }
        }
    }
    public void printDebugCam1(Telemetry t) {
        t.addLine("Path chosen: " + path);
    }

    public void stopAll() {
        for (int i = 0; i < camNum; i++) {
            cams[i].stopStreaming();
        }
    }
    public void stop(int i) {
        cams[i].stopStreaming();
    }

    /*public void changePipeLine(int i, int pipeline) {
        OpenCvCamera cam = cams[i];
        switch (pipeline) {
            case 0:
                cam.setPipeline(new ShippingElementPipeLine());
                break;
            case 1:
                cam.setPipeline(new CubesPipeLine());
                break;
            case 2:
                cam.setPipeline(new TestPipeLine());
        }
    }*/

    class TestPipeLine extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            return input;
        }
    }

    class SignalPipeLine extends OpenCvPipeline {

        private Cluster biggestCluster = null;

        @Override
        public Mat processFrame(Mat input1) {
            int signal = 0;
            int counter1 = 0;
            int counter2 = 0;



            double a = (255) / 255f;
            double b = (255) / 255f;
            double c = (255) / 255f;
            double d = (255) / 255f;
            //int ly = 306;
            //int lx = 345;
            //int ry = input1.rows() - 33;
            //int rx = input1.cols() - 147;
            int ly = (int)(a * input1.rows());
            int lx = (int)(b * input1.cols());
            int ry = (int)(c * input1.rows());
            int rx = (int)(d * input1.cols());

            Mat fView = new Mat(ry - ly, rx - lx, input1.type(), new Scalar(0));
            for (int i = ly; i < ry; i++) {	   //height
                for (int j = lx; j < rx; j++) { //Width
                    double red = input1.get(i, j)[0];
                    double green = input1.get(i, j)[1];
                    double blue = input1.get(i, j)[2];
                    boolean tot = green > 120;
                    boolean dif = (Math.abs(red - green) + Math.abs(blue - green) + Math.abs(blue - red))/3 > 16;

                    if (dif) {
                        fView.put(i - ly,  j - lx, new double[] {0, 0, 255});
                    }
                    if (tot) {
                        fView.put(i - ly,  j - lx, new double[] {0, 255, fView.get(i - ly, j - lx)[2]});
                    }
                }
            }

            Imgproc.resize(fView, fView, new Size(80, 60));
            findCorners(fView);
            Mat fView2 = new Mat(80, 60, input1.type(), new Scalar(0));

            if (biggestCluster != null)
            {
                lx = biggestCluster.getLow();
                ly = biggestCluster.getLeft();
                rx = biggestCluster.getHigh();
                ry = biggestCluster.getRight();
                fView2 = new Mat(ry - ly, rx - lx, input1.type(), new Scalar(0));
                for (int i = ly; i < ry; i++) {	   //height
                    for (int j = lx; j < rx; j++) { //Width
                        double red = fView.get(i, j)[0];
                        double green = fView.get(i, j)[1];
                        double blue = fView.get(i, j)[2];
                        fView2.put(i - ly,  j - lx, new double[] {red, green, blue});

                    }
                }

                Mat img = new Mat(fView2.rows(), fView2.cols(), fView2.type(), new Scalar(255));

                {
                    for (int i = 0; i < fView2.rows(); i++) {	   //height
                        for (int j = 0; j < fView2.cols(); j++) { //Width
                            double r = fView2.get(i, j)[0];
                            double g = fView2.get(i, j)[1];
                            double blue = fView2.get(i, j)[2];
                            img.put(i,  j, new double[] {r, g, blue});
                        }
                    }
                    Mat structElem = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(a + 1, b + 1));

                    Imgproc.dilate(img, img, structElem, new Point(-1, -1) );
                    Imgproc.erode( img, img, structElem, new Point(-1, -1) );

                    for (int i = 0; i < img.rows(); i++) {	   //height
                        for (int j = 0; j < img.cols(); j++) { //Width
                            double red = img.get(i, j)[0];
                            double green = img.get(i, j)[1];
                            double blue = img.get(i, j)[2];
                            boolean tot = (red + green + blue)/3 < 100;
                            boolean dif = Math.abs(red - green) + Math.abs(blue - green) + Math.abs(blue - red) < 30;

                            if (dif && tot) {
                                //fView.put(i,  j, new double[] {255, fView.get(i, j)[1], fView.get(i, j)[2]});
                                counter1++;
                            }
                        }
                    }
                }
			/*{
				for (int i = 0; i < fView2.rows(); i++) {	   //height
					for (int j = 0; j < fView2.cols(); j++) { //Width
						double r = fView2.get(i, j)[0];
						double g = fView2.get(i, j)[1];
						double blue = fView2.get(i, j)[2];
						img.put(i,  j, new double[] {r, g, blue});
					}
				}
				Mat structElem = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(a + 1, b + 1));

				Imgproc.dilate(img, img, structElem, new Point(-1, -1) );
				Imgproc.erode( img, img, structElem, new Point(-1, -1) );

				for (int i = 0; i < img.rows(); i++) {	   //height
					for (int j = 0; j < img.cols(); j++) { //Width
						double red = img.get(i, j)[0];
						double green = img.get(i, j)[1];
						double blue = img.get(i, j)[2];
						boolean tot = (red + green + blue)/3 < 100;
						boolean dif = Math.abs(red - green) + Math.abs(blue - green) + Math.abs(blue - red) < 30;

						if (dif && tot) {
							//fView.put(i,  j, new double[] {255, fView.get(i, j)[1], fView.get(i, j)[2]});
							counter2++;
						}
					}
				}
			}*/


            }

            if (counter1 > counter2) { signal = 1; } else { signal = 2; }

            if (Math.abs(counter1 - counter2) < 100) { signal = 3; }

            System.out.println(signal);
            return fView;
        }


        public void findCorners(Mat src) {

            biggestCluster = null;
            ArrayList<Cluster> clusters = new ArrayList<Cluster>();
            for (int i = 0; i < src.rows(); i++) {	   //height
                for (int j = 0; j < src.cols(); j++) { //Width

                    double red = src.get(i, j)[0];
                    double green = src.get(i, j)[1];
                    double blue = src.get(i, j)[2];

                    if (blue > 200 && green > 200 && red < 200) {
                        Cluster c = new Cluster(i, j);
                        clusters.add(c);
                        yellowFill(src, c, i, j);
                    }
                }
            }
            for (int i = 0; i < clusters.size(); i++) {
                Cluster c = clusters.get(i);
                c.clean();
                if (biggestCluster == null || c.getCount() > biggestCluster.getCount()) {
                    biggestCluster = c;
                }

            }

            if (biggestCluster != null) {
                int starSize = 2;
                placeDot(src, biggestCluster.getX(), biggestCluster.getY(), starSize, 255);
                placeDot(src, biggestCluster.getLeft(), biggestCluster.getHigh(), starSize, 100);
                placeDot(src, biggestCluster.getLeft(), biggestCluster.getLow(), starSize, 100);
                placeDot(src, biggestCluster.getRight(), biggestCluster.getHigh(), starSize, 100);
                placeDot(src, biggestCluster.getRight(), biggestCluster.getLow(), starSize, 100);
            }
        }

        public void yellowFill(Mat src, Cluster p, int x, int y) {
            if (!valid(src, x, y)) { return; }
            double red = src.get(x, y)[0];
            double green = src.get(x, y)[1];
            double blue = src.get(x, y)[2];
            if (blue > 200 && green > 200 && red < 200) {
                src.put(x, y, new double[]{255, 0, 0});
                p.add(x, y);
            } else {
                return;
            }

            int tol = 3;

            yellowFill(src, p, x+tol, y);
            yellowFill(src, p, x, y+tol);
            yellowFill(src, p, x, y-tol);
            yellowFill(src, p, x-tol, y);

            yellowFill(src, p, x+1, y);
            yellowFill(src, p, x, y+1);
            yellowFill(src, p, x, y-1);
            yellowFill(src, p, x-1, y);
        }

        public boolean valid(Mat src, int x, int y) {
            return !(x < 0 || y < 0 || x >= src.rows() || y >= src.cols() || src.get(x, y) == null);
        }

        public void placeDot(Mat mat, int x, int y, int dist, int color) {
            if (!valid(mat, x, y) || dist < 0) { return; }
            mat.put(x,  y, new double[] {color, color, color});
            placeDot(mat, x-1, y, dist-1, color);
            placeDot(mat, x+1, y, dist-1, color);
            placeDot(mat, x, y-1, dist-1, color);
            placeDot(mat, x, y+1, dist-1, color);
        }
    }
}
