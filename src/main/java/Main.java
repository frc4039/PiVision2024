// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
       "switched cameras": [
           {
               "name": <virtual camera name>
               "key": <network table key used for selection>
               // if NT value is a string, it's treated as a name
               // if NT value is a double, it's treated as an integer index
           }
       ]
   }
 */

public final class Main {
  private static String configFile = "/boot/frc.json";

  @SuppressWarnings("MemberName")
  public static class CameraConfig {
    public String name;
    public String path;
    public JsonObject config;
    public JsonElement streamConfig;
  }

  @SuppressWarnings("MemberName")
  public static class SwitchedCameraConfig {
    public String name;
    public String key;
  };

  public static int team = 4039;
  public static boolean server;
  public static List<CameraConfig> cameraConfigs = new ArrayList<>();
  public static List<SwitchedCameraConfig> switchedCameraConfigs = new ArrayList<>();
  public static List<VideoSource> cameras = new ArrayList<>();

  private static int threadCounter = 0;
  private static long threadCounterTime = 0;
  private static long elapsedThreadTime = 0;
  private static long startTime;


  private Main() {
    
  }

/**
   * Example pipeline.
   
  public static class MyPipeline implements VisionPipeline {
    public int val;

    @Override
    public void process(Mat mat) {
      val += 1;
    }
  }
*/

  /*** Main ***/
  public static void main(String... args) {
    if (args.length > 0) {
      configFile = args[0];
    }
    
    //initialize counter and timer
   startTime = System.currentTimeMillis();
  
    // read configuration
    if (!readConfig()) {
      return;
    }

    // start NetworkTables
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntinst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntinst.startClient4("wpilibpi");
      ntinst.setServerTeam(team);
      ntinst.startDSClient();
    }

    // start cameras
    for (CameraConfig config : cameraConfigs) {
      cameras.add(startCamera(config));
    }

    // start switched cameras
    for (SwitchedCameraConfig config : switchedCameraConfigs) {
      startSwitchedCamera(config);
    }
    CvSource outputStream = CameraServer.putVideo("DetectedObjectFeed", 320, 240);
    // start image processing on camera 0 if present
    if (cameras.size() >= 1) {
      VisionThread visionThread = new VisionThread(cameras.get(0),
  //            new GreenBinGripPL(), pipeline -> {
            new NoteGripPipeline(), pipeline -> {
    
        // do something with pipeline results
        Mat currentFrame = pipeline.maskOutput();
        startTime = System.currentTimeMillis();
        
        if (!pipeline.filterContoursOutput().isEmpty()) {

          int largestContourIndex = 0;
          float largestContourArea = 0;
          int currentIndex = 0;
  
          // Loop through all detected object contours and select the one with the largest area as our main target
          for (MatOfPoint matOfPoint : pipeline.filterContoursOutput()) {
            int currentContourArea = Imgproc.boundingRect(matOfPoint).height * Imgproc.boundingRect(matOfPoint).width;
  
            if (currentContourArea > largestContourArea) {
              largestContourIndex = currentIndex;
              largestContourArea = currentContourArea;
            }
            currentIndex++;
          }
  
          Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(largestContourIndex));
  
  
          int centerX = r.x + (r.width/2);
          int centerY = r.y + (r.height/2);
   
          
          Imgproc.rectangle(currentFrame, r, new Scalar(0, 255,0),5);
          Imgproc.drawContours(currentFrame, pipeline.filterContoursOutput(), largestContourIndex, new Scalar(255, 0, 0));

                  //Update Shuffleboard
        outputStream.putFrame(currentFrame);
        SmartDashboard.putNumber("/PI/Detected Object/xCenter", centerX);
        SmartDashboard.putNumber("/PI/Detected Object/yCenter", centerY);
        SmartDashboard.putNumber("/PI/Detected Object/width", r.width);
        SmartDashboard.putNumber("/PI/Detected Object/height", r.height);
        SmartDashboard.putNumber("/PI/Detected Object/area", r.height*r.width);
        SmartDashboard.putNumber("/PI/Detected Object/Angle", -1);

        } 
        else{
          Imgproc.putText(currentFrame, "No Note detected!!!", new Point(100, 50), 0, 1.0, new Scalar(0, 0, 255), 3);
        //Update Shuffleboard
        outputStream.putFrame(currentFrame);
        SmartDashboard.putNumber("/PI/Detected Object/xCenter", -1);
        SmartDashboard.putNumber("/PI/Detected Object/yCenter", -1);
        SmartDashboard.putNumber("/PI/Detected Object/width", -1);
        SmartDashboard.putNumber("/PI/Detected Object/height", -1);
        SmartDashboard.putNumber("/PI/Detected Object/area", -1);
        SmartDashboard.putNumber("/PI/Detected Object/Angle", -1);
        }


        // Calculate Threads per Second
        threadCounter++;
        elapsedThreadTime = (System.currentTimeMillis() - startTime);
        threadCounterTime = threadCounterTime + elapsedThreadTime;
        if (threadCounterTime < 1) elapsedThreadTime = 1;  // fix divide by 0 below  
        SmartDashboard.putNumber("/PI/Detected Object/Iterations", threadCounter);
        SmartDashboard.putNumber("/PI/Detected Object/ThreadCounterTime", threadCounterTime);
        SmartDashboard.putNumber("/PI/Detected Object/ThreadsperSecond", threadCounter/threadCounterTime*1000);
        SmartDashboard.putNumber("/PI/Detected Object/MilliSecondsPerThread", elapsedThreadTime);
       

       //reset counter every 100 cycles
       if (threadCounter > 100){
        threadCounter = 0;
        threadCounterTime = 0;
       }
         
      });

      visionThread.start();
     
    }

    // loop forever
    for (;;) {
      try {
        Thread.sleep(10000);
      } catch (InterruptedException ex) {
        return;
      }
    }
  }

  /**
   * Report parse error.
   */
  public static void parseError(String str) {
    System.err.println("config error in '" + configFile + "': " + str);
  }

  /**
   * Read single camera configuration.
   */
  public static boolean readCameraConfig(JsonObject config) {
    CameraConfig cam = new CameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement pathElement = config.get("path");
    if (pathElement == null) {
      parseError("camera '" + cam.name + "': could not read path");
      return false;
    }
    cam.path = pathElement.getAsString();

    // stream properties
    cam.streamConfig = config.get("stream");

    cam.config = config;

    cameraConfigs.add(cam);
    return true;
  }

  /**
   * Read single switched camera configuration.
   */
  public static boolean readSwitchedCameraConfig(JsonObject config) {
    SwitchedCameraConfig cam = new SwitchedCameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read switched camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement keyElement = config.get("key");
    if (keyElement == null) {
      parseError("switched camera '" + cam.name + "': could not read key");
      return false;
    }
    cam.key = keyElement.getAsString();

    switchedCameraConfigs.add(cam);
    return true;
  }

  /**
   * Read configuration file.
   */
  @SuppressWarnings("PMD.CyclomaticComplexity")
  public static boolean readConfig() {
    // parse file
    JsonElement top;
    try {
      top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
    } catch (IOException ex) {
      System.err.println("could not open '" + configFile + "': " + ex);
      return false;
    }

    // top level must be an object
    if (!top.isJsonObject()) {
      parseError("must be JSON object");
      return false;
    }
    JsonObject obj = top.getAsJsonObject();

    // team number
    JsonElement teamElement = obj.get("team");
    if (teamElement == null) {
      parseError("could not read team number");
      return false;
    }
    team = teamElement.getAsInt();

    // ntmode (optional)
    if (obj.has("ntmode")) {
      String str = obj.get("ntmode").getAsString();
      if ("client".equalsIgnoreCase(str)) {
        server = false;
      } else if ("server".equalsIgnoreCase(str)) {
        server = true;
      } else {
        parseError("could not understand ntmode value '" + str + "'");
      }
    }

    // cameras
    JsonElement camerasElement = obj.get("cameras");
    if (camerasElement == null) {
      parseError("could not read cameras");
      return false;
    }
    JsonArray cameras = camerasElement.getAsJsonArray();
    for (JsonElement camera : cameras) {
      if (!readCameraConfig(camera.getAsJsonObject())) {
        return false;
      }
    }

    if (obj.has("switched cameras")) {
      JsonArray switchedCameras = obj.get("switched cameras").getAsJsonArray();
      for (JsonElement camera : switchedCameras) {
        if (!readSwitchedCameraConfig(camera.getAsJsonObject())) {
          return false;
        }
      }
    }

    return true;
  }

  /**
   * Start running the camera.
   */
  public static VideoSource startCamera(CameraConfig config) {
    System.out.println("Starting camera '" + config.name + "' on " + config.path);
    UsbCamera camera = new UsbCamera(config.name, config.path);
    MjpegServer server = CameraServer.startAutomaticCapture(camera);

    Gson gson = new GsonBuilder().create();

    camera.setConfigJson(gson.toJson(config.config));
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    if (config.streamConfig != null) {
      server.setConfigJson(gson.toJson(config.streamConfig));
    }

    return camera;
  }

  /**
   * Start running the switched camera.
   */
  public static MjpegServer startSwitchedCamera(SwitchedCameraConfig config) {
    System.out.println("Starting switched camera '" + config.name + "' on " + config.key);
    MjpegServer server = CameraServer.addSwitchedCamera(config.name);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.addListener(
        inst.getTopic(config.key),
        EnumSet.of(NetworkTableEvent.Kind.kImmediate, NetworkTableEvent.Kind.kValueAll),
        event -> {
          if (event.valueData != null) {
            if (event.valueData.value.isInteger()) {
              int i = (int) event.valueData.value.getInteger();
              if (i >= 0 && i < cameras.size()) {
                server.setSource(cameras.get(i));
              }
            } else if (event.valueData.value.isDouble()) {
              int i = (int) event.valueData.value.getDouble();
              if (i >= 0 && i < cameras.size()) {
                server.setSource(cameras.get(i));
              }
            } else if (event.valueData.value.isString()) {
              String str = event.valueData.value.getString();
              for (int i = 0; i < cameraConfigs.size(); i++) {
                if (str.equals(cameraConfigs.get(i).name)) {
                  server.setSource(cameras.get(i));
                  break;
                }
              }
            }
          }
        });

    return server;
  }

}
