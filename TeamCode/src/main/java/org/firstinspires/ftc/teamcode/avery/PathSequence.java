package org.firstinspires.ftc.teamcode.avery;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.avery.Path;

import java.util.ArrayList;
import java.util.Arrays;

class PathSequence {
  public static final double deadBand = 0.1;
  
  public ArrayList<Path> paths = new ArrayList<>();

  public int currentPath = 0;
  public Vector2D start;

  public PathSequence(Path[] paths){
    this.start = paths[0].getStart();
    for(Path path : paths){
      this.paths.add(path);
    }
  }

  public PathSequence(Vector2D start){
    this.start = start;
  }

  public PathSequence(double x, double y){
    this.start = new Vector2D(x, y);
  }

  public PathSequence LineTo(double x, double y){
    Vector2D end = new Vector2D(x, y);
    Vector2D start = this.start;
    if(paths.size() == 0){
    }
    else{
      start = paths.get(paths.size() - 1).getEnd();
    }

    Path[] output = Arrays.copyOf(this.getPaths(), paths.size() + 1);
    output[paths.size()] = new Line(start, end);
    return new PathSequence(output);
  }

  public PathSequence LineTo(Vector2D end){
    Vector2D start = this.start;
    if(paths.size() == 0){
    }
    else{
      start = paths.get(paths.size() - 1).getEnd();
    }

    Path[] output = Arrays.copyOf(this.getPaths(), paths.size() + 1);
    output[paths.size()] = new Line(start, end);
    return new PathSequence(output);
  }

  public Path[] getPaths(){
    Path[] output = new Path[paths.size()];

    for(int i = 0; i < paths.size(); i++){
      output[i] = paths.get(i);
    }
    return output;
  }

  public PathSequence SplineTo(double x, double y, double cx, double cy, double startVelo){
    Path last = paths.get(paths.size() - 1);
    Vector2D lastEnd = last.controlPoints[last.controlPoints.length - 1];
    Vector2D lastStart = last.controlPoints[last.controlPoints.length - 2];
    
    Vector2D end = new Vector2D(x, y);

    Vector2D controlPoint1 =
      lastEnd.sub(lastStart)
      .normalize()
      .mult(startVelo);
      
    Vector2D controlPoint2 = new Vector2D(cx - x, cy - y);
    Path[] output = Arrays.copyOf(this.getPaths(), paths.size() + 1);
    output[paths.size()] = new Spline(lastEnd, controlPoint1, controlPoint2, end);
    return new PathSequence(output);
  }

  public PathSequence SplineTo(Vector2D end, Vector2D control, double startVelo){
    Path last = paths.get(paths.size() - 1);
    Vector2D lastEnd = last.controlPoints[last.controlPoints.length - 1];
    Vector2D lastStart = last.controlPoints[last.controlPoints.length - 2];


    Vector2D controlPoint1 =
      lastEnd.sub(lastStart)
      .normalize()
      .mult(startVelo);

    Vector2D controlPoint2 = control.sub(end);
    Path[] output = Arrays.copyOf(this.getPaths(), paths.size() + 1);
    output[paths.size()] = new Spline(lastEnd, controlPoint1, controlPoint2, end);
    return new PathSequence(output);
  }

  public PathSequence SplineTo(Vector2D start, Vector2D control1, Vector2D end, Vector2D control2){

    control1 = control1.sub(start);
    control2 = control2.sub(end);

    Path[] output = Arrays.copyOf(this.getPaths(), paths.size() + 1);
    output[paths.size()] = new Spline(start, control1, control2, end);
    return new PathSequence(output);
  }

  public PathSequence SplineTo(double startX, double startY, double cx1, double cy1, double endX, double endY, double cx2, double cy2){
    return SplineTo(new Vector2D(startX, startY), new Vector2D(cx1, cy1), new Vector2D(endX, endY), new Vector2D(cx2, cy2));

  }

  public PathSequence SplineTo(Spline spline){
    Path[] output = Arrays.copyOf(this.getPaths(), paths.size() + 1);
    output[paths.size()] = spline.withEnd(paths.get(paths.size() - 1).getEnd());
    return new PathSequence(output);
  }

  public Pose2d follow(Pose2d robot){
    Vector2D point = new Vector2D(robot);
    Path current = paths.get(currentPath);

    double closestT = current.closestT(point);
    Vector2D closest = current.point(closestT);
    if(closestT >= 0.95){
      if(currentPath < paths.size() - 2){
        currentPath ++;
      }
      else{
        //TODO: finish path
      }
      return follow(robot);
    } //switch paths
    Vector2D normal = closest.sub(point);

    Vector2D error = new Vector2D(0, 0);
    if(closestT < 1){
      error = current.derivative(closestT);
    }
    error.normalize();
    error.mult(current.aggressiveness);

    Vector2D output =
            normal
                    .add(error)
                    .normalize();

    double dist = current.distance(point);
    if(dist <= current.deccelRadius){
      double t = dist / current.deccelRadius;
      output =
              output.mult(t)
                      .add(output.mult(current.Kstatic * (1 - t)));
    }

    return new Pose2d(output.x, output.y, current.headingPID(robot, dist, output));

  }
  
}