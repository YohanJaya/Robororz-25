import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.LED;
import java.util.*;

public class my_controller {

  
  // --- Movement / robot constants ---
  // Wheel and robot geometry
  private static final double WHEEL_RADIUS = 0.0205; // meters (from your code)
  private static final double AXLE_WIDTH = 0.08; // meters (distance between wheels) - adjust if needed
  private static final double CELL_SIZE = 0.20; // meters per grid cell - change to match your world

  // Speeds (rad/s for wheel motors)
  private static final double DRIVE_SPEED = 3.0;   // wheel angular velocity while driving
  private static final double ROTATE_SPEED = 2.0;  // wheel angular velocity while rotating

  // Thresholds
  private static final double WALL_THRESHOLD = 80.0; // sensor threshold used in your code
  private static final double GREEN_PIXEL_RATIO = 0.10; // 10% of pixels

  // Heading mapping as you requested:
  // 0 = North (+Y)
  // 1 = West  (+X)
  // 2 = South (-Y)
  // 3 = East  (-X)

  // Helper: normalize heading to [0,3]
  private static int normHeading(int h) {
    h %= 4;
    if (h < 0) h += 4;
    return h;
  }

  public static void main(String[] args) {

    Robot robot = new Robot();

    // Time step in milliseconds
    int timeStep = 64;

    // Initialize motors
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");

    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);

    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);

    // Initialize distance sensors (8 sensors around the robot)
    DistanceSensor[] ds = new DistanceSensor[8];
    for (int i = 0; i < 8; i++) {
      ds[i] = robot.getDistanceSensor("ps" + i);
      if (ds[i] != null) ds[i].enable(timeStep);
    }

    // Initialize position sensors (encoders)
    PositionSensor leftEncoder = robot.getPositionSensor("left wheel sensor");
    PositionSensor rightEncoder = robot.getPositionSensor("right wheel sensor");
    if (leftEncoder != null) leftEncoder.enable(timeStep);
    if (rightEncoder != null) rightEncoder.enable(timeStep);

    // Initialize camera for checkpoint detection
    Camera camera = robot.getCamera("camera");
    if (camera != null) {
      camera.enable(timeStep);
    }

    // Initialize LED
    LED led = robot.getLED("led0");
    if (led != null) {
      led.set(0); // Start with LED off
    }

    // Direction vectors: {dx, dy}
    int[][] directions = {
        {0, 1},   // North (+Y)
        {1, 0},   // East (+X) -- note: we will map dx=+1 as heading 1 (West) per your custom mapping below
        {0, -1},  // South (-Y)
        {-1, 0}   // West (-X)
    };

    // Robot grid coordinates (start at 0,0) - use arrays to allow inner class modification
    final int[] position = {0, 0}; // [distX, distY]

    // Current robot heading (start facing North) - use array for inner class
    final int[] headingArray = {0}; // 0 = North

    // Encoder baseline values for relative motion
    double leftEncoderStart = (leftEncoder != null) ? leftEncoder.getValue() : 0.0;
    double rightEncoderStart = (rightEncoder != null) ? rightEncoder.getValue() : 0.0;

    // Checkpoint tracking - use arrays for inner class modification
    final int[] checkpointSum = {0, 0}; // [sumX, sumY]
    Set<String> checkpointVisited = new HashSet<>();

    // Column-by-column navigation state (visited & queue)
    Set<String> visited = new HashSet<>();

    // Use a queue; preserve your priority idea but implement as simple queue to avoid comparator complexities.
    Queue<Cell> queue = new LinkedList<>();

    // Helper method to create cell key
    final class Utils {
      String cellKey(int x, int y) {
        return x + "," + y;
      }
    }
    Utils utils = new Utils();

    // Mark starting position
    visited.add(utils.cellKey(position[0], position[1]));
    queue.add(new Cell(position[0], position[1]));

    // Motion helper class
    final class Motion {
      void setWheelVel(double leftVel, double rightVel) {
        leftMotor.setVelocity(leftVel);
        rightMotor.setVelocity(rightVel);
      }

      void rotateToHeading(int targetHeading) {
        targetHeading = normHeading(targetHeading);
        int delta = (targetHeading - headingArray[0] + 4) % 4;
        if (delta == 0) return;

        int steps;
        int direction;
        if (delta == 1) { steps = 1; direction = 1; }
        else if (delta == 2) { steps = 2; direction = 1; }
        else { steps = 1; direction = -1; }

        for (int s = 0; s < steps; s++) {
          double theta = Math.PI / 2.0;
          double arcPerWheel = (AXLE_WIDTH / 2.0) * theta;
          double requiredWheelRadians = arcPerWheel / WHEEL_RADIUS;

          double leftBase = (leftEncoder != null) ? leftEncoder.getValue() : 0.0;
          double rightBase = (rightEncoder != null) ? rightEncoder.getValue() : 0.0;

          if (direction == 1) {
            leftMotor.setVelocity(-ROTATE_SPEED);
            rightMotor.setVelocity(ROTATE_SPEED);
          } else {
            leftMotor.setVelocity(ROTATE_SPEED);
            rightMotor.setVelocity(-ROTATE_SPEED);
          }

          while (robot.step(timeStep) != -1) {
            double leftDelta = Math.abs((leftEncoder != null ? leftEncoder.getValue() : 0.0) - leftBase);
            double rightDelta = Math.abs((rightEncoder != null ? rightEncoder.getValue() : 0.0) - rightBase);
            double avg = (leftDelta + rightDelta) / 2.0;
            if (avg >= requiredWheelRadians * 0.98) break;
          }

          leftMotor.setVelocity(0.0);
          rightMotor.setVelocity(0.0);
          headingArray[0] = normHeading(headingArray[0] + (direction == 1 ? 1 : -1));
        }
      }

      void driveOneCellForward() {
        double requiredDistance = CELL_SIZE;
        double requiredWheelRadians = requiredDistance / WHEEL_RADIUS;

        double leftBase = (leftEncoder != null) ? leftEncoder.getValue() : 0.0;
        double rightBase = (rightEncoder != null) ? rightEncoder.getValue() : 0.0;

        leftMotor.setVelocity(DRIVE_SPEED);
        rightMotor.setVelocity(DRIVE_SPEED);

        while (robot.step(timeStep) != -1) {
          double leftDelta = Math.abs((leftEncoder != null ? leftEncoder.getValue() : 0.0) - leftBase);
          double rightDelta = Math.abs((rightEncoder != null ? rightEncoder.getValue() : 0.0) - rightBase);
          double avg = (leftDelta + rightDelta) / 2.0;
          if (avg >= requiredWheelRadians * 0.98) break;
        }

        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);
      }
    }
    Motion motion = new Motion();

    // Checkpoint detection helper class
    final class Checkpoint {
      boolean detectAndMark(int cx, int cy) {
        if (camera == null) return false;

        int width = camera.getWidth();
        int height = camera.getHeight();
        int totalPixels = width * height;
        int greenPixelCount = 0;
        int[] image = camera.getImage();

        for (int i = 0; i < totalPixels; i++) {
          int pixel = image[i];
          int r = Camera.pixelGetRed(pixel);
          int g = Camera.pixelGetGreen(pixel);
          int b = Camera.pixelGetBlue(pixel);

          if (g > 150 && g > r * 1.5 && g > b * 1.5) {
            greenPixelCount++;
          }
        }

        String key = utils.cellKey(cx, cy);
        if (greenPixelCount > totalPixels * GREEN_PIXEL_RATIO && !checkpointVisited.contains(key)) {
          checkpointVisited.add(key);
          checkpointSum[0] += cx;
          checkpointSum[1] += cy;

          System.out.println("CHECKPOINT DETECTED at (" + cx + ", " + cy + ")");
          System.out.println("Running sum: X=" + checkpointSum[0] + ", Y=" + checkpointSum[1]);

          if (led != null) led.set(1);
          return true;
        } else {
          if (led != null && !checkpointVisited.contains(key)) {
            led.set(0);
          }
          return false;
        }
      }
    }
    Checkpoint checkpoint = new Checkpoint();

    // --- Exploration: while queue not empty, poll next cell and move there step-by-step ---
    while (robot.step(timeStep) != -1 && !queue.isEmpty()) {

      // poll next cell to visit
      Cell next = queue.poll();
      if (next == null) break;

      // If next is same as current, still check checkpoint and expand neighbors
      if (next.x == position[0] && next.y == position[1]) {
        // detect checkpoint at current cell
        checkpoint.detectAndMark(position[0], position[1]);
      } else {
        // Determine required movement direction from current (position[0],position[1]) -> (next.x,next.y)
        int dx = next.x - position[0];
        int dy = next.y - position[1];

        // determine desired heading using your mapping:
        int desiredHeading = headingArray[0]; // default keep same if unknown
        if (dx == 1 && dy == 0) {
          // moving +X according to your earlier confirmation: +X mapping -> heading 1 (West per your mapping)
          desiredHeading = 1;
        } else if (dx == -1 && dy == 0) {
          // moving -X -> heading 3 (East per mapping)
          desiredHeading = 3;
        } else if (dx == 0 && dy == 1) {
          desiredHeading = 0; // North
        } else if (dx == 0 && dy == -1) {
          desiredHeading = 2; // South
        } else {
          // Non-adjacent cell (shouldn't happen in your neighbor-based queue); skip
          continue;
        }

        // Rotate to face desired heading
        motion.rotateToHeading(desiredHeading);

        // Drive one cell forward
        motion.driveOneCellForward();

        // Update current grid position & encoder baselines
        position[0] = next.x;
        position[1] = next.y;
        leftEncoderStart = (leftEncoder != null) ? leftEncoder.getValue() : leftEncoderStart;
        rightEncoderStart = (rightEncoder != null) ? rightEncoder.getValue() : rightEncoderStart;

        // After moving into new cell, detect checkpoint if any
        checkpoint.detectAndMark(position[0], position[1]);
      }

      // Read sensors to expand neighbors (use readings at new position)
      double frontRight = ds[6] != null ? ds[6].getValue() : 0.0;
      double frontLeft = ds[7] != null ? ds[7].getValue() : 0.0;
      double frontCenter = ds[0] != null ? ds[0].getValue() : 0.0;
      double rightSide = ds[5] != null ? ds[5].getValue() : 0.0;
      double leftSide = ds[1] != null ? ds[1].getValue() : 0.0;
      double backCenter = ds[3] != null ? ds[3].getValue() : 0.0;

      // Expand neighbors using your original sensor/wall logic (with corrected direction comments)
      // For each cardinal direction around robot (North, East, South, West) we check sensors
      // and add non-wall neighbors to queue if not visited.
      // Map directions to checks:
      // North (0, +1) -> frontCenter
      // East  (+1, 0) -> rightSide
      // South (0, -1) -> backCenter
      // West  (-1, 0) -> leftSide

      int[][] neighborDirs = {
          {0, 1},   // North
          {1, 0},   // +X
          {0, -1},  // South
          {-1, 0}   // -X
      };

      for (int[] dir : neighborDirs) {
        int newX = position[0] + dir[0];
        int newY = position[1] + dir[1];
        String newKey = utils.cellKey(newX, newY);

        boolean isWall = false;
        if (dir[0] == 0 && dir[1] == 1 && frontCenter > WALL_THRESHOLD) isWall = true; // North
        if (dir[0] == 1 && dir[1] == 0 && rightSide > WALL_THRESHOLD) isWall = true;   // +X (right side)
        if (dir[0] == 0 && dir[1] == -1 && backCenter > WALL_THRESHOLD) isWall = true;  // South
        if (dir[0] == -1 && dir[1] == 0 && leftSide > WALL_THRESHOLD) isWall = true;   // -X (left side)

        if (!isWall && !visited.contains(newKey)) {
          visited.add(newKey);
          queue.add(new Cell(newX, newY, new Cell(position[0], position[1])));
        }
      }

      // Continue until queue exhausted
    } // end exploration loop

    // Exploration finished. Compute final coordinates mod 12
    int finalX = ((checkpointSum[0] % 12) + 12) % 12;
    int finalY = ((checkpointSum[1] % 12) + 12) % 12;
    System.out.println("Exploration complete. Checkpoint sum X=" + checkpointSum[0] + " Y=" + checkpointSum[1]);
    System.out.println("Final target after mod 12: (" + finalX + ", " + finalY + ")");

    // Navigate to (finalX, finalY) from current (position[0], position[1]) using simple Manhattan moves
    // Move in X direction first, then Y (preserves your column-by-column flavor)
    while (position[0] != finalX) {
      int dx = finalX - position[0];
      int stepX = (dx > 0) ? 1 : -1;

      int desiredHeading;
      if (stepX == 1) desiredHeading = 1; else desiredHeading = 3; // +X -> heading 1; -X -> heading 3

      motion.rotateToHeading(desiredHeading);
      motion.driveOneCellForward();

      position[0] += stepX;
      // detect checkpoint at each step (optional)
      checkpoint.detectAndMark(position[0], position[1]);
    }

    while (position[1] != finalY) {
      int dy = finalY - position[1];
      int stepY = (dy > 0) ? 1 : -1;

      int desiredHeading;
      if (stepY == 1) desiredHeading = 0; else desiredHeading = 2; // +Y -> 0; -Y -> 2

      motion.rotateToHeading(desiredHeading);
      motion.driveOneCellForward();

      position[1] += stepY;
      checkpoint.detectAndMark(position[0], position[1]);
    }

    // At final cell: light LED to signal completion (blink a few times)
    if (led != null) {
      for (int i = 0; i < 6; i++) {
        led.set((i % 2 == 0) ? 1 : 0);
        // wait one step
        if (robot.step(timeStep) == -1) break;
      }
      led.set(1); // leave LED on
    }

    System.out.println("Arrived at final target (" + position[0] + ", " + position[1] + "). Task complete.");
  }
}
