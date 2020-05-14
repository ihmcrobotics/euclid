package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class Pose2DTest
{
   private static final double EPSILON = 1e-7;

   @Test
   public void testConstructors()
   {
      Random random = new Random(52942L);
      double x, y, yaw;
      Vector2D tuple;
      Orientation2D orientation;
      Pose2D allDoubles, toCopy, copyPose, fromComponents;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         yaw = Math.PI - 2.0 * Math.PI * random.nextDouble();

         allDoubles = new Pose2D(x, y, yaw);
         assertEquals(x, allDoubles.getX(), EPSILON);
         assertEquals(y, allDoubles.getY(), EPSILON);
         assertEquals(yaw, allDoubles.getYaw(), EPSILON);

         toCopy = new Pose2D(x, y, yaw);
         copyPose = new Pose2D(toCopy);

         assertEquals(x, copyPose.getX(), EPSILON);
         assertEquals(y, copyPose.getY(), EPSILON);
         assertEquals(yaw, copyPose.getYaw(), EPSILON);

         tuple = new Vector2D(x, y);
         orientation = new Orientation2D(yaw);
         fromComponents = new Pose2D(tuple, orientation);

         assertEquals(x, fromComponents.getX(), EPSILON);
         assertEquals(y, fromComponents.getY(), EPSILON);
         assertEquals(yaw, fromComponents.getYaw(), EPSILON);
      }
   }
}
