package us.ihmc.euclid.geometry;

import org.junit.Test;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;

import java.util.Random;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class Pose2DTest
{
   private int ITERATIONS = 1000;

   @Test
   public void testGeometricallyEquals() {
      Random random = new Random(19732L);
      Pose2D firstPose, secondPose;
      double epsilon = 1e-7;

      // Orientation
      for (int i = 0; i < ITERATIONS; ++i) {
         firstPose = EuclidGeometryRandomTools.generateRandomPose2D(random);
         secondPose = new Pose2D(firstPose);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));
         assertTrue(secondPose.geometricallyEquals(firstPose, epsilon));
         assertTrue(firstPose.geometricallyEquals(firstPose, epsilon));
         assertTrue(secondPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setYaw(firstPose.getYaw() + 0.99 * epsilon);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setYaw(firstPose.getYaw() - 0.99 * epsilon);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setYaw(firstPose.getYaw() + 1.01 * epsilon);

         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setYaw(firstPose.getYaw() - 1.01 * epsilon);

         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));
      }

      // Point
      for (int i = 0; i < ITERATIONS; ++i) {
         firstPose = EuclidGeometryRandomTools.generateRandomPose2D(random);
         secondPose = new Pose2D(firstPose);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));
         assertTrue(secondPose.geometricallyEquals(firstPose, epsilon));
         assertTrue(firstPose.geometricallyEquals(firstPose, epsilon));
         assertTrue(secondPose.geometricallyEquals(secondPose, epsilon));

         // True
         
         secondPose.setX(firstPose.getX() + 0.99 * epsilon);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setX(firstPose.getX() - 0.99 * epsilon);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));
         
         // reset
         secondPose.set(firstPose);

         secondPose.setY(firstPose.getY() + 0.99 * epsilon);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setY(firstPose.getY() - 0.99 * epsilon);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));

         // reset
         secondPose.set(firstPose);
         
         // False

         secondPose.setX(firstPose.getX() + 1.01 * epsilon);

         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setX(firstPose.getX() - 1.01 * epsilon);

         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));

         // reset
         secondPose.set(firstPose);

         secondPose.setY(firstPose.getY() + 1.01 * epsilon);

         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setY(firstPose.getY() - 1.01 * epsilon);

         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));
      }
   }
}
