package us.ihmc.euclid.geometry;

import org.junit.Test;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

import java.util.Random;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class Pose3DTest
{
   private int ITERATIONS = 1000;

   @Test
   public void testGeometricallyEquals() {
      Random random = new Random(19732L);
      Pose3D firstPose, secondPose;
      double epsilon = 1e-7;

      // Orientation
      for (int i = 0; i < ITERATIONS; ++i) {
         firstPose = EuclidGeometryRandomTools.generateRandomPose3D(random);
         secondPose = new Pose3D(firstPose);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));
         assertTrue(secondPose.geometricallyEquals(firstPose, epsilon));
         assertTrue(firstPose.geometricallyEquals(firstPose, epsilon));
         assertTrue(secondPose.geometricallyEquals(secondPose, epsilon));
         
         secondPose.appendRotation(new RotationMatrix(new AxisAngle(EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0), 0.99 * epsilon)));
         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));
         
         secondPose = new Pose3D(firstPose);
         secondPose.appendRotation(new RotationMatrix(new AxisAngle(EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0), 1.01 * epsilon)));
         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));
      }

      // Point
      for (int i = 0; i < ITERATIONS; ++i) {
         firstPose = EuclidGeometryRandomTools.generateRandomPose3D(random);
         secondPose = new Pose3D(firstPose);

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

         secondPose.setZ(firstPose.getZ() + 0.99 * epsilon);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setZ(firstPose.getZ() - 0.99 * epsilon);

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

         // reset
         secondPose.set(firstPose);

         secondPose.setZ(firstPose.getZ() + 1.01 * epsilon);

         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setZ(firstPose.getZ() - 1.01 * epsilon);

         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));
      }
   }
}
