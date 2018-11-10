package us.ihmc.euclid.tools;

import static org.junit.Assert.*;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class AxisAngleToolsTest
{
   private static final int NUMBER_OF_ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-14;

   @Test
   public void testDistance()
   {
      Random random = new Random(32434L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         AxisAngleReadOnly aa1 = nextAxisAngle(random);
         AxisAngleReadOnly aa2 = nextAxisAngle(random);

         Quaternion q1 = new Quaternion(aa1);
         Quaternion q2 = new Quaternion(aa2);

         double actualDistance = AxisAngleTools.distance(aa1, aa2);
         double expectedDistance = q1.distance(q2);
         assertEquals(expectedDistance, actualDistance, EPSILON);
         assertEquals(0.0, aa1.distance(aa1), EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3D u1 = nextVector3DWithFixedLength(random, nextDouble(random, 0.1, 1.0));
         Vector3D u2 = nextVector3DWithFixedLength(random, nextDouble(random, 0.1, 1.0));
         AxisAngleReadOnly aa1 = new AxisAngle(u1, nextDouble(random, Math.PI));
         AxisAngleReadOnly aa2 = new AxisAngle(u2, nextDouble(random, Math.PI));

         Quaternion q1 = new Quaternion(aa1);
         Quaternion q2 = new Quaternion(aa2);

         double actualDistance = AxisAngleTools.distance(aa1, aa2);
         double expectedDistance = q1.distance(q2);
         assertEquals(expectedDistance, actualDistance, EPSILON);
         assertEquals(0.0, aa1.distance(aa1), EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3D u1 = nextVector3DWithFixedLength(random, nextDouble(random, 0.0, 0.9 * AxisAngleTools.EPS));
         Vector3D u2 = nextVector3DWithFixedLength(random, nextDouble(random, 0.1, 1.0));
         AxisAngleReadOnly aa1 = new AxisAngle(u1, nextDouble(random, Math.PI));
         AxisAngleReadOnly aa2 = new AxisAngle(u2, nextDouble(random, Math.PI));

         double distance = AxisAngleTools.distance(aa1, aa2);
         assertTrue(Double.isNaN(distance));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3D u1 = nextVector3DWithFixedLength(random, nextDouble(random, 0.1, 1.0));
         Vector3D u2 = nextVector3DWithFixedLength(random, nextDouble(random, 0.0, 0.9 * AxisAngleTools.EPS));
         AxisAngleReadOnly aa1 = new AxisAngle(u1, nextDouble(random, Math.PI));
         AxisAngleReadOnly aa2 = new AxisAngle(u2, nextDouble(random, Math.PI));

         double distance = AxisAngleTools.distance(aa1, aa2);
         assertTrue(Double.isNaN(distance));
      }
   }
}
