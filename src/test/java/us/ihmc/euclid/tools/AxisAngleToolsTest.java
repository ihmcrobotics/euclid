package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextAxisAngle;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextVector3DWithFixedLength;

import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

public class AxisAngleToolsTest
{
   private static final double EPSILON = 1.0e-14;

   @Test
   public void testDistance()
   {
      Random random = new Random(32434L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AxisAngleReadOnly aa1 = nextAxisAngle(random);
         AxisAngleReadOnly aa2 = nextAxisAngle(random);

         Quaternion q1 = new Quaternion(aa1);
         Quaternion q2 = new Quaternion(aa2);

         double actualDistance = AxisAngleTools.distance(aa1, aa2, false);
         double expectedDistance = q1.distance(q2);
         assertEquals(expectedDistance, actualDistance, EPSILON);
         assertEquals(0.0, aa1.distance(aa1), EPSILON);
      }


      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D u1 = nextVector3DWithFixedLength(random, nextDouble(random, 0.1, 1.0));
         Vector3D u2 = nextVector3DWithFixedLength(random, nextDouble(random, 0.1, 1.0));
         AxisAngleReadOnly aa1 = new AxisAngle(u1, nextDouble(random, Math.PI));
         AxisAngleReadOnly aa2 = new AxisAngle(u2, nextDouble(random, Math.PI));

         Quaternion q1 = new Quaternion(aa1);
         Quaternion q2 = new Quaternion(aa2);

         double actualDistance = AxisAngleTools.distance(aa1, aa2, false);
         double expectedDistance = q1.distance(q2);
         assertEquals(expectedDistance, actualDistance, EPSILON);
         assertEquals(0.0, aa1.distance(aa1), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {// Cross Platform distance method testing: (AxisAngle , Quaternion)
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion converted = new Quaternion(axisAngle);

         double actualDistance = AxisAngleTools.distance(axisAngle, quaternion, false);
         double expectedDistance = QuaternionTools.distance(converted, quaternion, false);

         assertEquals(actualDistance, expectedDistance, EPSILON);
      }
      
      for (int i = 0; i < ITERATIONS; ++i)
      {// Cross Platform distance method testing: (AxisAngle , RotationMatrix)
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix converted = new RotationMatrix(axisAngle);

         double actualDistance = AxisAngleTools.distance(axisAngle, rotationMatrix);
         double expectedDistance = RotationMatrixTools.distance(converted, rotationMatrix);

         assertEquals(actualDistance, expectedDistance, EPSILON);
      }
      
      for (int i = 0; i < ITERATIONS; ++i)
      {// Cross Platform distance method testing: (AxisAngle , Yaw Pitch roll)
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         YawPitchRoll yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRoll(random);
         AxisAngle converted = new AxisAngle(yawPitchRoll);

         double actualDistance = AxisAngleTools.distance(axisAngle, yawPitchRoll, false);
         double expectedDistance = AxisAngleTools.distance(converted, axisAngle, false);

         assertEquals(actualDistance, expectedDistance, EPSILON);
      }
      
      for (int i = 0; i < ITERATIONS; ++i)
      {// Type check test in distance method
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics orientation = EuclidCoreRandomTools.nextOrientation3D(random);
         
         double notCastedResult = AxisAngleTools.distance(axisAngle, orientation,false);
         if(orientation instanceof QuaternionReadOnly)
         {
            orientation = (Quaternion) orientation;
         }
         else if(orientation instanceof YawPitchRollReadOnly)
         {
            orientation = (YawPitchRoll) orientation;
         }
         else if(orientation instanceof AxisAngleReadOnly)
         {
            orientation = (AxisAngle) orientation;
         }
         else
         {
            orientation = (RotationMatrix) orientation;
         }
         double castedResult = AxisAngleTools.distance(axisAngle, orientation,false);
         
         assertEquals(notCastedResult, castedResult);
      }
   }

   @Test
   public void testDistanceWithLimitToPi() throws Exception
   {// Test distance method with limit to PI. 
      double min = Math.PI;
      double max = 2 * min;
      Random random = new Random(23523L);
      for (int i = 0; i < ITERATIONS; ++i)
      {
         double randomAngle = ThreadLocalRandom.current().nextDouble(min, max);
         AxisAngle aa1 = EuclidCoreRandomTools.nextAxisAngle(random);
         AxisAngle distance = EuclidCoreRandomTools.nextAxisAngle(random);
         distance.setAngle(randomAngle);
         AxisAngle aa2 = new AxisAngle();
         AxisAngleTools.multiply(aa1, distance, aa2);
         Quaternion q1 = new Quaternion(aa1);
         Quaternion q2 = new Quaternion(aa2);

         double expected = QuaternionTools.distance(q1, q2, true);
         double actual = AxisAngleTools.distance(aa1, aa2, true);
         assertEquals(expected, actual, EPSILON);
      }

   }
   
   @Test
   public void myTest() throws Exception
   {
      AxisAngle aa = new AxisAngle();
      FrameQuaternion f = new FrameQuaternion();
      double d = AxisAngleTools.distance(aa, f, true);
   }

}
