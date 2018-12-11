package us.ihmc.euclid.tuple4D;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;

public class QuaternionTest extends QuaternionBasicsTest<Quaternion>
{
   public static final double EPS = 1e-14;

   @Test
   public void testQuaternion()
   {
      Random random = new Random(613615L);
      Quaternion quaternion = new Quaternion();
      Quaternion quaternionCopy;
      Quaternion expected = new Quaternion();

      { // Test Quaternion()
         expected.setToZero();
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(QuaternionBasics other)
         quaternion = quaternionCopy = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion quaternion2 = new Quaternion(quaternion);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, quaternion2, EPS);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(double x, double y, double z, double s)
         expected = EuclidCoreRandomTools.nextQuaternion(random);
         expected.normalizeAndLimitToPi();
         quaternion = new Quaternion(expected.getX(), expected.getY(), expected.getZ(), expected.getS());

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(double[] quaternionArray)
         expected = EuclidCoreRandomTools.nextQuaternion(random);

         double[] quaternionArray;
         quaternionArray = new double[] {expected.getX(), expected.getY(), expected.getZ(), expected.getS()};

         quaternion = new Quaternion(quaternionArray);

         EuclidCoreTestTools.assertQuaternionEquals(expected, quaternion, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(RotationMatrix rotationMatrix)
         RotationMatrix rotationMatrix, rotationMatrixCopy;
         rotationMatrix = rotationMatrixCopy = EuclidCoreRandomTools.nextRotationMatrix(random);

         quaternion = new Quaternion(rotationMatrix);
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, expected);

         EuclidCoreTestTools.assertQuaternionEquals(expected, quaternion, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(VectorBasics rotationVector)
         Vector3D rotationVector, rotationVectorCopy;
         rotationVector = rotationVectorCopy = EuclidCoreRandomTools.nextRotationVector(random);

         quaternion = new Quaternion(rotationVector);
         QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, expected);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
         EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(rotationVector, rotationVectorCopy, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(double yaw, double pitch, double roll)
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double pitch = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         quaternion = new Quaternion(yaw, pitch, roll);
         QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, expected);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Quaternion q = EuclidCoreRandomTools.nextQuaternion(random);

      int newHashCode, previousHashCode;
      newHashCode = q.hashCode();
      assertEquals(newHashCode, q.hashCode());

      previousHashCode = q.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         double qx = q.getX();
         double qy = q.getY();
         double qz = q.getZ();
         double qs = q.getS();
         switch (random.nextInt(4))
         {
         case 0:
            qx = random.nextDouble();
            break;
         case 1:
            qy = random.nextDouble();
            break;
         case 2:
            qz = random.nextDouble();
            break;
         case 3:
            qs = random.nextDouble();
            break;
         }
         q.setUnsafe(qx, qy, qz, qs);
         newHashCode = q.hashCode();
         assertNotEquals((long) newHashCode, (long) previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public Quaternion createEmptyTuple()
   {
      return new Quaternion();
   }

   @Override
   public Quaternion createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextQuaternion(random);
   }

   @Override
   public Quaternion createTuple(double x, double y, double z, double s)
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setUnsafe(x, y, z, s);
      return quaternion;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-14;
   }

   @Test
   public void testSingularitiesYawPitchRoll() throws Exception
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setYawPitchRoll(0.5 * Math.PI, 0.0, 1e-9 - 0.5 * Math.PI);
      double[] yawPitchRoll = new double[3];
      quaternion.getYawPitchRoll(yawPitchRoll);
   }
}
