package us.ihmc.euclid.tuple4D;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Quaternion32Test extends QuaternionBasicsTest<Quaternion32>
{
   public static final float EPS = (float) 1e-6;

   @Test
   public void testQuaternion32()
   {
      Random random = new Random(613615L);
      Quaternion32 quaternion = new Quaternion32();
      Quaternion32 quaternionCopy;
      Quaternion32 expected = new Quaternion32();

      { // Test Quaternion32()
         expected.setToZero();
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion32(QuaternionBasics other)
         quaternion = quaternionCopy = EuclidCoreRandomTools.nextQuaternion32(random);
         Quaternion32 quaternion2 = new Quaternion32(quaternion);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, quaternion2, EPS);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion32(float[] quaternionArray)
         expected = EuclidCoreRandomTools.nextQuaternion32(random);

         float[] quaternionArray;
         quaternionArray = new float[] {expected.getX32(), expected.getY32(), expected.getZ32(), expected.getS32()};

         quaternion = new Quaternion32(quaternionArray);

         EuclidCoreTestTools.assertQuaternionEquals(expected, quaternion, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion32(RotationMatrix rotationMatrix) about X-axis
         RotationMatrix rotationMatrix, rotationMatrixCopy;
         rotationMatrix = rotationMatrixCopy = EuclidCoreRandomTools.nextRotationMatrix(random);

         quaternion = new Quaternion32(rotationMatrix);
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, expected);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion32(VectorBasics rotationVector)
         Vector3D rotationVector, rotationVectorCopy;
         rotationVector = rotationVectorCopy = EuclidCoreRandomTools.nextRotationVector(random);

         quaternion = new Quaternion32(rotationVector);
         QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, expected);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
         EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(rotationVector, rotationVectorCopy, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(double x, double y, double z, double s)
         expected = EuclidCoreRandomTools.nextQuaternion32(random);
         quaternion = new Quaternion32(expected.getX32(), expected.getY32(), expected.getZ32(), expected.getS32());

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(double yaw, double pitch, double roll)
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double pitch = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         quaternion = new Quaternion32(yaw, pitch, roll);
         QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, expected);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Quaternion32 q = EuclidCoreRandomTools.nextQuaternion32(random);

      int newHashCode, previousHashCode;
      newHashCode = q.hashCode();
      assertEquals(newHashCode, q.hashCode());

      previousHashCode = q.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         float qx = q.getX32();
         float qy = q.getY32();
         float qz = q.getZ32();
         float qs = q.getS32();
         switch (random.nextInt(4))
         {
            case 0:
               qx = random.nextFloat();
               break;
            case 1:
               qy = random.nextFloat();
               break;
            case 2:
               qz = random.nextFloat();
               break;
            case 3:
               qs = random.nextFloat();
               break;
         }
         q.setUnsafe(qx, qy, qz, qs);
         newHashCode = q.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public Quaternion32 createEmptyTuple()
   {
      return new Quaternion32();
   }

   @Override
   public Quaternion32 createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextQuaternion32(random);
   }

   @Override
   public Quaternion32 createTuple(double x, double y, double z, double s)
   {
      Quaternion32 quaternion = new Quaternion32();
      quaternion.setUnsafe(x, y, z, s);
      return quaternion;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-6;
   }
}
