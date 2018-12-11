package us.ihmc.euclid.yawPitchRoll;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.YawPitchRollTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;

public abstract class YawPitchRollBasicsTest<T extends YawPitchRollBasics> extends YawPitchRollReadOnlyTest<T>
{
   @Test
   public void testComponentSetters()
   {
      Random random = new Random(5464);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setYaw(double yaw)
         T ypr = createEmptyYawPitchRoll();
         assertTrue(ypr.getYaw() == 0.0);
         assertTrue(ypr.getPitch() == 0.0);
         assertTrue(ypr.getRoll() == 0.0);

         double expectedYaw = random.nextDouble();
         ypr.setYaw(expectedYaw);

         assertTrue(ypr.getYaw() == expectedYaw);
         assertTrue(ypr.getPitch() == 0.0);
         assertTrue(ypr.getRoll() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setPitch(double pitch)
         T ypr = createEmptyYawPitchRoll();
         assertTrue(ypr.getYaw() == 0.0);
         assertTrue(ypr.getPitch() == 0.0);
         assertTrue(ypr.getRoll() == 0.0);

         double expectedPitch = random.nextDouble();
         ypr.setPitch(expectedPitch);

         assertTrue(ypr.getYaw() == 0.0);
         assertTrue(ypr.getPitch() == expectedPitch);
         assertTrue(ypr.getRoll() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setRoll(double roll)
         T ypr = createEmptyYawPitchRoll();
         assertTrue(ypr.getYaw() == 0.0);
         assertTrue(ypr.getPitch() == 0.0);
         assertTrue(ypr.getRoll() == 0.0);

         double expectedRoll = random.nextDouble();
         ypr.setRoll(expectedRoll);

         assertTrue(ypr.getYaw() == 0.0);
         assertTrue(ypr.getPitch() == 0.0);
         assertTrue(ypr.getRoll() == expectedRoll);
      }
   }

   @Test
   public void testSetToZero()
   {
      T ypr = createYawPitchRoll(Double.NaN, Double.NaN, Double.NaN);
      assertTrue(Double.isNaN(ypr.getYaw()));
      assertTrue(Double.isNaN(ypr.getPitch()));
      assertTrue(Double.isNaN(ypr.getRoll()));

      ypr.setToZero();
      assertTrue(ypr.getYaw() == 0.0);
      assertTrue(ypr.getPitch() == 0.0);
      assertTrue(ypr.getRoll() == 0.0);
   }

   @Test
   public void testSetToNaN()
   {
      T ypr = createYawPitchRoll(0.0, 0.0, 0.0);
      assertTrue(ypr.getYaw() == 0.0);
      assertTrue(ypr.getPitch() == 0.0);
      assertTrue(ypr.getRoll() == 0.0);
      ypr.setToNaN();
      assertTrue(Double.isNaN(ypr.getYaw()));
      assertTrue(Double.isNaN(ypr.getPitch()));
      assertTrue(Double.isNaN(ypr.getRoll()));
   }

   @Test
   public void testNegate()
   {
      Random random = new Random(5646541L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double yaw = EuclidCoreRandomTools.nextDouble(random);
         double pitch = EuclidCoreRandomTools.nextDouble(random);
         double roll = EuclidCoreRandomTools.nextDouble(random);
         T expectedYPR = createYawPitchRoll(-yaw, -pitch, -roll);
         T actualYPR = createYawPitchRoll(yaw, pitch, roll);
         actualYPR.negate();
         assertTrue(expectedYPR.equals(actualYPR));
      }
   }

   @Test
   public void testAbsolute()
   {
      Random random = new Random(5646541L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double yaw = EuclidCoreRandomTools.nextDouble(random);
         double pitch = EuclidCoreRandomTools.nextDouble(random);
         double roll = EuclidCoreRandomTools.nextDouble(random);
         T expectedYPR = createYawPitchRoll(Math.abs(yaw), Math.abs(pitch), Math.abs(roll));
         T actualYPR = createYawPitchRoll(yaw, pitch, roll);
         actualYPR.absolute();
         assertTrue(expectedYPR.equals(actualYPR));
      }
   }

   @Test
   public void testInvert() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T ypr = createRandomYawPitchRoll(random);
         T inverseYPR = createEmptyYawPitchRoll();
         inverseYPR.set(ypr);
         inverseYPR.invert();

         RotationMatrix rotationMatrix = new RotationMatrix(ypr);
         RotationMatrix inverseRotationMatrix = new RotationMatrix(inverseYPR);
         RotationMatrix identityExpected = new RotationMatrix();
         identityExpected.set(rotationMatrix);
         identityExpected.multiply(inverseRotationMatrix);
         EuclidCoreTestTools.assertIdentity(identityExpected, getEpsilon());

         T zeroExpected = createEmptyYawPitchRoll();
         YawPitchRollTools.multiply(ypr, false, inverseYPR, false, zeroExpected);
         EuclidCoreTestTools.assertAngleEquals(0.0, zeroExpected.getYaw(), getEpsilon());
         EuclidCoreTestTools.assertAngleEquals(0.0, zeroExpected.getPitch(), getEpsilon());
         EuclidCoreTestTools.assertAngleEquals(0.0, zeroExpected.getRoll(), getEpsilon());
      }
   }

   @Test
   public void testSetWithDoubles()
   {
      Random random = new Random(5646541L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T expectedYPR = createRandomYawPitchRoll(random);
         T actualYPR = createEmptyYawPitchRoll();
         actualYPR.set(expectedYPR.getYaw(), expectedYPR.getPitch(), expectedYPR.getRoll());
         assertTrue(expectedYPR.equals(actualYPR));
      }
   }

   @Test
   public void testSet()
   {
      T actualYPR = createEmptyYawPitchRoll();
      T expectedYPR = createEmptyYawPitchRoll();
      Random random = new Random(64654L);

      { // Test set(T other)
         for (int i = 0; i < ITERATIONS; i++)
         {
            expectedYPR = createRandomYawPitchRoll(random);
            actualYPR.set(expectedYPR);
            EuclidCoreTestTools.assertYawPitchRollEquals(actualYPR, expectedYPR, getEpsilon());
         }
      }

      { // Test set(YawPitchRollReadOnly other)
         for (int i = 0; i < ITERATIONS; i++)
         {
            expectedYPR = createRandomYawPitchRoll(random);
            actualYPR.set(expectedYPR);
            EuclidCoreTestTools.assertYawPitchRollEquals(actualYPR, expectedYPR, getEpsilon());
         }
      }

      { // Test set(double[] yawPitchRollArray)
         for (int i = 0; i < ITERATIONS; i++)
         {
            expectedYPR = createRandomYawPitchRoll(random);
            double[] axisAngleArray = new double[] {expectedYPR.getYaw(), expectedYPR.getPitch(), expectedYPR.getRoll()};
            actualYPR.set(axisAngleArray);
            EuclidCoreTestTools.assertYawPitchRollEquals(expectedYPR, actualYPR, getEpsilon());
         }
      }

      { // Test set(int startIndex, double[] yawPitchRollArray)
         for (int i = 0; i < ITERATIONS; i++)
         {
            expectedYPR = createRandomYawPitchRoll(random);
            int startIndex = random.nextInt(10);
            double[] axisAngleArray = new double[4 + startIndex + random.nextInt(10)];
            expectedYPR.get(startIndex, axisAngleArray);
            actualYPR.set(startIndex, axisAngleArray);
            EuclidCoreTestTools.assertYawPitchRollEquals(expectedYPR, actualYPR, getEpsilon());
         }
      }

      { // Test set(float[] yawPitchRollArray)
         for (int i = 0; i < ITERATIONS; i++)
         {
            expectedYPR = createRandomYawPitchRoll(random);
            float[] axisAngleArray = new float[] {expectedYPR.getYaw32(), expectedYPR.getPitch32(), expectedYPR.getRoll32()};
            actualYPR.set(axisAngleArray);
            assertTrue(expectedYPR.getYaw32() == actualYPR.getYaw32());
            assertTrue(expectedYPR.getPitch32() == actualYPR.getPitch32());
            assertTrue(expectedYPR.getRoll32() == actualYPR.getRoll32());
         }
      }

      { // Test set(int startIndex, float[] yawPitchRollArray)
         for (int i = 0; i < ITERATIONS; i++)
         {
            expectedYPR = createRandomYawPitchRoll(random);
            int startIndex = random.nextInt(10);
            float[] axisAngleArray = new float[4 + startIndex + random.nextInt(10)];
            expectedYPR.get(startIndex, axisAngleArray);
            actualYPR.set(startIndex, axisAngleArray);
            assertTrue(expectedYPR.getYaw32() == actualYPR.getYaw32());
            assertTrue(expectedYPR.getPitch32() == actualYPR.getPitch32());
            assertTrue(expectedYPR.getRoll32() == actualYPR.getRoll32());
         }
      }

      { // Test set(QuaternionReadOnly quaternion)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
            actualYPR.set(quaternion);
            YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, expectedYPR);
            EuclidCoreTestTools.assertYawPitchRollEquals(expectedYPR, actualYPR, getEpsilon());
         }
      }

      { // Test set(RotationMatrixReadOnly rotationMatrix)
         for (int i = 0; i < ITERATIONS; i++)
         {
            RotationMatrix matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
            actualYPR.set(matrix);
            YawPitchRollConversion.convertMatrixToYawPitchRoll(matrix, expectedYPR);
            EuclidCoreTestTools.assertYawPitchRollEquals(expectedYPR, actualYPR, getEpsilon());
         }
      }

      { // Test set(Vector3DReadOnly rotationVector)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Vector3D rotationVector = EuclidCoreRandomTools.nextRotationVector(random);
            actualYPR.setRotationVector(rotationVector);
            YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, expectedYPR);
            EuclidCoreTestTools.assertYawPitchRollEquals(expectedYPR, actualYPR, getEpsilon());
         }
      }

      { // Test setYawPitchRoll(AxisAngleReadOnly axisAngle)
         for (int i = 0; i < ITERATIONS; i++)
         {
            AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
            actualYPR.set(axisAngle);
            YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, expectedYPR);
            EuclidCoreTestTools.assertYawPitchRollEquals(expectedYPR, actualYPR, getEpsilon());
         }
      }

      { // Test setYawPitchRoll(double yaw, double pitch, double roll)
         for (int i = 0; i < ITERATIONS; i++)
         {
            double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
            double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI / 2.0);
            double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
            actualYPR.setYawPitchRoll(yaw, pitch, roll);
            expectedYPR.set(yaw, pitch, roll);
            EuclidCoreTestTools.assertYawPitchRollEquals(expectedYPR, actualYPR, getEpsilon());
         }
      }

      { // Test setElement(int index, double value)
         for (int i = 0; i < ITERATIONS; i++)
         {
            actualYPR = createRandomYawPitchRoll(random);

            for (int index = 0; index < 3; index++)
            {
               double expectedValue = random.nextDouble();
               actualYPR.setElement(index, expectedValue);
               double actualValue = actualYPR.getElement(index);
               assertEqualsDelta(expectedValue, actualValue, getEpsilon());
            }
         }
      }
   }

   @Test
   public void testAppend()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test append(Orientation3DReadOnly other)
         Orientation3DReadOnly orientation;
         switch (random.nextInt(5))
         {
         case 0:
            orientation = EuclidCoreRandomTools.nextAxisAngle(random);
            break;
         case 1:
            orientation = EuclidCoreRandomTools.nextQuaternion(random);
            break;
         case 2:
            orientation = EuclidCoreRandomTools.nextYawPitchRoll(random);
            break;
         default:
            orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
            break;
         }

         T original = createRandomYawPitchRoll(random);
         Quaternion q = new Quaternion(original);

         T actual = createEmptyYawPitchRoll();
         actual.set(original);
         actual.append(orientation);

         q.set(original);
         q.append(orientation);
         T expected = createYawPitchRoll(q);
         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testAppendInvertOther()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test appendInvertOther(Orientation3DReadOnly other)
         Orientation3DReadOnly orientation;
         switch (random.nextInt(5))
         {
         case 0:
            orientation = EuclidCoreRandomTools.nextAxisAngle(random);
            break;
         case 1:
            orientation = EuclidCoreRandomTools.nextQuaternion(random);
            break;
         case 2:
            orientation = EuclidCoreRandomTools.nextYawPitchRoll(random);
            break;
         default:
            orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
            break;
         }

         T original = createRandomYawPitchRoll(random);
         Quaternion q = new Quaternion(original);

         T actual = createEmptyYawPitchRoll();
         actual.set(original);
         actual.appendInvertOther(orientation);

         q.set(original);
         q.appendInvertOther(orientation);
         T expected = createYawPitchRoll(q);
         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testAppendYawRotation()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T original = createRandomYawPitchRoll(random);
         double yaw = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         T actual = createYawPitchRoll(original);
         actual.appendYawRotation(yaw);

         Quaternion q = new Quaternion(original);
         q.appendYawRotation(yaw);
         T expected = createYawPitchRoll(q);

         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testAppendPitchRotation()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T original = createRandomYawPitchRoll(random);
         double pitch = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         T actual = createYawPitchRoll(original);
         actual.appendPitchRotation(pitch);

         Quaternion q = new Quaternion(original);
         q.appendPitchRotation(pitch);
         T expected = createYawPitchRoll(q);

         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testAppendRollRotation()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T original = createRandomYawPitchRoll(random);
         double roll = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         T actual = createYawPitchRoll(original);
         actual.appendRollRotation(roll);

         Quaternion q = new Quaternion(original);
         q.appendRollRotation(roll);
         T expected = createYawPitchRoll(q);

         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrepend()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test prepend(Orientation3DReadOnly other)
         Orientation3DReadOnly orientation;
         switch (random.nextInt(5))
         {
         case 0:
            orientation = EuclidCoreRandomTools.nextAxisAngle(random);
            break;
         case 1:
            orientation = EuclidCoreRandomTools.nextQuaternion(random);
            break;
         case 2:
            orientation = EuclidCoreRandomTools.nextYawPitchRoll(random);
            break;
         default:
            orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
            break;
         }

         T original = createRandomYawPitchRoll(random);
         Quaternion q = new Quaternion(original);

         T actual = createEmptyYawPitchRoll();
         actual.set(original);
         actual.prepend(orientation);

         q.set(original);
         q.prepend(orientation);
         T expected = createYawPitchRoll(q);
         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependInvertOther()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test prependInvertOther(Orientation3DReadOnly other)
         Orientation3DReadOnly orientation;
         switch (random.nextInt(5))
         {
         case 0:
            orientation = EuclidCoreRandomTools.nextAxisAngle(random);
            break;
         case 1:
            orientation = EuclidCoreRandomTools.nextQuaternion(random);
            break;
         case 2:
            orientation = EuclidCoreRandomTools.nextYawPitchRoll(random);
            break;
         default:
            orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
            break;
         }

         T original = createRandomYawPitchRoll(random);
         Quaternion q = new Quaternion(original);

         T actual = createEmptyYawPitchRoll();
         actual.set(original);
         actual.prependInvertOther(orientation);

         q.set(original);
         q.prependInvertOther(orientation);
         T expected = createYawPitchRoll(q);
         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependYawRotation()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T original = createRandomYawPitchRoll(random);
         double yaw = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         T actual = createYawPitchRoll(original);
         actual.prependYawRotation(yaw);

         Quaternion q = new Quaternion(original);
         q.prependYawRotation(yaw);
         T expected = createYawPitchRoll(q);

         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependPitchRotation()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T original = createRandomYawPitchRoll(random);
         double pitch = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         T actual = createYawPitchRoll(original);
         actual.prependPitchRotation(pitch);

         Quaternion q = new Quaternion(original);
         q.prependPitchRotation(pitch);
         T expected = createYawPitchRoll(q);

         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependRollRotation()
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T original = createRandomYawPitchRoll(random);
         double roll = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         T actual = createYawPitchRoll(original);
         actual.prependRollRotation(roll);

         Quaternion q = new Quaternion(original);
         q.prependRollRotation(roll);
         T expected = createYawPitchRoll(q);

         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, actual, getEpsilon());
      }
   }
}
