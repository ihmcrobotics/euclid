package us.ihmc.euclid.axisAngle;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

public abstract class AxisAngleReadOnlyTest<T extends AxisAngleReadOnly>
{
   public abstract T createEmptyAxisAngle();

   public abstract T createAxisAngle(Vector3DReadOnly axis, double angle);

   public abstract T createAxisAngle(double ux, double uy, double uz, double angle);

   public abstract T createRandomAxisAngle(Random random);

   public abstract double getEpsilon();

   public abstract double getSmallestEpsilon();

   @Test
   public void testGetAngle()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < ITERATIONS; i++)
      {
         double expectedAngle = random.nextInt(100);
         axisAngle = createAxisAngle(new Vector3D(), expectedAngle);
         double actualAngle = axisAngle.getAngle();

         assertTrue(expectedAngle == actualAngle);
      }
   }

   @Test
   public void testGetters()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < ITERATIONS; i++)
      {
         double angle = EuclidCoreRandomTools.nextDouble(random, 10.0);
         UnitVector3D axis = EuclidCoreRandomTools.nextUnitVector3D(random);
         axisAngle = createAxisAngle(axis, angle);
         assertEquals(axis.getX(), axisAngle.getX(), getEpsilon());
         assertEquals(axis.getY(), axisAngle.getY(), getEpsilon());
         assertEquals(axis.getZ(), axisAngle.getZ(), getEpsilon());
         assertEquals(axis.getX32(), axisAngle.getX32(), getEpsilon());
         assertEquals(axis.getY32(), axisAngle.getY32(), getEpsilon());
         assertEquals(axis.getZ32(), axisAngle.getZ32(), getEpsilon());
         assertEquals(angle, axisAngle.getAngle(), getEpsilon());
         assertEquals((float) angle, axisAngle.getAngle32());
      }
   }

   @Test
   public void testContainsNaN()
   {
      T axisAngle;

      axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
      assertFalse(axisAngle.containsNaN());
      axisAngle = createAxisAngle(Double.NaN, 0.0, 0.0, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle = createAxisAngle(0.0, Double.NaN, 0.0, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle = createAxisAngle(0.0, 0.0, Double.NaN, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle = createAxisAngle(0.0, 0.0, 0.0, Double.NaN);
      assertTrue(axisAngle.containsNaN());
   }

   @Test
   @Deprecated
   public void testIsAxisUnitary() throws Exception
   {
      Random random = new Random(51651L);
      AxisAngle axisAngle = new AxisAngle();

      for (int i = 0; i < 20; i++)
      {
         double smallScale = EuclidCoreRandomTools.nextDouble(random, 0.90, 0.95);
         double bigScale = EuclidCoreRandomTools.nextDouble(random, 1.05, 1.10);

         double ux = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double uy = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double uz = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double angle = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double norm = EuclidCoreTools.norm(ux, uy, uz);

         ux /= norm;
         uy /= norm;
         uz /= norm;

         axisAngle.set(ux, uy, uz, angle);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));

         axisAngle.set(ux * smallScale, uy, uz, angle);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy * smallScale, uz, angle);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz * smallScale, angle);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz, angle * smallScale);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));

         axisAngle.set(ux * bigScale, uy, uz, angle);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy * bigScale, uz, angle);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz * bigScale, angle);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz, angle * bigScale);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));
      }
   }

   @Test
   public void testIsOrientation2D() throws Exception
   {
      Random random = new Random(23905872L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double ux = random.nextDouble();
         double uy = random.nextDouble();
         double uz = random.nextDouble();
         double angle = random.nextDouble();
         T axisAngle = createEmptyAxisAngle();
         assertTrue(axisAngle.isOrientation2D(getEpsilon()));

         axisAngle = createAxisAngle(ux, uy, uz, 0.0);
         assertTrue(axisAngle.isOrientation2D(getEpsilon()));

         axisAngle = createAxisAngle(ux, uy, uz, angle);
         assertFalse(axisAngle.isOrientation2D(getEpsilon()));

         axisAngle = createAxisAngle(0.0, uy, uz, angle);
         assertFalse(axisAngle.isOrientation2D(getEpsilon()));

         axisAngle = createAxisAngle(ux, 0.0, uz, angle);
         assertFalse(axisAngle.isOrientation2D(getEpsilon()));

         axisAngle = createAxisAngle(0.0, 0.0, uz, angle);
         assertTrue(axisAngle.isOrientation2D(getEpsilon()));

         axisAngle = createAxisAngle(2.0 * getEpsilon(), 0.0, uz, angle);
         assertFalse(axisAngle.isOrientation2D(getEpsilon()));
         axisAngle = createAxisAngle(0.0, 2.0 * getEpsilon(), uz, angle);
         assertFalse(axisAngle.isOrientation2D(getEpsilon()));
      }
   }

   @Test
   public void testCheckIfIsZOnly() throws Exception
   {
      Random random = new Random(23905872L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double ux = random.nextDouble();
         double uy = random.nextDouble();
         double uz = random.nextDouble();
         double angle = random.nextDouble();
         T axisAngle = createEmptyAxisAngle();
         axisAngle.checkIfOrientation2D(getEpsilon());

         axisAngle = createAxisAngle(ux, uy, uz, 0.0);
         axisAngle.checkIfOrientation2D(getEpsilon());

         axisAngle = createAxisAngle(ux, uy, uz, angle);

         try
         {
            axisAngle.checkIfOrientation2D(getEpsilon());
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a RuntimeException");
         }

         axisAngle = createAxisAngle(0.0, uy, uz, angle);
         try
         {
            axisAngle.checkIfOrientation2D(getEpsilon());
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a RuntimeException");
         }

         axisAngle = createAxisAngle(ux, 0.0, uz, angle);

         try
         {
            axisAngle.checkIfOrientation2D(getEpsilon());
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a RuntimeException");
         }

         axisAngle = createAxisAngle(0.0, 0.0, uz, angle);
         axisAngle.checkIfOrientation2D(getEpsilon());
      }
   }

   @Test
   public void testDistance() throws Exception
   {
      Random random = new Random(32434L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AxisAngleReadOnly aa1 = createRandomAxisAngle(random);
         AxisAngleReadOnly aa2 = createRandomAxisAngle(random);

         Quaternion q1 = new Quaternion(aa1);
         Quaternion q2 = new Quaternion(aa2);

         double actualDistance = aa1.distance(aa2);
         double expectedDistance = q1.distance(q2);
         assertEquals(expectedDistance, actualDistance, getEpsilon());
         assertEquals(0.0, aa1.distance(aa1), getEpsilon());
      }
   }
   
   @Test
   public void testAngle() throws Exception
   {      
      Random random = new Random(564648L);
      T axisAngle;
      for (int i = 0; i < ITERATIONS; i++)
      {
         double angle = EuclidCoreRandomTools.nextDouble(random, 10.0);
         UnitVector3D axis = EuclidCoreRandomTools.nextUnitVector3D(random);
         axisAngle = createAxisAngle(axis, angle);
         assertEquals(angle, axisAngle.angle(), getEpsilon());
      }
   }

   @Test
   public void testGetRotationVector()
   {
      Random random = new Random(2343456L);
      T axisAngle;
      Vector3D actualVector = new Vector3D();
      Vector3D expectedVector = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         axisAngle = createRandomAxisAngle(random);

         actualVector.setToNaN();
         axisAngle.getRotationVector(actualVector);
         RotationVectorConversion.convertAxisAngleToRotationVector(axisAngle, expectedVector);

         EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(actualVector, expectedVector, getEpsilon());
      }
   }

   @Test
   public void testGetYawPitchRoll() throws Exception
   {
      Random random = new Random(2342L);
      T axisAngle;
      for (int i = 0; i < ITERATIONS; i++)
      {
         axisAngle = createRandomAxisAngle(random);

         { // Test getYaw()
            double yaw = axisAngle.getYaw();
            double expectedYaw = YawPitchRollConversion.computeYaw(axisAngle);
            assertEquals(yaw, expectedYaw, getEpsilon());
         }

         { // Test getPitch()
            double pitch = axisAngle.getPitch();
            double expectedPitch = YawPitchRollConversion.computePitch(axisAngle);
            assertEquals(pitch, expectedPitch, getEpsilon());
         }

         { // Test getRoll()
            double roll = axisAngle.getRoll();
            double expectedRoll = YawPitchRollConversion.computeRoll(axisAngle);
            assertEquals(roll, expectedRoll, getEpsilon());
         }
      }
   }

   @Test
   public void testGetDoubleArray()
   {
      Random random = new Random(3513515L);
      T axisAngle;

      { // Test get(double[] axisAngleArray)
         for (int i = 0; i < ITERATIONS; i++)
         {
            double[] axisAngleArray = new double[4];
            axisAngle = createRandomAxisAngle(random);
            axisAngle.get(axisAngleArray);

            assertTrue(axisAngle.getX() == axisAngleArray[0]);
            assertTrue(axisAngle.getY() == axisAngleArray[1]);
            assertTrue(axisAngle.getZ() == axisAngleArray[2]);
            assertTrue(axisAngle.getAngle() == axisAngleArray[3]);
         }
      }

      { // Test get(double[] axisAngleArray, int startIndex)
         for (int i = 0; i < ITERATIONS; i++)
         {
            int startIndex = random.nextInt(20);
            double[] axisAngleArray = new double[startIndex + 4 + random.nextInt(10)];
            axisAngle = createRandomAxisAngle(random);

            axisAngle.get(startIndex, axisAngleArray);

            assertTrue(axisAngle.getX() == axisAngleArray[startIndex + 0]);
            assertTrue(axisAngle.getY() == axisAngleArray[startIndex + 1]);
            assertTrue(axisAngle.getZ() == axisAngleArray[startIndex + 2]);
            assertTrue(axisAngle.getAngle() == axisAngleArray[startIndex + 3]);
         }
      }
   }

   @Test
   public void testGetFloatArray()
   {
      Random random = new Random(3513515L);
      T axisAngle;

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(float[] axisAngleArray)
         float[] axisAngleArray = new float[4];
         axisAngle = createRandomAxisAngle(random);
         axisAngle.get(axisAngleArray);

         assertTrue(axisAngle.getX32() == axisAngleArray[0]);
         assertTrue(axisAngle.getY32() == axisAngleArray[1]);
         assertTrue(axisAngle.getZ32() == axisAngleArray[2]);
         assertTrue(axisAngle.getAngle32() == axisAngleArray[3]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(float[] axisAngleArray, int startIndex)
         int startIndex = random.nextInt(20);
         float[] axisAngleArray = new float[startIndex + 4 + random.nextInt(10)];
         axisAngle = createRandomAxisAngle(random);

         axisAngle.get(startIndex, axisAngleArray);

         assertTrue(axisAngle.getX32() == axisAngleArray[startIndex + 0]);
         assertTrue(axisAngle.getY32() == axisAngleArray[startIndex + 1]);
         assertTrue(axisAngle.getZ32() == axisAngleArray[startIndex + 2]);
         assertTrue(axisAngle.getAngle32() == axisAngleArray[startIndex + 3]);
      }
   }

   @Test
   public void testGetElement() throws Exception
   {
      Random random = new Random(324234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T axisAngle = createRandomAxisAngle(random);
         assertTrue(axisAngle.getX() == axisAngle.getElement(0));
         assertTrue(axisAngle.getY() == axisAngle.getElement(1));
         assertTrue(axisAngle.getZ() == axisAngle.getElement(2));
         assertTrue(axisAngle.getAngle() == axisAngle.getElement(3));

         assertTrue(axisAngle.getX32() == axisAngle.getElement32(0));
         assertTrue(axisAngle.getY32() == axisAngle.getElement32(1));
         assertTrue(axisAngle.getZ32() == axisAngle.getElement32(2));
         assertTrue(axisAngle.getAngle32() == axisAngle.getElement32(3));
      }
   }

   @Test
   public void testTransform()
   {
      Random random = new Random(6787L);
      T axisAngle = createEmptyAxisAngle();
      Quaternion quaternion = new Quaternion();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple3DBasics tupleToTransform)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = EuclidCoreRandomTools.nextVector3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, tuple, expectedTuple);
         axisAngle.transform(actualTuple);

         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = EuclidCoreRandomTools.nextVector3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, tuple, expectedTuple);
         axisAngle.transform(tuple, actualTuple);

         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());

         actualTuple = new Vector3D();
         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(tuple, actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(tuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addTransform(Tuple3DBasics tupleToTransform)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = new Vector3D(tuple);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.addTransform(quaternion, tuple, expectedTuple);
         axisAngle.addTransform(actualTuple);

         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics actualTuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics expectedTuple = new Vector3D(actualTuple);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.addTransform(quaternion, tuple, expectedTuple);
         axisAngle.addTransform(tuple, actualTuple);

         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());

         actualTuple = new Vector3D();
         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(tuple, actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(tuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test subTransform(Tuple3DBasics tupleToTransform)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = new Vector3D(tuple);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.subTransform(quaternion, tuple, expectedTuple);
         axisAngle.subTransform(actualTuple);

         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test subTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics actualTuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics expectedTuple = new Vector3D(actualTuple);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.subTransform(quaternion, tuple, expectedTuple);
         axisAngle.subTransform(tuple, actualTuple);

         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());

         actualTuple = new Vector3D();
         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(tuple, actualTuple);
         EuclidCoreTestTools.assertTuple3DEquals(tuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleToTransform)
         Tuple2DReadOnly tuple = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = EuclidCoreRandomTools.nextVector2D(random);
         double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         axisAngle = createAxisAngle(0.0, 0.0, 1.0, theta);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, tuple, expectedTuple, false);
         axisAngle.transform(actualTuple);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         axisAngle.transform(actualTuple, true);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         axisAngle.transform(actualTuple, false);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(actualTuple);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleTransformed)
         Tuple2DReadOnly tuple = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = EuclidCoreRandomTools.nextVector2D(random);
         double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         axisAngle = createAxisAngle(0.0, 0.0, 1.0, theta);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, tuple, expectedTuple, false);
         axisAngle.transform(tuple, actualTuple);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         axisAngle.transform(tuple, actualTuple, true);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         axisAngle.transform(tuple, actualTuple, false);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());

         actualTuple = new Vector2D();
         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(tuple, actualTuple);
         EuclidCoreTestTools.assertTuple2DEquals(tuple, actualTuple, getEpsilon());
      }

      // Test exceptions
      EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomAxisAngle(random).transform(new Vector2D()), NotAnOrientation2DException.class);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomAxisAngle(random).transform(new Vector2D(), new Vector2D()),
                                                  NotAnOrientation2DException.class);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomAxisAngle(random).transform(new Vector2D(), true), NotAnOrientation2DException.class);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomAxisAngle(random).transform(new Vector2D(), new Vector2D(), true),
                                                  NotAnOrientation2DException.class);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform Matrix3D
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());

         Matrix3D matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D matrixExpected = new Matrix3D();
         Matrix3D matrixActual = new Matrix3D();
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, matrixOriginal, matrixExpected);

         axisAngle.transform(matrixOriginal, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         matrixActual.set(matrixOriginal);
         axisAngle.transform(matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         matrixActual = new Matrix3D();
         axisAngle.transform(matrixOriginal, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixOriginal, matrixActual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {// Test transform quaternion
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());

         Quaternion qOriginal = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion qExpected = new Quaternion();
         Quaternion qActual = new Quaternion();
         quaternion.set(axisAngle);

         qExpected.multiply(quaternion, qOriginal);

         axisAngle.transform(qOriginal, qActual);
         EuclidCoreTestTools.assertEquals(qExpected, qActual, getEpsilon());

         qActual.set(qOriginal);
         axisAngle.transform(qActual);
         EuclidCoreTestTools.assertEquals(qExpected, qActual, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(qActual);
         EuclidCoreTestTools.assertEquals(qExpected, qActual, getEpsilon());

         qActual = new Quaternion();
         axisAngle.transform(qOriginal, qActual);
         EuclidCoreTestTools.assertEquals(qOriginal, qActual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {// Test transform Vector4D
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());

         Vector4D vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D vectorExpected = new Vector4D();
         Vector4D vectorActual = new Vector4D();
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, vectorOriginal, vectorExpected);

         axisAngle.transform(vectorOriginal, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, getEpsilon());

         vectorActual.set(vectorOriginal);
         axisAngle.transform(vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, getEpsilon());

         vectorActual = new Vector4D();
         axisAngle.transform(vectorOriginal, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorOriginal, vectorActual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform RotationMatrix
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());

         RotationMatrix matrixOriginal = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix matrixExpected = new RotationMatrix();
         RotationMatrix matrixActual = new RotationMatrix();
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, matrixOriginal, matrixExpected);

         axisAngle.transform(matrixOriginal, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         matrixActual.set(matrixOriginal);
         axisAngle.transform(matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         matrixActual = new RotationMatrix();
         axisAngle.transform(matrixOriginal, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixOriginal, matrixActual, getEpsilon());
      }
   }

   @Test
   public void testInverseTransform()
   {
      Random random = new Random(6787L);
      T axisAngle = createEmptyAxisAngle();
      Quaternion quaternion = new Quaternion();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple3DBasics tupleToTransform)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = EuclidCoreRandomTools.nextVector3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple);
         axisAngle.inverseTransform(actualTuple);

         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = EuclidCoreRandomTools.nextVector3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple);
         axisAngle.inverseTransform(tuple, actualTuple);

         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleToTransform)
         Tuple2DReadOnly tuple = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = EuclidCoreRandomTools.nextVector2D(random);
         double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         axisAngle = createAxisAngle(0.0, 0.0, 1.0, theta);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple, false);
         axisAngle.inverseTransform(actualTuple);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         axisAngle.inverseTransform(actualTuple, true);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         axisAngle.inverseTransform(actualTuple, false);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleTransformed)
         Tuple2DReadOnly tuple = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = EuclidCoreRandomTools.nextVector2D(random);
         double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         axisAngle = createAxisAngle(0.0, 0.0, 1.0, theta);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple, false);
         axisAngle.inverseTransform(tuple, actualTuple);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         axisAngle.inverseTransform(tuple, actualTuple, true);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         axisAngle.inverseTransform(tuple, actualTuple, false);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      // Test exceptions
      EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomAxisAngle(random).inverseTransform(new Vector2D()), NotAnOrientation2DException.class);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomAxisAngle(random).inverseTransform(new Vector2D(), new Vector2D()),
                                                  NotAnOrientation2DException.class);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomAxisAngle(random).inverseTransform(new Vector2D(), true),
                                                  NotAnOrientation2DException.class);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomAxisAngle(random).inverseTransform(new Vector2D(), new Vector2D(), true),
                                                  NotAnOrientation2DException.class);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(QuaternionBasics quaternionToTransform)
         QuaternionReadOnly original = EuclidCoreRandomTools.nextQuaternion(random);
         QuaternionBasics actual = new Quaternion(original);
         QuaternionBasics expected = EuclidCoreRandomTools.nextQuaternion(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(actual);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
         QuaternionReadOnly original = EuclidCoreRandomTools.nextQuaternion(random);
         QuaternionBasics actual = new Quaternion(original);
         QuaternionBasics expected = EuclidCoreRandomTools.nextQuaternion(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(original, actual);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Vector4DBasics vectorToTransform)
         Vector4DReadOnly original = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D(original);
         Vector4DBasics expected = EuclidCoreRandomTools.nextVector4D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(actual);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
         Vector4DReadOnly original = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D(original);
         Vector4DBasics expected = EuclidCoreRandomTools.nextVector4D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(original, actual);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Matrix3D matrixToTransform)
         Matrix3DReadOnly original = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D actual = new Matrix3D(original);
         Matrix3D expected = EuclidCoreRandomTools.nextMatrix3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
         Matrix3DReadOnly original = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D actual = new Matrix3D(original);
         Matrix3D expected = EuclidCoreRandomTools.nextMatrix3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(original, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(RotationMatrix matrixToTransform)
         RotationMatrixReadOnly original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix(original);
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
         RotationMatrixReadOnly original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix(original);
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(original, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);

      T axisAngle = createRandomAxisAngle(random);

      assertFalse(axisAngle.equals(createEmptyAxisAngle()));
      Object emptyAxisAngleAsObject = createEmptyAxisAngle();
      assertFalse(axisAngle.equals(emptyAxisAngleAsObject));
      Object axisAngleAsObject = axisAngle;
      assertTrue(axisAngle.equals(axisAngleAsObject));
      assertFalse(axisAngle.equals(null));
      assertFalse(axisAngle.equals(new double[5]));

      double x = axisAngle.getX();
      double y = axisAngle.getY();
      double z = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      assertTrue(axisAngle.equals(createAxisAngle(x, y, z, angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x + getSmallestEpsilon(), y, z, angle)));
      assertFalse(axisAngle.equals(createAxisAngle(x - getSmallestEpsilon(), y, z, angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x, y + getSmallestEpsilon(), z, angle)));
      assertFalse(axisAngle.equals(createAxisAngle(x, y - getSmallestEpsilon(), z, angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x, y, z + getSmallestEpsilon(), angle)));
      assertFalse(axisAngle.equals(createAxisAngle(x, y, z - getSmallestEpsilon(), angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x, y, z, angle + getSmallestEpsilon())));
      assertFalse(axisAngle.equals(createAxisAngle(x, y, z, angle - getSmallestEpsilon())));
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();

      T axisAngle = createRandomAxisAngle(random);
      double x = axisAngle.getX();
      double y = axisAngle.getY();
      double z = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle + 0.999 * epsilon), epsilon));
      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle - 0.999 * epsilon), epsilon));

      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle + 1.001 * epsilon), epsilon));
      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle - 1.001 * epsilon), epsilon));

      UnitVector3D vector1 = EuclidCoreRandomTools.nextUnitVector3D(random);
      UnitVector3D vector2 = EuclidCoreRandomTools.nextUnitVector3D(random);

      vector2.set(vector1);
      assertTrue(vector1.epsilonEquals(vector2, epsilon));
      AxisAngle transform = new AxisAngle(EuclidCoreRandomTools.nextOrthogonalVector3D(random, vector1, true), 0.0);
      transform.setAngle(0.999 * epsilon);
      transform.transform(vector1, vector2);
      assertTrue(createAxisAngle(vector1, angle).epsilonEquals(createAxisAngle(vector2, angle), epsilon));
      transform.setAngle(-0.999 * epsilon);
      transform.transform(vector1, vector2);
      assertTrue(createAxisAngle(vector1, angle).epsilonEquals(createAxisAngle(vector2, angle), epsilon));
      transform.setAngle(2.0 * epsilon);
      transform.transform(vector1, vector2);
      assertFalse(createAxisAngle(vector1, angle).epsilonEquals(createAxisAngle(vector2, angle), epsilon));
      transform.setAngle(-2.0 * epsilon);
      transform.transform(vector1, vector2);
      assertFalse(createAxisAngle(vector1, angle).epsilonEquals(createAxisAngle(vector2, angle), epsilon));
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(35454L);

      AxisAngleReadOnly aabA;
      AxisAngle aabB;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         aabA = EuclidCoreRandomTools.nextAxisAngle(random);
         double angleEps = epsilon * 0.99;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleEps);

         aabB = new AxisAngle(aa);
         aabB.preMultiply(aabA);

         assertTrue(aabA.geometricallyEquals(aabB, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         aabA = EuclidCoreRandomTools.nextAxisAngle(random);
         double angleEps = epsilon * 1.01;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleEps);

         aabB = new AxisAngle(aa);
         aabB.preMultiply(aabA);

         assertFalse(aabA.geometricallyEquals(aabB, epsilon));
      }
   }
   

   
}