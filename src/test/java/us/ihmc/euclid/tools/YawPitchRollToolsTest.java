package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;
import static us.ihmc.euclid.tools.EuclidCoreTestTools.assertExceptionIsThrown;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public class YawPitchRollToolsTest
{
   public static final double EPSILON = 1.0e-12;

   @Test
   public void testIsZero()
   {
      Random random = new Random(342);
      double yaw, pitch, roll;

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = random.nextDouble();
         yaw = pitch = roll = 0.0;
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         yaw = Double.NaN;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = Double.NaN;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         roll = Double.NaN;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));

         yaw = pitch = roll = 0.0;
         yaw = epsilon;
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = epsilon;
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         roll = epsilon;
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));

         yaw = pitch = roll = 0.0;
         yaw = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));

         yaw = pitch = roll = 0.0;
         yaw = 1.01 * epsilon;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = 1.01 * epsilon;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         roll = 1.01 * epsilon;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
      }
   }

   @Test
   public void testIsOrientation2D()
   {
      Random random = new Random(342);
      double yaw, pitch, roll;

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = random.nextDouble();
         yaw = pitch = roll = 0.0;
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         yaw = Double.NaN;
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = Double.NaN;
         assertFalse(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         roll = Double.NaN;
         assertFalse(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));

         yaw = EuclidCoreRandomTools.nextDouble(random, 100.0);
         pitch = roll = 0.0;
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         pitch = roll = 0.0;
         pitch = epsilon;
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         pitch = roll = 0.0;
         roll = epsilon;
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));

         pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));

         pitch = roll = 0.0;
         pitch = 1.01 * epsilon;
         assertFalse(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         pitch = roll = 0.0;
         roll = 1.01 * epsilon;
         assertFalse(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
      }
   }

   @Test
   public void testDistance() throws Exception
   {
      Random random = new Random(5321);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll firstYPR = EuclidCoreRandomTools.nextYawPitchRoll(random);
         YawPitchRoll secondYPR = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Quaternion firstQ = new Quaternion(firstYPR);
         Quaternion secondQ = new Quaternion(secondYPR);

         assertEquals(firstQ.distance(secondQ), YawPitchRollTools.distance(firstYPR, secondYPR), EPSILON);
         assertEquals(firstQ.distance(secondQ),
                      YawPitchRollTools.distance(firstYPR.getYaw(),
                                                 firstYPR.getPitch(),
                                                 firstYPR.getRoll(),
                                                 secondYPR.getYaw(),
                                                 secondYPR.getPitch(),
                                                 secondYPR.getRoll()),
                      EPSILON);
      }
      
      // distance test between self(yawPitchRoll) and other(quaternion)
      for(int i = 0; i < ITERATIONS; ++i)
      {
         YawPitchRoll yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRoll(random);
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         YawPitchRoll converted = new YawPitchRoll(quaternion);
         
         Quaternion selfConverted = new Quaternion(yawPitchRoll);
         
         double actualDistance = YawPitchRollTools.distance(yawPitchRoll, quaternion);
         double expectedDistance = QuaternionTools.distance(selfConverted, quaternion);
         
         assertEquals(expectedDistance, actualDistance, EPSILON);
      }
      
      // distance test between self(yawPitchRoll) and other(rotation matrix)
      for(int i = 0; i < ITERATIONS; ++i)
      {
//         System.out.println("iter = " + i);
         YawPitchRoll yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRoll(random);
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix converted = new RotationMatrix(yawPitchRoll);
         
         double actualDistance = YawPitchRollTools.distance(yawPitchRoll, rotationMatrix);
         double expectedDistance = RotationMatrixTools.distance(rotationMatrix, converted);
         
         assertEquals(actualDistance,expectedDistance,EPSILON);
      }
      
   // distance test between self(yawPitchRoll) and other(axis angle)
      for(int i = 0; i < ITERATIONS; ++i)
      {
         System.out.println("iter = " + i);
         YawPitchRoll yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRoll(random);
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         AxisAngle converted = new AxisAngle(yawPitchRoll);
         
         double actualDistance = YawPitchRollTools.distance(yawPitchRoll, axisAngle);
         double expectedDistance = AxisAngleTools.distance(axisAngle, converted);
         
         assertEquals(actualDistance,expectedDistance,EPSILON);
      }

   }

   @Test
   public void testTransform() throws Exception
   {
      Random random = new Random(24546654);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(double yaw, double pitch, double roll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(YawPitchRollReadOnly yawPitchRoll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         YawPitchRollTools.transform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addTransform(double yaw, double pitch, double roll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         YawPitchRollTools.addTransform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).addTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addTransform(YawPitchRollReadOnly yawPitchRoll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         YawPitchRollTools.addTransform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).addTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(double yaw, double pitch, double roll, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics actual = new Point2D();
         Tuple2DBasics expected = new Point2D();

         YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual, false);
         new RotationMatrix(ypr).transform(tupleOriginal, expected, false);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);

         assertExceptionIsThrown(() -> YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual, true),
                                 NotAnOrientation2DException.class);
         ypr.setPitch(0.0);
         assertExceptionIsThrown(() -> YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual, true),
                                 NotAnOrientation2DException.class);
         ypr.setRoll(0.0);
         YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual, true);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(YawPitchRollReadOnly yawPitchRoll, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics actual = new Point2D();
         Tuple2DBasics expected = new Point2D();

         YawPitchRollTools.transform(ypr, tupleOriginal, actual, false);
         new RotationMatrix(ypr).transform(tupleOriginal, expected, false);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);

         assertExceptionIsThrown(() -> YawPitchRollTools.transform(ypr, tupleOriginal, actual, true), NotAnOrientation2DException.class);
         ypr.setPitch(0.0);
         assertExceptionIsThrown(() -> YawPitchRollTools.transform(ypr, tupleOriginal, actual, true), NotAnOrientation2DException.class);
         ypr.setRoll(0.0);
         YawPitchRollTools.transform(ypr, tupleOriginal, actual, true);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(double yaw, double pitch, double roll, Matrix3DReadOnly tupleOriginal, Matrix3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Matrix3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DBasics actual = new Matrix3D();
         Matrix3DBasics expected = new Matrix3D();

         YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(YawPitchRollReadOnly yawPitchRoll, Matrix3DReadOnly tupleOriginal, Matrix3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Matrix3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DBasics actual = new Matrix3D();
         Matrix3DBasics expected = new Matrix3D();

         YawPitchRollTools.transform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(double yaw, double pitch, double roll, RotationMatrixReadOnly tupleOriginal, RotationMatrixBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         RotationMatrixReadOnly tupleOriginal = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix();
         RotationMatrix expected = new RotationMatrix();

         YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(YawPitchRollReadOnly yawPitchRoll, RotationMatrixReadOnly tupleOriginal, RotationMatrixBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         RotationMatrixReadOnly tupleOriginal = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix();
         RotationMatrix expected = new RotationMatrix();

         YawPitchRollTools.transform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(double yaw, double pitch, double roll, Vector4DReadOnly tupleOriginal, Vector4DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Vector4DReadOnly tupleOriginal = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D();
         Vector4DBasics expected = new Vector4D();

         YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(YawPitchRollReadOnly yawPitchRoll, Vector4DReadOnly tupleOriginal, Vector4DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Vector4DReadOnly tupleOriginal = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D();
         Vector4DBasics expected = new Vector4D();

         YawPitchRollTools.transform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testInverseTransform() throws Exception
   {
      Random random = new Random(24546654);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(double yaw, double pitch, double roll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleinverseTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         YawPitchRollTools.inverseTransform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(YawPitchRollReadOnly yawPitchRoll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleinverseTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         YawPitchRollTools.inverseTransform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(double yaw, double pitch, double roll, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleinverseTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics actual = new Point2D();
         Tuple2DBasics expected = new Point2D();

         YawPitchRollTools.inverseTransform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual, false);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected, false);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);

         assertExceptionIsThrown(() -> YawPitchRollTools.inverseTransform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual, true),
                                 NotAnOrientation2DException.class);
         ypr.setPitch(0.0);
         assertExceptionIsThrown(() -> YawPitchRollTools.inverseTransform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual, true),
                                 NotAnOrientation2DException.class);
         ypr.setRoll(0.0);
         YawPitchRollTools.inverseTransform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual, true);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(YawPitchRollReadOnly yawPitchRoll, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleinverseTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics actual = new Point2D();
         Tuple2DBasics expected = new Point2D();

         YawPitchRollTools.inverseTransform(ypr, tupleOriginal, actual, false);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected, false);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);

         assertExceptionIsThrown(() -> YawPitchRollTools.inverseTransform(ypr, tupleOriginal, actual, true), NotAnOrientation2DException.class);
         ypr.setPitch(0.0);
         assertExceptionIsThrown(() -> YawPitchRollTools.inverseTransform(ypr, tupleOriginal, actual, true), NotAnOrientation2DException.class);
         ypr.setRoll(0.0);
         YawPitchRollTools.inverseTransform(ypr, tupleOriginal, actual, true);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(double yaw, double pitch, double roll, Matrix3DReadOnly tupleOriginal, Matrix3DBasics tupleinverseTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Matrix3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DBasics actual = new Matrix3D();
         Matrix3DBasics expected = new Matrix3D();

         YawPitchRollTools.inverseTransform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(YawPitchRollReadOnly yawPitchRoll, Matrix3DReadOnly tupleOriginal, Matrix3DBasics tupleinverseTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Matrix3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DBasics actual = new Matrix3D();
         Matrix3DBasics expected = new Matrix3D();

         YawPitchRollTools.inverseTransform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(double yaw, double pitch, double roll, RotationMatrixReadOnly tupleOriginal, RotationMatrixBasics tupleinverseTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         RotationMatrixReadOnly tupleOriginal = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix();
         RotationMatrix expected = new RotationMatrix();

         YawPitchRollTools.inverseTransform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(YawPitchRollReadOnly yawPitchRoll, RotationMatrixReadOnly tupleOriginal, RotationMatrixBasics tupleinverseTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         RotationMatrixReadOnly tupleOriginal = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix();
         RotationMatrix expected = new RotationMatrix();

         YawPitchRollTools.inverseTransform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(double yaw, double pitch, double roll, Vector4DReadOnly tupleOriginal, Vector4DBasics tupleinverseTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Vector4DReadOnly tupleOriginal = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D();
         Vector4DBasics expected = new Vector4D();

         YawPitchRollTools.inverseTransform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(YawPitchRollReadOnly yawPitchRoll, Vector4DReadOnly tupleOriginal, Vector4DBasics tupleinverseTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Vector4DReadOnly tupleOriginal = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D();
         Vector4DBasics expected = new Vector4D();

         YawPitchRollTools.inverseTransform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(5303298);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll ypr1 = EuclidCoreRandomTools.nextYawPitchRoll(random);
         AxisAngle aa1 = new AxisAngle(ypr1);
         Quaternion q1 = new Quaternion(ypr1);
         RotationMatrix r1 = new RotationMatrix(ypr1);

         YawPitchRoll ypr2 = EuclidCoreRandomTools.nextYawPitchRoll(random);
         AxisAngle aa2 = new AxisAngle(ypr2);
         Quaternion q2 = new Quaternion(ypr2);
         RotationMatrix r2 = new RotationMatrix(ypr2);

         for (byte invertByte = 0; invertByte < 4; invertByte++)
         {
            boolean invert1 = (invertByte & 1) != 0;
            boolean invert2 = (invertByte & 2) != 0;

            Quaternion qExpected = new Quaternion();
            if (invert1)
               qExpected.setAndInvert(q1);
            else
               qExpected.set(q1);
            if (invert2)
               qExpected.multiplyConjugateOther(q2);
            else
               qExpected.multiply(q2);

            YawPitchRoll yprExpected = new YawPitchRoll(qExpected);

            YawPitchRoll yprActual = new YawPitchRoll();
            YawPitchRollTools.multiply(ypr1, invert1, ypr2, invert2, yprActual);
            EuclidCoreTestTools.assertYawPitchRollEquals(yprExpected, yprActual, EPSILON);

            YawPitchRollTools.multiply(aa1, invert1, ypr2, invert2, yprActual);
            EuclidCoreTestTools.assertYawPitchRollEquals(yprExpected, yprActual, EPSILON);
            YawPitchRollTools.multiply(q1, invert1, ypr2, invert2, yprActual);
            EuclidCoreTestTools.assertYawPitchRollEquals(yprExpected, yprActual, EPSILON);
            YawPitchRollTools.multiply(r1, invert1, ypr2, invert2, yprActual);
            EuclidCoreTestTools.assertYawPitchRollEquals(yprExpected, yprActual, EPSILON);

            YawPitchRollTools.multiply(ypr1, invert1, aa2, invert2, yprActual);
            EuclidCoreTestTools.assertYawPitchRollEquals(yprExpected, yprActual, EPSILON);
            YawPitchRollTools.multiply(ypr1, invert1, q2, invert2, yprActual);
            EuclidCoreTestTools.assertYawPitchRollEquals(yprExpected, yprActual, EPSILON);
            YawPitchRollTools.multiply(ypr1, invert1, r2, invert2, yprActual);
            EuclidCoreTestTools.assertYawPitchRollEquals(yprExpected, yprActual, EPSILON);
         }
      }
   }

   @Test
   public void testPrependYawRotation() throws Exception
   {
      Random random = new Random(97);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll original = EuclidCoreRandomTools.nextYawPitchRoll(random);
         YawPitchRoll actual = new YawPitchRoll();
         YawPitchRoll expected = new YawPitchRoll();

         double yaw = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         Quaternion q = new Quaternion();
         q.set(original);
         q.prependYawRotation(yaw);
         expected.set(q);

         YawPitchRollTools.prependYawRotation(original, yaw, actual);
         EuclidCoreTestTools.assertYawPitchRollEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testAppendYawRotation() throws Exception
   {
      Random random = new Random(97);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll original = EuclidCoreRandomTools.nextYawPitchRoll(random);
         YawPitchRoll actual = new YawPitchRoll();
         YawPitchRoll expected = new YawPitchRoll();

         double yaw = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         Quaternion q = new Quaternion();
         q.set(original);
         q.appendYawRotation(yaw);
         expected.set(q);

         YawPitchRollTools.appendYawRotation(original, yaw, actual);
         EuclidCoreTestTools.assertYawPitchRollEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testPrependPitchRotation() throws Exception
   {
      Random random = new Random(97);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll original = EuclidCoreRandomTools.nextYawPitchRoll(random);
         YawPitchRoll actual = new YawPitchRoll();
         YawPitchRoll expected = new YawPitchRoll();

         double pitch = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         Quaternion q = new Quaternion();
         q.set(original);
         q.prependPitchRotation(pitch);
         expected.set(q);

         YawPitchRollTools.prependPitchRotation(original, pitch, actual);
         EuclidCoreTestTools.assertYawPitchRollEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testAppendPitchRotation() throws Exception
   {
      Random random = new Random(97);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll original = EuclidCoreRandomTools.nextYawPitchRoll(random);
         YawPitchRoll actual = new YawPitchRoll();
         YawPitchRoll expected = new YawPitchRoll();

         double pitch = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         Quaternion q = new Quaternion();
         q.set(original);
         q.appendPitchRotation(pitch);
         expected.set(q);

         YawPitchRollTools.appendPitchRotation(original, pitch, actual);
         EuclidCoreTestTools.assertYawPitchRollEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testPrependRollRotation() throws Exception
   {
      Random random = new Random(97);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll original = EuclidCoreRandomTools.nextYawPitchRoll(random);
         YawPitchRoll actual = new YawPitchRoll();
         YawPitchRoll expected = new YawPitchRoll();

         double roll = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         Quaternion q = new Quaternion();
         q.set(original);
         q.prependRollRotation(roll);
         expected.set(q);

         YawPitchRollTools.prependRollRotation(original, roll, actual);
         EuclidCoreTestTools.assertYawPitchRollEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testAppendRollRotation() throws Exception
   {
      Random random = new Random(97);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll original = EuclidCoreRandomTools.nextYawPitchRoll(random);
         YawPitchRoll actual = new YawPitchRoll();
         YawPitchRoll expected = new YawPitchRoll();

         double roll = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);

         Quaternion q = new Quaternion();
         q.set(original);
         q.appendRollRotation(roll);
         expected.set(q);

         YawPitchRollTools.appendRollRotation(original, roll, actual);
         EuclidCoreTestTools.assertYawPitchRollEquals(expected, actual, EPSILON);
      }
   }
}
