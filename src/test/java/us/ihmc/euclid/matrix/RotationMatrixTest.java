package us.ihmc.euclid.matrix;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasicsTest;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

public class RotationMatrixTest extends CommonMatrix3DBasicsTest<RotationMatrix>
{
   public static final double EPS = 1.0e-10;

   @Test
   public void testRotationMatrix()
   {
      Random random = new Random(46876L);
      RotationMatrix actualRotationMatrix = new RotationMatrix();
      RotationMatrix expectedRotationMatrix = new RotationMatrix();

      { // Test RotationMatrix()
         assertTrue(new RotationMatrix().isIdentity());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test RotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         expectedRotationMatrix = createRandomMatrix(random);
         double m00 = expectedRotationMatrix.getM00();
         double m01 = expectedRotationMatrix.getM01();
         double m02 = expectedRotationMatrix.getM02();
         double m10 = expectedRotationMatrix.getM10();
         double m11 = expectedRotationMatrix.getM11();
         double m12 = expectedRotationMatrix.getM12();
         double m20 = expectedRotationMatrix.getM20();
         double m21 = expectedRotationMatrix.getM21();
         double m22 = expectedRotationMatrix.getM22();
         actualRotationMatrix = new RotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationMatrix, actualRotationMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test RotationMatrix(double[] rotationMatrixArray)
         expectedRotationMatrix = createRandomMatrix(random);
         double[] rotationMatrixArray = new double[50];
         expectedRotationMatrix.get(rotationMatrixArray);
         actualRotationMatrix = new RotationMatrix(rotationMatrixArray);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationMatrix, actualRotationMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test RotationMatrix(DenseMatrix64F rotationMatrix)
         expectedRotationMatrix = createRandomMatrix(random);
         DenseMatrix64F rotationMatrixDenseMatrix = new DenseMatrix64F(3, 3);
         expectedRotationMatrix.get(rotationMatrixDenseMatrix);
         actualRotationMatrix = new RotationMatrix(rotationMatrixDenseMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationMatrix, actualRotationMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test RotationMatrix(Matrix3DReadOnly rotationMatrix)
         expectedRotationMatrix = createRandomMatrix(random);
         Matrix3D matrix3D = new Matrix3D(expectedRotationMatrix);
         actualRotationMatrix = new RotationMatrix(matrix3D);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationMatrix, actualRotationMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test RotationMatrix(Matrix3DReadOnly rotationMatrix)
         expectedRotationMatrix = createRandomMatrix(random);
         actualRotationMatrix = new RotationMatrix(expectedRotationMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationMatrix, actualRotationMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test RotationMatrix(AxisAngleBasics axisAngle)
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);

         actualRotationMatrix = new RotationMatrix(axisAngle);
         RotationMatrixConversion.convertAxisAngleToMatrix(axisAngle, expectedRotationMatrix);

         EuclidCoreTestTools.assertMatrix3DEquals(actualRotationMatrix, expectedRotationMatrix, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test RotationMatrix(QuaternionBasics quaternion)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         actualRotationMatrix = new RotationMatrix(quaternion);
         RotationMatrixConversion.convertQuaternionToMatrix(quaternion, expectedRotationMatrix);

         EuclidCoreTestTools.assertMatrix3DEquals(actualRotationMatrix, expectedRotationMatrix, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test RotationMatrix(VectorBasics rotationVector)
         Vector3D rotationVector = EuclidCoreRandomTools.nextVector3D(random);

         actualRotationMatrix = new RotationMatrix(rotationVector);
         RotationMatrixConversion.convertRotationVectorToMatrix(rotationVector, expectedRotationMatrix);

         EuclidCoreTestTools.assertMatrix3DEquals(actualRotationMatrix, expectedRotationMatrix, EPS);
      }
   }

   @Override
   public void testSetDoubles()
   {
      super.testSetDoubles();

      try
      {
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(45.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
         fail("should have thrown a NotARotationMatrixException");
      }
      catch (NotARotationMatrixException e)
      {
         // good
      }
      catch (Exception e)
      {
         fail("should have thrown a NotARotationMatrixException");
      }
   }

   @Test
   public void testSetToZero()
   {
      RotationMatrix rotationMatrix = new RotationMatrix();
      RotationMatrix identityMatrix = new RotationMatrix();
      identityMatrix.setToNaN();
      rotationMatrix.setToNaN();

      EuclidCoreTestTools.assertMatrix3DContainsOnlyNaN(identityMatrix);
      EuclidCoreTestTools.assertMatrix3DContainsOnlyNaN(rotationMatrix);

      identityMatrix.setIdentity();
      rotationMatrix.setToZero();

      EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, identityMatrix, EPS);
   }

   @Test
   public void testCheckIfMatrixProper() throws Exception
   {
      Random random = new Random(46876L);
      Matrix3D matrix, matrixCopy = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         matrixCopy.set(matrix);

         RotationMatrix rotationMatrix = new RotationMatrix();

         try
         {
            rotationMatrix = new RotationMatrix(matrix);
            rotationMatrix.checkIfRotationMatrix();
            assertTrue(matrix.isRotationMatrix());
         }
         catch (RuntimeException e)
         {
            if (matrix.isRotationMatrix())
               throw e;
            // else it is good
         }

         EuclidCoreTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
      }
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(648967L);

      { // Test set(RotationMatrix other)
         Matrix3D expectedMatrix;
         RotationMatrix rotationMatrix = new RotationMatrix(), expectedRotationMatrix;

         for (int i = 0; i < ITERATIONS; i++)
         {
            expectedMatrix = EuclidCoreRandomTools.nextMatrix3D(random);
            expectedRotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
            rotationMatrix.setToNaN();

            rotationMatrix.set(expectedRotationMatrix);
            EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, expectedRotationMatrix, EPS);

            try
            {
               rotationMatrix.set(expectedMatrix);
               EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, rotationMatrix, EPS);
            }
            catch (RuntimeException e)
            {
               if (expectedMatrix.isRotationMatrix())
                  throw e;
               // else it is good
            }
         }
      }

      { // Test set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         RotationMatrix rotationMatrix = new RotationMatrix();

         try
         {
            rotationMatrix.set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            fail("Should have thrown a NotARotationMatrixException.");
         }
         catch (NotARotationMatrixException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotARotationMatrixException.");
         }

         for (int i = 0; i < ITERATIONS; i++)
         {

            RotationMatrix other = EuclidCoreRandomTools.nextRotationMatrix(random);
            double m00 = other.getM00();
            double m01 = other.getM01();
            double m02 = other.getM02();
            double m10 = other.getM10();
            double m11 = other.getM11();
            double m12 = other.getM12();
            double m20 = other.getM20();
            double m21 = other.getM21();
            double m22 = other.getM22();
            rotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
            EuclidCoreTestTools.assertMatrix3DEquals(other, rotationMatrix, SMALL_EPS);
         }
      }

      { // Test setUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         RotationMatrix rotationMatrix = new RotationMatrix();

         rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == 0.0);
         }

         for (int i = 0; i < ITERATIONS; i++)
         {

            RotationMatrix other = EuclidCoreRandomTools.nextRotationMatrix(random);
            double m00 = other.getM00();
            double m01 = other.getM01();
            double m02 = other.getM02();
            double m10 = other.getM10();
            double m11 = other.getM11();
            double m12 = other.getM12();
            double m20 = other.getM20();
            double m21 = other.getM21();
            double m22 = other.getM22();
            rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
            EuclidCoreTestTools.assertMatrix3DEquals(other, rotationMatrix, SMALL_EPS);
         }
      }

      { // Test set(double[] rotationMatrixArray)
         double[] matrixArray = new double[9];

         RotationMatrix rotationMatrix = new RotationMatrix();
         try
         {
            rotationMatrix.set(matrixArray);
            fail("Should have thrown a NotARotationMatrixException.");
         }
         catch (NotARotationMatrixException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotARotationMatrixException.");
         }

         for (int i = 0; i < ITERATIONS; i++)
         {
            RotationMatrix other = EuclidCoreRandomTools.nextRotationMatrix(random);
            double m00 = other.getM00();
            double m01 = other.getM01();
            double m02 = other.getM02();
            double m10 = other.getM10();
            double m11 = other.getM11();
            double m12 = other.getM12();
            double m20 = other.getM20();
            double m21 = other.getM21();
            double m22 = other.getM22();

            matrixArray = new double[] {m00, m01, m02, m10, m11, m12, m20, m21, m22};
            rotationMatrix.set(matrixArray);
            EuclidCoreTestTools.assertMatrix3DEquals(other, rotationMatrix, SMALL_EPS);
         }
      }

      { // Test set(DenseMatrix64F matrix)
         for (int i = 0; i < ITERATIONS; i++)
         {
            RotationMatrix actualMatrix = new RotationMatrix();
            RotationMatrix randomRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
            DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
            for (int row = 0; row < 3; row++)
            {
               for (int column = 0; column < 3; column++)
               {
                  denseMatrix.set(row, column, randomRotation.getElement(row, column));
               }
            }

            actualMatrix.set(denseMatrix);

            for (int row = 0; row < 3; row++)
            {
               for (int column = 0; column < 3; column++)
               {
                  assertTrue(denseMatrix.get(row, column) == actualMatrix.getElement(row, column));
               }
            }
         }
      }

      { // Test set(DenseMatrix64F matrix, int startRow, int startColumn)
         for (int i = 0; i < ITERATIONS; i++)
         {
            RotationMatrix actualMatrix = new RotationMatrix();
            RotationMatrix randomRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
            int startRow = random.nextInt(10);
            int startColumn = random.nextInt(10);
            DenseMatrix64F denseMatrix = new DenseMatrix64F(3 + startRow, 3 + startColumn);

            for (int row = 0; row < 3; row++)
            {
               for (int column = 0; column < 3; column++)
               {
                  denseMatrix.set(row + startRow, column + startColumn, randomRotation.getElement(row, column));
               }
            }

            actualMatrix.set(startRow, startColumn, denseMatrix);

            for (int row = 0; row < 3; row++)
            {
               for (int column = 0; column < 3; column++)
               {
                  assertTrue(denseMatrix.get(row + startRow, column + startColumn) == actualMatrix.getElement(row, column));
               }
            }
         }
      }

      { // Test setColumns(Tuple3DReadOnly firstColumn, Tuple3DReadOnly secondColumn, Tuple3DReadOnly thirdColumn)
         RotationMatrix rotationMatrix = new RotationMatrix();
         Vector3D first = new Vector3D();
         Vector3D second = new Vector3D();
         Vector3D third = new Vector3D();

         try
         {
            rotationMatrix.setColumns(first, second, third);
            fail("Should have thrown a NotARotationMatrixException.");
         }
         catch (NotARotationMatrixException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotARotationMatrixException.");
         }

         for (int i = 0; i < ITERATIONS; i++)
         {
            RotationMatrix other = EuclidCoreRandomTools.nextRotationMatrix(random);

            for (int row = 0; row < 3; row++)
            {
               first.setElement(row, other.getElement(row, 0));
               second.setElement(row, other.getElement(row, 1));
               third.setElement(row, other.getElement(row, 2));
            }

            rotationMatrix.setColumns(first, second, third);
            EuclidCoreTestTools.assertMatrix3DEquals(other, rotationMatrix, SMALL_EPS);
         }
      }

      { // Test setRows(Tuple3DReadOnly firstRow, Tuple3DReadOnly secondRow, Tuple3DReadOnly thirdRow)
         RotationMatrix rotationMatrix = new RotationMatrix();
         Vector3D first = new Vector3D();
         Vector3D second = new Vector3D();
         Vector3D third = new Vector3D();

         try
         {
            rotationMatrix.setRows(first, second, third);
            fail("Should have thrown a NotARotationMatrixException.");
         }
         catch (NotARotationMatrixException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotARotationMatrixException.");
         }

         for (int i = 0; i < ITERATIONS; i++)
         {

            RotationMatrix other = EuclidCoreRandomTools.nextRotationMatrix(random);
            for (int column = 0; column < 3; column++)
            {
               first.setElement(column, other.getElement(0, column));
               second.setElement(column, other.getElement(1, column));
               third.setElement(column, other.getElement(2, column));
            }

            rotationMatrix.setRows(first, second, third);
            EuclidCoreTestTools.assertMatrix3DEquals(other, rotationMatrix, SMALL_EPS);
         }
      }
   }

   @Test
   public void testAppendYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);

      RotationMatrix expected = new RotationMatrix();
      RotationMatrix actual = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      { // appendYawRotation(double yaw)
         RotationMatrix original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix yawRotation = new RotationMatrix();
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         yawRotation.setToYawMatrix(yaw);
         RotationMatrixTools.multiply(original, yawRotation, expected);

         actual.set(original);
         actual.appendYawRotation(yaw);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendPitchRotation(double pitch)
         RotationMatrix original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix pitchRotation = new RotationMatrix();
         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         pitchRotation.setToPitchMatrix(pitch);
         RotationMatrixTools.multiply(original, pitchRotation, expected);

         actual.set(original);
         actual.appendPitchRotation(pitch);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendRollRotation(double roll)
         RotationMatrix original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix rollRotation = new RotationMatrix();
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         rollRotation.setToRollMatrix(roll);
         RotationMatrixTools.multiply(original, rollRotation, expected);

         actual.set(original);
         actual.appendRollRotation(roll);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPrependYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);

      RotationMatrix expected = new RotationMatrix();
      RotationMatrix actual = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      { // prependYawRotation(double yaw)
         RotationMatrix original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix yawRotation = new RotationMatrix();
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         yawRotation.setToYawMatrix(yaw);
         RotationMatrixTools.multiply(yawRotation, original, expected);

         actual.set(original);
         actual.prependYawRotation(yaw);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependPitchRotation(double pitch)
         RotationMatrix original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix pitchRotation = new RotationMatrix();
         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         pitchRotation.setToPitchMatrix(pitch);
         RotationMatrixTools.multiply(pitchRotation, original, expected);

         actual.set(original);
         actual.prependPitchRotation(pitch);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependRollRotation(double roll)
         RotationMatrix original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix rollRotation = new RotationMatrix();
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         rollRotation.setToRollMatrix(roll);
         RotationMatrixTools.multiply(rollRotation, original, expected);

         actual.set(original);
         actual.prependRollRotation(roll);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(3245235);

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests interpolate(RotationMatrixReadOnly rf, double alpha)
         RotationMatrix actual = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix expected = new RotationMatrix();

         RotationMatrix rf = EuclidCoreRandomTools.nextRotationMatrix(random);
         Quaternion qf = new Quaternion(rf);

         double alpha = random.nextDouble();

         Quaternion qInterpolated = new Quaternion(actual);
         qInterpolated.interpolate(qf, alpha);
         expected.set(qInterpolated);

         actual.interpolate(rf, alpha);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, 1.0e-5);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests interpolate(RotationMatrixReadOnly r0, RotationMatrixReadOnly rf, double alpha)
         RotationMatrix r0 = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix rf = EuclidCoreRandomTools.nextRotationMatrix(random);
         Quaternion q0 = new Quaternion(r0);
         Quaternion qf = new Quaternion(rf);

         RotationMatrix actual = new RotationMatrix();
         RotationMatrix expected = new RotationMatrix();

         double alpha = random.nextDouble();

         Quaternion qInterpolated = new Quaternion();
         qInterpolated.interpolate(q0, qf, alpha);
         expected.set(qInterpolated);

         actual.interpolate(r0, rf, alpha);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, 1.0e-5);
      }
   }

   @Test
   public void testSetToYawPitchRollMatrix()
   {
      Random random = new Random(35454L);
      RotationMatrix rotationMatrix, rotationMatrixCopy = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);

         { // Test setToPitchMatrix()
            double pitch, pitchCopy;
            pitch = pitchCopy = random.nextDouble();

            rotationMatrix.setToNaN();
            rotationMatrixCopy.setToNaN();

            rotationMatrix.setToPitchMatrix(pitch);
            RotationMatrixConversion.computePitchMatrix(pitch, rotationMatrixCopy);

            EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);

            assertTrue(pitch == pitchCopy);
         }

         { // Test setToRollMatrix()
            double roll, rollCopy;
            roll = rollCopy = random.nextDouble();

            rotationMatrix.setToNaN();
            rotationMatrixCopy.setToNaN();

            rotationMatrix.setToRollMatrix(roll);
            RotationMatrixConversion.computeRollMatrix(roll, rotationMatrixCopy);

            EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);

            assertTrue(roll == rollCopy);
         }

         { // Test setToYawMatrix()
            double yaw, yawCopy;
            yaw = yawCopy = random.nextDouble();

            rotationMatrix.setToNaN();
            rotationMatrixCopy.setToNaN();

            rotationMatrix.setToYawMatrix(yaw);
            RotationMatrixConversion.computeYawMatrix(yaw, rotationMatrixCopy);

            EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);

            assertTrue(yaw == yawCopy);
         }
      }
   }

   @Test
   public void testSetYawPitchRoll()
   {
      Random random = new Random(6465L);
      RotationMatrix rotationMatrix, rotationMatrixCopy = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setYawPitchRoll (double[] yawPitchRoll)
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);

         double[] yawPitchRoll, yawPitchRollCopy;
         yawPitchRoll = yawPitchRollCopy = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble()};

         rotationMatrix.setYawPitchRoll(yawPitchRoll);
         RotationMatrixConversion.convertYawPitchRollToMatrix(yawPitchRoll, rotationMatrixCopy);

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         assertTrue(yawPitchRoll == yawPitchRollCopy);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setYawPitchRoll(double yaw, double pitch, double roll)
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);

         double[] yawPitchRoll, yawPitchRollCopy;
         yawPitchRoll = yawPitchRollCopy = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble()};

         rotationMatrix.setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
         RotationMatrixConversion.convertYawPitchRollToMatrix(yawPitchRoll, rotationMatrixCopy);

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         assertTrue(yawPitchRoll == yawPitchRollCopy);
      }
   }

   @Test
   public void testSetEuler()
   {
      Random random = new Random(65466L);
      RotationMatrix rotationMatrix, rotationMatrixCopy;
      RotationMatrix yawPitchRoll = new RotationMatrix();
      RotationMatrix expected = new RotationMatrix();
      Vector3D eulerAngles, eulerAnglesCopy;

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setEuler(VectorBasics eulerAngles)
         rotationMatrix = rotationMatrixCopy = EuclidCoreRandomTools.nextRotationMatrix(random);
         eulerAngles = eulerAnglesCopy = EuclidCoreRandomTools.nextVector3D(random);
         yawPitchRoll.setEuler(eulerAngles);
         RotationMatrixConversion.convertYawPitchRollToMatrix(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX(), expected);

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(eulerAngles, eulerAnglesCopy, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(yawPitchRoll, expected, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setEuler(double rotX, double rotY, double rotZ)
         rotationMatrix = rotationMatrixCopy = EuclidCoreRandomTools.nextRotationMatrix(random);
         eulerAngles = eulerAnglesCopy = EuclidCoreRandomTools.nextVector3D(random);
         yawPitchRoll.setEuler(eulerAngles.getX(), eulerAngles.getY(), eulerAngles.getZ());
         RotationMatrixConversion.convertYawPitchRollToMatrix(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX(), expected);

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(eulerAngles, eulerAnglesCopy, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(yawPitchRoll, expected, EPS);
      }
   }

   @Test
   public void testDistance() throws Exception
   {
      Random random = new Random(3242L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix m1 = createRandomMatrix(random);
         RotationMatrix m2 = createRandomMatrix(random);

         double actualDistance = m1.distance(m2);
         double expectedDistance = RotationMatrixTools.distance(m1, m2);
         assertEquals(expectedDistance, actualDistance, EPS);
      }
   }

   @Test
   public void testGet()
   {
      Random random = new Random(6841L);
      RotationMatrix rotationMatrix, rotationMatrixCopy = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(VectorBasics rotationVectorToPack)
         rotationMatrix = rotationMatrixCopy = EuclidCoreRandomTools.nextRotationMatrix(random);

         Vector3D vector = new Vector3D();
         Vector3D expectedVector = new Vector3D();

         rotationMatrix.getRotationVector(vector);
         RotationVectorConversion.convertMatrixToRotationVector(rotationMatrix, expectedVector);

         EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(vector, expectedVector, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }
   }

   @Test
   public void testGetEuler()
   {
      Random random = new Random(65466L);
      RotationMatrix yawPitchRoll = new RotationMatrix(), expected;
      Vector3D eulerAngles = new Vector3D();
      Vector3D eulerAnglesCopy = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
         try
         {
            expected = yawPitchRoll = EuclidCoreRandomTools.nextRotationMatrix(random);
            yawPitchRoll.getEuler(eulerAngles);
            YawPitchRollConversion.convertMatrixToYawPitchRoll(expected, eulerAnglesCopy);

            EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(eulerAngles, eulerAnglesCopy, EPS);
            EuclidCoreTestTools.assertMatrix3DEquals(yawPitchRoll, expected, EPS);
         }
         catch (AssertionError e)
         {
            double pitch = YawPitchRollConversion.computePitch(yawPitchRoll);
            if (!Double.isNaN(pitch))
               throw e;
         }
   }

   @Test
   public void testGetToYawPitchRollMatrix()
   {
      Random random = new Random(35454L);
      RotationMatrix rotationMatrix, expectedMatrix;

      for (int i = 0; i < ITERATIONS; i++)
      {
         rotationMatrix = expectedMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);

         { // Test getToPitchMatrix()
            double pitch = rotationMatrix.getPitch();
            double expectedPitch = YawPitchRollConversion.computePitch(expectedMatrix);

            assertEquals(pitch, expectedPitch, EPS);

            EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);
         }

         { // Test getToRollMatrix()

            double roll = rotationMatrix.getRoll();
            double expectedRoll = YawPitchRollConversion.computeRoll(expectedMatrix);

            assertEquals(roll, expectedRoll, EPS);

            EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);
         }

         { // Test getToYawMatrix()
            double yaw = rotationMatrix.getYaw();
            double expectedYaw = YawPitchRollConversion.computeYaw(expectedMatrix);

            assertEquals(yaw, expectedYaw, EPS);

            EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);

            EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);
         }
      }
   }

   @Test
   public void testGetYawPitchRoll()
   {
      Random random = new Random(35454L);
      RotationMatrix rotationMatrix = new RotationMatrix(), expectedMatrix = new RotationMatrix();
      double[] yawPitchRoll, yawPitchRollCopy = new double[3];

      for (int i = 0; i < ITERATIONS; i++)
      {
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         expectedMatrix.set(rotationMatrix);
         yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         yawPitchRollCopy = yawPitchRoll;

         rotationMatrix.getYawPitchRoll(yawPitchRoll);
         RotationMatrixConversion.convertYawPitchRollToMatrix(yawPitchRoll, expectedMatrix);

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);
         assertTrue(yawPitchRoll == yawPitchRollCopy);
      }
   }

   @Test
   public void testApplyTransform()
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RotationMatrix original = createRandomMatrix(random);
         RotationMatrix expected = new RotationMatrix();
         RotationMatrix actual = new RotationMatrix();

         expected.set(original);
         expected.preMultiply(transform.getRotationMatrix());
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);

         double m00 = original.getM00();
         double m01 = original.getM01();
         double m02 = original.getM02();
         double m10 = original.getM10();
         double m11 = original.getM11();
         double m12 = original.getM12();
         double m20 = original.getM20();
         double m21 = original.getM21();
         double m22 = original.getM22();
         actual.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         RotationMatrix original = createRandomMatrix(random);
         RotationMatrix expected = new RotationMatrix();
         RotationMatrix actual = new RotationMatrix();

         expected.set(original);
         expected.prepend(transform.getQuaternion());
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         RotationMatrix original = createRandomMatrix(random);
         RotationMatrix expected = new RotationMatrix();
         RotationMatrix actual = new RotationMatrix();

         expected.set(original);
         expected.preMultiply(transform.getRotationMatrix());
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testApplyInverseTransform()
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RotationMatrix original = createRandomMatrix(random);
         RotationMatrix expected = new RotationMatrix();
         RotationMatrix actual = new RotationMatrix();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         RotationMatrix original = createRandomMatrix(random);
         RotationMatrix expected = new RotationMatrix();
         RotationMatrix actual = new RotationMatrix();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         RotationMatrix original = createRandomMatrix(random);
         RotationMatrix expected = new RotationMatrix();
         RotationMatrix actual = new RotationMatrix();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testInvert()
   {
      Random random = new Random(65474L);
      RotationMatrix rotationMatrix, expectedMatrix = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         expectedMatrix.set(rotationMatrix);

         rotationMatrix.invert();
         expectedMatrix.transpose();

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);
      }
   }

   @Test
   public void testMultiply()
   {
      Random random = new Random(645864L);
      RotationMatrix multiplied, expected = new RotationMatrix();
      RotationMatrix rotationMatrix, rotationMatrixCopy = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test multiply(RotationMatrixReadOnly other)
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = EuclidCoreRandomTools.nextRotationMatrix(random);
         expected.set(multiplied);

         multiplied.multiply(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiply(expected, rotationMatrixCopy, expected);
         expected.normalize();

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test multiplyTransposeThis(RotationMatrixReadOnly other)
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = EuclidCoreRandomTools.nextRotationMatrix(random);
         expected.set(multiplied);

         multiplied.multiplyTransposeThis(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeLeft(expected, rotationMatrixCopy, expected);
         expected.normalize();

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test multiplyTransposeOther(RotationMatrixReadOnly other)
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = EuclidCoreRandomTools.nextRotationMatrix(random);
         expected.set(multiplied);

         multiplied.multiplyTransposeOther(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeRight(expected, rotationMatrixCopy, expected);
         expected.normalize();

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test multiplyTransposeBoth(RotationMatrixReadOnly other)
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = EuclidCoreRandomTools.nextRotationMatrix(random);
         expected.set(multiplied);

         multiplied.multiplyTransposeBoth(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeBoth(expected, rotationMatrixCopy, expected);
         expected.normalize();

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }
   }

   @Test
   public void testNumericalError() throws Exception
   {
      Random random = new Random(53463);

      { // Chain multiplication to control numerical errors.
         RotationMatrix unnormalized = new RotationMatrix();
         RotationMatrix normalized = new RotationMatrix();

         for (int i = 0; i < 269000; i++)
         {
            RotationMatrixReadOnly multiplyWith = EuclidCoreRandomTools.nextRotationMatrix(random);
            unnormalized.multiply(multiplyWith);
            normalized.multiply(multiplyWith);
            normalized.normalize();
            EuclidCoreTestTools.assertMatrix3DEquals(normalized, unnormalized, EPS);
            assertTrue(unnormalized.isRotationMatrix(1.0e-11), "At multiplication #" + i + ".");
         }
      }

      { // Chain multiplication with quaternion to control numerical errors.
         RotationMatrix unnormalized = new RotationMatrix();
         RotationMatrix normalized = new RotationMatrix();

         for (int i = 0; i < 116000; i++)
         {
            Quaternion multiplyWith = EuclidCoreRandomTools.nextQuaternion(random);
            unnormalized.append(multiplyWith);
            normalized.append(multiplyWith);
            normalized.normalize();
            EuclidCoreTestTools.assertMatrix3DEquals(normalized, unnormalized, EPS);
            assertTrue(unnormalized.isRotationMatrix(1.0e-11), "At multiplication #" + i + ".");
         }
      }

      { // Chain multiplication with axis-angle to control numerical errors.
         RotationMatrix unnormalized = new RotationMatrix();
         RotationMatrix normalized = new RotationMatrix();

         for (int i = 0; i < 268000; i++)
         {
            AxisAngle multiplyWith = EuclidCoreRandomTools.nextAxisAngle(random);
            unnormalized.append(multiplyWith);
            normalized.append(multiplyWith);
            normalized.normalize();
            EuclidCoreTestTools.assertMatrix3DEquals(normalized, unnormalized, EPS);
            assertTrue(unnormalized.isRotationMatrix(1.0e-11), "At multiplication #" + i + ".");
         }
      }
   }

   @Test
   public void testPreMultiply()
   {
      Random random = new Random(645864L);
      RotationMatrix multiplied, expected = new RotationMatrix();
      RotationMatrix rotationMatrix, rotationMatrixCopy = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test preMultiply(RotationMatrixReadOnly other)
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = EuclidCoreRandomTools.nextRotationMatrix(random);
         expected.set(multiplied);

         multiplied.preMultiply(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiply(rotationMatrixCopy, expected, expected);
         expected.normalize();

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test preMultiplyTransposeThis(RotationMatrixReadOnly other)
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = EuclidCoreRandomTools.nextRotationMatrix(random);
         expected.set(multiplied);

         multiplied.preMultiplyTransposeThis(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeRight(rotationMatrixCopy, expected, expected);
         expected.normalize();

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test preMultiplyTransposeOther(RotationMatrixReadOnly other)
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = EuclidCoreRandomTools.nextRotationMatrix(random);
         expected.set(multiplied);

         multiplied.preMultiplyTransposeOther(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeLeft(rotationMatrixCopy, expected, expected);
         expected.normalize();

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test preMultiplyTransposeBoth(RotationMatrixReadOnly other)
         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = EuclidCoreRandomTools.nextRotationMatrix(random);
         expected.set(multiplied);

         multiplied.preMultiplyTransposeBoth(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeBoth(rotationMatrixCopy, expected, expected);
         expected.normalize();

         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }
   }

   @Test
   public void testNormalize() throws Exception
   {
      Random random = new Random(39456L);
      RotationMatrix matrixExpected = new RotationMatrix();
      RotationMatrix matrixActual = new RotationMatrix();

      { // Check that identity does not get modified
         matrixActual.setIdentity();
         matrixExpected.setIdentity();

         matrixActual.normalize();
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      // Test that normalizing a proper rotation matrix does not change it.
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrixExpected.set(EuclidCoreRandomTools.nextRotationMatrix(random));
         matrixActual.set(matrixExpected);

         matrixActual.normalize();
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      Vector3D vector1 = new Vector3D();
      Vector3D vector2 = new Vector3D();

      // Test that it actually makes a random matrix ortho-normal
      double corruptionFactor = 0.1;
      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix randomRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         double m00 = randomRotation.getM00() + corruptionFactor * random.nextDouble();
         double m01 = randomRotation.getM01() + corruptionFactor * random.nextDouble();
         double m02 = randomRotation.getM02() + corruptionFactor * random.nextDouble();
         double m10 = randomRotation.getM10() + corruptionFactor * random.nextDouble();
         double m11 = randomRotation.getM11() + corruptionFactor * random.nextDouble();
         double m12 = randomRotation.getM12() + corruptionFactor * random.nextDouble();
         double m20 = randomRotation.getM20() + corruptionFactor * random.nextDouble();
         double m21 = randomRotation.getM21() + corruptionFactor * random.nextDouble();
         double m22 = randomRotation.getM22() + corruptionFactor * random.nextDouble();
         matrixActual.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         matrixActual.normalize();

         // Test that each row & column vectors are unit-length
         for (int j = 0; j < 3; j++)
         {
            matrixActual.getRow(j, vector1);
            assertEquals(1.0, vector1.length(), EPS);

            matrixActual.getColumn(j, vector1);
            assertEquals(1.0, vector1.length(), EPS);
         }

         // Test that each pair of rows and each pair of columns are orthogonal
         for (int j = 0; j < 3; j++)
         {
            matrixActual.getRow(j, vector1);
            matrixActual.getRow((j + 1) % 3, vector2);
            assertEquals(0.0, vector1.dot(vector2), EPS);

            matrixActual.getColumn(j, vector1);
            matrixActual.getColumn((j + 1) % 3, vector2);
            assertEquals(0.0, vector1.dot(vector2), EPS);
         }
      }
   }

   @Test
   public void testSetAndNormalize() throws Exception
   {
      Random random = new Random(39456L);
      RotationMatrix matrixExpected = new RotationMatrix();
      RotationMatrix matrixActual = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         matrixExpected = EuclidCoreRandomTools.nextRotationMatrix(random);
         double m00 = matrixExpected.getM00();
         double m01 = matrixExpected.getM01();
         double m02 = matrixExpected.getM02();
         double m10 = matrixExpected.getM10();
         double m11 = matrixExpected.getM11();
         double m12 = matrixExpected.getM12();
         double m20 = matrixExpected.getM20();
         double m21 = matrixExpected.getM21();
         double m22 = matrixExpected.getM22();
         matrixActual.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

         matrixActual.setToNaN();
         matrixActual.setAndNormalize(matrixExpected);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

         matrixActual.setToNaN();
         matrixActual.setAndNormalize((Matrix3DReadOnly) matrixExpected);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

         matrixActual.setToNaN();
         matrixActual.setAndNormalize((Orientation3DReadOnly) matrixExpected);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      double corruptionFactor = 0.1;
      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix corrupted = EuclidCoreRandomTools.nextRotationMatrix(random);
         double m00 = corrupted.getM00() + corruptionFactor * random.nextDouble();
         double m01 = corrupted.getM01() + corruptionFactor * random.nextDouble();
         double m02 = corrupted.getM02() + corruptionFactor * random.nextDouble();
         double m10 = corrupted.getM10() + corruptionFactor * random.nextDouble();
         double m11 = corrupted.getM11() + corruptionFactor * random.nextDouble();
         double m12 = corrupted.getM12() + corruptionFactor * random.nextDouble();
         double m20 = corrupted.getM20() + corruptionFactor * random.nextDouble();
         double m21 = corrupted.getM21() + corruptionFactor * random.nextDouble();
         double m22 = corrupted.getM22() + corruptionFactor * random.nextDouble();
         corrupted.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);

         matrixExpected.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         matrixExpected.normalize();
         matrixActual.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

         matrixActual.setToNaN();
         matrixActual.setAndNormalize(corrupted);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

         matrixActual.setToNaN();
         matrixActual.setAndNormalize((Matrix3DReadOnly) corrupted);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

         matrixActual.setToNaN();
         matrixActual.setAndNormalize((Orientation3DReadOnly) corrupted);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }
   }

   @Test
   public void testSetAndInvert() throws Exception
   {
      Random random = new Random(545L);
      RotationMatrix matrixActual = new RotationMatrix();
      RotationMatrix matrixExpected = new RotationMatrix();

      RotationMatrix randomMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
      matrixExpected.set(randomMatrix);
      matrixExpected.invert();

      matrixActual.setAndInvert(randomMatrix);
      EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

      matrixActual.setToNaN();
      matrixActual.setAndInvert((Matrix3DReadOnly) randomMatrix);
      EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
   }

   @Test
   public void testSetAndTranspose() throws Exception
   {
      Random random = new Random(545L);
      RotationMatrix matrixActual = new RotationMatrix();
      RotationMatrix matrixExpected = new RotationMatrix();

      RotationMatrix randomMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
      matrixExpected.set(randomMatrix);
      matrixExpected.transpose();

      matrixActual.setAndTranspose(randomMatrix);
      EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

      matrixActual.setToNaN();
      matrixActual.setAndTranspose((Matrix3DReadOnly) randomMatrix);
      EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
   }

   @Test
   public void testTransformTuple() throws Exception
   {
      Random random = new Random(435L);
      Vector3D actual = new Vector3D();
      Vector3D expected = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector3D original = EuclidCoreRandomTools.nextVector3D(random);

         Matrix3DTools.transform(matrix, original, expected);
         actual.set(original);
         matrix.transform(actual);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformTuple2D() throws Exception
   {
      Random random = new Random(435L);
      Vector2D actual = new Vector2D();
      Vector2D expected = new Vector2D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix matrix = new RotationMatrix();
         matrix.setToYawMatrix(2.0 * Math.PI * random.nextDouble());
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);

         Matrix3DTools.transform(matrix, original, expected, true);
         actual.set(original);
         matrix.transform(actual);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);

         actual.setToNaN();
         Matrix3DTools.transform(matrix, original, expected, true);
         actual.set(original);
         matrix.transform(actual, true);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual, true);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformMatrix() throws Exception
   {
      Random random = new Random(435L);
      Matrix3D actual = new Matrix3D();
      Matrix3D expected = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Matrix3D original = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.transform(matrix, original, expected);
         actual.set(original);
         matrix.transform(actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformQuaternion() throws Exception
   {
      Random random = new Random(435L);
      Quaternion actual = new Quaternion();
      Quaternion expected = new Quaternion();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Quaternion original = EuclidCoreRandomTools.nextQuaternion(random);

         matrix.transform(original, expected);
         actual.set(original);
         matrix.transform(actual);
         EuclidCoreTestTools.assertQuaternionEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual);
         EuclidCoreTestTools.assertQuaternionEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformVector4D() throws Exception
   {
      Random random = new Random(435L);
      Vector4D actual = new Vector4D();
      Vector4D expected = new Vector4D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector4D original = EuclidCoreRandomTools.nextVector4D(random);

         matrix.transform(original, expected);
         actual.set(original);
         matrix.transform(actual);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformRotationMatrix() throws Exception
   {
      Random random = new Random(435L);
      RotationMatrix actual = new RotationMatrix();
      Matrix3D expected = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix original = EuclidCoreRandomTools.nextRotationMatrix(random);

         Matrix3DTools.multiply(matrix, original, expected);
         matrix.transform(original, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
         actual.set(original);
         matrix.transform(actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformRotationScaleMatrix() throws Exception
   {
      Random random = new Random(435L);
      RotationScaleMatrix actual = new RotationScaleMatrix();
      RotationScaleMatrix expected = new RotationScaleMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix originalRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix expectedRotation = new RotationMatrix();
         Vector3D scales = EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0);
         RotationScaleMatrix original = new RotationScaleMatrix(originalRotation, scales);

         matrix.transform(originalRotation, expectedRotation);
         expected.set(expectedRotation, scales);

         matrix.transform(original, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
         actual.set(original);
         matrix.transform(actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Override
   @Test
   public void testInverseTransform() throws Exception
   {
      super.testInverseTransform();

      Random random = new Random(6787L);
      RotationMatrix rotationMatrix = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(TupleBasics tupleToTransform)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = EuclidCoreRandomTools.nextVector3D(random);
         rotationMatrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(rotationMatrix, tuple, expectedTuple);
         rotationMatrix.inverseTransform(actualTuple);

         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(TupleBasics tupleOriginal, TupleBasics tupleTransformed)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = EuclidCoreRandomTools.nextVector3D(random);
         rotationMatrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(rotationMatrix, tuple, expectedTuple);
         rotationMatrix.inverseTransform(tuple, actualTuple);

         EuclidCoreTestTools.assertTuple3DEquals(expectedTuple, actualTuple, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleToTransform)
         Tuple2DReadOnly tuple = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = EuclidCoreRandomTools.nextVector2D(random);
         double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         rotationMatrix.setToYawMatrix(theta);

         Matrix3DTools.inverseTransform(rotationMatrix, tuple, expectedTuple, false);
         rotationMatrix.inverseTransform(actualTuple);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, EPS);
         actualTuple.set(tuple);
         rotationMatrix.inverseTransform(actualTuple, true);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, EPS);
         actualTuple.set(tuple);
         rotationMatrix.inverseTransform(actualTuple, false);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleTransformed)
         Tuple2DReadOnly tuple = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = EuclidCoreRandomTools.nextVector2D(random);
         double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         rotationMatrix.setToYawMatrix(theta);

         Matrix3DTools.inverseTransform(rotationMatrix, tuple, expectedTuple, false);
         rotationMatrix.inverseTransform(tuple, actualTuple);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, EPS);
         rotationMatrix.inverseTransform(tuple, actualTuple, true);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, EPS);
         rotationMatrix.inverseTransform(tuple, actualTuple, false);
         EuclidCoreTestTools.assertTuple2DEquals(expectedTuple, actualTuple, EPS);
      }

      // Test exceptions
      try
      {
         rotationMatrix = createRandomMatrix(random);
         rotationMatrix.inverseTransform(new Vector2D());
         fail("Should have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         rotationMatrix = createRandomMatrix(random);
         rotationMatrix.inverseTransform(new Vector2D(), new Vector2D());
         fail("Should have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }
      try
      {
         rotationMatrix = createRandomMatrix(random);
         rotationMatrix.inverseTransform(new Vector2D(), true);
         fail("Should have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         rotationMatrix = createRandomMatrix(random);
         rotationMatrix.inverseTransform(new Vector2D(), new Vector2D(), true);
         fail("Should have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(QuaternionBasics quaternionToTransform)
         QuaternionReadOnly original = EuclidCoreRandomTools.nextQuaternion(random);
         QuaternionBasics actual = new Quaternion(original);
         QuaternionBasics expected = EuclidCoreRandomTools.nextQuaternion(random);
         rotationMatrix = createRandomMatrix(random);

         QuaternionTools.multiply(rotationMatrix, true, original, false, expected);
         rotationMatrix.inverseTransform(actual);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
         QuaternionReadOnly original = EuclidCoreRandomTools.nextQuaternion(random);
         QuaternionBasics actual = new Quaternion(original);
         QuaternionBasics expected = EuclidCoreRandomTools.nextQuaternion(random);
         rotationMatrix = createRandomMatrix(random);

         QuaternionTools.multiply(rotationMatrix, true, original, false, expected);
         rotationMatrix.inverseTransform(original, actual);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Vector4DBasics vectorToTransform)
         Vector4DReadOnly original = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D(original);
         Vector4DBasics expected = EuclidCoreRandomTools.nextVector4D(random);
         rotationMatrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(rotationMatrix, original, expected);
         rotationMatrix.inverseTransform(actual);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
         Vector4DReadOnly original = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D(original);
         Vector4DBasics expected = EuclidCoreRandomTools.nextVector4D(random);
         rotationMatrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(rotationMatrix, original, expected);
         rotationMatrix.inverseTransform(original, actual);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Matrix3D matrixToTransform)
         Matrix3DReadOnly original = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D actual = new Matrix3D(original);
         Matrix3D expected = EuclidCoreRandomTools.nextMatrix3D(random);
         rotationMatrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(rotationMatrix, original, expected);
         rotationMatrix.inverseTransform(actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
         Matrix3DReadOnly original = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D actual = new Matrix3D(original);
         Matrix3D expected = EuclidCoreRandomTools.nextMatrix3D(random);
         rotationMatrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(rotationMatrix, original, expected);
         rotationMatrix.inverseTransform(original, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(RotationMatrix matrixToTransform)
         RotationMatrixReadOnly original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix(original);
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrix = createRandomMatrix(random);

         RotationMatrixTools.multiplyTransposeLeft(rotationMatrix, original, expected);
         rotationMatrix.inverseTransform(actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
         RotationMatrixReadOnly original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix(original);
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         rotationMatrix = createRandomMatrix(random);

         RotationMatrixTools.multiplyTransposeLeft(rotationMatrix, original, expected);
         rotationMatrix.inverseTransform(original, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // inverseTransform(RotationScaleMatrixReadOnly matrixOriginal, RotationScaleMatrix matrixTransformed)
         RotationScaleMatrix actual = new RotationScaleMatrix();
         RotationScaleMatrix expected = new RotationScaleMatrix();
         RotationMatrix matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix originalRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix expectedRotation = new RotationMatrix();
         Vector3D scales = EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0);
         RotationScaleMatrix original = new RotationScaleMatrix(originalRotation, scales);

         matrix.inverseTransform(originalRotation, expectedRotation);
         expected.set(expectedRotation, scales);

         matrix.inverseTransform(original, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
         actual.set(original);
         matrix.inverseTransform(actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(2354L);
      RotationMatrix m1 = EuclidCoreRandomTools.nextRotationMatrix(random);
      RotationMatrix m2 = new RotationMatrix();

      assertFalse(m1.equals(m2));
      assertFalse(m1.equals(null));
      assertFalse(m1.equals(new double[4]));
      m2.set(m1);
      assertTrue(m1.equals(m2));
      Object m2AsObject = m2;
      assertTrue(m1.equals(m2AsObject));

      double smallestEpsilon = 1.0e-16;
      double[] coeffs = new double[9];

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            m2.set(m1);
            assertTrue(m1.equals(m2));
            m1.get(coeffs);
            coeffs[3 * row + column] += smallestEpsilon;
            m2.set(coeffs);
            assertFalse(m1.equals(m2));

            m2.set(m1);
            assertTrue(m1.equals(m2));
            m1.get(coeffs);
            coeffs[3 * row + column] -= smallestEpsilon;
            m2.set(coeffs);
            assertFalse(m1.equals(m2));
         }
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      RotationMatrix mA;
      RotationMatrix mB;
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 1.0e-12, 1.0e-11);
         mA = createRandomMatrix(random);
         double angleDiff = 0.99 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         mB = new RotationMatrix(aa);
         mB.preMultiply(mA);

         assertTrue(mA.geometricallyEquals(mB, epsilon), "Epsilon = " + epsilon);
         assertTrue(mA.geometricallyEquals(mA, 0.0));
         assertTrue(mB.geometricallyEquals(mB, 0.0));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 1.0e-12, 1.0e-11);
         mA = createRandomMatrix(random);
         double angleDiff = 1.01 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         mB = new RotationMatrix(aa);
         mB.preMultiply(mA);

         assertFalse(mA.geometricallyEquals(mB, epsilon), "Epsilon = " + epsilon);
         assertTrue(mA.geometricallyEquals(mA, 0.0));
         assertTrue(mB.geometricallyEquals(mB, 0.0));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0 * Math.PI);
         mA = createRandomMatrix(random);
         double angleDiff = 0.99 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         mB = new RotationMatrix(aa);
         mB.preMultiply(mA);

         assertTrue(mA.geometricallyEquals(mB, epsilon), "Epsilon = " + epsilon);
         assertTrue(mA.geometricallyEquals(mA, 0.0));
         assertTrue(mB.geometricallyEquals(mB, 0.0));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 1.02); // Make sure to not go over Math.PI
         mA = createRandomMatrix(random);
         double angleDiff = 1.01 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         mB = new RotationMatrix(aa);
         mB.preMultiply(mA);

         assertFalse(mA.geometricallyEquals(mB, epsilon), "Epsilon = " + epsilon);
         assertTrue(mA.geometricallyEquals(mA, 0.0));
         assertTrue(mB.geometricallyEquals(mB, 0.0));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // If epsilon >= Math.PI, any pair of two quaternions will be equal
         double epsilon = EuclidCoreRandomTools.nextDouble(random, Math.PI, 2.0 * Math.PI);
         mA = createRandomMatrix(random);
         mB = createRandomMatrix(random);

         assertTrue(mA.geometricallyEquals(mB, epsilon), "Epsilon = " + epsilon);
      }
   }

   @Test
   public void testEpsilonConsistencyWithQuaternionAndAxisAngle() throws Exception
   {
      Random random = new Random(9762344L);
      RotationMatrix rotationMatrixA;
      RotationMatrix rotationMatrixB;
      Quaternion quaternionA;
      Quaternion quaternionB;
      AxisAngle axisAngleA;
      AxisAngle axisAngleB;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         rotationMatrixA = EuclidCoreRandomTools.nextRotationMatrix(random);
         double angleDiff = 0.99 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         rotationMatrixB = new RotationMatrix(aa);
         rotationMatrixB.preMultiply(rotationMatrixA);

         assertTrue(rotationMatrixA.geometricallyEquals(rotationMatrixB, epsilon));

         quaternionA = new Quaternion(rotationMatrixA);
         quaternionB = new Quaternion(rotationMatrixB);

         assertTrue(quaternionA.geometricallyEquals(quaternionB, epsilon));

         axisAngleA = new AxisAngle(rotationMatrixA);
         axisAngleB = new AxisAngle(rotationMatrixB);

         assertTrue(axisAngleA.geometricallyEquals(axisAngleB, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         rotationMatrixA = EuclidCoreRandomTools.nextRotationMatrix(random);
         double angleDiff = 1.01 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         rotationMatrixB = new RotationMatrix(aa);
         rotationMatrixB.preMultiply(rotationMatrixA);

         quaternionA = new Quaternion(rotationMatrixA);
         quaternionB = new Quaternion(rotationMatrixB);

         assertFalse(quaternionA.geometricallyEquals(quaternionB, epsilon));

         axisAngleA = new AxisAngle(rotationMatrixA);
         axisAngleB = new AxisAngle(rotationMatrixB);

         assertFalse(axisAngleA.geometricallyEquals(axisAngleB, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Matrix3D matrix = EuclidCoreRandomTools.nextMatrix3D(random);
      RotationMatrix rotationMatrix = new RotationMatrix();

      int newHashCode, previousHashCode;
      newHashCode = matrix.hashCode();
      assertEquals(newHashCode, matrix.hashCode());

      previousHashCode = matrix.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         int row = random.nextInt(3);
         int column = random.nextInt(3);
         matrix.setElement(row, column, random.nextDouble());

         double m00 = matrix.getM00();
         double m01 = matrix.getM01();
         double m02 = matrix.getM02();
         double m10 = matrix.getM10();
         double m11 = matrix.getM11();
         double m12 = matrix.getM12();
         double m20 = matrix.getM20();
         double m21 = matrix.getM21();
         double m22 = matrix.getM22();
         rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);

         newHashCode = rotationMatrix.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Test
   public void testOrientation3DBasicsFeatures() throws Throwable
   {
      Orientation3DBasicsTest test = new Orientation3DBasicsTest()
      {
         @Override
         public Orientation3DBasics createEmptyOrientation3DBasics()
         {
            return createEmptyMatrix();
         }

         @Override
         public double getEpsilon()
         {
            return EPS;
         }
      };
      for (Method testMethod : test.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         try
         {
            testMethod.invoke(test);
         }
         catch (InvocationTargetException e)
         {
            throw e.getTargetException();
         }
      }
   }

   @Override
   public RotationMatrix createEmptyMatrix()
   {
      return new RotationMatrix();
   }

   @Override
   public RotationMatrix createMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      return rotationMatrix;
   }

   @Override
   public RotationMatrix createRandomMatrix(Random random)
   {
      return EuclidCoreRandomTools.nextRotationMatrix(random);
   }
}
