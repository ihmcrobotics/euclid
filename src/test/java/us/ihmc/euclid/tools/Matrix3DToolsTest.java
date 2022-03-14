package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;

public class Matrix3DToolsTest
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testInvert() throws Exception
   {
      Random random = new Random(298364L);

      { // Test that the first of invert(matrix, inverseToPack) remains unchanged
         Matrix3D matrix = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 1.0, 2.0); // Make sure that it generates a non-singular matrix.
         Matrix3D matrixCopy = new Matrix3D(matrix);
         boolean success = Matrix3DTools.invert(matrix, new Matrix3D());
         assertTrue(success);
         assertEquals(matrix, matrixCopy);
      }

      { // Test that invert returns false when the matrix is not invertible
         Matrix3D zero = new Matrix3D();
         Matrix3D zeroCopy = new Matrix3D();

         boolean success = Matrix3DTools.invert(zero);
         assertFalse(success);
         assertEquals(zero, zeroCopy);

         success = Matrix3DTools.invert(zero, new Matrix3D());
         assertFalse(success);
      }

      { // Test that the inverse of the identity matrix is equal to the identity matrix
         Matrix3D identity = new Matrix3D(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
         Matrix3D matrix = new Matrix3D(identity);
         Matrix3D inverse = new Matrix3D();

         boolean success = Matrix3DTools.invert(matrix);
         assertTrue(success);
         assertEquals(identity, matrix);

         success = Matrix3DTools.invert(matrix, inverse);
         assertTrue(success);
         assertEquals(identity, inverse);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test the inverse of a rotation matrix
         RotationMatrix original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix expected = new RotationMatrix();
         expected.setAndTranspose(original);
         Matrix3D actual = new Matrix3D(original);

         boolean success = Matrix3DTools.invert(actual);
         assertTrue(success);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);

         actual.setToZero();
         success = Matrix3DTools.invert(original, actual);
         assertTrue(success);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Finally test against EJML
         Matrix3D original = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D expected = new Matrix3D();
         Matrix3D actual = new Matrix3D(original);

         DMatrixRMaj originalDenseMatrix = new DMatrixRMaj(3, 3);
         original.get(originalDenseMatrix);
         DMatrixRMaj expectedDenseMatrix = new DMatrixRMaj(3, 3);
         boolean expectedSuccess = CommonOps_DDRM.invert(originalDenseMatrix, expectedDenseMatrix);
         expected.set(expectedDenseMatrix);

         boolean actualSuccess = Matrix3DTools.invert(actual);
         assertTrue(expectedSuccess == actualSuccess);
         assertTrue(expected.epsilonEquals(actual, EPS));

         actual.setToZero();
         actualSuccess = Matrix3DTools.invert(original, actual);
         assertTrue(expectedSuccess == actualSuccess);
         assertTrue(expected.epsilonEquals(actual, EPS));
      }
   }

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(53463L);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      // Test that m times the identity equals m
      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D m = EuclidCoreRandomTools.nextMatrix3D(random);
         // Fill some random data in matrixActual
         matrixActual = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D mCopy = new Matrix3D(m);

         Matrix3D identity = new Matrix3D();
         identity.setIdentity();
         Matrix3D identityCopy = new Matrix3D(identity);

         matrixExpected.set(m);
         Matrix3DTools.multiply(m, identity, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
         assertEquals(m, mCopy);
         assertEquals(identity, identityCopy);
         Matrix3DTools.multiply(identity, m, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      // Test that m * m^-1 = identity
      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D m = EuclidCoreRandomTools.nextMatrix3D(random);
         // Fill some random data in matrixActual
         matrixActual = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3D mInverse = new Matrix3D();
         boolean success = Matrix3DTools.invert(m, mInverse);

         if (!success)
         {
            m = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 1.0, 10.0);
            Matrix3DTools.invert(m, mInverse);
         }

         matrixExpected.setIdentity();
         Matrix3DTools.multiply(m, mInverse, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
         Matrix3DTools.multiply(mInverse, m, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      // Assert that multiplying two matrices describing two rotations around the same axis is equivalent to adding the angles of these
      // two rotations. TODO move out to RotationMatrixToolsTest
      for (int i = 0; i < ITERATIONS; i++)
      {
         double angle1 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double angle2 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         RotationMatrix rotationMatrixExpected = new RotationMatrix();
         RotationMatrix rotationMatrixActual = new RotationMatrix();
         RotationMatrix m1 = new RotationMatrix();
         RotationMatrix m2 = new RotationMatrix();

         RotationMatrixConversion.convertAxisAngleToMatrix(axis.getX(), axis.getY(), axis.getZ(), angle1, m1);
         RotationMatrixConversion.convertAxisAngleToMatrix(axis.getX(), axis.getY(), axis.getZ(), angle2, m2);
         RotationMatrixConversion.convertAxisAngleToMatrix(axis.getX(), axis.getY(), axis.getZ(), angle1 + angle2, rotationMatrixExpected);

         RotationMatrixTools.multiply(m1, m2, rotationMatrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrixExpected, rotationMatrixActual, EPS);
         RotationMatrixTools.multiply(m2, m1, rotationMatrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrixExpected, rotationMatrixActual, EPS);
      }

      // Check that we can do in-place multiplication TODO move out to RotationMatrixToolsTest
      for (int i = 0; i < ITERATIONS; i++)
      {
         double angle = EuclidCoreRandomTools.nextDouble(random, 1.0);
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         RotationMatrix rotationMatrixExpected = new RotationMatrix();
         RotationMatrix rotationMatrixActual = new RotationMatrix();

         RotationMatrixConversion.convertAxisAngleToMatrix(axis.getX(), axis.getY(), axis.getZ(), angle, rotationMatrixActual);
         RotationMatrixConversion.convertAxisAngleToMatrix(axis.getX(), axis.getY(), axis.getZ(), 2.0 * angle, rotationMatrixExpected);

         RotationMatrixTools.multiply(rotationMatrixActual, rotationMatrixActual, rotationMatrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrixExpected, rotationMatrixActual, EPS);
      }

      DMatrixRMaj dm1 = new DMatrixRMaj(3, 3);
      DMatrixRMaj dm2 = new DMatrixRMaj(3, 3);
      DMatrixRMaj dmResult = new DMatrixRMaj(3, 3);

      // Finally check against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         m1.get(dm1);
         m2.get(dm2);

         CommonOps_DDRM.mult(dm1, dm2, dmResult);
         matrixExpected.set(dmResult);

         Matrix3DTools.multiply(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      for (int i = 0; i < 4 * ITERATIONS; i++)
      { // Test multiply(Matrix3DReadOnly m1, boolean transpose1, boolean invert1, Matrix3DReadOnly m2, boolean transpose2, boolean invert2, CommonMatrix3DBasics matrixToPack)
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         if (random.nextBoolean())
            m1.setIdentity();
         if (random.nextBoolean())
            m2.setIdentity();

         boolean transpose1 = random.nextBoolean();
         boolean invert1 = random.nextBoolean();
         boolean transpose2 = random.nextBoolean();
         boolean invert2 = random.nextBoolean();

         m1.get(dm1);
         m2.get(dm2);

         if (transpose1)
            CommonOps_DDRM.transpose(dm1);
         if (invert1)
            CommonOps_DDRM.invert(dm1);

         if (transpose2)
            CommonOps_DDRM.transpose(dm2);
         if (invert2)
            CommonOps_DDRM.invert(dm2);

         CommonOps_DDRM.mult(dm1, dm2, dmResult);
         matrixExpected.set(dmResult);

         Matrix3DTools.multiply(m1, transpose1, invert1, m2, transpose2, invert2, matrixActual);
         if (invert1 || invert2)
            EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, 20.0 * EPS);
         else
            EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      for (int i = 0; i < 4 * ITERATIONS; i++)
      { // Test multiply(Matrix3DReadOnly matrix, boolean transposeMatrix, boolean invertMatrix, Orientation3DReadOnly orientation, boolean invertOrientation, CommonMatrix3DBasics matrixToPack)
         Matrix3D matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         Orientation3DBasics orientation = EuclidCoreRandomTools.nextOrientation3D(random);

         if (random.nextBoolean())
            matrix.setIdentity();
         if (random.nextBoolean())
            orientation.setToZero();

         boolean transposeMatrix = random.nextBoolean();
         boolean invertMatrix = random.nextBoolean();
         boolean invertOrientation = random.nextBoolean();

         Matrix3D expected = new Matrix3D();
         RotationMatrix rotationMatrix = new RotationMatrix(orientation);
         Matrix3DTools.multiply(matrix, transposeMatrix, invertMatrix, rotationMatrix, invertOrientation, false, expected);
         Matrix3D actual = new Matrix3D();
         Matrix3DTools.multiply(matrix, transposeMatrix, invertMatrix, orientation, invertOrientation, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      for (int i = 0; i < 4 * ITERATIONS; i++)
      { // Test multiply(Matrix3DReadOnly matrix, boolean transposeMatrix, boolean invertMatrix, Orientation3DReadOnly orientation, boolean invertOrientation, CommonMatrix3DBasics matrixToPack)
         Matrix3D matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         Orientation3DBasics orientation = EuclidCoreRandomTools.nextOrientation3D(random);

         if (random.nextBoolean())
            matrix.setIdentity();
         if (random.nextBoolean())
            orientation.setToZero();

         boolean transposeMatrix = random.nextBoolean();
         boolean invertMatrix = random.nextBoolean();
         boolean invertOrientation = random.nextBoolean();

         Matrix3D expected = new Matrix3D();
         RotationMatrix rotationMatrix = new RotationMatrix(orientation);
         Matrix3DTools.multiply(matrix, transposeMatrix, invertMatrix, rotationMatrix, invertOrientation, false, expected);
         Matrix3D actual = new Matrix3D();
         Matrix3DTools.multiply(matrix, transposeMatrix, invertMatrix, orientation, invertOrientation, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyTransposeBoth() throws Exception
   {
      Random random = new Random(53463L);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      // Test against multiply
      for (int i = 0; i < ITERATIONS; i++)
      {
         // Fill some random data in matrixActual
         matrixActual = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1Copy = new Matrix3D(m1);
         Matrix3D m2Copy = new Matrix3D(m2);

         Matrix3D m1Transpose = new Matrix3D();
         m1Transpose.setAndTranspose(m1);
         Matrix3D m2Transpose = new Matrix3D();
         m2Transpose.setAndTranspose(m2);

         Matrix3DTools.multiply(m1Transpose, m2Transpose, matrixExpected);
         Matrix3DTools.multiplyTransposeBoth(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
         assertEquals(m1, m1Copy);
         assertEquals(m2, m2Copy);
      }

      DMatrixRMaj dm1 = new DMatrixRMaj(3, 3);
      DMatrixRMaj dm2 = new DMatrixRMaj(3, 3);
      DMatrixRMaj dmResult = new DMatrixRMaj(3, 3);

      // Finally check against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         m1.get(dm1);
         m2.get(dm2);

         CommonOps_DDRM.multTransAB(dm1, dm2, dmResult);
         matrixExpected.set(dmResult);

         Matrix3DTools.multiplyTransposeBoth(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertBoth() throws Exception
   {
      Random random = new Random(53463L);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      // Test that it throws an exception when the matrix is singular
      matrixActual.setToZero();
      try
      {
         Matrix3DTools.multiplyInvertBoth(matrixActual, matrixActual, matrixActual);
         fail("Should have thrown a " + SingularMatrixException.class.getSimpleName());
      }
      catch (SingularMatrixException e)
      {
         // good
      }

      // Test with a generic matrix against multiply
      for (int i = 0; i < ITERATIONS; i++)
      {
         // Fill some random data in matrixActual
         matrixActual = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random, 1.0, 10.0);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random, 1.0, 10.0);
         Matrix3D m1Copy = new Matrix3D(m1);
         Matrix3D m2Copy = new Matrix3D(m2);

         Matrix3D m1Inverse = new Matrix3D();
         m1Inverse.setAndInvert(m1);
         Matrix3D m2Inverse = new Matrix3D();
         m2Inverse.setAndInvert(m2);

         Matrix3DTools.multiply(m1Inverse, m2Inverse, matrixExpected);
         Matrix3DTools.multiplyInvertBoth(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, 1.0e-7);
         assertEquals(m1, m1Copy);
         assertEquals(m2, m2Copy);
      }

      // Test with a rotation matrix against multiply TODO move
      for (int i = 0; i < ITERATIONS; i++)
      {
         // Fill some random data in matrixActual
         RotationMatrix rotationMatrixActual = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix m1 = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix m2 = EuclidCoreRandomTools.nextRotationMatrix(random);

         Matrix3D m1Inverse = new Matrix3D();
         m1Inverse.setAndInvert(m1);
         Matrix3D m2Inverse = new Matrix3D();
         m2Inverse.setAndInvert(m2);

         Matrix3DTools.multiply(m1Inverse, m2Inverse, matrixExpected);
         RotationMatrixTools.multiplyTransposeBoth(m1, m2, rotationMatrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, rotationMatrixActual, 1.0e-7);
      }
   }

   @Test
   public void testMultiplyTransposeLeft() throws Exception
   {
      Random random = new Random(53463L);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      // Test against multiply
      for (int i = 0; i < ITERATIONS; i++)
      {
         // Fill some random data in matrixActual
         matrixActual = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1Copy = new Matrix3D(m1);
         Matrix3D m2Copy = new Matrix3D(m2);

         Matrix3D m1Transpose = new Matrix3D();
         m1Transpose.setAndTranspose(m1);

         Matrix3DTools.multiply(m1Transpose, m2, matrixExpected);
         Matrix3DTools.multiplyTransposeLeft(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
         assertEquals(m1, m1Copy);
         assertEquals(m2, m2Copy);
      }

      DMatrixRMaj dm1 = new DMatrixRMaj(3, 3);
      DMatrixRMaj dm2 = new DMatrixRMaj(3, 3);
      DMatrixRMaj dmResult = new DMatrixRMaj(3, 3);

      // Finally check against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         m1.get(dm1);
         m2.get(dm2);

         CommonOps_DDRM.multTransA(dm1, dm2, dmResult);
         matrixExpected.set(dmResult);

         Matrix3DTools.multiplyTransposeLeft(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertLeft() throws Exception
   {
      Random random = new Random(53463L);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      // Test that it throws an exception when the matrix is singular
      matrixActual.setToZero();
      try
      {
         Matrix3DTools.multiplyInvertLeft(matrixActual, matrixActual, matrixActual);
         fail("Should have thrown a " + SingularMatrixException.class.getSimpleName());
      }
      catch (SingularMatrixException e)
      {
         // good
      }

      // Test with a generic matrix against multiply
      for (int i = 0; i < ITERATIONS; i++)
      {
         // Fill some random data in matrixActual
         matrixActual = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1Copy = new Matrix3D(m1);
         Matrix3D m2Copy = new Matrix3D(m2);

         Matrix3D m1Inverse = new Matrix3D();
         m1Inverse.setAndInvert(m1);

         Matrix3DTools.multiply(m1Inverse, m2, matrixExpected);
         Matrix3DTools.multiplyInvertLeft(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
         assertEquals(m1, m1Copy);
         assertEquals(m2, m2Copy);
      }

      // Test with a rotation matrix against multiply
      for (int i = 0; i < ITERATIONS; i++)
      {
         // Fill some random data in matrixActual
         matrixActual = EuclidCoreRandomTools.nextMatrix3D(random);
         RotationMatrix m1 = EuclidCoreRandomTools.nextRotationMatrix(random);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3D m1Inverse = new Matrix3D();
         m1Inverse.setAndInvert(m1);

         Matrix3DTools.multiply(m1Inverse, m2, matrixExpected);
         Matrix3DTools.multiplyInvertLeft(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }
   }

   @Test
   public void testMultiplyTransposeRight() throws Exception
   {
      Random random = new Random(53463L);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      // Test against multiply
      for (int i = 0; i < ITERATIONS; i++)
      {
         // Fill some random data in matrixActual
         matrixActual = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1Copy = new Matrix3D(m1);
         Matrix3D m2Copy = new Matrix3D(m2);

         Matrix3D m2Transpose = new Matrix3D();
         m2Transpose.setAndTranspose(m2);

         Matrix3DTools.multiply(m1, m2Transpose, matrixExpected);
         Matrix3DTools.multiplyTransposeRight(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
         assertEquals(m1, m1Copy);
         assertEquals(m2, m2Copy);
      }

      DMatrixRMaj dm1 = new DMatrixRMaj(3, 3);
      DMatrixRMaj dm2 = new DMatrixRMaj(3, 3);
      DMatrixRMaj dmResult = new DMatrixRMaj(3, 3);

      // Finally check against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         m1.get(dm1);
         m2.get(dm2);

         CommonOps_DDRM.multTransB(dm1, dm2, dmResult);
         matrixExpected.set(dmResult);

         Matrix3DTools.multiplyTransposeRight(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertRight() throws Exception
   {
      Random random = new Random(53463L);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      // Test that it throws an exception when the matrix is singular
      matrixActual.setToZero();
      try
      {
         Matrix3DTools.multiplyInvertRight(matrixActual, matrixActual, matrixActual);
         fail("Should have thrown a " + SingularMatrixException.class.getSimpleName());
      }
      catch (SingularMatrixException e)
      {
         // good
      }

      // Test with a generic matrix against multiply
      for (int i = 0; i < ITERATIONS; i++)
      {
         // Fill some random data in matrixActual
         matrixActual = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m2 = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1Copy = new Matrix3D(m1);
         Matrix3D m2Copy = new Matrix3D(m2);

         Matrix3D m2Inverse = new Matrix3D();
         m2Inverse.setAndInvert(m2);

         Matrix3DTools.multiply(m1, m2Inverse, matrixExpected);
         Matrix3DTools.multiplyInvertRight(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
         assertEquals(m1, m1Copy);
         assertEquals(m2, m2Copy);
      }

      // Test with a rotation matrix against multiply
      for (int i = 0; i < ITERATIONS; i++)
      {
         // Fill some random data in matrixActual
         matrixActual = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         RotationMatrix m2 = EuclidCoreRandomTools.nextRotationMatrix(random);

         Matrix3D m2Inverse = new Matrix3D();
         m2Inverse.setAndInvert(m2);

         Matrix3DTools.multiply(m1, m2Inverse, matrixExpected);
         Matrix3DTools.multiplyInvertRight(m1, m2, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }
   }

   @Test
   public void testMultiplyInner()
   {
      Random random = new Random(3245);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D original = EuclidCoreRandomTools.nextMatrix3D(random, 10.0);
         Matrix3D expected = new Matrix3D();
         Matrix3DTools.multiplyTransposeLeft(original, original, expected);
         Matrix3D actual = new Matrix3D();
         Matrix3DTools.multiplyInner(original, actual);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyOuter()
   {
      Random random = new Random(3245);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D original = EuclidCoreRandomTools.nextMatrix3D(random, 10.0);
         Matrix3D expected = new Matrix3D();
         Matrix3DTools.multiplyTransposeRight(original, original, expected);
         Matrix3D actual = new Matrix3D();
         Matrix3DTools.multiplyOuter(original, actual);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testNormalize() throws Exception
   {
      Random random = new Random(39456L);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      { // Check that identity does not get modified
         matrixActual.setIdentity();
         matrixExpected.setIdentity();

         Matrix3DTools.normalize(matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      // Test that normalizing a proper rotation matrix does not change it.
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrixExpected.set(EuclidCoreRandomTools.nextRotationMatrix(random));
         matrixActual.set(matrixExpected);

         Matrix3DTools.normalize(matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      Vector3D vector1 = new Vector3D();
      Vector3D vector2 = new Vector3D();

      // Test that it actually makes a random matrix ortho-normal
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrixActual = EuclidCoreRandomTools.nextMatrix3D(random, 1.0, 2.0);
         Matrix3DTools.normalize(matrixActual);

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
   public void testTransformTuple() throws Exception
   {
      Random random = new Random(3489756L);
      Matrix3D matrix = new Matrix3D();
      Tuple3DBasics tupleOriginal = new Vector3D();
      Tuple3DBasics tupleActual = new Vector3D();
      Tuple3DBasics tupleExpected = new Vector3D();

      { // Test transforming with the zero matrix zero out the tuple.
         matrix.setToZero();
         Matrix3D matrixCopy = new Matrix3D(matrix);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics tupleOriginalCopy = new Vector3D(tupleOriginal);
         tupleExpected.setToZero();
         Matrix3DTools.transform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
         assertEquals(tupleOriginal, tupleOriginalCopy);
         assertEquals(matrix, matrixCopy);
      }

      { // Test that transforming with identity does not do anything.
         matrix.setIdentity();
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         tupleExpected.set(tupleOriginal);
         Matrix3DTools.transform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test some random scaling
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 1.0, 10.0);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         tupleExpected.setX(matrix.getM00() * tupleOriginal.getX());
         tupleExpected.setY(matrix.getM11() * tupleOriginal.getY());
         tupleExpected.setZ(matrix.getM22() * tupleOriginal.getZ());

         Matrix3DTools.transform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      DMatrixRMaj denseMatrix = new DMatrixRMaj(3, 3);
      DMatrixRMaj denseVectorOriginal = new DMatrixRMaj(3, 1);
      DMatrixRMaj denseVectorTransformed = new DMatrixRMaj(3, 1);

      // Test against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         matrix.get(denseMatrix);
         tupleOriginal.get(denseVectorOriginal);

         CommonOps_DDRM.mult(denseMatrix, denseVectorOriginal, denseVectorTransformed);
         tupleExpected.set(denseVectorTransformed);

         Matrix3DTools.transform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test transforming in-place
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);

         Matrix3DTools.transform(matrix, tupleOriginal, tupleExpected);
         tupleActual.set(tupleOriginal);
         Matrix3DTools.transform(matrix, tupleActual, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }
   }

   @Test
   public void testAddTransformTuple() throws Exception
   {
      Random random = new Random(3489756L);
      Matrix3D matrix = new Matrix3D();
      Tuple3DBasics tupleOriginal = new Vector3D();
      Tuple3DBasics tupleActual = new Vector3D();
      Tuple3DBasics tupleExpected = new Vector3D();

      { // Test transforming with the zero matrix does not do anything.
         matrix.setToZero();
         Matrix3D matrixCopy = new Matrix3D(matrix);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics tupleOriginalCopy = new Vector3D(tupleOriginal);
         tupleExpected = EuclidCoreRandomTools.nextVector3D(random);
         tupleActual.set(tupleExpected);
         Matrix3DTools.addTransform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
         assertEquals(tupleOriginal, tupleOriginalCopy);
         assertEquals(matrix, matrixCopy);
      }

      { // Test that transforming with identity simply adds up the two tuples.
         matrix.setIdentity();
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         tupleActual = EuclidCoreRandomTools.nextVector3D(random);
         tupleExpected.add(tupleActual, tupleOriginal);
         Matrix3DTools.addTransform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test some random scaling
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 1.0, 10.0);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         tupleActual = EuclidCoreRandomTools.nextVector3D(random);
         tupleExpected.setX(matrix.getM00() * tupleOriginal.getX());
         tupleExpected.setY(matrix.getM11() * tupleOriginal.getY());
         tupleExpected.setZ(matrix.getM22() * tupleOriginal.getZ());
         tupleExpected.add(tupleActual);

         Matrix3DTools.addTransform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      DMatrixRMaj denseMatrix = new DMatrixRMaj(3, 3);
      DMatrixRMaj denseVectorOriginal = new DMatrixRMaj(3, 1);
      DMatrixRMaj denseVectorTransformed = new DMatrixRMaj(3, 1);

      // Test against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         tupleActual = EuclidCoreRandomTools.nextVector3D(random);
         matrix.get(denseMatrix);
         tupleOriginal.get(denseVectorOriginal);
         tupleActual.get(denseVectorTransformed);

         CommonOps_DDRM.multAdd(denseMatrix, denseVectorOriginal, denseVectorTransformed);
         tupleExpected.set(denseVectorTransformed);

         Matrix3DTools.addTransform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test transforming in-place
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         tupleActual = EuclidCoreRandomTools.nextVector3D(random);
         tupleExpected.set(tupleActual);

         Matrix3DTools.addTransform(matrix, tupleActual, tupleExpected);
         Matrix3DTools.addTransform(matrix, tupleActual, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }
   }

   @Test
   public void testSubTransformTuple() throws Exception
   {
      Random random = new Random(3489756L);
      Matrix3D matrix = new Matrix3D();
      Tuple3DBasics tupleOriginal = new Vector3D();
      Tuple3DBasics tupleActual = new Vector3D();
      Tuple3DBasics tupleExpected = new Vector3D();

      { // Test transforming with the zero matrix does not do anything.
         matrix.setToZero();
         Matrix3D matrixCopy = new Matrix3D(matrix);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics tupleOriginalCopy = new Vector3D(tupleOriginal);
         tupleExpected = EuclidCoreRandomTools.nextVector3D(random);
         tupleActual.set(tupleExpected);
         Matrix3DTools.subTransform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
         assertEquals(tupleOriginal, tupleOriginalCopy);
         assertEquals(matrix, matrixCopy);
      }

      { // Test that transforming with identity simply adds up the two tuples.
         matrix.setIdentity();
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         tupleActual = EuclidCoreRandomTools.nextVector3D(random);
         tupleExpected.sub(tupleActual, tupleOriginal);
         Matrix3DTools.subTransform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test some random scaling
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 1.0, 10.0);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         tupleActual = EuclidCoreRandomTools.nextVector3D(random);
         tupleExpected.setX(matrix.getM00() * tupleOriginal.getX());
         tupleExpected.setY(matrix.getM11() * tupleOriginal.getY());
         tupleExpected.setZ(matrix.getM22() * tupleOriginal.getZ());
         tupleExpected.sub(tupleActual, tupleExpected);

         Matrix3DTools.subTransform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      DMatrixRMaj denseMatrix = new DMatrixRMaj(3, 3);
      DMatrixRMaj denseVectorOriginal = new DMatrixRMaj(3, 1);
      DMatrixRMaj denseVectorTransformed = new DMatrixRMaj(3, 1);

      // Test against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         tupleActual = EuclidCoreRandomTools.nextVector3D(random);
         matrix.get(denseMatrix);
         tupleOriginal.get(denseVectorOriginal);
         tupleActual.get(denseVectorTransformed);

         CommonOps_DDRM.multAdd(-1.0, denseMatrix, denseVectorOriginal, denseVectorTransformed);
         tupleExpected.set(denseVectorTransformed);

         Matrix3DTools.subTransform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test transforming in-place
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         tupleActual = EuclidCoreRandomTools.nextVector3D(random);
         tupleExpected.set(tupleActual);

         Matrix3DTools.subTransform(matrix, tupleActual, tupleExpected);
         Matrix3DTools.subTransform(matrix, tupleActual, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }
   }

   @Test
   public void testTransformTuple2D() throws Exception
   {
      Random random = new Random(3489756L);
      Matrix3D matrix = new Matrix3D();
      Tuple2DBasics tupleOriginal = new Vector2D();
      Tuple2DBasics tupleActual = new Vector2D();
      Tuple2DBasics tupleExpected = new Vector2D();

      { // Test transforming with the zero matrix zero out the tuple.
         matrix.setToZero();
         Matrix3D matrixCopy = new Matrix3D(matrix);
         tupleOriginal = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DBasics tupleOriginalCopy = new Vector2D(tupleOriginal);
         tupleExpected.setToZero();
         Matrix3DTools.transform(matrix, tupleOriginal, tupleActual, false);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPS);
         assertEquals(tupleOriginal, tupleOriginalCopy);
         assertEquals(matrix, matrixCopy);
      }

      { // Test that transforming with identity does not do anything.
         matrix.setIdentity();
         tupleOriginal = EuclidCoreRandomTools.nextVector2D(random);
         tupleExpected.set(tupleOriginal);
         Matrix3DTools.transform(matrix, tupleOriginal, tupleActual, false);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test some random scaling
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 1.0, 10.0);
         tupleOriginal = EuclidCoreRandomTools.nextVector2D(random);
         tupleExpected.setX(matrix.getM00() * tupleOriginal.getX());
         tupleExpected.setY(matrix.getM11() * tupleOriginal.getY());

         Matrix3DTools.transform(matrix, tupleOriginal, tupleActual, false);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPS);
      }

      DMatrixRMaj denseMatrix = new DMatrixRMaj(2, 2);
      DMatrixRMaj denseVectorOriginal = new DMatrixRMaj(2, 1);
      DMatrixRMaj denseVectorTransformed = new DMatrixRMaj(2, 1);

      // Test against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         tupleOriginal = EuclidCoreRandomTools.nextVector2D(random);
         denseMatrix.set(0, 0, matrix.getM00());
         denseMatrix.set(0, 1, matrix.getM01());
         denseMatrix.set(1, 0, matrix.getM10());
         denseMatrix.set(1, 1, matrix.getM11());
         tupleOriginal.get(denseVectorOriginal);

         CommonOps_DDRM.mult(denseMatrix, denseVectorOriginal, denseVectorTransformed);
         tupleExpected.set(denseVectorTransformed);

         Matrix3DTools.transform(matrix, tupleOriginal, tupleActual, false);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test transforming in-place
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         tupleOriginal = EuclidCoreRandomTools.nextVector2D(random);

         Matrix3DTools.transform(matrix, tupleOriginal, tupleExpected, false);
         tupleActual.set(tupleOriginal);
         Matrix3DTools.transform(matrix, tupleActual, tupleActual, false);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test exceptions are thrown with checkIfTransformInXYPlane set to true and given a 3D matrix.
      matrix.setIdentity();
      matrix.setM02(2.0);
      try
      {
         Matrix3DTools.transform(matrix, tupleActual, tupleActual, true);
         fail("Should have thrown a " + NotAMatrix2DException.class.getSimpleName());
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }

      matrix.setIdentity();
      matrix.setM12(2.0);
      try
      {
         Matrix3DTools.transform(matrix, tupleActual, tupleActual, true);
         fail("Should have thrown a " + NotAMatrix2DException.class.getSimpleName());
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
      matrix.setIdentity();
      matrix.setM22(2.0);
      try
      {
         Matrix3DTools.transform(matrix, tupleActual, tupleActual, true);
         fail("Should have thrown a " + NotAMatrix2DException.class.getSimpleName());
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
      matrix.setIdentity();
      matrix.setM20(2.0);
      try
      {
         Matrix3DTools.transform(matrix, tupleActual, tupleActual, true);
         fail("Should have thrown a " + NotAMatrix2DException.class.getSimpleName());
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
      matrix.setIdentity();
      matrix.setM21(2.0);
      try
      {
         Matrix3DTools.transform(matrix, tupleActual, tupleActual, true);
         fail("Should have thrown a " + NotAMatrix2DException.class.getSimpleName());
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }

      // Test that the math does not change with checkIfTransformInXYPlane set to true.
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         matrix.setColumn(2, 0.0, 0.0, 1.0);
         matrix.setRow(2, 0.0, 0.0, 1.0);
         tupleOriginal = EuclidCoreRandomTools.nextVector2D(random);

         Matrix3DTools.transform(matrix, tupleOriginal, tupleExpected, false);
         Matrix3DTools.transform(matrix, tupleOriginal, tupleActual, true);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPS);
      }
   }

   @Test
   public void testTransformQuaternion() throws Exception
   {
      Random random = new Random(324636L);
      RotationMatrix matrix = new RotationMatrix();
      RotationMatrix matrixOriginal = new RotationMatrix();
      RotationMatrix matrixExpected = new RotationMatrix();

      Quaternion qOriginal = new Quaternion();
      Quaternion qExpected = new Quaternion();
      Quaternion qActual = new Quaternion();

      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         qOriginal = EuclidCoreRandomTools.nextQuaternion(random);
         matrixOriginal.set(qOriginal);

         matrixExpected.set(matrix);
         matrixExpected.multiply(matrixOriginal);
         qExpected.set(matrixExpected);
         matrix.transform(qOriginal, qActual);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(qExpected, qActual, EPS);
      }
   }

   @Test
   public void testTransformVector4D() throws Exception
   {
      Random random = new Random(3489756L);
      Matrix3D matrix = new Matrix3D();
      Vector4D vectorOriginal = new Vector4D();
      Vector4D vectorActual = new Vector4D();
      Vector4D vectorExpected = new Vector4D();

      { // Test transforming with the zero matrix zero out the tuple.
         matrix.setToZero();
         Matrix3D matrixCopy = new Matrix3D(matrix);
         vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D vectorOriginalCopy = new Vector4D(vectorOriginal);
         vectorExpected.set(0.0, 0.0, 0.0, vectorOriginal.getS());
         Matrix3DTools.transform(matrix, vectorOriginal, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
         assertEquals(vectorOriginal, vectorOriginalCopy);
         assertEquals(matrix, matrixCopy);
      }

      { // Test that transforming with identity does not do anything.
         matrix.setIdentity();
         vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);
         vectorExpected.set(vectorOriginal);
         Matrix3DTools.transform(matrix, vectorOriginal, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
      }

      // Test some random scaling
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 1.0, 10.0);
         vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);
         vectorExpected.setX(matrix.getM00() * vectorOriginal.getX());
         vectorExpected.setY(matrix.getM11() * vectorOriginal.getY());
         vectorExpected.setZ(matrix.getM22() * vectorOriginal.getZ());
         vectorExpected.setS(vectorOriginal.getS());

         Matrix3DTools.transform(matrix, vectorOriginal, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
      }

      DMatrixRMaj denseMatrix = new DMatrixRMaj(3, 3);
      DMatrixRMaj denseVectorOriginal = new DMatrixRMaj(3, 1);
      DMatrixRMaj denseVectorTransformed = new DMatrixRMaj(3, 1);

      // Test against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);
         matrix.get(denseMatrix);
         for (int index = 0; index < 3; index++)
            denseVectorOriginal.set(index, vectorOriginal.getElement(index));

         CommonOps_DDRM.mult(denseMatrix, denseVectorOriginal, denseVectorTransformed);
         for (int index = 0; index < 3; index++)
            vectorExpected.setElement(index, denseVectorTransformed.get(index));
         vectorExpected.setS(vectorOriginal.getS());

         Matrix3DTools.transform(matrix, vectorOriginal, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
      }

      // Test transforming in-place
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);

         Matrix3DTools.transform(matrix, vectorOriginal, vectorExpected);
         vectorActual.set(vectorOriginal);
         Matrix3DTools.transform(matrix, vectorActual, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
      }
   }

   @Test
   public void testTransformMatrix() throws Exception
   {
      // Compare with transforming a quaternion TODO move + reimplement here
      //      Random random = new Random(234L);
      //      RotationMatrix transformationMatrix = new RotationMatrix();
      //
      //      RotationMatrix rotationMatrixOriginal = new RotationMatrix();
      //      RotationMatrix rotationMatrixExpected = new RotationMatrix();
      //      RotationMatrix rotationMatrixActual = new RotationMatrix();
      //
      //      Quaternion quaternion = new Quaternion();
      //
      //      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      //      {
      //         transformationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      //         rotationMatrixOriginal = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      //         QuaternionConversion.convertMatrixToQuaternion(rotationMatrixOriginal, quaternion);
      //         RotationMatrixTools.transform(transformationMatrix, quaternion, quaternion);
      //         RotationMatrixConversion.convertQuaternionToMatrix(quaternion, rotationMatrixExpected);
      //
      //         RotationMatrixTools.transform(transformationMatrix, rotationMatrixOriginal, rotationMatrixActual);
      //         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrixExpected, rotationMatrixActual, EPS);
      //
      //         RotationMatrixTools.transform(transformationMatrix, rotationMatrixOriginal, rotationMatrixActual);
      //         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrixExpected, rotationMatrixActual, EPS);
      //      }
      //
      //      // Test the transform with a rotation scale matrix against the transform with a generic matrix
      //      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      //      {
      //         RotationScaleMatrix transformationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);
      //         Matrix3D transformationGenericMatrix = new Matrix3D(transformationScaleMatrix);
      //         Matrix3D genericMatrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      //         Matrix3D genericMatrixExpected = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      //         Matrix3D genericMatrixActual = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      //
      //         Matrix3DTools.transform(transformationGenericMatrix, genericMatrixOriginal, genericMatrixExpected);
      //         Matrix3DTools.transform(transformationScaleMatrix, genericMatrixOriginal, genericMatrixActual);
      //         GeometryBasicsTestTools.assertMatrix3DEquals(genericMatrixExpected, genericMatrixActual, EPS);
      //      }
   }

   @Test
   public void testInverseTransformTuple() throws Exception
   {
      Random random = new Random(3489756L);
      Matrix3D matrix = new Matrix3D();
      Matrix3D matrixInverse = new Matrix3D();
      Tuple3DBasics tupleOriginal = new Vector3D();
      Tuple3DBasics tupleActual = new Vector3D();
      Tuple3DBasics tupleExpected = new Vector3D();

      // Test that it throws an axception when the matrix is singular
      matrix.setToZero();
      try
      {
         Matrix3DTools.inverseTransform(matrix, tupleOriginal, tupleActual);
         fail("Should have thrown a " + SingularMatrixException.class.getSimpleName());
      }
      catch (SingularMatrixException e)
      {
         // good
      }

      // Test against transform
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DTools.invert(matrix, matrixInverse);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);

         Matrix3DTools.transform(matrixInverse, tupleOriginal, tupleExpected);
         Matrix3DTools.inverseTransform(matrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test that: tuple == inverseTransform(transform(tuple))
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);
         tupleExpected.set(tupleOriginal);

         Matrix3DTools.transform(matrix, tupleOriginal, tupleActual);
         Matrix3DTools.inverseTransform(matrix, tupleActual, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test with rotation matrix
      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         matrix.set(rotationMatrix);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);

         Matrix3DTools.inverseTransform(matrix, tupleOriginal, tupleExpected);
         Matrix3DTools.inverseTransform(rotationMatrix, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test transform in-place
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         tupleOriginal = EuclidCoreRandomTools.nextVector3D(random);

         Matrix3DTools.inverseTransform(matrix, tupleOriginal, tupleExpected);
         tupleActual.set(tupleOriginal);
         Matrix3DTools.inverseTransform(matrix, tupleActual, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPS);
      }
   }

   @Test
   public void testInverseTransformTuple2D() throws Exception
   {
      Random random = new Random(3489756L);
      Matrix3D matrix = new Matrix3D();
      Matrix3D matrixInverse = new Matrix3D();
      Tuple2DBasics tupleOriginal = new Vector2D();
      Tuple2DBasics tupleActual = new Vector2D();
      Tuple2DBasics tupleExpected = new Vector2D();

      // Test that it throws an axception when the matrix is singular
      matrix.setToZero();
      try
      {
         Matrix3DTools.inverseTransform(matrix, tupleOriginal, tupleActual, false);
         fail("Should have thrown a " + SingularMatrixException.class.getSimpleName());
      }
      catch (SingularMatrixException e)
      {
         // good
      }

      // Test against transform
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         matrix.setRow(2, 0.0, 0.0, 1.0);
         matrix.setColumn(2, 0.0, 0.0, 1.0);
         Matrix3DTools.invert(matrix, matrixInverse);
         tupleOriginal = EuclidCoreRandomTools.nextVector2D(random);

         Matrix3DTools.transform(matrixInverse, tupleOriginal, tupleExpected, false);
         Matrix3DTools.inverseTransform(matrix, tupleOriginal, tupleActual, false);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test with rotation matrix
      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.setToYawOrientation(2.0 * Math.PI * random.nextDouble());
         matrix.set(rotationMatrix);
         tupleOriginal = EuclidCoreRandomTools.nextVector2D(random);

         Matrix3DTools.inverseTransform(matrix, tupleOriginal, tupleExpected, false);
         Matrix3DTools.inverseTransform(rotationMatrix, tupleOriginal, tupleActual, false);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test transform in-place
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         tupleOriginal = EuclidCoreRandomTools.nextVector2D(random);

         Matrix3DTools.inverseTransform(matrix, tupleOriginal, tupleExpected, false);
         tupleActual.set(tupleOriginal);
         Matrix3DTools.inverseTransform(matrix, tupleActual, tupleActual, false);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPS);
      }

      // Test exceptions are thrown with checkIfTransformInXYPlane set to true and given a 3D matrix.
      matrix = EuclidCoreRandomTools.nextMatrix3D(random);
      try
      {
         Matrix3DTools.inverseTransform(matrix, tupleActual, tupleActual, true);
         fail("Should have thrown a " + NotAMatrix2DException.class.getSimpleName());
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }

      RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
      try
      {
         Matrix3DTools.inverseTransform(rotationMatrix, tupleActual, tupleActual, true);
         fail("Should have thrown a " + NotAMatrix2DException.class.getSimpleName());
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
   }

   @Test
   public void testInverseTransformTuple4D() throws Exception
   {
      Random random = new Random(3489756L);
      Matrix3D matrix = new Matrix3D();
      Matrix3D matrixInverse = new Matrix3D();
      Vector4D vectorOriginal = new Vector4D();
      Vector4D vectorActual = new Vector4D();
      Vector4D vectorExpected = new Vector4D();

      // Test that it throws an axception when the matrix is singular
      matrix.setToZero();
      try
      {
         Matrix3DTools.inverseTransform(matrix, vectorOriginal, vectorActual);
         fail("Should have thrown a " + SingularMatrixException.class.getSimpleName());
      }
      catch (SingularMatrixException e)
      {
         // good
      }

      // Test against transform
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DTools.invert(matrix, matrixInverse);
         vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);

         Matrix3DTools.transform(matrixInverse, vectorOriginal, vectorExpected);
         Matrix3DTools.inverseTransform(matrix, vectorOriginal, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
      }

      // Test that: tuple == inverseTransform(transform(tuple))
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);
         vectorExpected.set(vectorOriginal);

         Matrix3DTools.transform(matrix, vectorOriginal, vectorActual);
         Matrix3DTools.inverseTransform(matrix, vectorActual, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
      }

      // Test with rotation matrix
      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         matrix.set(rotationMatrix);
         vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);

         Matrix3DTools.inverseTransform(matrix, vectorOriginal, vectorExpected);
         Matrix3DTools.inverseTransform(rotationMatrix, vectorOriginal, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
      }

      // Test transform in-place
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);

         Matrix3DTools.inverseTransform(matrix, vectorOriginal, vectorExpected);
         vectorActual.set(vectorOriginal);
         Matrix3DTools.inverseTransform(matrix, vectorActual, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
      }
   }
}
