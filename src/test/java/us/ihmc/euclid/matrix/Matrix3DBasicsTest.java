package us.ihmc.euclid.matrix;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.ejml.data.DMatrix;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.LinearTransform3D;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public abstract class Matrix3DBasicsTest<T extends Matrix3DBasics> extends CommonMatrix3DBasicsTest<T>
{
   private static final double MID_EPS = 1.0e-10;

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(6345L);
      T actualMatrix = createEmptyMatrix();

      for (int j = 0; j < ITERATIONS; j++)
      {
         double m00 = random.nextDouble();
         double m01 = random.nextDouble();
         double m02 = random.nextDouble();
         double m10 = random.nextDouble();
         double m11 = random.nextDouble();
         double m12 = random.nextDouble();
         double m20 = random.nextDouble();
         double m21 = random.nextDouble();
         double m22 = random.nextDouble();
         actualMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         assertTrue(m00 == actualMatrix.getM00());
         assertTrue(m01 == actualMatrix.getM01());
         assertTrue(m02 == actualMatrix.getM02());
         assertTrue(m10 == actualMatrix.getM10());
         assertTrue(m11 == actualMatrix.getM11());
         assertTrue(m12 == actualMatrix.getM12());
         assertTrue(m20 == actualMatrix.getM20());
         assertTrue(m21 == actualMatrix.getM21());
         assertTrue(m22 == actualMatrix.getM22());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D expectedMatrix = EuclidCoreRandomTools.nextMatrix3D(random);
         actualMatrix.set(expectedMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix expectedMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         actualMatrix.set(expectedMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         LinearTransform3D expectedMatrix = EuclidCoreRandomTools.nextLinearTransform3D(random);
         actualMatrix.set(expectedMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D expectedMatrix = EuclidCoreRandomTools.nextMatrix3D(random);
         actualMatrix.set((Matrix3DReadOnly) expectedMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] matrixArray = new double[9];
         for (int index = 0; index < 9; index++)
            matrixArray[index] = random.nextDouble();
         actualMatrix.set(matrixArray);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertTrue(actualMatrix.getElement(row, column) == matrixArray[row * 3 + column]);
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         DMatrix denseMatrix = EuclidCoreRandomTools.nextDMatrixRMaj(random, 3, 3);
         actualMatrix.set(denseMatrix);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertTrue(actualMatrix.getElement(row, column) == denseMatrix.get(row, column));
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int startRow = random.nextInt(10);
         int startColumn = random.nextInt(10);
         DMatrix denseMatrix = EuclidCoreRandomTools.nextDMatrixRMaj(random, 3 + startRow, 3 + startColumn);
         actualMatrix.set(startRow, startColumn, denseMatrix);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertTrue(actualMatrix.getElement(row, column) == denseMatrix.get(row + startRow, column + startColumn));
            }
         }
      }
   }

   @Test
   public void testSetToDiagonal() throws Exception
   {
      Random random = new Random(23423);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setToDiagonal(double m00, double m11, double m22)
         double m00 = random.nextDouble();
         double m11 = random.nextDouble();
         double m22 = random.nextDouble();

         T matrix = createRandomMatrix(random);

         matrix.setToDiagonal(m00, m11, m22);

         assertTrue(matrix.getM00() == m00);
         assertTrue(matrix.getM11() == m11);
         assertTrue(matrix.getM22() == m22);

         for (int row = 0; row < 3; row++)
         {
            for (int col = 0; col < 3; col++)
            {
               if (row != col)
                  assertTrue(matrix.getElement(row, col) == 0.0);
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // setToDiagonal(Tuple3DReadOnly tuple)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();

         T matrix = createRandomMatrix(random);

         matrix.setToDiagonal(new Point3D(x, y, z));

         assertTrue(matrix.getM00() == x);
         assertTrue(matrix.getM11() == y);
         assertTrue(matrix.getM22() == z);

         for (int row = 0; row < 3; row++)
         {
            for (int col = 0; col < 3; col++)
            {
               if (row != col)
                  assertTrue(matrix.getElement(row, col) == 0.0);
            }
         }
      }
   }

   @Test
   public void testSetToTildeForm()
   {
      Random random = new Random(646L);
      Matrix3D tildeMatrix = new Matrix3D();
      Vector3D expected = new Vector3D();
      Vector3D actual = new Vector3D();

      for (int k = 0; k < ITERATIONS; k++)
      {
         Vector3D vector1 = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D vector2 = EuclidCoreRandomTools.nextVector3D(random);
         expected.cross(vector1, vector2);

         tildeMatrix.setToTildeForm(vector1);

         tildeMatrix.transform(vector2, actual);

         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testSetToYawPitchRollMatrix()
   {
      Random random = new Random(35454L);
      RotationMatrix rotationMatrix = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         T matrix = createRandomMatrix(random);

         { // Test setToPitchMatrix()
            double pitch = random.nextDouble();

            matrix.setToNaN();
            rotationMatrix.setToNaN();

            matrix.setToPitchMatrix(pitch);
            RotationMatrixConversion.computePitchMatrix(pitch, rotationMatrix);
            EuclidCoreTestTools.assertMatrix3DEquals(matrix, rotationMatrix, SMALL_EPS);
         }

         { // Test setToRollMatrix()
            double roll = random.nextDouble();

            matrix.setToNaN();
            rotationMatrix.setToNaN();

            matrix.setToRollMatrix(roll);
            RotationMatrixConversion.computeRollMatrix(roll, rotationMatrix);
            EuclidCoreTestTools.assertMatrix3DEquals(matrix, rotationMatrix, SMALL_EPS);
         }

         { // Test setToYawMatrix()
            double yaw = random.nextDouble();

            matrix.setToNaN();
            rotationMatrix.setToNaN();

            matrix.setToYawMatrix(yaw);
            RotationMatrixConversion.computeYawMatrix(yaw, rotationMatrix);
            EuclidCoreTestTools.assertMatrix3DEquals(matrix, rotationMatrix, SMALL_EPS);
         }
      }
   }

   @Test
   public void testAdd()
   {
      Random random = new Random(6345L);
      Matrix3D a = new Matrix3D();
      Matrix3D b = new Matrix3D();
      // c = a + b
      T c = createEmptyMatrix();

      for (int p = 0; p < ITERATIONS; p++)
      {
         c.setToNaN();

         a = EuclidCoreRandomTools.nextMatrix3D(random);
         b = EuclidCoreRandomTools.nextMatrix3D(random);

         c.set(a);
         c.add(b);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double aij = a.getElement(row, column);
               double bij = b.getElement(row, column);
               double cij = c.getElement(row, column);
               assertEquals(aij + bij, cij, SMALL_EPS);
            }
         }

         c.setToNaN();
         a = EuclidCoreRandomTools.nextMatrix3D(random);
         b = EuclidCoreRandomTools.nextMatrix3D(random);

         c.add(a, b);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double aij = a.getElement(row, column);
               double bij = b.getElement(row, column);
               double cij = c.getElement(row, column);
               assertEquals(aij + bij, cij, SMALL_EPS);
            }
         }
      }
   }

   @Test
   public void testSub()
   {
      Random random = new Random(6345L);
      Matrix3D a = new Matrix3D();
      Matrix3D b = new Matrix3D();
      // c = a - b
      T c = createEmptyMatrix();

      for (int p = 0; p < ITERATIONS; p++)
      {
         c.setToNaN();

         a = EuclidCoreRandomTools.nextMatrix3D(random);
         b = EuclidCoreRandomTools.nextMatrix3D(random);

         c.set(a);
         c.sub(b);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double aij = a.getElement(row, column);
               double bij = b.getElement(row, column);
               double cij = c.getElement(row, column);
               assertEquals(aij - bij, cij, SMALL_EPS);
            }
         }

         c.setToNaN();
         a = EuclidCoreRandomTools.nextMatrix3D(random);
         b = EuclidCoreRandomTools.nextMatrix3D(random);

         c.sub(a, b);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double aij = a.getElement(row, column);
               double bij = b.getElement(row, column);
               double cij = c.getElement(row, column);
               assertEquals(aij - bij, cij, SMALL_EPS);
            }
         }
      }
   }

   @Test
   public void testScale()
   {
      Random random = new Random(6345L);
      double scalar = Double.NaN;
      Matrix3D matrixOriginal = new Matrix3D();
      T scaledMatrix = createEmptyMatrix();

      Vector3D scales = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         scalar = random.nextDouble();

         scaledMatrix.set(matrixOriginal);
         scaledMatrix.scale(scalar);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double mij = matrixOriginal.getElement(row, column);
               double sij = scaledMatrix.getElement(row, column);
               assertEquals(scalar * mij, sij, SMALL_EPS);
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         scalar = random.nextDouble();
         int scaledRow = random.nextInt(3);

         scaledMatrix.set(matrixOriginal);
         scaledMatrix.scaleRow(scaledRow, scalar);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double mij = matrixOriginal.getElement(row, column);
               double sij = scaledMatrix.getElement(row, column);
               if (row == scaledRow)
                  assertEquals(scalar * mij, sij, SMALL_EPS);
               else
                  assertEquals(mij, sij, SMALL_EPS);
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         scalar = random.nextDouble();
         int scaledColumn = random.nextInt(3);

         scaledMatrix.set(matrixOriginal);
         scaledMatrix.scaleColumn(scaledColumn, scalar);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double mij = matrixOriginal.getElement(row, column);
               double sij = scaledMatrix.getElement(row, column);
               if (column == scaledColumn)
                  assertEquals(scalar * mij, sij, SMALL_EPS);
               else
                  assertEquals(mij, sij, SMALL_EPS);
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         scales = EuclidCoreRandomTools.nextRotationVector(random, 2.0);

         scaledMatrix.set(matrixOriginal);
         scaledMatrix.scaleColumns(scales.getX(), scales.getY(), scales.getZ());

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double mij = matrixOriginal.getElement(row, column);
               double sij = scaledMatrix.getElement(row, column);
               assertEquals(scales.getElement(column) * mij, sij, SMALL_EPS);
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         scales = EuclidCoreRandomTools.nextRotationVector(random, 2.0);

         scaledMatrix.set(matrixOriginal);
         scaledMatrix.scaleRows(scales.getX(), scales.getY(), scales.getZ());

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double mij = matrixOriginal.getElement(row, column);
               double sij = scaledMatrix.getElement(row, column);
               assertEquals(scales.getElement(row) * mij, sij, SMALL_EPS);
            }
         }
      }

      matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM00(scalar);
      assertEquals(scalar * matrixOriginal.getM00(), scaledMatrix.getM00(), SMALL_EPS);

      matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM01(scalar);
      assertEquals(scalar * matrixOriginal.getM01(), scaledMatrix.getM01(), SMALL_EPS);

      matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM02(scalar);
      assertEquals(scalar * matrixOriginal.getM02(), scaledMatrix.getM02(), SMALL_EPS);

      matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM10(scalar);
      assertEquals(scalar * matrixOriginal.getM10(), scaledMatrix.getM10(), SMALL_EPS);

      matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM11(scalar);
      assertEquals(scalar * matrixOriginal.getM11(), scaledMatrix.getM11(), SMALL_EPS);

      matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM12(scalar);
      assertEquals(scalar * matrixOriginal.getM12(), scaledMatrix.getM12(), SMALL_EPS);

      matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM20(scalar);
      assertEquals(scalar * matrixOriginal.getM20(), scaledMatrix.getM20(), SMALL_EPS);

      matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM21(scalar);
      assertEquals(scalar * matrixOriginal.getM21(), scaledMatrix.getM21(), SMALL_EPS);

      matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM22(scalar);
      assertEquals(scalar * matrixOriginal.getM22(), scaledMatrix.getM22(), SMALL_EPS);

      // Test some exceptions
      double scalarFinal = scalar;
      assertThrows(ArrayIndexOutOfBoundsException.class, () -> scaledMatrix.scaleRow(3, scalarFinal));
      assertThrows(ArrayIndexOutOfBoundsException.class, () -> scaledMatrix.scaleColumn(3, scalarFinal));
   }

   @Test
   public void testMultiplyOuter()
   {
      Random random = new Random(51665L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test multiplyOuter()
         T actualMatrix = createRandomMatrix(random);
         T expectedMatrix = createEmptyMatrix();

         Matrix3DTools.multiplyTransposeRight(actualMatrix, actualMatrix, expectedMatrix);
         actualMatrix.multiplyOuter();

         EuclidCoreTestTools.assertMatrix3DEquals(actualMatrix, expectedMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test multiplyOuter(Matrix3DBasics other)
         T actualMatrix = createRandomMatrix(random);
         T expectedMatrix = createEmptyMatrix();

         Matrix3D m = EuclidCoreRandomTools.nextMatrix3D(random);
         actualMatrix.setAndMultiplyOuter(m);
         expectedMatrix.set(m);
         expectedMatrix.multiplyOuter();

         EuclidCoreTestTools.assertMatrix3DEquals(actualMatrix, expectedMatrix, SMALL_EPS);
      }
   }

   @Test
   public void testInvert() throws Exception
   {
      Random random = new Random(54654L);
      Matrix3D matrix = new Matrix3D();
      T expectedInvert = createEmptyMatrix();
      T actualInvert = createEmptyMatrix();

      actualInvert.set(0, 0, 0, 0, 0, 0, 0, 0, 0);
      assertThrows(SingularMatrixException.class, () -> actualInvert.invert());

      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DTools.invert(matrix, expectedInvert);
         actualInvert.set(matrix);
         actualInvert.invert();
         EuclidCoreTestTools.assertMatrix3DEquals(expectedInvert, actualInvert, SMALL_EPS);

         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DTools.invert(matrix, expectedInvert);
         actualInvert.setAndInvert(matrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedInvert, actualInvert, SMALL_EPS);
      }
   }

   @Test
   public void testNormalize() throws Exception
   {
      Random random = new Random(541654L);
      T actualMatrix = createEmptyMatrix();
      T expectedMatrix = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D matrix = EuclidCoreRandomTools.nextMatrix3D(random, 2.0);

         expectedMatrix.set(matrix);
         Matrix3DTools.normalize(expectedMatrix);
         actualMatrix.set(matrix);
         actualMatrix.normalize();
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D matrix = EuclidCoreRandomTools.nextMatrix3D(random, 2.0);

         expectedMatrix.set(matrix);
         Matrix3DTools.normalize(expectedMatrix);
         actualMatrix.setAndNormalize(matrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }
   }

   @Test
   public void testTranspose() throws Exception
   {
      super.testTranspose();

      Random random = new Random(65451L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T matrix = createRandomMatrix(random);
         T matrixTranspose = createEmptyMatrix();

         matrixTranspose.setAndTranspose(matrix);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertEquals(matrix.getElement(row, column), matrixTranspose.getElement(column, row), SMALL_EPS);
            }
         }
      }
   }

   @Test
   public void testNegate() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T original = createRandomMatrix(random);
         T negated = createRandomMatrix(random);

         negated.set(original);
         negated.negate();

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertEquals(-original.getElement(row, column), negated.getElement(row, column), SMALL_EPS);
            }
         }

         negated.setToNaN();
         EuclidCoreTestTools.assertMatrix3DContainsOnlyNaN(negated);
         negated.setAndNegate(original);
         original.negate();
         EuclidCoreTestTools.assertMatrix3DEquals(original, negated, SMALL_EPS);
      }
   }

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(65561L);

      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiply(m1, m2, expected);
         actual.set(m1);
         actual.multiply(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testMultiplyTransposeThis() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiplyTransposeLeft(m1, m2, expected);
         actual.set(m1);
         actual.multiplyTransposeThis(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testMultiplyInvertThis() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiplyInvertLeft(m1, m2, expected);
         actual.set(m1);
         actual.multiplyInvertThis(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testMultiplyTransposeOther() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiplyTransposeRight(m1, m2, expected);
         actual.set(m1);
         actual.multiplyTransposeOther(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testMultiplyInvertOther() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiplyInvertRight(m1, m2, expected);
         actual.set(m1);
         actual.multiplyInvertOther(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);

         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Matrix3DTools.multiplyInvertRight(m1, rotationMatrix, expected);
         actual.set(m1);
         actual.multiplyInvertOther(rotationMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testMultiplyTransposeBoth() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiplyTransposeBoth(m1, m2, expected);
         actual.set(m1);
         actual.multiplyTransposeBoth(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testPreMultiply() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiply(m2, m1, expected);
         actual.set(m1);
         actual.preMultiply(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testPreMultiplyTransposeThis() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiplyTransposeRight(m2, m1, expected);
         actual.set(m1);
         actual.preMultiplyTransposeThis(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThis() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiplyInvertRight(m2, m1, expected);
         actual.set(m1);
         actual.preMultiplyInvertThis(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testPreMultiplyTransposeOther() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiplyTransposeLeft(m2, m1, expected);
         actual.set(m1);
         actual.preMultiplyTransposeOther(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOther() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiplyInvertLeft(m2, m1, expected);
         actual.set(m1);
         actual.preMultiplyInvertOther(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);

         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Matrix3DTools.multiplyInvertLeft(rotationMatrix, m1, expected);
         actual.set(m1);
         actual.preMultiplyInvertOther(rotationMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testPreMultiplyTransposeBoth() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      T expected = createEmptyMatrix();
      T actual = createEmptyMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random);
         m2 = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3DTools.multiplyTransposeBoth(m2, m1, expected);
         actual.set(m1);
         actual.preMultiplyTransposeBoth(m2);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testSetRow() throws Exception
   {
      Random random = new Random(6465L);
      T matrix = createEmptyMatrix();

      double[] rowArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble()};
      matrix.setRow(0, rowArray);
      assertTrue(matrix.getM00() == rowArray[0]);
      assertTrue(matrix.getM01() == rowArray[1]);
      assertTrue(matrix.getM02() == rowArray[2]);

      matrix.setRow(1, rowArray);
      assertTrue(matrix.getM10() == rowArray[0]);
      assertTrue(matrix.getM11() == rowArray[1]);
      assertTrue(matrix.getM12() == rowArray[2]);

      matrix.setRow(2, rowArray);
      assertTrue(matrix.getM20() == rowArray[0]);
      assertTrue(matrix.getM21() == rowArray[1]);
      assertTrue(matrix.getM22() == rowArray[2]);

      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setRow(3, rowArray));

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();
      matrix.setRow(0, x, y, z);
      assertTrue(matrix.getM00() == x);
      assertTrue(matrix.getM01() == y);
      assertTrue(matrix.getM02() == z);

      matrix.setRow(1, x, y, z);
      assertTrue(matrix.getM10() == x);
      assertTrue(matrix.getM11() == y);
      assertTrue(matrix.getM12() == z);

      matrix.setRow(2, x, y, z);
      assertTrue(matrix.getM20() == x);
      assertTrue(matrix.getM21() == y);
      assertTrue(matrix.getM22() == z);

      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setRow(3, x, y, z));

      Vector3D rowVector = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      matrix.setRow(0, rowVector);
      assertTrue(matrix.getM00() == rowVector.getElement(0));
      assertTrue(matrix.getM01() == rowVector.getElement(1));
      assertTrue(matrix.getM02() == rowVector.getElement(2));

      matrix.setRow(1, rowVector);
      assertTrue(matrix.getM10() == rowVector.getElement(0));
      assertTrue(matrix.getM11() == rowVector.getElement(1));
      assertTrue(matrix.getM12() == rowVector.getElement(2));

      matrix.setRow(2, rowVector);
      assertTrue(matrix.getM20() == rowVector.getElement(0));
      assertTrue(matrix.getM21() == rowVector.getElement(1));
      assertTrue(matrix.getM22() == rowVector.getElement(2));

      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setRow(3, rowVector));
   }

   @Test
   public void testSetColumn() throws Exception
   {
      Random random = new Random(6465L);
      T matrix = createEmptyMatrix();

      double[] columnArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble()};
      matrix.setColumn(0, columnArray);
      assertTrue(matrix.getM00() == columnArray[0]);
      assertTrue(matrix.getM10() == columnArray[1]);
      assertTrue(matrix.getM20() == columnArray[2]);

      matrix.setColumn(1, columnArray);
      assertTrue(matrix.getM01() == columnArray[0]);
      assertTrue(matrix.getM11() == columnArray[1]);
      assertTrue(matrix.getM21() == columnArray[2]);

      matrix.setColumn(2, columnArray);
      assertTrue(matrix.getM02() == columnArray[0]);
      assertTrue(matrix.getM12() == columnArray[1]);
      assertTrue(matrix.getM22() == columnArray[2]);

      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setColumn(3, columnArray));

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();
      matrix.setColumn(0, x, y, z);
      assertTrue(matrix.getM00() == x);
      assertTrue(matrix.getM10() == y);
      assertTrue(matrix.getM20() == z);

      matrix.setColumn(1, x, y, z);
      assertTrue(matrix.getM01() == x);
      assertTrue(matrix.getM11() == y);
      assertTrue(matrix.getM21() == z);

      matrix.setColumn(2, x, y, z);
      assertTrue(matrix.getM02() == x);
      assertTrue(matrix.getM12() == y);
      assertTrue(matrix.getM22() == z);

      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setColumn(3, x, y, z));

      Vector3D columnVector = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      matrix.setColumn(0, columnVector);
      assertTrue(matrix.getM00() == columnVector.getElement(0));
      assertTrue(matrix.getM10() == columnVector.getElement(1));
      assertTrue(matrix.getM20() == columnVector.getElement(2));

      matrix.setColumn(1, columnVector);
      assertTrue(matrix.getM01() == columnVector.getElement(0));
      assertTrue(matrix.getM11() == columnVector.getElement(1));
      assertTrue(matrix.getM21() == columnVector.getElement(2));

      matrix.setColumn(2, columnVector);
      assertTrue(matrix.getM02() == columnVector.getElement(0));
      assertTrue(matrix.getM12() == columnVector.getElement(1));
      assertTrue(matrix.getM22() == columnVector.getElement(2));

      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setColumn(3, columnVector));
   }

   @Test
   public void testSetGetElement() throws Exception
   {
      Random random = new Random(5464L);
      T matrix = createEmptyMatrix();
      double coeff;

      matrix.setElement(0, 0, coeff = random.nextDouble());
      assertTrue(matrix.getM00() == coeff);
      assertTrue(matrix.getElement(0, 0) == coeff);
      matrix.setElement(0, 1, coeff = random.nextDouble());
      assertTrue(matrix.getM01() == coeff);
      assertTrue(matrix.getElement(0, 1) == coeff);
      matrix.setElement(0, 2, coeff = random.nextDouble());
      assertTrue(matrix.getM02() == coeff);
      assertTrue(matrix.getElement(0, 2) == coeff);
      matrix.setElement(1, 0, coeff = random.nextDouble());
      assertTrue(matrix.getM10() == coeff);
      assertTrue(matrix.getElement(1, 0) == coeff);
      matrix.setElement(1, 1, coeff = random.nextDouble());
      assertTrue(matrix.getM11() == coeff);
      assertTrue(matrix.getElement(1, 1) == coeff);
      matrix.setElement(1, 2, coeff = random.nextDouble());
      assertTrue(matrix.getM12() == coeff);
      assertTrue(matrix.getElement(1, 2) == coeff);
      matrix.setElement(2, 0, coeff = random.nextDouble());
      assertTrue(matrix.getM20() == coeff);
      assertTrue(matrix.getElement(2, 0) == coeff);
      matrix.setElement(2, 1, coeff = random.nextDouble());
      assertTrue(matrix.getM21() == coeff);
      assertTrue(matrix.getElement(2, 1) == coeff);
      matrix.setElement(2, 2, coeff = random.nextDouble());
      assertTrue(matrix.getM22() == coeff);
      assertTrue(matrix.getElement(2, 2) == coeff);

      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setElement(0, 3, random.nextDouble()));
      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setElement(1, 3, random.nextDouble()));
      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setElement(2, 3, random.nextDouble()));
      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setElement(3, 0, random.nextDouble()));
      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setElement(3, 1, random.nextDouble()));
      assertThrows(ArrayIndexOutOfBoundsException.class, () -> matrix.setElement(3, 2, random.nextDouble()));
   }

   @Test
   public void testFieldSettersAndGetters() throws Exception
   {
      Random random = new Random(654654L);
      T matrix = createEmptyMatrix();
      double coeff;

      matrix.setM00(coeff = random.nextDouble());
      assertTrue(matrix.getM00() == coeff);
      matrix.setM01(coeff = random.nextDouble());
      assertTrue(matrix.getM01() == coeff);
      matrix.setM02(coeff = random.nextDouble());
      assertTrue(matrix.getM02() == coeff);
      matrix.setM10(coeff = random.nextDouble());
      assertTrue(matrix.getM10() == coeff);
      matrix.setM11(coeff = random.nextDouble());
      assertTrue(matrix.getM11() == coeff);
      matrix.setM12(coeff = random.nextDouble());
      assertTrue(matrix.getM12() == coeff);
      matrix.setM20(coeff = random.nextDouble());
      assertTrue(matrix.getM20() == coeff);
      matrix.setM21(coeff = random.nextDouble());
      assertTrue(matrix.getM21() == coeff);
      matrix.setM22(coeff = random.nextDouble());
      assertTrue(matrix.getM22() == coeff);
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(2354L);
      Matrix3D m1 = EuclidCoreRandomTools.nextMatrix3D(random);
      Matrix3D m2 = new Matrix3D();

      assertFalse(m1.equals(m2));
      assertFalse(m1.equals(null));
      assertFalse(m1.equals(new double[4]));
      m2.set(m1);
      assertTrue(m1.equals(m2));
      Object m2AsObject = m2;
      assertTrue(m1.equals(m2AsObject));

      double smallestEpsilon = 1.0e-16;

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            m2.set(m1);
            assertTrue(m1.equals(m2));
            m2.setElement(row, column, m2.getElement(row, column) + smallestEpsilon);
            assertFalse(m1.equals(m2));

            m2.set(m1);
            assertTrue(m1.equals(m2));
            m2.setElement(row, column, m2.getElement(row, column) - smallestEpsilon);
            assertFalse(m1.equals(m2));
         }
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      T matrix = createRandomMatrix(random);

      int newHashCode, previousHashCode;
      newHashCode = matrix.hashCode();
      assertEquals(newHashCode, matrix.hashCode());

      previousHashCode = matrix.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         int row = random.nextInt(3);
         int column = random.nextInt(3);
         matrix.setElement(row, column, random.nextDouble());
         newHashCode = matrix.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T original = createRandomMatrix(random);
         T expected = createEmptyMatrix();
         T actual = createEmptyMatrix();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T original = createRandomMatrix(random);
         T expected = createEmptyMatrix();
         T actual = createEmptyMatrix();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         T original = createRandomMatrix(random);
         T expected = createEmptyMatrix();
         T actual = createEmptyMatrix();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testApplyInverseTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T original = createRandomMatrix(random);
         T expected = createEmptyMatrix();
         T actual = createEmptyMatrix();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T original = createRandomMatrix(random);
         T expected = createEmptyMatrix();
         T actual = createEmptyMatrix();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextNonSingularAffineTransform(random);
         T original = createRandomMatrix(random);
         T expected = createEmptyMatrix();
         T actual = createEmptyMatrix();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, MID_EPS);
      }
   }
}
