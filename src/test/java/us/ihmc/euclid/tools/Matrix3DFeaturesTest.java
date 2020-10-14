package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.ejml.MatrixDimensionException;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;

public class Matrix3DFeaturesTest
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testContainsNaN() throws Exception
   {
      Matrix3D matrix = new Matrix3D();
      matrix.set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertFalse(matrix.containsNaN());
      matrix.set(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.set(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.set(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.set(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.set(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.set(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN);
      assertTrue(matrix.containsNaN());
   }

   /** This is a tough one to test. */
   @Test
   public void testCheckIfRotationMatrixAndIsRotationMatrix() throws Exception
   {
      Random random = new Random(93486534L);
      Matrix3D matrix = new Matrix3D();

      matrix.setIdentity();
      testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, true);
      matrix.setToZero();
      testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, false);
      matrix.setToNaN();
      testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, false);

      // Let's first test that it returns true for an actual rotation matrix
      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = new Matrix3D(EuclidCoreRandomTools.nextRotationMatrix(random));
         testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, true);

         // Test when the matrix is not a rotation matrix, by assuming that it is most unlikely to generate a rotation matrix by generating random elements
         matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, false);

         // Start from a rotation matrix and apply slight changes to make not a rotation matrix
         // Add an offset to one element
         matrix = new Matrix3D(EuclidCoreRandomTools.nextRotationMatrix(random));
         int row = random.nextInt(3);
         int column = random.nextInt(3);
         matrix.setElement(row, column, matrix.getElement(row, column) + random.nextDouble());
         testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, false);

         // Scale a row
         matrix = new Matrix3D(EuclidCoreRandomTools.nextRotationMatrix(random));
         row = random.nextInt(3);
         double scale = EuclidCoreRandomTools.nextDouble(random, 1.0, 1.5);
         matrix.setElement(row, 0, scale * matrix.getElement(row, 0));
         matrix.setElement(row, 1, scale * matrix.getElement(row, 1));
         matrix.setElement(row, 2, scale * matrix.getElement(row, 2));
         testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, false);

         // Scale a column
         matrix = new Matrix3D(EuclidCoreRandomTools.nextRotationMatrix(random));
         column = random.nextInt(3);
         scale = EuclidCoreRandomTools.nextDouble(random, 1.0, 1.5);
         matrix.setElement(0, column, scale * matrix.getElement(0, column));
         matrix.setElement(1, column, scale * matrix.getElement(1, column));
         matrix.setElement(2, column, scale * matrix.getElement(2, column));
         testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, false);

         // Set an element to zero
         matrix = new Matrix3D(EuclidCoreRandomTools.nextRotationMatrix(random));
         row = random.nextInt(3);
         column = random.nextInt(3);
         double previousValue = matrix.getElement(row, column);
         matrix.setElement(row, column, 0.0);
         testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, Math.abs(previousValue) < Matrix3DFeatures.EPS_CHECK_ROTATION);

         // Swap two elements
         matrix = new Matrix3D(EuclidCoreRandomTools.nextRotationMatrix(random));
         row = random.nextInt(3);
         column = random.nextInt(3);
         int row2 = random.nextInt(2);
         int column2 = random.nextInt(2);
         if (row2 == row)
            row2++;
         if (column2 == column)
            column2++;
         double temp = matrix.getElement(row, column);
         matrix.setElement(row, column, matrix.getElement(row2, column2));
         matrix.setElement(row2, column2, temp);
         testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, false);

         // Swap 2 rows
         matrix = new Matrix3D(EuclidCoreRandomTools.nextRotationMatrix(random));
         row = random.nextInt(3);
         row2 = random.nextInt(2);
         if (row2 == row)
            row2++;
         double[] swapArray = new double[3];
         double[] swapArray2 = new double[3];
         matrix.getRow(row, swapArray);
         matrix.getRow(row2, swapArray2);
         matrix.setRow(row, swapArray2);
         matrix.setRow(row2, swapArray);
         testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, false);

         // Swap 2 columns
         matrix = new Matrix3D(EuclidCoreRandomTools.nextRotationMatrix(random));
         column = random.nextInt(3);
         column2 = random.nextInt(2);
         if (column2 == column)
            column2++;
         matrix.getColumn(column, swapArray);
         matrix.getColumn(column2, swapArray2);
         matrix.setColumn(column, swapArray2);
         matrix.setColumn(column2, swapArray);
         testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(matrix, false);
      }

      // Also test the check on the DMatrixRMaj dimension
      assertThrows(MatrixDimensionException.class, () -> Matrix3DFeatures.checkIfRotationMatrix(new DMatrixRMaj(2, 3)));
      assertThrows(MatrixDimensionException.class, () -> Matrix3DFeatures.checkIfRotationMatrix(new DMatrixRMaj(3, 2)));
      assertThrows(MatrixDimensionException.class, () -> Matrix3DFeatures.checkIfRotationMatrix(new DMatrixRMaj(4, 3)));
      assertThrows(MatrixDimensionException.class, () -> Matrix3DFeatures.checkIfRotationMatrix(new DMatrixRMaj(3, 4)));
   }

   private void testAllCheckIfRotationMatrixAndIsRotationMatrixMethods(Matrix3DReadOnly matrix, boolean isRotationMatrix)
   {
      Matrix3D matrixCopy = new Matrix3D(matrix);
      DMatrixRMaj denseMatrix = new DMatrixRMaj(3, 3);
      matrix.get(denseMatrix);
      DMatrixRMaj denseMatrixCopy = new DMatrixRMaj(denseMatrix);
      double[] matrixArray = new double[9];
      matrix.get(matrixArray);
      double[] matrixArrayCopy = new double[9];
      System.arraycopy(matrixArray, 0, matrixArrayCopy, 0, 9);

      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();

      try
      {
         Matrix3DFeatures.checkIfRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         if (!isRotationMatrix)
            fail("Should have thrown a NotARotationMatrixException.");
      }
      catch (NotARotationMatrixException e)
      {
         if (isRotationMatrix)
            throw e;
         // else it is good
         assertTrue(e.getMessage().equals("The matrix is not a rotation matrix: \n" + matrix));
      }
      assertTrue(Matrix3DFeatures.isRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22) == isRotationMatrix);

      try
      {
         matrix.checkIfRotationMatrix();
         if (!isRotationMatrix)
            fail("Should have thrown a NotARotationMatrixException.");
      }
      catch (NotARotationMatrixException e)
      {
         if (isRotationMatrix)
            throw e;
         // else it is good
         assertTrue(e.getMessage().equals("The matrix is not a rotation matrix: \n" + matrix));
      }
      for (int row = 0; row < 3; row++)
         for (int column = 0; column < 3; column++)
            assertTrue(Double.compare(matrix.getElement(row, column), matrixCopy.getElement(row, column)) == 0);
      assertTrue(matrix.isRotationMatrix() == isRotationMatrix);
      for (int row = 0; row < 3; row++)
         for (int column = 0; column < 3; column++)
            assertTrue(Double.compare(matrix.getElement(row, column), matrixCopy.getElement(row, column)) == 0);

      try
      {
         Matrix3DFeatures.checkIfRotationMatrix(denseMatrix);
         if (!isRotationMatrix)
            fail("Should have thrown a NotARotationMatrixException.");
      }
      catch (NotARotationMatrixException e)
      {
         if (isRotationMatrix)
            throw e;
         // else it is good
         assertTrue(e.getMessage().contains("The matrix is not a rotation matrix: \n" + matrix));
      }
      for (int index = 0; index < denseMatrix.getNumElements(); index++)
         assertTrue(Double.compare(denseMatrix.get(index), denseMatrixCopy.get(index)) == 0);
      assertTrue(Matrix3DFeatures.isRotationMatrix(denseMatrix) == isRotationMatrix);

      try
      {
         Matrix3DFeatures.checkIfRotationMatrix(matrixArray);
         if (!isRotationMatrix)
            fail("Should have thrown a NotARotationMatrixException.");
      }
      catch (NotARotationMatrixException e)
      {
         if (isRotationMatrix)
            throw e;
         // else it is good
         assertTrue(e.getMessage().contains("The matrix is not a rotation matrix: \n" + matrix));
      }
      for (int index = 0; index < 9; index++)
         assertTrue(Double.compare(matrixArray[index], matrixArrayCopy[index]) == 0);
      assertTrue(Matrix3DFeatures.isRotationMatrix(matrixArray) == isRotationMatrix);
      for (int index = 0; index < 9; index++)
         assertTrue(Double.compare(matrixArray[index], matrixArrayCopy[index]) == 0);
   }

   @Test
   public void testCheckIfMatrix2DAndIsMatrix2D() throws Exception
   {
      Random random = new Random(93486534L);

      Matrix3D matrix = new Matrix3D();

      matrix.setIdentity();
      testCheckIfMatrix2DAndIsMatrix2DMethods(matrix, true);
      matrix.setToZero();
      testCheckIfMatrix2DAndIsMatrix2DMethods(matrix, false);
      matrix.setToNaN();
      testCheckIfMatrix2DAndIsMatrix2DMethods(matrix, false);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D randomMatrix = EuclidCoreRandomTools.nextMatrix3D(random);
         randomMatrix.setM02(0.0);
         randomMatrix.setM20(0.0);
         randomMatrix.setM12(0.0);
         randomMatrix.setM21(0.0);
         randomMatrix.setM22(1.0);
         testCheckIfMatrix2DAndIsMatrix2DMethods(randomMatrix, true);

         double delta = random.nextBoolean() ? 1.0 : -1.0;
         delta *= random.nextDouble();
         // corrupt the last diagonal element
         randomMatrix.setM22(1.0 + delta);
         testCheckIfMatrix2DAndIsMatrix2DMethods(randomMatrix, Math.abs(delta) < Matrix3DFeatures.EPS_CHECK_2D);
         // reset changes
         randomMatrix.setM22(1.0);

         // corrupt m02
         randomMatrix.setM02(0.0 + delta);
         testCheckIfMatrix2DAndIsMatrix2DMethods(randomMatrix, Math.abs(delta) < Matrix3DFeatures.EPS_CHECK_2D);
         // reset changes
         randomMatrix.setM02(0.0);

         // corrupt m02
         randomMatrix.setM20(0.0 + delta);
         testCheckIfMatrix2DAndIsMatrix2DMethods(randomMatrix, Math.abs(delta) < Matrix3DFeatures.EPS_CHECK_2D);
         // reset changes
         randomMatrix.setM20(0.0);

         // corrupt m02
         randomMatrix.setM12(0.0 + delta);
         testCheckIfMatrix2DAndIsMatrix2DMethods(randomMatrix, Math.abs(delta) < Matrix3DFeatures.EPS_CHECK_2D);
         // reset changes
         randomMatrix.setM12(0.0);

         // corrupt m02
         randomMatrix.setM21(0.0 + delta);
         testCheckIfMatrix2DAndIsMatrix2DMethods(randomMatrix, Math.abs(delta) < Matrix3DFeatures.EPS_CHECK_2D);
      }
   }

   private void testCheckIfMatrix2DAndIsMatrix2DMethods(Matrix3DReadOnly matrix, boolean isMatrix2D)
   {
      Matrix3D matrixCopy = new Matrix3D(matrix);

      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();
      String matrixAsString = EuclidCoreIOTools.getMatrix3DString(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      assertTrue(Matrix3DFeatures.isMatrix2D(m00, m01, m02, m10, m11, m12, m20, m21, m22, Matrix3DFeatures.EPS_CHECK_2D) == isMatrix2D);

      try
      {
         matrix.checkIfMatrix2D();
         if (!isMatrix2D)
            fail("Should have thrown a NotAMatrix2DException.");
      }
      catch (NotAMatrix2DException e)
      {
         if (isMatrix2D)
            throw e;
         // else it is good
         assertTrue(e.getMessage().equals("The matrix is not in XY plane: \n" + matrixAsString));
      }
      for (int row = 0; row < 3; row++)
         for (int column = 0; column < 3; column++)
            assertTrue(Double.compare(matrix.getElement(row, column), matrixCopy.getElement(row, column)) == 0);

      assertTrue(matrix.isMatrix2D() == isMatrix2D);

      for (int row = 0; row < 3; row++)
         for (int column = 0; column < 3; column++)
            assertTrue(Double.compare(matrix.getElement(row, column), matrixCopy.getElement(row, column)) == 0);
   }

   @Test
   public void testCheckMatrixSize() throws Exception
   {
      // Also test the check on the DMatrixRMaj dimension
      try
      {
         Matrix3DFeatures.checkIfRotationMatrix(new DMatrixRMaj(2, 3));
         fail("Should have got a RuntimeException for providing a matrix with wrong size.");
      }
      catch (RuntimeException e)
      {
         assertTrue(e.getMessage().equals("Unexpected matrix size: 2-by-3. Must be 3-by-3."));
      }

      try
      {
         Matrix3DFeatures.checkIfRotationMatrix(new DMatrixRMaj(3, 2));
         fail("Should have got a RuntimeException for providing a matrix with wrong size.");
      }
      catch (RuntimeException e)
      {
         assertTrue(e.getMessage().equals("Unexpected matrix size: 3-by-2. Must be 3-by-3."));
      }

      try
      {
         Matrix3DFeatures.checkIfRotationMatrix(new DMatrixRMaj(4, 3));
         fail("Should have got a RuntimeException for providing a matrix with wrong size.");
      }
      catch (RuntimeException e)
      {
         assertTrue(e.getMessage().equals("Unexpected matrix size: 4-by-3. Must be 3-by-3."));
      }

      try
      {
         Matrix3DFeatures.checkIfRotationMatrix(new DMatrixRMaj(3, 4));
         fail("Should have got a RuntimeException for providing a matrix with wrong size.");
      }
      catch (RuntimeException e)
      {
         assertTrue(e.getMessage().equals("Unexpected matrix size: 3-by-4. Must be 3-by-3."));
      }
   }

   @Test
   public void testDeterminant() throws Exception
   {
      Random random = new Random(641651L);
      DMatrixRMaj denseMatrix = new DMatrixRMaj(3, 3);

      { // Test that the identity's determinant is equal to 1.0
         Matrix3D identity = new Matrix3D();
         identity.setIdentity();
         assertEquals(1.0, identity.determinant(), EPS);
         assertEquals(1.0, Matrix3DFeatures.determinant(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test that the determinant of a random rotation matrix is also equal to 1.0
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         assertEquals(1.0, rotationMatrix.determinant(), EPS);
         double m00 = rotationMatrix.getM00();
         double m01 = rotationMatrix.getM01();
         double m02 = rotationMatrix.getM02();
         double m10 = rotationMatrix.getM10();
         double m11 = rotationMatrix.getM11();
         double m12 = rotationMatrix.getM12();
         double m20 = rotationMatrix.getM20();
         double m21 = rotationMatrix.getM21();
         double m22 = rotationMatrix.getM22();
         assertEquals(1.0, Matrix3DFeatures.determinant(m00, m01, m02, m10, m11, m12, m20, m21, m22), EPS);
      }

      // Check det == 0.0 when a column is zero
      for (int i = 0; i < ITERATIONS; i++)
      {
         int zeroColumn = random.nextInt(3);
         double[] row0 = new double[3];
         double[] row1 = new double[3];
         double[] row2 = new double[3];

         for (int column = 0; column < 3; column++)
         {
            row0[column] = column == zeroColumn ? 0.0 : random.nextDouble();
            row1[column] = column == zeroColumn ? 0.0 : random.nextDouble();
            row2[column] = column == zeroColumn ? 0.0 : random.nextDouble();
         }

         double det = Matrix3DFeatures.determinant(row0[0], row0[1], row0[2], row1[0], row1[1], row1[2], row2[0], row2[1], row2[2]);
         assertEquals(0.0, det, EPS);
      }

      // Check that row swap negates the determinant
      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] column0 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] column1 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] column2 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};

         double det = Matrix3DFeatures.determinant(column0[0], column1[0], column2[0], column0[1], column1[1], column2[1], column0[2], column1[2], column2[2]);

         int rowSwap1 = random.nextInt(3);
         int rowSwap2 = (rowSwap1 + 1 + random.nextInt(2)) % 3;

         double temp0 = column0[rowSwap1];
         double temp1 = column1[rowSwap1];
         double temp2 = column2[rowSwap1];
         column0[rowSwap1] = column0[rowSwap2];
         column1[rowSwap1] = column1[rowSwap2];
         column2[rowSwap1] = column2[rowSwap2];
         column0[rowSwap2] = temp0;
         column1[rowSwap2] = temp1;
         column2[rowSwap2] = temp2;

         double detSwapped = Matrix3DFeatures.determinant(column0[0],
                                                          column1[0],
                                                          column2[0],
                                                          column0[1],
                                                          column1[1],
                                                          column2[1],
                                                          column0[2],
                                                          column1[2],
                                                          column2[2]);
         assertEquals(detSwapped, -det, EPS);
      }

      // Check that column swap negates the determinant
      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] row0 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] row1 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] row2 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};

         double det = Matrix3DFeatures.determinant(row0[0], row0[1], row0[2], row1[0], row1[1], row1[2], row2[0], row2[1], row2[2]);

         int columnSwap1 = random.nextInt(3);
         int columnSwap2 = (columnSwap1 + 1 + random.nextInt(2)) % 3;

         double temp0 = row0[columnSwap1];
         double temp1 = row1[columnSwap1];
         double temp2 = row2[columnSwap1];
         row0[columnSwap1] = row0[columnSwap2];
         row1[columnSwap1] = row1[columnSwap2];
         row2[columnSwap1] = row2[columnSwap2];
         row0[columnSwap2] = temp0;
         row1[columnSwap2] = temp1;
         row2[columnSwap2] = temp2;

         double detSwapped = Matrix3DFeatures.determinant(row0[0], row0[1], row0[2], row1[0], row1[1], row1[2], row2[0], row2[1], row2[2]);
         assertEquals(detSwapped, -det, EPS);
      }

      // Check that scaling a row scales the determinant
      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] column0 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] column1 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] column2 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};

         double det = Matrix3DFeatures.determinant(column0[0], column1[0], column2[0], column0[1], column1[1], column2[1], column0[2], column1[2], column2[2]);

         int rowScale = random.nextInt(3);
         double scale = EuclidCoreRandomTools.nextDouble(random, 5.0);

         column0[rowScale] *= scale;
         column1[rowScale] *= scale;
         column2[rowScale] *= scale;

         double detScaled = Matrix3DFeatures.determinant(column0[0],
                                                         column1[0],
                                                         column2[0],
                                                         column0[1],
                                                         column1[1],
                                                         column2[1],
                                                         column0[2],
                                                         column1[2],
                                                         column2[2]);
         assertEquals(detScaled, scale * det, EPS);
      }

      // Check that scaling a column scales the determinant
      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] row0 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] row1 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] row2 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};

         double det = Matrix3DFeatures.determinant(row0[0], row0[1], row0[2], row1[0], row1[1], row1[2], row2[0], row2[1], row2[2]);

         int columnScale = random.nextInt(3);
         double scale = EuclidCoreRandomTools.nextDouble(random, 5.0);

         row0[columnScale] *= scale;
         row1[columnScale] *= scale;
         row2[columnScale] *= scale;

         double detScaled = Matrix3DFeatures.determinant(row0[0], row0[1], row0[2], row1[0], row1[1], row1[2], row2[0], row2[1], row2[2]);
         assertEquals(detScaled, scale * det, EPS);
      }

      // Check that det(M) == 0 when M has two equal rows
      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] column0 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] column1 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] column2 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};

         int rowCopyDest = random.nextInt(3);
         int rowCopySrc = (rowCopyDest + 1 + random.nextInt(2)) % 3;

         column0[rowCopyDest] = column0[rowCopySrc];
         column1[rowCopyDest] = column1[rowCopySrc];
         column2[rowCopyDest] = column2[rowCopySrc];

         double det = Matrix3DFeatures.determinant(column0[0], column1[0], column2[0], column0[1], column1[1], column2[1], column0[2], column1[2], column2[2]);
         assertEquals(0.0, det, EPS);
      }

      // Check that det(M) == 0 when M has two equal columns
      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] row0 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] row1 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] row2 = {random.nextDouble(), random.nextDouble(), random.nextDouble()};

         int columnCopyDest = random.nextInt(3);
         int columnCopySrc = (columnCopyDest + 1 + random.nextInt(2)) % 3;

         row0[columnCopyDest] = row0[columnCopySrc];
         row1[columnCopyDest] = row1[columnCopySrc];
         row2[columnCopyDest] = row2[columnCopySrc];

         double det = Matrix3DFeatures.determinant(row0[0], row0[1], row0[2], row1[0], row1[1], row1[2], row2[0], row2[1], row2[2]);
         assertEquals(0.0, det, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Finally test against EJML
         Matrix3D matrix = EuclidCoreRandomTools.nextMatrix3D(random);
         matrix.get(denseMatrix);

         assertEquals(CommonOps_DDRM.det(denseMatrix), matrix.determinant(), EPS);

         double m00 = matrix.getM00();
         double m01 = matrix.getM01();
         double m02 = matrix.getM02();
         double m10 = matrix.getM10();
         double m11 = matrix.getM11();
         double m12 = matrix.getM12();
         double m20 = matrix.getM20();
         double m21 = matrix.getM21();
         double m22 = matrix.getM22();
         assertEquals(CommonOps_DDRM.det(denseMatrix), Matrix3DFeatures.determinant(m00, m01, m02, m10, m11, m12, m20, m21, m22), EPS);
      }
   }

   @Test
   public void testIsIdentity() throws Exception
   {
      Random random = new Random(982364L);
      Matrix3D matrix = new Matrix3D();
      // Test with a zero matrix
      matrix.setToZero();
      testAllIsIdentityMethods(matrix, false);
      // Test with a NaN matrix
      matrix.setToNaN();
      testAllIsIdentityMethods(matrix, false);

      // Test with identity
      matrix.setIdentity();
      testAllIsIdentityMethods(matrix, true);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double delta = random.nextBoolean() ? 1.0 : -1.0;
         delta *= random.nextDouble();
         matrix.setIdentity();
         matrix.setM00(matrix.getM00() + delta);
         testAllIsIdentityMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setIdentity();
         matrix.setM01(matrix.getM01() + delta);
         testAllIsIdentityMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setIdentity();
         matrix.setM02(matrix.getM02() + delta);
         testAllIsIdentityMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setIdentity();
         matrix.setM10(matrix.getM10() + delta);
         testAllIsIdentityMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setIdentity();
         matrix.setM11(matrix.getM11() + delta);
         testAllIsIdentityMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setIdentity();
         matrix.setM12(matrix.getM12() + delta);
         testAllIsIdentityMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setIdentity();
         matrix.setM20(matrix.getM20() + delta);
         testAllIsIdentityMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setIdentity();
         matrix.setM21(matrix.getM21() + delta);
         testAllIsIdentityMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setIdentity();
         matrix.setM22(matrix.getM22() + delta);
         testAllIsIdentityMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
      }
   }

   private void testAllIsIdentityMethods(Matrix3DReadOnly matrix, boolean isIdentity)
   {
      Matrix3D matrixCopy = new Matrix3D(matrix);

      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();

      assertTrue(Matrix3DFeatures.isIdentity(m00, m01, m02, m10, m11, m12, m20, m21, m22) == isIdentity);
      assertTrue(Matrix3DFeatures.isIdentity(m00, m01, m02, m10, m11, m12, m20, m21, m22, Matrix3DFeatures.EPS_CHECK_IDENTITY) == isIdentity);

      assertTrue(matrix.isIdentity(Matrix3DFeatures.EPS_CHECK_IDENTITY) == isIdentity);

      for (int row = 0; row < 3; row++)
         for (int column = 0; column < 3; column++)
            assertTrue(Double.compare(matrix.getElement(row, column), matrixCopy.getElement(row, column)) == 0);
   }

   @Test
   public void testIsZero() throws Exception
   {
      Random random = new Random(982364L);
      Matrix3D matrix = new Matrix3D();
      // Test with a identity matrix
      matrix.setIdentity();
      testAllIsZeroMethods(matrix, false);
      // Test with a NaN matrix
      matrix.setToNaN();
      testAllIsZeroMethods(matrix, false);

      // Test with zero
      matrix.setToZero();
      testAllIsZeroMethods(matrix, true);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double delta = random.nextBoolean() ? 1.0 : -1.0;
         delta *= random.nextDouble();
         matrix.setToZero();
         matrix.setM00(matrix.getM00() + delta);
         testAllIsZeroMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setToZero();
         matrix.setM01(matrix.getM01() + delta);
         testAllIsZeroMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setToZero();
         matrix.setM02(matrix.getM02() + delta);
         testAllIsZeroMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setToZero();
         matrix.setM10(matrix.getM10() + delta);
         testAllIsZeroMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setToZero();
         matrix.setM11(matrix.getM11() + delta);
         testAllIsZeroMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setToZero();
         matrix.setM12(matrix.getM12() + delta);
         testAllIsZeroMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setToZero();
         matrix.setM20(matrix.getM20() + delta);
         testAllIsZeroMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setToZero();
         matrix.setM21(matrix.getM21() + delta);
         testAllIsZeroMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
         matrix.setToZero();
         matrix.setM22(matrix.getM22() + delta);
         testAllIsZeroMethods(matrix, Math.abs(delta) <= Matrix3DFeatures.EPS_CHECK_IDENTITY);
      }
   }

   private void testAllIsZeroMethods(Matrix3DReadOnly matrix, boolean isZero)
   {
      Matrix3D matrixCopy = new Matrix3D(matrix);

      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();

      assertTrue(Matrix3DFeatures.isZero(m00, m01, m02, m10, m11, m12, m20, m21, m22) == isZero);
      assertTrue(Matrix3DFeatures.isZero(m00, m01, m02, m10, m11, m12, m20, m21, m22, Matrix3DFeatures.EPS_CHECK_IDENTITY) == isZero);

      assertTrue(matrix.isZero(Matrix3DFeatures.EPS_CHECK_IDENTITY) == isZero);

      for (int row = 0; row < 3; row++)
         for (int column = 0; column < 3; column++)
            assertTrue(Double.compare(matrix.getElement(row, column), matrixCopy.getElement(row, column)) == 0);
   }

   @Test
   public void testIsMatrixSkewSymmetric() throws Exception
   {
      Random random = new Random(982364L);
      Matrix3D matrix = new Matrix3D();
      Matrix3D matrixCorrupted = new Matrix3D();
      // Test with a zero matrix
      matrix.setToZero();
      testAllIsMatrixSkewSymmetrixMethods(matrix, true);

      // Test with identity
      matrix.setIdentity();
      testAllIsMatrixSkewSymmetrixMethods(matrix, false);

      // Test with NaN
      matrix.setToNaN();
      testAllIsMatrixSkewSymmetrixMethods(matrix, false);

      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix.setToZero();
         matrix.setM01(EuclidCoreRandomTools.nextDouble(random));
         matrix.setM02(EuclidCoreRandomTools.nextDouble(random));
         matrix.setM12(EuclidCoreRandomTools.nextDouble(random));
         matrix.setM10(-matrix.getM01());
         matrix.setM20(-matrix.getM02());
         matrix.setM21(-matrix.getM12());

         testAllIsMatrixSkewSymmetrixMethods(matrix, true);

         double delta = EuclidCoreRandomTools.nextDouble(random, 10.0 * Matrix3DFeatures.EPS_CHECK_SKEW);
         boolean isSkewSymmetric = Math.abs(delta) < Matrix3DFeatures.EPS_CHECK_SKEW;

         matrixCorrupted.set(matrix);
         matrixCorrupted.setM00(matrixCorrupted.getM00() + delta);
         testAllIsMatrixSkewSymmetrixMethods(matrixCorrupted, isSkewSymmetric);

         matrixCorrupted.set(matrix);
         matrixCorrupted.setM01(matrixCorrupted.getM01() + delta);
         testAllIsMatrixSkewSymmetrixMethods(matrixCorrupted, isSkewSymmetric);

         matrixCorrupted.set(matrix);
         matrixCorrupted.setM02(matrixCorrupted.getM02() + delta);
         testAllIsMatrixSkewSymmetrixMethods(matrixCorrupted, isSkewSymmetric);

         matrixCorrupted.set(matrix);
         matrixCorrupted.setM10(matrixCorrupted.getM10() + delta);
         testAllIsMatrixSkewSymmetrixMethods(matrixCorrupted, isSkewSymmetric);

         matrixCorrupted.set(matrix);
         matrixCorrupted.setM11(matrixCorrupted.getM11() + delta);
         testAllIsMatrixSkewSymmetrixMethods(matrixCorrupted, isSkewSymmetric);

         matrixCorrupted.set(matrix);
         matrixCorrupted.setM12(matrixCorrupted.getM12() + delta);
         testAllIsMatrixSkewSymmetrixMethods(matrixCorrupted, isSkewSymmetric);

         matrixCorrupted.set(matrix);
         matrixCorrupted.setM20(matrixCorrupted.getM20() + delta);
         testAllIsMatrixSkewSymmetrixMethods(matrixCorrupted, isSkewSymmetric);

         matrixCorrupted.set(matrix);
         matrixCorrupted.setM21(matrixCorrupted.getM21() + delta);
         testAllIsMatrixSkewSymmetrixMethods(matrixCorrupted, isSkewSymmetric);

         matrixCorrupted.set(matrix);
         matrixCorrupted.setM22(matrixCorrupted.getM22() + delta);
         testAllIsMatrixSkewSymmetrixMethods(matrixCorrupted, isSkewSymmetric);
      }
   }

   private void testAllIsMatrixSkewSymmetrixMethods(Matrix3DReadOnly matrix, boolean isSkewSymmetric)
   {
      Matrix3D matrixCopy = new Matrix3D(matrix);

      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();

      assertTrue(Matrix3DFeatures.isMatrixSkewSymmetric(m00, m01, m02, m10, m11, m12, m20, m21, m22) == isSkewSymmetric);
      assertTrue(Matrix3DFeatures.isMatrixSkewSymmetric(m00, m01, m02, m10, m11, m12, m20, m21, m22, Matrix3DFeatures.EPS_CHECK_SKEW) == isSkewSymmetric);

      assertTrue(matrix.isMatrixSkewSymmetric(Matrix3DFeatures.EPS_CHECK_SKEW) == isSkewSymmetric);

      for (int row = 0; row < 3; row++)
         for (int column = 0; column < 3; column++)
            assertTrue(Double.compare(matrix.getElement(row, column), matrixCopy.getElement(row, column)) == 0);
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(346L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D delta = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random, 10.0);
         double epsilon = random.nextDouble();

         delta.fill(0.1 * epsilon);

         m2.add(m1, delta);
         assertTrue(Matrix3DFeatures.epsilonEquals(m1, m2, epsilon));

         m2.sub(m1, delta);
         assertTrue(Matrix3DFeatures.epsilonEquals(m1, m2, epsilon));

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               delta.fill(0.1 * epsilon);
               delta.setElement(row, column, 1.1 * epsilon);
               m2.add(m1, delta);
               assertFalse(Matrix3DFeatures.epsilonEquals(m1, m2, epsilon));

               m2.sub(m1, delta);
               assertFalse(Matrix3DFeatures.epsilonEquals(m1, m2, epsilon));
            }
         }
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(346L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      double smallestEpsilon = 1.0e-15;

      // Test that it handles null pointers.
      assertFalse(Matrix3DFeatures.equals(m1, null));
      assertFalse(Matrix3DFeatures.equals(null, m2));
      assertFalse(Matrix3DFeatures.equals(new RotationMatrix(), m2));

      for (int i = 0; i < ITERATIONS; i++)
      {
         m1 = EuclidCoreRandomTools.nextMatrix3D(random, 10.0);

         m2.set(m1);
         assertTrue(Matrix3DFeatures.equals(m1, m2));

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               m2.set(m1);
               m2.setElement(row, column, m2.getElement(row, column) + smallestEpsilon);
               assertFalse(Matrix3DFeatures.equals(m1, m2));

               m2.set(m1);
               m2.setElement(row, column, m2.getElement(row, column) - smallestEpsilon);
               assertFalse(Matrix3DFeatures.equals(m1, m2));
            }
         }
      }
   }
}
