package us.ihmc.euclid.matrix;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

public abstract class CommonMatrix3DBasicsTest<T extends CommonMatrix3DBasics> extends Matrix3DReadOnlyTest<T>
{
   @Test
   public void testSetDoubles()
   {
      Random random = new Random(345634L);
      T expectedMatrix = createRandomMatrix(random);
      double m00 = expectedMatrix.getM00();
      double m01 = expectedMatrix.getM01();
      double m02 = expectedMatrix.getM02();
      double m10 = expectedMatrix.getM10();
      double m11 = expectedMatrix.getM11();
      double m12 = expectedMatrix.getM12();
      double m20 = expectedMatrix.getM20();
      double m21 = expectedMatrix.getM21();
      double m22 = expectedMatrix.getM22();

      T actualMatrix = createEmptyMatrix();

      actualMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      T matrix = createEmptyMatrix();
      for (int row = 0; row < 3; row++)
         for (int col = 0; col < 3; col++)
            assertFalse(Double.isNaN(matrix.getElement(row, col)));
      matrix.setToNaN();
      for (int row = 0; row < 3; row++)
         for (int col = 0; col < 3; col++)
            assertTrue(Double.isNaN(matrix.getElement(row, col)));
   }

   @Test
   public void testSetIdentity() throws Exception
   {
      Random random = new Random(2342L);
      T matrix = createRandomMatrix(random);
      assertFalse(matrix.isIdentity());
      matrix.setIdentity();
      assertTrue(matrix.isIdentity());
   }

   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Matrix3DReadOnly other)
         RotationMatrix expectedMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         T actualMatrix = createEmptyMatrix();
         actualMatrix.set(expectedMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(double[] matrixArray)
         RotationMatrix expectedMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         double[] array = new double[40];
         expectedMatrix.get(array);
         T actualMatrix = createEmptyMatrix();
         actualMatrix.set(array);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(int startIndex, double[] matrixArray)
         RotationMatrix expectedMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         double[] array = new double[40];
         int startIndex = random.nextInt(array.length - 9);
         expectedMatrix.get(startIndex, array);
         T actualMatrix = createEmptyMatrix();
         actualMatrix.set(startIndex, array);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(DenseMatrix64F matrix)
         RotationMatrix expectedMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(30, 30);
         expectedMatrix.get(denseMatrix);
         T actualMatrix = createEmptyMatrix();
         actualMatrix.set(denseMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(int startRow, int startColumn, DenseMatrix64F matrix)
         RotationMatrix expectedMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(30, 30);
         int startRow = random.nextInt(denseMatrix.getNumRows() - 3);
         int startColumn = random.nextInt(denseMatrix.getNumCols() - 3);
         expectedMatrix.get(startRow, startColumn, denseMatrix);
         T actualMatrix = createEmptyMatrix();
         actualMatrix.set(startRow, startColumn, denseMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }
   }
}