package us.ihmc.euclid.matrix;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

public class Matrix3DTest extends Matrix3DBasicsTest<Matrix3D>
{
   public static final double EPS = 1.0e-10;

   @Test
   public void testMatrix3D()
   {
      Random random = new Random(65431L);
      Matrix3D matrix = new Matrix3D(), matrixCopy;

      for (int k = 0; k < ITERATIONS; k++)
      { // Test Matrix3D(double[] matrixArray)
         double[] matrixArray = new double[9];
         double[] matrixArrayCopy = new double[9];

         for (int i = 0; i < matrixArray.length; i++)
            matrixArray[i] = matrixArrayCopy[i] = random.nextDouble();

         matrix = matrixCopy = new Matrix3D(matrixArray);
         Matrix3D expectedMatrix = new Matrix3D();
         expectedMatrix.set(matrixArray);

         for (int j = 0; j < matrixArray.length; j++)
            assertTrue(matrixArray[j] == matrixArrayCopy[j]);

         assertTrue(matrix.getM00() == matrixArray[0]);
         assertTrue(matrix.getM01() == matrixArray[1]);
         assertTrue(matrix.getM02() == matrixArray[2]);
         assertTrue(matrix.getM10() == matrixArray[3]);
         assertTrue(matrix.getM11() == matrixArray[4]);
         assertTrue(matrix.getM12() == matrixArray[5]);
         assertTrue(matrix.getM20() == matrixArray[6]);
         assertTrue(matrix.getM21() == matrixArray[7]);
         assertTrue(matrix.getM22() == matrixArray[8]);

         EuclidCoreTestTools.assertMatrix3DEquals(matrix, expectedMatrix, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Matrix3D(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         Matrix3D expectedMatrix = EuclidCoreRandomTools.nextMatrix3D(random);
         matrix = new Matrix3D(expectedMatrix.getM00(),
                               expectedMatrix.getM01(),
                               expectedMatrix.getM02(),
                               expectedMatrix.getM10(),
                               expectedMatrix.getM11(),
                               expectedMatrix.getM12(),
                               expectedMatrix.getM20(),
                               expectedMatrix.getM21(),
                               expectedMatrix.getM22());

         EuclidCoreTestTools.assertMatrix3DEquals(matrix, expectedMatrix, EPS);
      }

      for (int k = 0; k < ITERATIONS; k++)
      { // Test Matrix3D(Matrix3DBasics other)
         double[] matrixArray = new double[9];

         for (int i = 0; i < matrixArray.length; i++)
            matrixArray[i] = random.nextDouble();

         matrix = new Matrix3D(matrixArray);
         Matrix3D matrix2 = new Matrix3D(matrix);

         EuclidCoreTestTools.assertMatrix3DEquals(matrix, matrix2, EPS);

         assertTrue(matrix.getM00() == matrixArray[0]);
         assertTrue(matrix.getM01() == matrixArray[1]);
         assertTrue(matrix.getM02() == matrixArray[2]);
         assertTrue(matrix.getM10() == matrixArray[3]);
         assertTrue(matrix.getM11() == matrixArray[4]);
         assertTrue(matrix.getM12() == matrixArray[5]);
         assertTrue(matrix.getM20() == matrixArray[6]);
         assertTrue(matrix.getM21() == matrixArray[7]);
         assertTrue(matrix.getM22() == matrixArray[8]);
      }
   }

   @Test
   public void testSetToZero()
   {
      Random random = new Random(564L);
      Matrix3D matrix = EuclidCoreRandomTools.nextMatrix3D(random);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertFalse(matrix.getElement(row, column) == 0.0);
         }
      }

      matrix.setToZero();

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(matrix.getElement(row, column) == 0.0);
         }
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();
   
      Matrix3D matrix = createRandomMatrix(random);
      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();
   
      double small = 0.999 * epsilon;
      double big = 1.001 * epsilon;
   
      assertTrue(matrix.geometricallyEquals(createMatrix(m00 + small, m01, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01 + small, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02 + small, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10 + small, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11 + small, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12 + small, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20 + small, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21 + small, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22 + small), epsilon));
   
      assertTrue(matrix.geometricallyEquals(createMatrix(m00 - small, m01, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01 - small, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02 - small, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10 - small, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11 - small, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12 - small, m20, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20 - small, m21, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21 - small, m22), epsilon));
      assertTrue(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22 - small), epsilon));
   
      assertFalse(matrix.geometricallyEquals(createMatrix(m00 + big, m01, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01 + big, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02 + big, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10 + big, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11 + big, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12 + big, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20 + big, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21 + big, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22 + big), epsilon));
   
      assertFalse(matrix.geometricallyEquals(createMatrix(m00 - big, m01, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01 - big, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02 - big, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10 - big, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11 - big, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12 - big, m20, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20 - big, m21, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21 - big, m22), epsilon));
      assertFalse(matrix.geometricallyEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22 - big), epsilon));
   }

   @Override
   public Matrix3D createEmptyMatrix()
   {
      return new Matrix3D();
   }

   @Override
   public Matrix3D createMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return new Matrix3D(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   @Override
   public Matrix3D createRandomMatrix(Random random)
   {
      return EuclidCoreRandomTools.nextMatrix3D(random);
   }
}