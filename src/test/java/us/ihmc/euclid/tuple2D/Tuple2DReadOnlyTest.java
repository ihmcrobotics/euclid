package us.ihmc.euclid.tuple2D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public abstract class Tuple2DReadOnlyTest<T extends Tuple2DReadOnly>
{
   public abstract T createEmptyTuple();

   public abstract T createTuple(double x, double y);

   public abstract T createRandomTuple(Random random);

   public abstract double getEpsilon();

   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.getX(), Tuple2DReadOnly.getY()
         double x = random.nextDouble();
         double y = random.nextDouble();
         T tuple = createTuple(x, y);

         assertEquals(tuple.getX(), x, getEpsilon());
         assertEquals(tuple.getY(), y, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.getX32(), Tuple2DReadOnly.getY32()
         float x = random.nextFloat();
         float y = random.nextFloat();
         T tuple = createTuple(x, y);

         assertTrue(tuple.getX32() == x);
         assertTrue(tuple.getY32() == y);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.getElement(int index)
         double x = random.nextDouble();
         double y = random.nextDouble();
         T tuple = createTuple(x, y);

         assertEquals(tuple.getElement(0), x, getEpsilon());
         assertEquals(tuple.getElement(1), y, getEpsilon());

         assertThrows(IndexOutOfBoundsException.class, () -> tuple.getElement(-1));
         assertThrows(IndexOutOfBoundsException.class, () -> tuple.getElement(2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.getElement(Axis2D)
         double x = random.nextDouble();
         double y = random.nextDouble();
         T tuple = createTuple(x, y);

         assertEquals(tuple.getElement(Axis2D.X), x, getEpsilon());
         assertEquals(tuple.getElement(Axis2D.Y), y, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.getElement32(int index)
         float x = random.nextFloat();
         float y = random.nextFloat();
         T tuple = createTuple(x, y);

         assertTrue(tuple.getElement32(0) == x);
         assertTrue(tuple.getElement32(1) == y);

         assertThrows(IndexOutOfBoundsException.class, () -> tuple.getElement32(-1));
         assertThrows(IndexOutOfBoundsException.class, () -> tuple.getElement32(2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.get(double[] tupleArrayToPack)
         T tuple = createRandomTuple(random);
         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         tuple.get(tupleArray);
         assertTrue(tuple.getX() == tupleArray[0]);
         assertTrue(tuple.getY() == tupleArray[1]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.get(double[] tupleArrayToPack, int startIndex)
         T tuple = createRandomTuple(random);
         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         tuple.get(2, tupleArray);
         assertTrue(tuple.getX() == tupleArray[2]);
         assertTrue(tuple.getY() == tupleArray[3]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.get(float[] tupleArrayToPack)
         T tuple = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(),
               random.nextFloat()};
         tuple.get(tupleArray);
         assertTrue(tuple.getX32() == tupleArray[0]);
         assertTrue(tuple.getY32() == tupleArray[1]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.get(double[] tupleArrayToPack, int startIndex)
         T tuple = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(),
               random.nextFloat()};
         tuple.get(2, tupleArray);
         assertTrue(tuple.getX32() == tupleArray[2]);
         assertTrue(tuple.getY32() == tupleArray[3]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.get(DenseMatrix64F tupleMatrixToPack)
         T tuple = createRandomTuple(random);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(matrix);
         assertTrue(tuple.getX() == matrix.get(0, 0));
         assertTrue(tuple.getY() == matrix.get(1, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.get(int startRow, DenseMatrix64F tupleMatrixToPack)
         T tuple = createRandomTuple(random);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(2, matrix);
         assertTrue(tuple.getX() == matrix.get(2, 0));
         assertTrue(tuple.getY() == matrix.get(3, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple2DReadOnly.get(int startRow, int startColumn, DenseMatrix64F tupleMatrixToPack)
         T tuple = createRandomTuple(random);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(2, 4, matrix);
         assertTrue(tuple.getX() == matrix.get(2, 4));
         assertTrue(tuple.getY() == matrix.get(3, 4));
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      assertFalse(createTuple(0.0, 0.0).containsNaN());
      assertTrue(createTuple(Double.NaN, 0.0).containsNaN());
      assertTrue(createTuple(0.0, Double.NaN).containsNaN());
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();

      T tuple = createRandomTuple(random);
      double x = tuple.getX();
      double y = tuple.getY();

      assertTrue(tuple.epsilonEquals(createTuple(x + 0.999 * epsilon, y), epsilon));
      assertTrue(tuple.epsilonEquals(createTuple(x - 0.999 * epsilon, y), epsilon));

      assertTrue(tuple.epsilonEquals(createTuple(x, y + 0.999 * epsilon), epsilon));
      assertTrue(tuple.epsilonEquals(createTuple(x, y - 0.999 * epsilon), epsilon));

      assertFalse(tuple.epsilonEquals(createTuple(x + 1.001 * epsilon, y), epsilon));
      assertFalse(tuple.epsilonEquals(createTuple(x - 1.001 * epsilon, y), epsilon));

      assertFalse(tuple.epsilonEquals(createTuple(x, y + 1.001 * epsilon), epsilon));
      assertFalse(tuple.epsilonEquals(createTuple(x, y - 1.001 * epsilon), epsilon));
   }

   @Test
   public void testNorm()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple1 = createRandomTuple(random);
         double norm1 = tuple1.norm();
         double scalar = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         T tuple2 = createTuple(scalar * tuple1.getX(), scalar * tuple1.getY());
         double expectedLength2 = scalar * norm1;
         double actualLength2 = tuple2.norm();
         assertEquals(expectedLength2, actualLength2, 2.0 * getEpsilon());
      }
   }

   @Test
   public void testNormSquared()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple1 = createRandomTuple(random);
         double norm1 = tuple1.norm();
         double scalar = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         T tuple2 = createTuple(scalar * tuple1.getX(), scalar * tuple1.getY());
         double expectedNorm2 = scalar * norm1;
         double actualNorm2 = tuple2.normSquared();
         assertEquals(expectedNorm2, EuclidCoreTools.squareRoot(actualNorm2), 2.0 * getEpsilon());
      }
   }

   @Test
   public void testDot()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple1 = createRandomTuple(random);
         double angle = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double x = EuclidCoreTools.cos(angle) * tuple1.getX() - EuclidCoreTools.sin(angle) * tuple1.getY();
         double y = EuclidCoreTools.sin(angle) * tuple1.getX() + EuclidCoreTools.cos(angle) * tuple1.getY();
         double scalar = EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0);
         T tuple2 = createTuple(scalar * x, scalar * y);

         double expectedDot = tuple1.norm() * tuple2.norm() * EuclidCoreTools.cos(angle);
         double actualDot = tuple1.dot(tuple2);
         assertEquals(expectedDot, actualDot, getEpsilon());
      }
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);

      T tuple = createRandomTuple(random);

      assertFalse(tuple.equals(createEmptyTuple()));
      assertFalse(tuple.equals(null));
      assertFalse(tuple.equals(new double[5]));
      Object tupleAsObject = tuple;
      assertTrue(tuple.equals(tupleAsObject));

      double x = tuple.getX();
      double y = tuple.getY();

      assertTrue(tuple.equals(createTuple(x, y)));

      assertFalse(tuple.equals(createTuple(x + getEpsilon(), y)));
      assertFalse(tuple.equals(createTuple(x - getEpsilon(), y)));

      assertFalse(tuple.equals(createTuple(x, y + getEpsilon())));
      assertFalse(tuple.equals(createTuple(x, y - getEpsilon())));
   }
}