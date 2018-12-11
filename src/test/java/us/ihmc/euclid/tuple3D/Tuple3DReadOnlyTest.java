package us.ihmc.euclid.tuple3D;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class Tuple3DReadOnlyTest<T extends Tuple3DReadOnly>
{
   public abstract T createEmptyTuple();

   public abstract T createTuple(double x, double y, double z);

   public abstract T createRandomTuple(Random random);

   public abstract double getEpsilon();

   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(621541L);
      T tuple = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getX(), getY()
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple = createTuple(x, y, z);

         assertEquals(tuple.getX(), x, getEpsilon());
         assertEquals(tuple.getY(), y, getEpsilon());
         assertEquals(tuple.getZ(), z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getX32(), getY32()
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         tuple = createTuple(x, y, z);

         assertEquals(tuple.getX32(), x, getEpsilon());
         assertEquals(tuple.getY32(), y, getEpsilon());
         assertEquals(tuple.getZ32(), z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int index)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple = createTuple(x, y, z);

         assertEquals(tuple.getElement(0), x, getEpsilon());
         assertEquals(tuple.getElement(1), y, getEpsilon());
         assertEquals(tuple.getElement(2), z, getEpsilon());

         try
         {
            tuple.getElement(-1);
            fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown IndexOutOfBoundsException.");
         }

         try
         {
            tuple.getElement(3);
            fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown IndexOutOfBoundsException.");
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get32(int index)
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         tuple = createTuple(x, y, z);

         assertTrue(tuple.getElement32(0) == x);
         assertTrue(tuple.getElement32(1) == y);
         assertTrue(tuple.getElement32(2) == z);

         try
         {
            tuple.getElement32(-1);
            fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown IndexOutOfBoundsException.");
         }

         try
         {
            tuple.getElement32(3);
            fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown IndexOutOfBoundsException.");
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(double[] tupleArrayToPack)
         tuple = createRandomTuple(random);
         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         tuple.get(tupleArray);
         assertTrue(tuple.getX() == tupleArray[0]);
         assertTrue(tuple.getY() == tupleArray[1]);
         assertTrue(tuple.getZ() == tupleArray[2]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(double[] tupleArrayToPack, int startIndex)
         tuple = createRandomTuple(random);
         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         tuple.get(2, tupleArray);
         assertTrue(tuple.getX() == tupleArray[2]);
         assertTrue(tuple.getY() == tupleArray[3]);
         assertTrue(tuple.getZ() == tupleArray[4]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(float[] tupleArrayToPack)
         tuple = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(),
               random.nextFloat()};
         tuple.get(tupleArray);
         assertTrue(tuple.getX32() == tupleArray[0]);
         assertTrue(tuple.getY32() == tupleArray[1]);
         assertTrue(tuple.getZ32() == tupleArray[2]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(double[] tupleArrayToPack, int startIndex)
         tuple = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(),
               random.nextFloat()};
         tuple.get(2, tupleArray);
         assertTrue(tuple.getX32() == tupleArray[2]);
         assertTrue(tuple.getY32() == tupleArray[3]);
         assertTrue(tuple.getZ32() == tupleArray[4]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(matrix);
         assertTrue(tuple.getX() == matrix.get(0, 0));
         assertTrue(tuple.getY() == matrix.get(1, 0));
         assertTrue(tuple.getZ() == matrix.get(2, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startRow, DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(2, matrix);
         assertTrue(tuple.getX() == matrix.get(2, 0));
         assertTrue(tuple.getY() == matrix.get(3, 0));
         assertTrue(tuple.getZ() == matrix.get(4, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startRow, int startColumn, DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(2, 4, matrix);
         assertTrue(tuple.getX() == matrix.get(2, 4));
         assertTrue(tuple.getY() == matrix.get(3, 4));
         assertTrue(tuple.getZ() == matrix.get(4, 4));
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      assertFalse(createTuple(0.0, 0.0, 0.0).containsNaN());
      assertTrue(createTuple(Double.NaN, 0.0, 0.0).containsNaN());
      assertTrue(createTuple(0.0, Double.NaN, 0.0).containsNaN());
      assertTrue(createTuple(0.0, 0.0, Double.NaN).containsNaN());
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();

      T tuple = createRandomTuple(random);
      double x = tuple.getX();
      double y = tuple.getY();
      double z = tuple.getZ();

      assertTrue(tuple.epsilonEquals(createTuple(x + 0.999 * epsilon, y, z), epsilon));
      assertTrue(tuple.epsilonEquals(createTuple(x - 0.999 * epsilon, y, z), epsilon));

      assertTrue(tuple.epsilonEquals(createTuple(x, y + 0.999 * epsilon, z), epsilon));
      assertTrue(tuple.epsilonEquals(createTuple(x, y - 0.999 * epsilon, z), epsilon));

      assertTrue(tuple.epsilonEquals(createTuple(x, y, z + 0.999 * epsilon), epsilon));
      assertTrue(tuple.epsilonEquals(createTuple(x, y, z - 0.999 * epsilon), epsilon));

      assertFalse(tuple.epsilonEquals(createTuple(x + 1.001 * epsilon, y, z), epsilon));
      assertFalse(tuple.epsilonEquals(createTuple(x - 1.001 * epsilon, y, z), epsilon));

      assertFalse(tuple.epsilonEquals(createTuple(x, y + 1.001 * epsilon, z), epsilon));
      assertFalse(tuple.epsilonEquals(createTuple(x, y - 1.001 * epsilon, z), epsilon));

      assertFalse(tuple.epsilonEquals(createTuple(x, y, z + 1.001 * epsilon), epsilon));
      assertFalse(tuple.epsilonEquals(createTuple(x, y, z - 1.001 * epsilon), epsilon));
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

      double x = tuple.getX();
      double y = tuple.getY();
      double z = tuple.getZ();

      assertTrue(tuple.equals(createTuple(x, y, z)));

      assertFalse(tuple.equals(createTuple(x + getEpsilon(), y, z)));
      assertFalse(tuple.equals(createTuple(x - getEpsilon(), y, z)));

      assertFalse(tuple.equals(createTuple(x, y + getEpsilon(), z)));
      assertFalse(tuple.equals(createTuple(x, y - getEpsilon(), z)));

      assertFalse(tuple.equals(createTuple(x, y, z + getEpsilon())));
      assertFalse(tuple.equals(createTuple(x, y, z - getEpsilon())));
   }
}