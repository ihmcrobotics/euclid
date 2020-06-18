package us.ihmc.euclid.tuple4D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

public abstract class Tuple4DReadOnlyTest<T extends Tuple4DReadOnly>
{
   public abstract T createEmptyTuple();

   public abstract T createTuple(double x, double y, double z, double s);

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
         double s = random.nextDouble();
         tuple = createTuple(x, y, z, s);

         assertEquals(tuple.getX(), x, getEpsilon());
         assertEquals(tuple.getY(), y, getEpsilon());
         assertEquals(tuple.getZ(), z, getEpsilon());
         assertEquals(tuple.getS(), s, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getX32(), getY32()
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         float s = random.nextFloat();
         tuple = createTuple(x, y, z, s);

         assertEquals(tuple.getX32(), x, getEpsilon());
         assertEquals(tuple.getY32(), y, getEpsilon());
         assertEquals(tuple.getZ32(), z, getEpsilon());
         assertEquals(tuple.getS32(), s, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int index)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         tuple = createTuple(x, y, z, s);

         assertEquals(tuple.getElement(0), x, getEpsilon());
         assertEquals(tuple.getElement(1), y, getEpsilon());
         assertEquals(tuple.getElement(2), z, getEpsilon());
         assertEquals(tuple.getElement(3), s, getEpsilon());

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
            tuple.getElement(4);
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
         float s = random.nextFloat();
         tuple = createTuple(x, y, z, s);

         assertTrue(tuple.getElement32(0) == x);
         assertTrue(tuple.getElement32(1) == y);
         assertTrue(tuple.getElement32(2) == z);
         assertTrue(tuple.getElement32(3) == s);

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
            tuple.getElement32(4);
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
         assertTrue(tuple.getS() == tupleArray[3]);
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
         assertTrue(tuple.getS() == tupleArray[5]);
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
         assertTrue(tuple.getS32() == tupleArray[3]);
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
         assertTrue(tuple.getS32() == tupleArray[5]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(DMatrix tupleMatrixToPack)
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(matrix);
         assertTrue(tuple.getX() == matrix.get(0, 0));
         assertTrue(tuple.getY() == matrix.get(1, 0));
         assertTrue(tuple.getZ() == matrix.get(2, 0));
         assertTrue(tuple.getS() == matrix.get(3, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startRow, DMatrix tupleMatrixToPack)
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(2, matrix);
         assertTrue(tuple.getX() == matrix.get(2, 0));
         assertTrue(tuple.getY() == matrix.get(3, 0));
         assertTrue(tuple.getZ() == matrix.get(4, 0));
         assertTrue(tuple.getS() == matrix.get(5, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startRow, int startColumn, DMatrix tupleMatrixToPack)
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(2, 4, matrix);
         assertTrue(tuple.getX() == matrix.get(2, 4));
         assertTrue(tuple.getY() == matrix.get(3, 4));
         assertTrue(tuple.getZ() == matrix.get(4, 4));
         assertTrue(tuple.getS() == matrix.get(5, 4));
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      assertFalse(createTuple(0.0, 0.0, 0.0, 0.0).containsNaN());
      assertTrue(createTuple(Double.NaN, 0.0, 0.0, 0.0).containsNaN());
      assertTrue(createTuple(0.0, Double.NaN, 0.0, 0.0).containsNaN());
      assertTrue(createTuple(0.0, 0.0, Double.NaN, 0.0).containsNaN());
      assertTrue(createTuple(0.0, 0.0, 0.0, Double.NaN).containsNaN());
   }

   @Test
   public void testLength()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple1 = createRandomTuple(random);
         double length1 = tuple1.norm();
         double scalar = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         T tuple2 = createTuple(scalar * tuple1.getX(), scalar * tuple1.getY(), scalar * tuple1.getZ(), scalar * tuple1.getS());
         double expectedLength2 = scalar * length1;
         double actualLength2 = tuple2.norm();
         assertEquals(expectedLength2, actualLength2, 5.0 * getEpsilon());
      }
   }

   @Test
   public void testLengthSquared()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple1 = createRandomTuple(random);
         double length1 = tuple1.norm();
         double scalar = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         T tuple2 = createTuple(scalar * tuple1.getX(), scalar * tuple1.getY(), scalar * tuple1.getZ(), scalar * tuple1.getS());
         double expectedLength2 = scalar * length1;
         double actualLength2 = tuple2.normSquared();
         assertEquals(expectedLength2, EuclidCoreTools.squareRoot(actualLength2), 5.0 * getEpsilon());
      }
   }

   /**
    * Using the property from <a href="https://en.wikipedia.org/wiki/Quaternion"> Wikipedia (section
    * Quaternion and the Geometry of R<sup>3</sup>)</a>.
    * <p>
    * p . q = s(p * q^-1) = s(q * p^-1) = s(p^-1 * q) = s(q^-1 * p) <br>
    * where s(q) is the function returning the scalar part of the quaternion q.
    * </p>
    */
   @Test
   public void testDot()
   {
      Random random = new Random(5461L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T p = createRandomTuple(random);
         T q = createRandomTuple(random);
         Vector4D product = new Vector4D();

         double expectedDot;
         double actualDot = p.dot(q);

         QuaternionTools.multiplyConjugateRight(p, q, product);
         expectedDot = product.getS();
         assertEquals(expectedDot, actualDot, getEpsilon());

         QuaternionTools.multiplyConjugateRight(q, p, product);
         expectedDot = product.getS();
         assertEquals(expectedDot, actualDot, getEpsilon());

         QuaternionTools.multiplyConjugateLeft(p, q, product);
         expectedDot = product.getS();
         assertEquals(expectedDot, actualDot, getEpsilon());

         QuaternionTools.multiplyConjugateLeft(q, p, product);
         expectedDot = product.getS();
         assertEquals(expectedDot, actualDot, getEpsilon());
      }
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
      double s = tuple.getS();

      assertTrue(tuple.epsilonEquals(createTuple(x + 0.999 * epsilon, y, z, s), epsilon));
      assertTrue(tuple.epsilonEquals(createTuple(x - 0.999 * epsilon, y, z, s), epsilon));

      assertTrue(tuple.epsilonEquals(createTuple(x, y + 0.999 * epsilon, z, s), epsilon));
      assertTrue(tuple.epsilonEquals(createTuple(x, y - 0.999 * epsilon, z, s), epsilon));

      assertTrue(tuple.epsilonEquals(createTuple(x, y, z + 0.999 * epsilon, s), epsilon));
      assertTrue(tuple.epsilonEquals(createTuple(x, y, z - 0.999 * epsilon, s), epsilon));

      assertTrue(tuple.epsilonEquals(createTuple(x, y, z, s + 0.999 * epsilon), epsilon));
      assertTrue(tuple.epsilonEquals(createTuple(x, y, z, s - 0.999 * epsilon), epsilon));

      assertFalse(tuple.epsilonEquals(createTuple(x + 1.001 * epsilon, y, z, s), epsilon));
      assertFalse(tuple.epsilonEquals(createTuple(x - 1.001 * epsilon, y, z, s), epsilon));

      assertFalse(tuple.epsilonEquals(createTuple(x, y + 1.001 * epsilon, z, s), epsilon));
      assertFalse(tuple.epsilonEquals(createTuple(x, y - 1.001 * epsilon, z, s), epsilon));

      assertFalse(tuple.epsilonEquals(createTuple(x, y, z + 1.001 * epsilon, s), epsilon));
      assertFalse(tuple.epsilonEquals(createTuple(x, y, z - 1.001 * epsilon, s), epsilon));

      assertFalse(tuple.epsilonEquals(createTuple(x, y, z, s + 1.001 * epsilon), epsilon));
      assertFalse(tuple.epsilonEquals(createTuple(x, y, z, s - 1.001 * epsilon), epsilon));
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
      double s = tuple.getS();

      assertTrue(tuple.equals(createTuple(x, y, z, s)));

      assertFalse(tuple.equals(createTuple(x + getEpsilon(), y, z, s)));
      assertFalse(tuple.equals(createTuple(x - getEpsilon(), y, z, s)));

      assertFalse(tuple.equals(createTuple(x, y + getEpsilon(), z, s)));
      assertFalse(tuple.equals(createTuple(x, y - getEpsilon(), z, s)));

      assertFalse(tuple.equals(createTuple(x, y, z + getEpsilon(), s)));
      assertFalse(tuple.equals(createTuple(x, y, z - getEpsilon(), s)));

      assertFalse(tuple.equals(createTuple(x, y, z, s + getEpsilon())));
      assertFalse(tuple.equals(createTuple(x, y, z, s - getEpsilon())));
   }
}