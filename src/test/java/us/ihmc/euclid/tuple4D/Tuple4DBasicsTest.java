package us.ihmc.euclid.tuple4D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DBasics;

public abstract class Tuple4DBasicsTest<T extends Tuple4DBasics> extends Tuple4DReadOnlyTest<T>
{
   @Test
   public abstract void testSetDoubles();

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple = createTuple(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple.setToNaN();
         assertTrue(Double.isNaN(tuple.getX()));
         assertTrue(Double.isNaN(tuple.getY()));
         assertTrue(Double.isNaN(tuple.getZ()));
      }
   }

   @Test
   public void testNormalize()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test normalize()
         T tuple1 = createRandomTuple(random);
         tuple1.normalize();

         double expectedLength = 1.0;
         double actualLength = tuple1.norm();
         assertEquals(expectedLength, actualLength, getEpsilon());

         T tuple2 = createRandomTuple(random);
         tuple2.normalize();
         double scale = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         tuple1 = createTuple(scale * tuple2.getX(), scale * tuple2.getY(), scale * tuple2.getZ(), scale * tuple2.getS());
         tuple1.normalize();
         EuclidCoreTestTools.assertTuple4DEquals(tuple1, tuple2, getEpsilon());

         tuple1.setToNaN();
         tuple1.normalize();
         for (int index = 0; index < 4; index++)
            assertTrue(Double.isNaN(tuple1.getElement(index)));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setAndNormalize(Tuple4DReadOnly other)
         T tuple1 = createRandomTuple(random);
         T tuple2 = createEmptyTuple();

         tuple2.setAndNormalize(tuple1);

         double expectedLength = 1.0;
         double actualLength = tuple2.norm();
         assertEquals(expectedLength, actualLength, getEpsilon());

         tuple2 = createRandomTuple(random);
         tuple2.normalize();
         T tuple3 = createEmptyTuple();
         double scale = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         tuple3 = createTuple(scale * tuple2.getX(), scale * tuple2.getY(), scale * tuple2.getZ(), scale * tuple2.getS());
         tuple1.setAndNormalize(tuple3);
         EuclidCoreTestTools.assertTuple4DEquals(tuple1, tuple2, getEpsilon());

         tuple3.setToNaN();
         tuple1.setToZero();
         tuple1.setAndNormalize(tuple3);
         for (int index = 0; index < 4; index++)
            assertTrue(Double.isNaN(tuple1.getElement(index)));
      }
   }

   @Test
   public void testAbsolute() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();

      for (double signX = -1.0; signX <= 1.0; signX += 2.0)
      {
         for (double signY = -1.0; signY <= 1.0; signY += 2.0)
         {
            for (double signZ = -1.0; signZ <= 1.0; signZ += 2.0)
            {
               for (double signS = -1.0; signS <= 1.0; signS += 2.0)
               {
                  T original = createRandomTuple(random);
                  double xPos = Math.abs(original.getX());
                  double yPos = Math.abs(original.getY());
                  double zPos = Math.abs(original.getZ());
                  double sPos = Math.abs(original.getS());
                  tuple1 = createTuple(signX * xPos, signY * yPos, signZ * zPos, signS * sPos);

                  tuple2.setAndAbsolute(tuple1);
                  assertEquals(tuple2.getX(), xPos, getEpsilon());
                  assertEquals(tuple2.getY(), yPos, getEpsilon());
                  assertEquals(tuple2.getZ(), zPos, getEpsilon());
                  assertEquals(tuple2.getS(), sPos, getEpsilon());
                  assertEquals(tuple1.getX(), signX * xPos, getEpsilon());
                  assertEquals(tuple1.getY(), signY * yPos, getEpsilon());
                  assertEquals(tuple1.getZ(), signZ * zPos, getEpsilon());
                  assertEquals(tuple1.getS(), signS * sPos, getEpsilon());

                  tuple1.absolute();
                  assertEquals(tuple1.getX(), xPos, getEpsilon());
                  assertEquals(tuple1.getY(), yPos, getEpsilon());
                  assertEquals(tuple1.getZ(), zPos, getEpsilon());
                  assertEquals(tuple1.getS(), sPos, getEpsilon());
               }
            }
         }
      }
   }

   @Test
   public void testNegate() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();

      for (double signX = -1.0; signX <= 1.0; signX += 2.0)
      {
         for (double signY = -1.0; signY <= 1.0; signY += 2.0)
         {
            for (double signZ = -1.0; signZ <= 1.0; signZ += 2.0)
            {
               for (double signS = -1.0; signS <= 1.0; signS += 2.0)
               {
                  T original = createRandomTuple(random);
                  double xOriginal = signX * original.getX();
                  double yOriginal = signY * original.getY();
                  double zOriginal = signZ * original.getZ();
                  double sOriginal = signS * original.getS();
                  tuple1 = createTuple(xOriginal, yOriginal, zOriginal, sOriginal);

                  tuple2.setToNaN();
                  tuple2.setAndNegate(tuple1);
                  assertEquals(tuple2.getX(), -xOriginal, getEpsilon());
                  assertEquals(tuple2.getY(), -yOriginal, getEpsilon());
                  assertEquals(tuple2.getZ(), -zOriginal, getEpsilon());
                  assertEquals(tuple2.getS(), -sOriginal, getEpsilon());
                  assertEquals(tuple1.getX(), xOriginal, getEpsilon());
                  assertEquals(tuple1.getY(), yOriginal, getEpsilon());
                  assertEquals(tuple1.getZ(), zOriginal, getEpsilon());
                  assertEquals(tuple1.getS(), sOriginal, getEpsilon());

                  tuple1.negate();
                  assertEquals(tuple1.getX(), -xOriginal, getEpsilon());
                  assertEquals(tuple1.getY(), -yOriginal, getEpsilon());
                  assertEquals(tuple1.getZ(), -zOriginal, getEpsilon());
                  assertEquals(tuple1.getS(), -sOriginal, getEpsilon());
               }
            }
         }
      }
   }

   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(T other)
         tuple2 = createRandomTuple(random);
         tuple1.set(tuple2);
         EuclidCoreTestTools.assertTuple4DEquals(tuple1, tuple2, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(Tuple3DReadOnly tupleReadOnly)
         tuple2 = createRandomTuple(random);
         tuple1.set(tuple2);
         EuclidCoreTestTools.assertTuple4DEquals(tuple1, tuple2, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double[] tupleArray);
         double[] tupleArray = new double[10];
         tuple2 = createRandomTuple(random);
         tuple2.get(tupleArray);
         tuple1.set(tupleArray);
         EuclidCoreTestTools.assertTuple4DEquals(tuple1, tuple2, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double[] tupleArray, int startIndex);
         double[] tupleArray = new double[10];
         tuple2 = createRandomTuple(random);
         tuple2.get(2, tupleArray);
         tuple1.set(2, tupleArray);
         EuclidCoreTestTools.assertTuple4DEquals(tuple1, tuple2, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(float[] tupleArray);
         float[] tupleArray = new float[10];
         tuple2 = createRandomTuple(random);
         tuple2.get(tupleArray);
         tuple1.set(tupleArray);
         for (int index = 0; index < 4; index++)
            assertEquals(tuple2.getElement32(index), tuple1.getElement32(index), Math.max(getEpsilon(), 1.0e-6));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(float[] tupleArray, int startIndex);
         float[] tupleArray = new float[10];
         tuple2 = createRandomTuple(random);
         tuple2.get(2, tupleArray);
         tuple1.set(2, tupleArray);
         for (int index = 0; index < 4; index++)
            assertEquals(tuple2.getElement32(index), tuple1.getElement32(index), Math.max(getEpsilon(), 1.0e-6));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DMatrixRMaj matrix);
         DMatrixRMaj matrix = new DMatrixRMaj(5, 4);
         tuple2 = createRandomTuple(random);
         tuple2.get(matrix);
         tuple1.set(matrix);
         EuclidCoreTestTools.assertTuple4DEquals(tuple1, tuple2, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DMatrixRMaj matrix, int startRow);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 4);
         tuple2 = createRandomTuple(random);
         tuple2.get(5, matrix);
         tuple1.set(5, matrix);
         EuclidCoreTestTools.assertTuple4DEquals(tuple1, tuple2, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DMatrixRMaj matrix, int startRow, int column);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 4);
         tuple2 = createRandomTuple(random);
         tuple2.get(5, 2, matrix);
         tuple1.set(5, 2, matrix);
         EuclidCoreTestTools.assertTuple4DEquals(tuple1, tuple2, getEpsilon());
      }
   }
}