package us.ihmc.euclid.tuple3D;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public abstract class Tuple3DBasicsTest<T extends Tuple3DBasics> extends Tuple3DReadOnlyTest<T>
{
   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setX(double x)
         double x = random.nextDouble();
         tuple1.setX(x);
         assertEqualsDelta(tuple1.getX(), x, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setY(double y)
         double y = random.nextDouble();
         tuple1.setY(y);
         assertEqualsDelta(tuple1.getY(), y, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setZ(double z)
         double z = random.nextDouble();
         tuple1.setZ(z);
         assertEqualsDelta(tuple1.getZ(), z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(int index, double value)
         try
         {
            tuple1.setElement(-1, random.nextDouble());
            fail("Should have thrown a IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a IndexOutOfBoundsException.");
         }

         try
         {
            tuple1.setElement(3, random.nextDouble());
            fail("Should have thrown a IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a IndexOutOfBoundsException.");
         }

         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple1.setElement(0, x);
         tuple1.setElement(1, y);
         tuple1.setElement(2, z);

         assertEqualsDelta(tuple1.getX(), x, getEpsilon());
         assertEqualsDelta(tuple1.getY(), y, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double x, double y, double z);
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple1.set(x, y, z);

         assertEqualsDelta(tuple1.getX(), x, getEpsilon());
         assertEqualsDelta(tuple1.getY(), y, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(T other)
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());

         tuple1.set(tuple2);
         EuclidCoreTestTools.assertTuple3DEquals(tuple1, tuple2, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(Tuple3DReadOnly tupleReadOnly)
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());

         tuple1.set(tuple2);
         EuclidCoreTestTools.assertTuple3DEquals(tuple1, tuple2, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(Tuple2DReadOnly tupleReadOnly)
         Tuple2DReadOnly tuple2D = EuclidCoreRandomTools.nextPoint2D(random);
         double expectedZ = tuple1.getZ();

         tuple1.set(tuple2D);
         assertEqualsDelta(tuple2D.getX(), tuple1.getX(), getEpsilon());
         assertEqualsDelta(tuple2D.getY(), tuple1.getY(), getEpsilon());
         assertEqualsDelta(expectedZ, tuple1.getZ(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(Tuple2DReadOnly tupleReadOnly, double)
         Tuple2DReadOnly tuple2D = EuclidCoreRandomTools.nextPoint2D(random);
         double expectedZ = random.nextDouble();

         tuple1.set(tuple2D, expectedZ);
         assertEqualsDelta(tuple2D.getX(), tuple1.getX(), getEpsilon());
         assertEqualsDelta(tuple2D.getY(), tuple1.getY(), getEpsilon());
         assertEqualsDelta(expectedZ, tuple1.getZ(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double[] tupleArray);
         double[] tupleArray = {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         tuple1.set(tupleArray);
         assertEqualsDelta(tuple1.getX(), tupleArray[0], getEpsilon());
         assertEqualsDelta(tuple1.getY(), tupleArray[1], getEpsilon());
         assertEqualsDelta(tuple1.getZ(), tupleArray[2], getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double[] tupleArray, int startIndex);
         double[] tupleArray = {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         tuple1.set(2, tupleArray);
         assertEqualsDelta(tuple1.getX(), tupleArray[2], getEpsilon());
         assertEqualsDelta(tuple1.getY(), tupleArray[3], getEpsilon());
         assertEqualsDelta(tuple1.getZ(), tupleArray[4], getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(float[] tupleArray);
         float[] tupleArray = {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         tuple1.set(tupleArray);
         assertTrue(tuple1.getX() == tupleArray[0]);
         assertTrue(tuple1.getY() == tupleArray[1]);
         assertTrue(tuple1.getZ() == tupleArray[2]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(float[] tupleArray, int startIndex);
         float[] tupleArray = {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         tuple1.set(2, tupleArray);
         assertTrue(tuple1.getX32() == tupleArray[2]);
         assertTrue(tuple1.getY32() == tupleArray[3]);
         assertTrue(tuple1.getZ32() == tupleArray[4]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DenseMatrix64F matrix);
         DenseMatrix64F matrix = new DenseMatrix64F(5, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         tuple1.set(matrix);
         assertEqualsDelta(tuple1.getX(), matrix.get(0, 0), getEpsilon());
         assertEqualsDelta(tuple1.getY(), matrix.get(1, 0), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), matrix.get(2, 0), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DenseMatrix64F matrix, int startRow);
         DenseMatrix64F matrix = new DenseMatrix64F(10, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         tuple1.set(5, matrix);
         assertEqualsDelta(tuple1.getX(), matrix.get(5, 0), getEpsilon());
         assertEqualsDelta(tuple1.getY(), matrix.get(6, 0), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), matrix.get(7, 0), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DenseMatrix64F matrix, int startRow, int column);
         DenseMatrix64F matrix = new DenseMatrix64F(10, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         tuple1.set(5, 2, matrix);
         assertEqualsDelta(tuple1.getX(), matrix.get(5, 2), getEpsilon());
         assertEqualsDelta(tuple1.getY(), matrix.get(6, 2), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), matrix.get(7, 2), getEpsilon());
      }
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple = createEmptyTuple();
         tuple.setX(random.nextDouble());
         tuple.setY(random.nextDouble());
         tuple.setZ(random.nextDouble());

         tuple.setToNaN();
         assertTrue(Double.isNaN(tuple.getX()));
         assertTrue(Double.isNaN(tuple.getY()));
         assertTrue(Double.isNaN(tuple.getZ()));
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple = createEmptyTuple();
         tuple.setX(random.nextDouble());
         tuple.setY(random.nextDouble());
         tuple.setZ(random.nextDouble());

         tuple.setToZero();
         assertTrue(tuple.getX() == 0.0);
         assertTrue(tuple.getY() == 0.0);
         assertTrue(tuple.getZ() == 0.0);
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
               double xPos = random.nextDouble();
               double yPos = random.nextDouble();
               double zPos = random.nextDouble();
               tuple1.setX(signX * xPos);
               tuple1.setY(signY * yPos);
               tuple1.setZ(signZ * zPos);

               tuple2.setAndAbsolute(tuple1);
               assertEqualsDelta(tuple2.getX(), xPos, getEpsilon());
               assertEqualsDelta(tuple2.getY(), yPos, getEpsilon());
               assertEqualsDelta(tuple2.getZ(), zPos, getEpsilon());
               assertEqualsDelta(tuple1.getX(), signX * xPos, getEpsilon());
               assertEqualsDelta(tuple1.getY(), signY * yPos, getEpsilon());
               assertEqualsDelta(tuple1.getZ(), signZ * zPos, getEpsilon());

               tuple1.absolute();
               assertEqualsDelta(tuple1.getX(), xPos, getEpsilon());
               assertEqualsDelta(tuple1.getY(), yPos, getEpsilon());
               assertEqualsDelta(tuple1.getZ(), zPos, getEpsilon());
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
               double xOriginal = signX * random.nextDouble();
               double yOriginal = signY * random.nextDouble();
               double zOriginal = signZ * random.nextDouble();
               tuple1.setX(xOriginal);
               tuple1.setY(yOriginal);
               tuple1.setZ(zOriginal);

               tuple2.setAndNegate(tuple1);
               assertEqualsDelta(tuple2.getX(), -xOriginal, getEpsilon());
               assertEqualsDelta(tuple2.getY(), -yOriginal, getEpsilon());
               assertEqualsDelta(tuple2.getZ(), -zOriginal, getEpsilon());
               assertEqualsDelta(tuple1.getX(), xOriginal, getEpsilon());
               assertEqualsDelta(tuple1.getY(), yOriginal, getEpsilon());
               assertEqualsDelta(tuple1.getZ(), zOriginal, getEpsilon());

               tuple1.negate();
               assertEqualsDelta(tuple1.getX(), -xOriginal, getEpsilon());
               assertEqualsDelta(tuple1.getY(), -yOriginal, getEpsilon());
               assertEqualsDelta(tuple1.getZ(), -zOriginal, getEpsilon());
            }
         }
      }
   }

   @Test
   public void testClip() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMax(double max)
         double max = random.nextDouble();
         tuple1.setX(max + random.nextDouble());
         tuple1.setY(max + random.nextDouble());
         tuple1.setZ(max + random.nextDouble());
         tuple1.clipToMax(max);
         assertEqualsDelta(tuple1.getX(), max, getEpsilon());
         assertEqualsDelta(tuple1.getY(), max, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), max, getEpsilon());

         max = random.nextDouble();
         tuple1.setX(max - random.nextDouble());
         tuple1.setY(max - random.nextDouble());
         tuple1.setZ(max - random.nextDouble());
         tuple1.set(tuple2);
         tuple1.clipToMax(max);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMax(double max, TupleBasics other)
         double max = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(max + random.nextDouble());
         tuple2.setY(max + random.nextDouble());
         tuple2.setZ(max + random.nextDouble());
         tuple1.setAndClipToMax(max, tuple2);
         assertEqualsDelta(tuple1.getX(), max, getEpsilon());
         assertEqualsDelta(tuple1.getY(), max, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), max, getEpsilon());

         max = random.nextDouble();
         tuple2.setX(max - random.nextDouble());
         tuple2.setY(max - random.nextDouble());
         tuple2.setZ(max - random.nextDouble());
         tuple1.setAndClipToMax(max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMin(double min)
         double min = random.nextDouble();
         tuple1.setX(min - random.nextDouble());
         tuple1.setY(min - random.nextDouble());
         tuple1.setZ(min - random.nextDouble());
         tuple1.clipToMin(min);
         assertEqualsDelta(tuple1.getX(), min, getEpsilon());
         assertEqualsDelta(tuple1.getY(), min, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), min, getEpsilon());

         min = random.nextDouble();
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple2.setZ(min + random.nextDouble());
         tuple1.set(tuple2);
         tuple1.clipToMin(min);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMin(double min, TupleBasics other)
         double min = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(min - random.nextDouble());
         tuple2.setY(min - random.nextDouble());
         tuple2.setZ(min - random.nextDouble());
         tuple1.setAndClipToMin(min, tuple2);
         assertEqualsDelta(tuple1.getX(), min, getEpsilon());
         assertEqualsDelta(tuple1.getY(), min, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), min, getEpsilon());

         min = random.nextDouble();
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple2.setZ(min + random.nextDouble());
         tuple1.setAndClipToMin(min, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMinMax(double min, double max)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         tuple1.setX(min - random.nextDouble());
         tuple1.setY(min - random.nextDouble());
         tuple1.setZ(min - random.nextDouble());
         tuple1.clipToMinMax(min, max);
         assertEqualsDelta(tuple1.getX(), min, getEpsilon());
         assertEqualsDelta(tuple1.getY(), min, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), min, getEpsilon());

         min = random.nextDouble() - 0.5;
         max = random.nextDouble() + 0.5;
         tuple1.setX(max + random.nextDouble());
         tuple1.setY(max + random.nextDouble());
         tuple1.setZ(max + random.nextDouble());
         tuple1.clipToMinMax(min, max);
         assertEqualsDelta(tuple1.getX(), max, getEpsilon());
         assertEqualsDelta(tuple1.getY(), max, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), max, getEpsilon());

         min = random.nextDouble() - 1.0;
         max = random.nextDouble() + 1.0;
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple2.setZ(min + random.nextDouble());
         tuple1.set(tuple2);
         tuple1.clipToMinMax(min, max);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMinMax(double min, double max, TupleBasics other)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(min - random.nextDouble());
         tuple2.setY(min - random.nextDouble());
         tuple2.setZ(min - random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertEqualsDelta(tuple1.getX(), min, getEpsilon());
         assertEqualsDelta(tuple1.getY(), min, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), min, getEpsilon());

         min = random.nextDouble() - 0.5;
         max = random.nextDouble() + 0.5;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(max + random.nextDouble());
         tuple2.setY(max + random.nextDouble());
         tuple2.setZ(max + random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertEqualsDelta(tuple1.getX(), max, getEpsilon());
         assertEqualsDelta(tuple1.getY(), max, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), max, getEpsilon());

         min = random.nextDouble() - 1.0;
         max = random.nextDouble() + 1.0;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple2.setZ(min + random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }
   }

   @Test
   public void testAdd() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addX(double x), addY(double y), and addZ(double z)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);

         tuple1.addX(x);
         assertEqualsDelta(tuple1.getX(), xOld + x, getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld, getEpsilon());

         tuple1.addY(y);
         assertEqualsDelta(tuple1.getX(), xOld + x, getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld + y, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld, getEpsilon());

         tuple1.addZ(z);
         assertEqualsDelta(tuple1.getX(), xOld + x, getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld + y, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld + z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(double x, double y)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);

         tuple1.add(x, y, z);
         assertEqualsDelta(tuple1.getX(), xOld + x, getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld + y, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld + z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(TupleBasics other)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);

         tuple1.add(tuple2);
         assertEqualsDelta(tuple1.getX(), xOld + tuple2.getX(), getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld + tuple2.getY(), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld + tuple2.getZ(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(TupleBasics other)
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple3.setX(random.nextDouble());
         tuple3.setY(random.nextDouble());
         tuple3.setZ(random.nextDouble());

         tuple1.add(tuple2, tuple3);
         assertEqualsDelta(tuple1.getX(), tuple2.getX() + tuple3.getX(), getEpsilon());
         assertEqualsDelta(tuple1.getY(), tuple2.getY() + tuple3.getY(), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), tuple2.getZ() + tuple3.getZ(), getEpsilon());
      }
   }

   @Test
   public void testSub() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test subX(double x), subY(double y), and subZ(double z)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);

         tuple1.subX(x);
         assertEqualsDelta(tuple1.getX(), xOld - x, getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld, getEpsilon());

         tuple1.subY(y);
         assertEqualsDelta(tuple1.getX(), xOld - x, getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld - y, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld, getEpsilon());

         tuple1.subZ(z);
         assertEqualsDelta(tuple1.getX(), xOld - x, getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld - y, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld - z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(double x, double y)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple1.set(xOld, yOld, zOld);

         tuple1.sub(x, y, z);
         assertEqualsDelta(tuple1.getX(), xOld - x, getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld - y, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld - z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(TupleBasics other)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);

         tuple1.sub(tuple2);
         assertEqualsDelta(tuple1.getX(), xOld - tuple2.getX(), getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld - tuple2.getY(), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld - tuple2.getZ(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(TupleBasics tuple1, TupleBasics tuple2)
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(random.nextDouble(), random.nextDouble(), random.nextDouble());

         tuple1.sub(tuple2, tuple3);
         assertEqualsDelta(tuple1.getX(), tuple2.getX() - tuple3.getX(), getEpsilon());
         assertEqualsDelta(tuple1.getY(), tuple2.getY() - tuple3.getY(), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), tuple2.getZ() - tuple3.getZ(), getEpsilon());
      }
   }

   @Test
   public void testScale() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalarX, double scalarY)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double scale = random.nextDouble();
         tuple1.set(xOld, yOld, zOld);

         tuple1.scale(scale);
         assertEqualsDelta(tuple1.getX(), xOld * scale, getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld * scale, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld * scale, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalarX, double scalarY)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double scaleX = random.nextDouble();
         double scaleY = random.nextDouble();
         double scaleZ = random.nextDouble();
         tuple1.set(xOld, yOld, zOld);

         tuple1.scale(scaleX, scaleY, scaleZ);
         assertEqualsDelta(tuple1.getX(), xOld * scaleX, getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld * scaleY, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld * scaleZ, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalar, TupleBasics other)
         double scale = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble());

         tuple1.setAndScale(scale, tuple2);
         assertEqualsDelta(tuple1.getX(), tuple2.getX() * scale, getEpsilon());
         assertEqualsDelta(tuple1.getY(), tuple2.getY() * scale, getEpsilon());
         assertEqualsDelta(tuple1.getZ(), tuple2.getZ() * scale, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, TupleBasics other)
         double scale = random.nextDouble();
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());

         tuple1.scaleAdd(scale, tuple2);
         assertEqualsDelta(tuple1.getX(), xOld * scale + tuple2.getX(), getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld * scale + tuple2.getY(), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld * scale + tuple2.getZ(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, TupleBasics tuple1, TupleBasics tuple2)
         double scale = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(random.nextDouble(), random.nextDouble(), random.nextDouble());

         tuple1.scaleAdd(scale, tuple2, tuple3);
         assertEqualsDelta(tuple1.getX(), tuple2.getX() * scale + tuple3.getX(), getEpsilon());
         assertEqualsDelta(tuple1.getY(), tuple2.getY() * scale + tuple3.getY(), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), tuple2.getZ() * scale + tuple3.getZ(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, TupleBasics tuple1, TupleBasics tuple2) with tuple2 == this
         double scale = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(tuple1);

         tuple1.scaleAdd(scale, tuple2, tuple1);
         assertEqualsDelta(tuple1.getX(), tuple2.getX() * scale + tuple3.getX(), getEpsilon());
         assertEqualsDelta(tuple1.getY(), tuple2.getY() * scale + tuple3.getY(), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), tuple2.getZ() * scale + tuple3.getZ(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, TupleBasics other)
         double scale = random.nextDouble();
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());

         tuple1.scaleSub(scale, tuple2);
         assertEqualsDelta(tuple1.getX(), xOld * scale - tuple2.getX(), getEpsilon());
         assertEqualsDelta(tuple1.getY(), yOld * scale - tuple2.getY(), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), zOld * scale - tuple2.getZ(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, TupleBasics tuple1, TupleBasics tuple2)
         double scale = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(random.nextDouble(), random.nextDouble(), random.nextDouble());

         tuple1.scaleSub(scale, tuple2, tuple3);
         assertEqualsDelta(tuple1.getX(), tuple2.getX() * scale - tuple3.getX(), getEpsilon());
         assertEqualsDelta(tuple1.getY(), tuple2.getY() * scale - tuple3.getY(), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), tuple2.getZ() * scale - tuple3.getZ(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, TupleBasics tuple1, TupleBasics tuple2) with tuple2 == this
         double scale = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(tuple1);

         tuple1.scaleSub(scale, tuple2, tuple1);
         assertEqualsDelta(tuple1.getX(), tuple2.getX() * scale - tuple3.getX(), getEpsilon());
         assertEqualsDelta(tuple1.getY(), tuple2.getY() * scale - tuple3.getY(), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), tuple2.getZ() * scale - tuple3.getZ(), getEpsilon());
      }
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test interpolate(TupleBasics other, double alpha)
         double alpha = random.nextDouble();
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());

         tuple1.interpolate(tuple2, alpha);
         assertEqualsDelta(tuple1.getX(), EuclidCoreTools.interpolate(xOld, tuple2.getX(), alpha), getEpsilon());
         assertEqualsDelta(tuple1.getY(), EuclidCoreTools.interpolate(yOld, tuple2.getY(), alpha), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), EuclidCoreTools.interpolate(zOld, tuple2.getZ(), alpha), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test interpolate(TupleBasics tuple1, TupleBasics tuple2, double alpha)
         double alpha = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(random.nextDouble(), random.nextDouble(), random.nextDouble());

         tuple1.interpolate(tuple2, tuple3, alpha);
         assertEqualsDelta(tuple1.getX(), EuclidCoreTools.interpolate(tuple2.getX(), tuple3.getX(), alpha), getEpsilon());
         assertEqualsDelta(tuple1.getY(), EuclidCoreTools.interpolate(tuple2.getY(), tuple3.getY(), alpha), getEpsilon());
         assertEqualsDelta(tuple1.getZ(), EuclidCoreTools.interpolate(tuple2.getZ(), tuple3.getZ(), alpha), getEpsilon());
      }
   }
}