package us.ihmc.euclid.tuple2D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class UnitVector2DTest extends Vector2DBasicsTest<UnitVector2D>
{
   private static final double DOUBLE_EPSILON = 1.0e-14;
   private static final double FLOAT_EPSILON = 1.0e-7;

   @Test
   public void testConstructor()
   {
      Random random = new Random(621541L);

      { // Test UnitVector2D()
         UnitVector2D actual = new UnitVector2D();
         assertTrue(1 == actual.getX());
         assertTrue(0 == actual.getY());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test UnitVector2D(double x, double y)
         double newX = random.nextDouble();
         double newY = random.nextDouble();

         UnitVector2D actual = new UnitVector2D(newX, newY);
         Vector2D expected = new Vector2D(newX, newY);
         expected.normalize();

         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test UnitVector2D(double[] vectorArray)
         double[] randomVectorArray = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         UnitVector2D actual = new UnitVector2D(randomVectorArray);
         Vector2D expected = new Vector2D(randomVectorArray);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test UnitVector2D(Tuple2DReadOnly tuple)
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         UnitVector2D actual = new UnitVector2D(other);
         Vector2D expected = new Vector2D(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testSetters() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setX(double x)
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);

         double x = random.nextDouble();
         actual.setX(x);
         expected.setX(x);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setY(double y)
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);

         double y = random.nextDouble();
         actual.setY(y);
         expected.setY(y);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(int index, double value)
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         try
         {
            actual.setElement(-1, random.nextDouble());
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
            actual.setElement(2, random.nextDouble());
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
         actual.setElement(0, x);
         actual.setElement(1, y);
         expected.setElement(0, x);
         expected.setElement(1, y);

         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double x, double y);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();
         double x = random.nextDouble();
         double y = random.nextDouble();
         actual.set(x, y);
         expected.set(x, y);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(T other)
         UnitVector2D other = EuclidCoreRandomTools.nextUnitVector2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.set(other);
         expected.set(other);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(Tuple2DReadOnly tupleReadOnly)
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.set(other);
         expected.set(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(Tuple3DReadOnly tupleReadOnly)
         Tuple3DReadOnly tuple3D = EuclidCoreRandomTools.nextPoint3D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.set(tuple3D);
         expected.set(tuple3D);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double[] tupleArray);
         double[] tupleArray = {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.set(tupleArray);
         expected.set(tupleArray);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double[] tupleArray, int startIndex);
         double[] tupleArray = {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.set(2, tupleArray);
         expected.set(2, tupleArray);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(float[] tupleArray);
         float[] tupleArray = {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.set(tupleArray);
         expected.set(tupleArray);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(float[] tupleArray, int startIndex);
         float[] tupleArray = {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.set(2, tupleArray);
         expected.set(2, tupleArray);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DMatrixRMaj matrix);
         DMatrixRMaj matrix = new DMatrixRMaj(5, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.set(matrix);
         expected.set(matrix);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DMatrixRMaj matrix, int startRow);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.set(5, matrix);
         expected.set(5, matrix);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DMatrixRMaj matrix, int startRow, int column);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.set(5, 2, matrix);
         expected.set(5, 2, matrix);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testAbsolute() throws Exception
   {
      Random random = new Random(45036);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.absolute();
         expected.absolute();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);

         actual.setAndAbsolute(other);
         expected.setAndAbsolute(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testNegate() throws Exception
   {
      Random random = new Random(45036);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.negate();
         expected.negate();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);

         actual.setAndNegate(other);
         expected.setAndNegate(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testGetters() throws Exception
   {
      Random random = new Random(621541L);
      UnitVector2D vector = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getX(), getY(), getZ()
         double x = random.nextDouble();
         double y = random.nextDouble();
         vector = createTuple(x, y);

         double length = EuclidCoreTools.norm(x, y);
         assertEquals(vector.getX(), x / length, DOUBLE_EPSILON);
         assertEquals(vector.getY(), y / length, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getX32(), getY32()
         float x = random.nextFloat();
         float y = random.nextFloat();
         vector = createTuple(x, y);

         double length = EuclidCoreTools.norm(x, y);
         assertEquals(vector.getX32(), x / length, FLOAT_EPSILON);
         assertEquals(vector.getY32(), y / length, FLOAT_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int index)
         double x = random.nextDouble();
         double y = random.nextDouble();
         vector = createTuple(x, y);

         double length = EuclidCoreTools.norm(x, y);
         assertEquals(vector.getElement(0), x / length, DOUBLE_EPSILON);
         assertEquals(vector.getElement(1), y / length, DOUBLE_EPSILON);

         try
         {
            vector.getElement(-1);
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
            vector.getElement(2);
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
         vector = createTuple(x, y);

         double length = EuclidCoreTools.norm(x, y);
         assertEquals(vector.getElement32(0), x / length, FLOAT_EPSILON);
         assertEquals(vector.getElement32(1), y / length, FLOAT_EPSILON);

         try
         {
            vector.getElement32(-1);
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
            vector.getElement32(2);
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
         vector = createRandomTuple(random);
         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         vector.get(tupleArray);
         assertTrue(vector.getX() == tupleArray[0]);
         assertTrue(vector.getY() == tupleArray[1]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(double[] tupleArrayToPack, int startIndex)
         vector = createRandomTuple(random);
         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         vector.get(2, tupleArray);
         assertTrue(vector.getX() == tupleArray[2]);
         assertTrue(vector.getY() == tupleArray[3]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(float[] tupleArrayToPack)
         vector = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(),
               random.nextFloat()};
         vector.get(tupleArray);
         assertTrue(vector.getX32() == tupleArray[0]);
         assertTrue(vector.getY32() == tupleArray[1]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(double[] tupleArrayToPack, int startIndex)
         vector = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(),
               random.nextFloat()};
         vector.get(2, tupleArray);
         assertTrue(vector.getX32() == tupleArray[2]);
         assertTrue(vector.getY32() == tupleArray[3]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(DMatrixRMaj tupleMatrixToPack)
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         vector.get(matrix);
         assertTrue(vector.getX() == matrix.get(0, 0));
         assertTrue(vector.getY() == matrix.get(1, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startRow, DMatrixRMaj tupleMatrixToPack)
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         vector.get(2, matrix);
         assertTrue(vector.getX() == matrix.get(2, 0));
         assertTrue(vector.getY() == matrix.get(3, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startRow, int startColumn, DMatrixRMaj tupleMatrixToPack)
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         vector.get(2, 4, matrix);
         assertTrue(vector.getX() == matrix.get(2, 4));
         assertTrue(vector.getY() == matrix.get(3, 4));
      }
   }

   @Test
   @Override
   public void testAdd() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addX(double x), addY(double y)
         UnitVector2D actual = new UnitVector2D();
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         double x = random.nextDouble();
         double y = random.nextDouble();
         actual.set(expected);

         actual.addX(x);
         expected.addX(x);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);

         actual.addY(y);
         expected.addY(y);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(double x, double y, double z)
         UnitVector2D actual = new UnitVector2D();
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         double x = random.nextDouble();
         double y = random.nextDouble();
         actual.set(expected);

         actual.add(x, y);
         expected.add(x, y);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(Tuple2DReadOnly other)
         UnitVector2D actual = new UnitVector2D();
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DReadOnly tupleToAdd = EuclidCoreRandomTools.nextPoint2D(random);
         actual.set(expected);

         actual.add(tupleToAdd);
         expected.add(tupleToAdd);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2)
         UnitVector2D actual = new UnitVector2D();
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DReadOnly tupleToAdd1 = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DReadOnly tupleToAdd2 = EuclidCoreRandomTools.nextPoint2D(random);
         actual.set(expected);

         actual.add(tupleToAdd1, tupleToAdd2);
         expected.add(tupleToAdd1, tupleToAdd2);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testSub() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test subX(double x), subY(double y)
         UnitVector2D actual = new UnitVector2D();
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         double x = random.nextDouble();
         double y = random.nextDouble();
         actual.set(expected);

         actual.subX(x);
         expected.subX(x);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);

         actual.subY(y);
         expected.subY(y);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(double x, double y, double z)
         UnitVector2D actual = new UnitVector2D();
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         double x = random.nextDouble();
         double y = random.nextDouble();
         actual.set(expected);

         actual.sub(x, y);
         expected.sub(x, y);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(Tuple2DReadOnly other)
         UnitVector2D actual = new UnitVector2D();
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DReadOnly tupleToSub = EuclidCoreRandomTools.nextPoint2D(random);
         actual.set(expected);

         actual.sub(tupleToSub);
         expected.sub(tupleToSub);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2)
         UnitVector2D actual = new UnitVector2D();
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DReadOnly tupleToSub1 = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DReadOnly tupleToSub2 = EuclidCoreRandomTools.nextPoint2D(random);
         actual.set(expected);

         actual.sub(tupleToSub1, tupleToSub2);
         expected.sub(tupleToSub1, tupleToSub2);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testClip() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMax(double max)
         double max = random.nextDouble();
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.clipToMax(max);
         expected.clipToMax(max);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMax(double max, Tuple2DReadOnly other)
         double max = random.nextDouble();
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.setAndClipToMax(max, other);
         expected.setAndClipToMax(max, other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMin(double min)
         double min = random.nextDouble();
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.clipToMin(min);
         expected.clipToMin(min);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMin(double min, TupleBasics other)
         double min = random.nextDouble();
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.setAndClipToMin(min, other);
         expected.setAndClipToMin(min, other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMinMax(double min, double max)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.clipToMinMax(min, max);
         expected.clipToMinMax(min, max);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMinMax(double min, double max, TupleBasics other)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         actual.setAndClipToMinMax(min, max, other);
         expected.setAndClipToMinMax(min, max, other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testScale() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalar)
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         double scale = random.nextDouble();

         actual.scale(scale);
         expected.scale(scale);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalarX, double scalarY)
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         double scaleX = random.nextDouble();
         double scaleY = random.nextDouble();

         actual.scale(scaleX, scaleY);
         expected.scale(scaleX, scaleY);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalar, Tuple2DReadOnly other)
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         double scale = random.nextDouble();

         actual.setAndScale(scale, other);
         expected.setAndScale(scale, other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, Tuple2DBasics other)
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         double scale = random.nextDouble();

         actual.scaleAdd(scale, other); // the scale is applied to unit vector which cannot be scaled.
         expected.add(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2)
         Tuple2DReadOnly other1 = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DReadOnly other2 = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         double scale = random.nextDouble();

         actual.scaleAdd(scale, other1, other2);
         expected.scaleAdd(scale, other1, other2);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2) with tuple2 == this
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         double scale = random.nextDouble();

         actual.scaleAdd(scale, other, actual);
         expected.scaleAdd(scale, other, expected);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, Tuple2DReadOnly other)
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         double scale = random.nextDouble();

         actual.scaleSub(scale, other); // the scale is applied to unit vector which cannot be scaled.
         expected.sub(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2)
         Tuple2DReadOnly other1 = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DReadOnly other2 = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         double scale = random.nextDouble();

         actual.scaleSub(scale, other1, other2);
         expected.scaleSub(scale, other1, other2);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2) with tuple2 == this
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         double scale = random.nextDouble();

         actual.scaleSub(scale, other, actual);
         expected.scaleSub(scale, other, expected);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testSetToZero() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         UnitVector2D tuple = EuclidCoreRandomTools.nextUnitVector2D(random);

         tuple.setToZero();
         assertTrue(tuple.getX() == 1.0);
         assertTrue(tuple.getY() == 0.0);
      }
   }

   @Test
   @Override
   public void testLengthSquared()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random, -100.0, 100.0);
         UnitVector2D actual = new UnitVector2D(expected);

         assertEquals(1.0, actual.lengthSquared());
      }
   }

   @Test
   @Override
   public void testInterpolate() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test interpolate(Tuple2DReadOnly other, double alpha)
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();
         double alpha = random.nextDouble();

         actual.interpolate(other, alpha);
         expected.interpolate(other, alpha);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test interpolate(Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2, double alpha)
         Tuple2DReadOnly other1 = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DReadOnly other2 = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();
         double alpha = random.nextDouble();

         actual.interpolate(other1, other2, alpha);
         expected.interpolate(other1, other2, alpha);
         expected.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      UnitVector2D tuple1 = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         tuple1.setElement(i % 2, random.nextDouble());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   @Test
   public void testLength()
   {
      Random random = new Random(234234);

      for (int i = 0; i < ITERATIONS; i++)
      {
         UnitVector2D vector = new UnitVector2D(EuclidCoreRandomTools.nextVector2D(random, -100.0, 100.0));
         assertTrue(vector.length() == 1.0);
      }
   }

   @Override
   @Test
   public void testClipToMaxLength() throws Exception
   {
      Random random = new Random(234234);

      for (int i = 0; i < ITERATIONS; i++)
      {
         UnitVector2D vector = EuclidCoreRandomTools.nextUnitVector2D(random);
         Vector2D oldValues = new Vector2D(vector);

         assertFalse(vector.clipToMaxLength(EuclidCoreRandomTools.nextDouble(random, 10.0)));
         assertTrue(oldValues.equals(vector));
      }
   }

   @Test
   @Override
   public void testCross()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < ITERATIONS; i++)
      { // cross(Tuple2DReadOnly other)
         Tuple2DReadOnly other = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         UnitVector2D actual = new UnitVector2D(expected);
         expected.normalize();

         assertEquals(actual.cross(other), expected.cross(other));
      }
   }

   @Test
   @Override
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();

      UnitVector2D vector1 = createRandomTuple(random);
      UnitVector2D vector2 = createRandomTuple(random);

      vector2.set(vector1);
      assertTrue(vector1.epsilonEquals(vector2, epsilon));
      AxisAngle axisAngle = new AxisAngle(Axis3D.Z, 0.0);
      axisAngle.setAngle(0.999 * epsilon);
      axisAngle.transform(vector1, vector2);
      assertTrue(vector1.epsilonEquals(vector2, epsilon));
      axisAngle.setAngle(-0.999 * epsilon);
      axisAngle.transform(vector1, vector2);
      assertTrue(vector1.epsilonEquals(vector2, epsilon));
      axisAngle.setAngle(2.0 * epsilon);
      axisAngle.transform(vector1, vector2);
      assertFalse(vector1.epsilonEquals(vector2, epsilon));
      axisAngle.setAngle(-2.0 * epsilon);
      axisAngle.transform(vector1, vector2);
      assertFalse(vector1.epsilonEquals(vector2, epsilon));
   }

   @Override
   @Test
   public void testGeometricallyEquals() throws Exception
   {
      super.testGeometricallyEquals();

      UnitVector2D vectorA;
      UnitVector2D vectorB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         vectorA = EuclidCoreRandomTools.nextUnitVector2D(random);
         vectorB = EuclidCoreRandomTools.nextUnitVector2D(random);

         if (((Vector2DReadOnly) vectorA).geometricallyEquals(vectorB, DOUBLE_EPSILON))
         {
            assertTrue(vectorA.geometricallyEquals(vectorB, DOUBLE_EPSILON));
         }
         else
         {
            assertFalse(vectorA.geometricallyEquals(vectorB, DOUBLE_EPSILON));
         }
      }
   }

   @Override
   public UnitVector2D createEmptyTuple()
   {
      return new UnitVector2D();
   }

   @Override
   public UnitVector2D createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextUnitVector2D(random);
   }

   @Override
   public UnitVector2D createTuple(double x, double y)
   {
      return new UnitVector2D(x, y);
   }

   @Override
   public double getEpsilon()
   {
      return DOUBLE_EPSILON;
   }
}
