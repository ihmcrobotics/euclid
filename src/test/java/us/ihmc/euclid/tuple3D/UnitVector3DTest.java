package us.ihmc.euclid.tuple3D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class UnitVector3DTest extends Vector3DBasicsTest<UnitVector3D>
{
   private static final double DOUBLE_EPSILON = 1.0e-14;
   private static final double FLOAT_EPSILON = 1.0e-7;

   @Test
   public void testConstructor()
   {
      Random random = new Random(621541L);

      { // Test UnitVector3D()
         UnitVector3D actual = new UnitVector3D();
         assertTrue(1 == actual.getX());
         assertTrue(0 == actual.getY());
         assertTrue(0 == actual.getZ());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test UnitVector3D(double x, double y, double z)
         double newX = random.nextDouble();
         double newY = random.nextDouble();
         double newZ = random.nextDouble();

         UnitVector3D actual = new UnitVector3D(newX, newY, newZ);
         Vector3D expected = new Vector3D(newX, newY, newZ);
         expected.normalize();

         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test UnitVector3D(double[] vectorArray)
         double[] randomVectorArray = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         UnitVector3D actual = new UnitVector3D(randomVectorArray);
         Vector3D expected = new Vector3D(randomVectorArray);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test UnitVector3D(Tuple3DReadOnly tuple)
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         UnitVector3D actual = new UnitVector3D(other);
         Vector3D expected = new Vector3D(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testSetters() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setX(double x)
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);

         double x = random.nextDouble();
         actual.setX(x);
         expected.setX(x);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setY(double y)
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);

         double y = random.nextDouble();
         actual.setY(y);
         expected.setY(y);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setZ(double z)
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);

         double z = random.nextDouble();
         actual.setZ(z);
         expected.setZ(z);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(int index, double value)
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
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
            actual.setElement(3, random.nextDouble());
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
         actual.setElement(0, x);
         actual.setElement(1, y);
         actual.setElement(2, z);
         expected.setElement(0, x);
         expected.setElement(1, y);
         expected.setElement(2, z);

         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double x, double y, double z);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         actual.set(x, y, z);
         expected.set(x, y, z);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(T other)
         UnitVector3D other = EuclidCoreRandomTools.nextUnitVector3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.set(other);
         expected.set(other);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(Tuple3DReadOnly tupleReadOnly)
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.set(other);
         expected.set(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(Tuple2DReadOnly tupleReadOnly)
         Tuple2DReadOnly tuple2D = EuclidCoreRandomTools.nextPoint2D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.set(tuple2D);
         expected.set(tuple2D);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(Tuple2DReadOnly tupleReadOnly, double)
         Tuple2DReadOnly tuple2D = EuclidCoreRandomTools.nextPoint2D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         double z = random.nextDouble();
         actual.set(tuple2D, z);
         expected.set(tuple2D, z);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double[] tupleArray);
         double[] tupleArray = {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.set(tupleArray);
         expected.set(tupleArray);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double[] tupleArray, int startIndex);
         double[] tupleArray = {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.set(2, tupleArray);
         expected.set(2, tupleArray);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(float[] tupleArray);
         float[] tupleArray = {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.set(tupleArray);
         expected.set(tupleArray);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(float[] tupleArray, int startIndex);
         float[] tupleArray = {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.set(2, tupleArray);
         expected.set(2, tupleArray);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DMatrixRMaj matrix);
         DMatrixRMaj matrix = new DMatrixRMaj(5, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.set(matrix);
         expected.set(matrix);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DMatrixRMaj matrix, int startRow);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.set(5, matrix);
         expected.set(5, matrix);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(DMatrixRMaj matrix, int startRow, int column);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.set(5, 2, matrix);
         expected.set(5, 2, matrix);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testAbsolute() throws Exception
   {
      Random random = new Random(45036);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.absolute();
         expected.absolute();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);

         actual.setAndAbsolute(other);
         expected.setAndAbsolute(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testNegate() throws Exception
   {
      Random random = new Random(45036);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.negate();
         expected.negate();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);

         actual.setAndNegate(other);
         expected.setAndNegate(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testGetters() throws Exception
   {
      Random random = new Random(621541L);
      UnitVector3D vector = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getX(), getY(), getZ()
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         vector = createTuple(x, y, z);

         double length = EuclidCoreTools.norm(x, y, z);
         assertEquals(vector.getX(), x / length, DOUBLE_EPSILON);
         assertEquals(vector.getY(), y / length, DOUBLE_EPSILON);
         assertEquals(vector.getZ(), z / length, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getX32(), getY32()
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         vector = createTuple(x, y, z);

         double length = EuclidCoreTools.norm(x, y, z);
         assertEquals(vector.getX32(), x / length, FLOAT_EPSILON);
         assertEquals(vector.getY32(), y / length, FLOAT_EPSILON);
         assertEquals(vector.getZ32(), z / length, FLOAT_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int index)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         vector = createTuple(x, y, z);

         double length = EuclidCoreTools.norm(x, y, z);
         assertEquals(vector.getElement(0), x / length, DOUBLE_EPSILON);
         assertEquals(vector.getElement(1), y / length, DOUBLE_EPSILON);
         assertEquals(vector.getElement(2), z / length, DOUBLE_EPSILON);

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
            vector.getElement(3);
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
         vector = createTuple(x, y, z);

         double length = EuclidCoreTools.norm(x, y, z);
         assertEquals(vector.getElement32(0), x / length, FLOAT_EPSILON);
         assertEquals(vector.getElement32(1), y / length, FLOAT_EPSILON);
         assertEquals(vector.getElement32(2), z / length, FLOAT_EPSILON);

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
            vector.getElement32(3);
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
         assertTrue(vector.getZ() == tupleArray[2]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(double[] tupleArrayToPack, int startIndex)
         vector = createRandomTuple(random);
         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         vector.get(2, tupleArray);
         assertTrue(vector.getX() == tupleArray[2]);
         assertTrue(vector.getY() == tupleArray[3]);
         assertTrue(vector.getZ() == tupleArray[4]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(float[] tupleArrayToPack)
         vector = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(),
               random.nextFloat()};
         vector.get(tupleArray);
         assertTrue(vector.getX32() == tupleArray[0]);
         assertTrue(vector.getY32() == tupleArray[1]);
         assertTrue(vector.getZ32() == tupleArray[2]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(double[] tupleArrayToPack, int startIndex)
         vector = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(),
               random.nextFloat()};
         vector.get(2, tupleArray);
         assertTrue(vector.getX32() == tupleArray[2]);
         assertTrue(vector.getY32() == tupleArray[3]);
         assertTrue(vector.getZ32() == tupleArray[4]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(DMatrixRMaj tupleMatrixToPack)
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         vector.get(matrix);
         assertTrue(vector.getX() == matrix.get(0, 0));
         assertTrue(vector.getY() == matrix.get(1, 0));
         assertTrue(vector.getZ() == matrix.get(2, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startRow, DMatrixRMaj tupleMatrixToPack)
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         vector.get(2, matrix);
         assertTrue(vector.getX() == matrix.get(2, 0));
         assertTrue(vector.getY() == matrix.get(3, 0));
         assertTrue(vector.getZ() == matrix.get(4, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startRow, int startColumn, DMatrixRMaj tupleMatrixToPack)
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         vector.get(2, 4, matrix);
         assertTrue(vector.getX() == matrix.get(2, 4));
         assertTrue(vector.getY() == matrix.get(3, 4));
         assertTrue(vector.getZ() == matrix.get(4, 4));
      }
   }

   @Test
   @Override
   public void testAdd() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addX(double x), addY(double y), and addZ(double z)
         UnitVector3D actual = new UnitVector3D();
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         actual.set(expected);

         actual.addX(x);
         expected.addX(x);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);

         actual.addY(y);
         expected.addY(y);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);

         actual.addZ(z);
         expected.addZ(z);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(double x, double y, double z)
         UnitVector3D actual = new UnitVector3D();
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         actual.set(expected);

         actual.add(x, y, z);
         expected.add(x, y, z);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(Tuple3DReadOnly other)
         UnitVector3D actual = new UnitVector3D();
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DReadOnly tupleToAdd = EuclidCoreRandomTools.nextPoint3D(random);
         actual.set(expected);

         actual.add(tupleToAdd);
         expected.add(tupleToAdd);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
         UnitVector3D actual = new UnitVector3D();
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DReadOnly tupleToAdd1 = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DReadOnly tupleToAdd2 = EuclidCoreRandomTools.nextPoint3D(random);
         actual.set(expected);

         actual.add(tupleToAdd1, tupleToAdd2);
         expected.add(tupleToAdd1, tupleToAdd2);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testSub() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test subX(double x), subY(double y), and subZ(double z)
         UnitVector3D actual = new UnitVector3D();
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         actual.set(expected);

         actual.subX(x);
         expected.subX(x);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);

         actual.subY(y);
         expected.subY(y);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);

         actual.subZ(z);
         expected.subZ(z);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(double x, double y, double z)
         UnitVector3D actual = new UnitVector3D();
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         actual.set(expected);

         actual.sub(x, y, z);
         expected.sub(x, y, z);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(Tuple3DReadOnly other)
         UnitVector3D actual = new UnitVector3D();
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DReadOnly tupleToSub = EuclidCoreRandomTools.nextPoint3D(random);
         actual.set(expected);

         actual.sub(tupleToSub);
         expected.sub(tupleToSub);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
         UnitVector3D actual = new UnitVector3D();
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DReadOnly tupleToSub1 = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DReadOnly tupleToSub2 = EuclidCoreRandomTools.nextPoint3D(random);
         actual.set(expected);

         actual.sub(tupleToSub1, tupleToSub2);
         expected.sub(tupleToSub1, tupleToSub2);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
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
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.clipToMax(max);
         expected.clipToMax(max);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMax(double max, Tuple3DReadOnly other)
         double max = random.nextDouble();
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.setAndClipToMax(max, other);
         expected.setAndClipToMax(max, other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMin(double min)
         double min = random.nextDouble();
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.clipToMin(min);
         expected.clipToMin(min);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMin(double min, TupleBasics other)
         double min = random.nextDouble();
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.setAndClipToMin(min, other);
         expected.setAndClipToMin(min, other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMinMax(double min, double max)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.clipToMinMax(min, max);
         expected.clipToMinMax(min, max);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMinMax(double min, double max, TupleBasics other)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.setAndClipToMinMax(min, max, other);
         expected.setAndClipToMinMax(min, max, other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testScale() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalarX, double scalarY)
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         double scale = random.nextDouble();

         actual.scale(scale);
         expected.scale(scale);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalarX, double scalarY)
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         double scaleX = random.nextDouble();
         double scaleY = random.nextDouble();
         double scaleZ = random.nextDouble();

         actual.scale(scaleX, scaleY, scaleZ);
         expected.scale(scaleX, scaleY, scaleZ);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalar, Tuple3DReadOnly other)
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         double scale = random.nextDouble();

         actual.setAndScale(scale, other);
         expected.setAndScale(scale, other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, Tuple3DBasics other)
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         double scale = random.nextDouble();

         actual.scaleAdd(scale, other); // the scale is applied to unit vector which cannot be scaled.
         expected.add(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
         Tuple3DReadOnly other1 = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DReadOnly other2 = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         double scale = random.nextDouble();

         actual.scaleAdd(scale, other1, other2);
         expected.scaleAdd(scale, other1, other2);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2) with tuple2 == this
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         double scale = random.nextDouble();

         actual.scaleAdd(scale, other, actual);
         expected.scaleAdd(scale, other, expected);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, Tuple3DReadOnly other)
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         double scale = random.nextDouble();

         actual.scaleSub(scale, other); // the scale is applied to unit vector which cannot be scaled.
         expected.sub(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
         Tuple3DReadOnly other1 = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DReadOnly other2 = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         double scale = random.nextDouble();

         actual.scaleSub(scale, other1, other2);
         expected.scaleSub(scale, other1, other2);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2) with tuple2 == this
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         double scale = random.nextDouble();

         actual.scaleSub(scale, other, actual);
         expected.scaleSub(scale, other, expected);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testSetToZero() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         UnitVector3D tuple = EuclidCoreRandomTools.nextUnitVector3D(random);

         tuple.setToZero();
         assertTrue(tuple.getX() == 1.0);
         assertTrue(tuple.getY() == 0.0);
         assertTrue(tuple.getZ() == 0.0);
      }
   }

   @Test
   @Override
   public void testLengthSquared()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random, -100.0, 100.0);
         UnitVector3D actual = new UnitVector3D(expected);

         assertEquals(1.0, actual.lengthSquared());
      }
   }

   @Test
   @Override
   public void testInterpolate() throws Exception
   {
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test interpolate(Tuple3DReadOnly other, double alpha)
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();
         double alpha = random.nextDouble();

         actual.interpolate(other, alpha);
         expected.interpolate(other, alpha);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test interpolate(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2, double alpha)
         Tuple3DReadOnly other1 = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DReadOnly other2 = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();
         double alpha = random.nextDouble();

         actual.interpolate(other1, other2, alpha);
         expected.interpolate(other1, other2, alpha);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      UnitVector3D tuple1 = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         tuple1.setElement(i % 3, random.nextDouble());
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
         UnitVector3D vector = new UnitVector3D(EuclidCoreRandomTools.nextVector3D(random, -100.0, 100.0));
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
         UnitVector3D vector = EuclidCoreRandomTools.nextUnitVector3D(random);
         Vector3D oldValues = new Vector3D(vector);

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
      { // cross(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
         Tuple3DReadOnly other1 = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DReadOnly other2 = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.cross(other1, other2);
         expected.cross(other1, other2);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // cross(Tuple3DReadOnly other)
         Tuple3DReadOnly other = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         UnitVector3D actual = new UnitVector3D(expected);
         expected.normalize();

         actual.cross(other);
         expected.cross(other);
         expected.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, DOUBLE_EPSILON);
      }
   }

   @Test
   @Override
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();

      UnitVector3D vector1 = createRandomTuple(random);
      UnitVector3D vector2 = createRandomTuple(random);

      vector2.set(vector1);
      assertTrue(vector1.epsilonEquals(vector2, epsilon));
      AxisAngle axisAngle = new AxisAngle(EuclidCoreRandomTools.nextOrthogonalVector3D(random, vector1, true), 0.0);
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

      UnitVector3D vectorA;
      UnitVector3D vectorB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         vectorA = EuclidCoreRandomTools.nextUnitVector3D(random);
         vectorB = EuclidCoreRandomTools.nextUnitVector3D(random);

         if (((Vector3DReadOnly) vectorA).geometricallyEquals(vectorB, DOUBLE_EPSILON))
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
   public UnitVector3D createEmptyTuple()
   {
      return new UnitVector3D();
   }

   @Override
   public UnitVector3D createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextUnitVector3D(random);
   }

   @Override
   public UnitVector3D createTuple(double x, double y, double z)
   {
      return new UnitVector3D(x, y, z);
   }

   @Override
   public double getEpsilon()
   {
      return DOUBLE_EPSILON;
   }
}
