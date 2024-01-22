package us.ihmc.euclid.tuple3D;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

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

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getX(), getY()
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         T tuple = createTuple(x, y, z);

         assertEquals(tuple.getX(), x, getEpsilon());
         assertEquals(tuple.getY(), y, getEpsilon());
         assertEquals(tuple.getZ(), z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getX32(), getY32()
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         T tuple = createTuple(x, y, z);

         assertEquals(tuple.getX32(), x, getEpsilon());
         assertEquals(tuple.getY32(), y, getEpsilon());
         assertEquals(tuple.getZ32(), z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getElement(int index)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         T tuple = createTuple(x, y, z);

         assertEquals(tuple.getElement(0), x, getEpsilon());
         assertEquals(tuple.getElement(1), y, getEpsilon());
         assertEquals(tuple.getElement(2), z, getEpsilon());

         assertThrows(IndexOutOfBoundsException.class, () -> tuple.getElement(-1));
         assertThrows(IndexOutOfBoundsException.class, () -> tuple.getElement(3));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getElement(Axis3D)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         T tuple = createTuple(x, y, z);

         assertEquals(tuple.getElement(Axis3D.X), x, getEpsilon());
         assertEquals(tuple.getElement(Axis3D.Y), y, getEpsilon());
         assertEquals(tuple.getElement(Axis3D.Z), z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getElement32(int index)
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         T tuple = createTuple(x, y, z);

         assertTrue(tuple.getElement32(0) == x);
         assertTrue(tuple.getElement32(1) == y);
         assertTrue(tuple.getElement32(2) == z);

         assertThrows(IndexOutOfBoundsException.class, () -> tuple.getElement32(-1));
         assertThrows(IndexOutOfBoundsException.class, () -> tuple.getElement32(3));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(double[] tupleArrayToPack)
         T tuple = createRandomTuple(random);
         double[] tupleArray = new double[] {random.nextDouble(),
                                             random.nextDouble(),
                                             random.nextDouble(),
                                             random.nextDouble(),
                                             random.nextDouble(),
                                             random.nextDouble()};
         tuple.get(tupleArray);
         assertTrue(tuple.getX() == tupleArray[0]);
         assertTrue(tuple.getY() == tupleArray[1]);
         assertTrue(tuple.getZ() == tupleArray[2]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(double[] tupleArrayToPack, int startIndex)
         T tuple = createRandomTuple(random);
         double[] tupleArray = new double[] {random.nextDouble(),
                                             random.nextDouble(),
                                             random.nextDouble(),
                                             random.nextDouble(),
                                             random.nextDouble(),
                                             random.nextDouble()};
         tuple.get(2, tupleArray);
         assertTrue(tuple.getX() == tupleArray[2]);
         assertTrue(tuple.getY() == tupleArray[3]);
         assertTrue(tuple.getZ() == tupleArray[4]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(float[] tupleArrayToPack)
         T tuple = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(),
                                           random.nextFloat(),
                                           random.nextFloat(),
                                           random.nextFloat(),
                                           random.nextFloat(),
                                           random.nextFloat()};
         tuple.get(tupleArray);
         assertTrue(tuple.getX32() == tupleArray[0]);
         assertTrue(tuple.getY32() == tupleArray[1]);
         assertTrue(tuple.getZ32() == tupleArray[2]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(double[] tupleArrayToPack, int startIndex)
         T tuple = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(),
                                           random.nextFloat(),
                                           random.nextFloat(),
                                           random.nextFloat(),
                                           random.nextFloat(),
                                           random.nextFloat()};
         tuple.get(2, tupleArray);
         assertTrue(tuple.getX32() == tupleArray[2]);
         assertTrue(tuple.getY32() == tupleArray[3]);
         assertTrue(tuple.getZ32() == tupleArray[4]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(DMatrixRMaj tupleMatrixToPack)
         T tuple = createRandomTuple(random);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(matrix);
         assertTrue(tuple.getX() == matrix.get(0, 0));
         assertTrue(tuple.getY() == matrix.get(1, 0));
         assertTrue(tuple.getZ() == matrix.get(2, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startRow, DMatrixRMaj tupleMatrixToPack)
         T tuple = createRandomTuple(random);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(2, matrix);
         assertTrue(tuple.getX() == matrix.get(2, 0));
         assertTrue(tuple.getY() == matrix.get(3, 0));
         assertTrue(tuple.getZ() == matrix.get(4, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startRow, int startColumn, DMatrixRMaj tupleMatrixToPack)
         T tuple = createRandomTuple(random);
         DMatrixRMaj matrix = new DMatrixRMaj(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(2, 4, matrix);
         assertTrue(tuple.getX() == matrix.get(2, 4));
         assertTrue(tuple.getY() == matrix.get(3, 4));
         assertTrue(tuple.getZ() == matrix.get(4, 4));
      }
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
         T tuple2 = createTuple(scalar * tuple1.getX(), scalar * tuple1.getY(), scalar * tuple1.getZ());
         double expectedNorm2 = scalar * norm1;
         double actualNorm2 = tuple2.norm();
         assertEquals(expectedNorm2, actualNorm2, Math.max(5.0, expectedNorm2) * getEpsilon());
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
         T tuple2 = createTuple(scalar * tuple1.getX(), scalar * tuple1.getY(), scalar * tuple1.getZ());
         double expectedNorm2 = scalar * norm1;
         double actualNorm2 = tuple2.normSquared();
         assertEquals(expectedNorm2, EuclidCoreTools.squareRoot(actualNorm2), Math.max(5.0, expectedNorm2) * getEpsilon());
      }
   }

   @Test
   public void testDot()
   {
      Random random = new Random(5461L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple1 = createRandomTuple(random);
         double scalar = EuclidCoreRandomTools.nextDouble(random, 2.0);
         tuple1 = createTuple(scalar * tuple1.getX(), scalar * tuple1.getY(), scalar * tuple1.getZ());
         Vector3DBasics axis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(tuple1), true);
         double angle = EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI);
         if (random.nextBoolean())
            angle = -angle;

         RotationMatrix rotationMatrix = new RotationMatrix(new AxisAngle(axis, angle));
         Vector3D rotated_tuple1 = new Vector3D();
         rotationMatrix.transform(tuple1, rotated_tuple1);
         scalar = EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0);
         T tuple2 = createTuple(scalar * rotated_tuple1.getX(), scalar * rotated_tuple1.getY(), scalar * rotated_tuple1.getZ());

         double expectedDot = tuple1.norm() * tuple2.norm() * EuclidCoreTools.cos(angle);
         double actualDot = tuple1.dot(tuple2);
         assertEquals(expectedDot,
                      actualDot,
                      Math.max(5.0, Math.abs(expectedDot)) * getEpsilon(),
                      "Iteration: " + i + ", error: " + Math.abs(expectedDot - actualDot));
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