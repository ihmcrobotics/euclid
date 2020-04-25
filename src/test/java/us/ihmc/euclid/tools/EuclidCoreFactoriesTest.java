package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.UnitVector2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidCoreFactoriesTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testNewLinkedPoint2DReadOnly()
   {
      Random random = new Random(5416);

      { // Test newLinkedPoint2DReadOnly(DoubleSupplier scaleSupplier, Tuple2DReadOnly originalTuple)
         double[] scale = new double[1];
         Point2D originalTuple = new Point2D();
         Point2DReadOnly actual = EuclidCoreFactories.newLinkedPoint2DReadOnly(() -> scale[0], originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.set(EuclidCoreRandomTools.nextPoint2D(random));
            scale[0] = random.nextDouble();

            Point2D expected = new Point2D();
            expected.setAndScale(scale[0], originalTuple);
            thoroughAssertionsTuple2D(expected, actual);
         }
      }

      { // Test newLinkedPoint2DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier)
         Point2D expected = new Point2D();
         Point2DReadOnly actual = EuclidCoreFactories.newLinkedPoint2DReadOnly(expected::getX, expected::getY);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidCoreRandomTools.nextPoint2D(random));
            thoroughAssertionsTuple2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewLinkedVector2DReadOnly()
   {
      Random random = new Random(5416);

      { // Test newLinkedVector2DReadOnly(DoubleSupplier scaleSupplier, Tuple2DReadOnly originalTuple)
         double[] scale = new double[1];
         Vector2D originalTuple = new Vector2D();
         Vector2DReadOnly actual = EuclidCoreFactories.newLinkedVector2DReadOnly(() -> scale[0], originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.set(EuclidCoreRandomTools.nextVector2D(random));
            scale[0] = random.nextDouble();

            Vector2D expected = new Vector2D();
            expected.setAndScale(scale[0], originalTuple);
            thoroughAssertionsTuple2D(expected, actual);
         }
      }

      { // Test newLinkedVector2DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier)
         Vector2D expected = new Vector2D();
         Vector2DReadOnly actual = EuclidCoreFactories.newLinkedVector2DReadOnly(expected::getX, expected::getY);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidCoreRandomTools.nextVector2D(random));

            thoroughAssertionsTuple2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewLinkedPoint3DReadOnly()
   {
      Random random = new Random(5416);

      { // Test newLinkedPoint3DReadOnly(DoubleSupplier scaleSupplier, Tuple3DReadOnly originalTuple)
         double[] scale = new double[1];
         Point3D originalTuple = new Point3D();
         Point3DReadOnly actual = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> scale[0], originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.set(EuclidCoreRandomTools.nextPoint3D(random));
            scale[0] = random.nextDouble();

            Point3D expected = new Point3D();
            expected.setAndScale(scale[0], originalTuple);
            thoroughAssertionsTuple3D(expected, actual);
         }
      }

      { // Test newLinkedPoint3DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier)
         Point3D expected = new Point3D();
         Point3DReadOnly actual = EuclidCoreFactories.newLinkedPoint3DReadOnly(expected::getX, expected::getY, expected::getZ);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidCoreRandomTools.nextPoint3D(random));

            thoroughAssertionsTuple3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewLinkedVector3DReadOnly()
   {
      Random random = new Random(5416);

      { // Test newLinkedVector3DReadOnly(DoubleSupplier scaleSupplier, Tuple3DReadOnly originalTuple)
         double[] scale = new double[1];
         Vector3D originalTuple = new Vector3D();
         Vector3DReadOnly actual = EuclidCoreFactories.newLinkedVector3DReadOnly(() -> scale[0], originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.set(EuclidCoreRandomTools.nextVector3D(random));
            scale[0] = random.nextDouble();

            Vector3D expected = new Vector3D();
            expected.setAndScale(scale[0], originalTuple);
            thoroughAssertionsTuple3D(expected, actual);
         }
      }

      { // Test newLinkedVector3DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier)
         Vector3D expected = new Vector3D();
         Vector3DReadOnly actual = EuclidCoreFactories.newLinkedVector3DReadOnly(expected::getX, expected::getY, expected::getZ);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidCoreRandomTools.nextVector3D(random));

            thoroughAssertionsTuple3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewNegativeLinkedPoint2D()
   {
      Random random = new Random(43);

      { // Test newNegativeLinkedPoint2D(Point2DReadOnly originalPoint)
         Point2D originalTuple = new Point2D();
         Point2DReadOnly actual = EuclidCoreFactories.newNegativeLinkedPoint2D(originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.set(EuclidCoreRandomTools.nextPoint2D(random));
            Point2D expected = new Point2D();
            expected.setAndNegate(originalTuple);

            thoroughAssertionsTuple2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewNegativeLinkedVector2D()
   {
      Random random = new Random(43);

      { // Test newNegativeLinkedVector2D(Vector2DReadOnly originalVector)
         Vector2D originalTuple = new Vector2D();
         Vector2DReadOnly actual = EuclidCoreFactories.newNegativeLinkedVector2D(originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.set(EuclidCoreRandomTools.nextVector2D(random));
            Vector2D expected = new Vector2D();
            expected.setAndNegate(originalTuple);

            thoroughAssertionsTuple2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewNegativeLinkedPoint3D()
   {
      Random random = new Random(43);

      { // Test newNegativeLinkedPoint3D(Point3DReadOnly originalPoint)
         Point3D originalTuple = new Point3D();
         Point3DReadOnly actual = EuclidCoreFactories.newNegativeLinkedPoint3D(originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.set(EuclidCoreRandomTools.nextPoint3D(random));
            Point3D expected = new Point3D();
            expected.setAndNegate(originalTuple);

            thoroughAssertionsTuple3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewNegativeLinkedVector3D()
   {
      Random random = new Random(43);

      { // Test newNegativeLinkedVector3D(Vector3DReadOnly originalVector)
         Vector3D originalTuple = new Vector3D();
         Vector3DReadOnly actual = EuclidCoreFactories.newNegativeLinkedVector3D(originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.set(EuclidCoreRandomTools.nextVector3D(random));
            Vector3D expected = new Vector3D();
            expected.setAndNegate(originalTuple);

            thoroughAssertionsTuple3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewNegativeLinkedUnitVector2D()
   {
      Random random = new Random(389);

      { // Test newNegativeLinkedUnitVector2D(UnitVector2DReadOnly originalUnitVector)
         UnitVector2D originalVector = EuclidCoreRandomTools.nextUnitVector2D(random);
         UnitVector2DReadOnly actual = EuclidCoreFactories.newNegativeLinkedUnitVector2D(originalVector);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalVector.set(EuclidCoreRandomTools.nextUnitVector2D(random));

            UnitVector2D expected = new UnitVector2D();
            expected.setAndNegate(originalVector);
            thoroughAssertionsTuple2D(expected, actual);
            assertEquals(expected.getRawX(), actual.getRawX());
            assertEquals(expected.getRawY(), actual.getRawY());

            assertEquals(originalVector.isDirty(), actual.isDirty());
            originalVector.setX(1.0);
            assertTrue(originalVector.isDirty());
            assertTrue(actual.isDirty());

            actual.getX(); // Trigger normalization
            assertFalse(actual.isDirty());
            assertFalse(originalVector.isDirty());
         }
      }
   }

   @Test
   public void testNewNegativeLinkedUnitVector3D()
   {
      Random random = new Random(389);

      { // Test newNegativeLinkedUnitVector3D(UnitVector3DReadOnly originalUnitVector)
         UnitVector3D originalVector = EuclidCoreRandomTools.nextUnitVector3D(random);
         UnitVector3DReadOnly actual = EuclidCoreFactories.newNegativeLinkedUnitVector3D(originalVector);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalVector.set(EuclidCoreRandomTools.nextUnitVector3D(random));

            UnitVector3D expected = new UnitVector3D();
            expected.setAndNegate(originalVector);
            thoroughAssertionsTuple3D(expected, actual);
            assertEquals(expected.getRawX(), actual.getRawX());
            assertEquals(expected.getRawY(), actual.getRawY());
            assertEquals(expected.getRawZ(), actual.getRawZ());

            assertEquals(originalVector.isDirty(), actual.isDirty());
            originalVector.setX(1.0);
            assertTrue(originalVector.isDirty());
            assertTrue(actual.isDirty());

            actual.getX(); // Trigger normalization
            assertFalse(actual.isDirty());
            assertFalse(originalVector.isDirty());
         }
      }
   }

   @Test
   public void testNewTransposeLinkedMatrix3DReadOnly()
   {
      Random random = new Random(43634677);

      { // Test newTransposeLinkedMatrix3DReadOnly(Matrix3DBasics original)
         Matrix3D originalMatrix = new Matrix3D();
         Matrix3DReadOnly actual = EuclidCoreFactories.newTransposeLinkedMatrix3DReadOnly(originalMatrix);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalMatrix.set(EuclidCoreRandomTools.nextMatrix3D(random));
            Matrix3D expected = new Matrix3D();
            expected.setAndTranspose(originalMatrix);
            thoroughAssertionsMatrix3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewTildeLinkedMatrix3DReadOnly()
   {
      Random random = new Random(43634677);

      { // Test newTildeLinkedMatrix3DReadOnly(Matrix3DBasics original)
         Tuple3DBasics originalTuple = new Point3D();
         Matrix3DReadOnly actual = EuclidCoreFactories.newTildeLinkedMatrix3DReadOnly(originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.set(EuclidCoreRandomTools.nextPoint3D(random));
            Matrix3D expected = new Matrix3D();
            expected.setToTildeForm(originalTuple);
            thoroughAssertionsMatrix3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewDiagonalLinkedMatrix3DReadOnly()
   {
      Random random = new Random(43634677);

      { // Test newDiagonalLinkedMatrix3DReadOnly(Matrix3DBasics original)
         Tuple3DBasics originalTuple = new Point3D();
         Matrix3DReadOnly actual = EuclidCoreFactories.newDiagonalLinkedMatrix3DReadOnly(originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.set(EuclidCoreRandomTools.nextPoint3D(random));
            Matrix3D expected = new Matrix3D();
            expected.setToDiagonal(originalTuple);
            thoroughAssertionsMatrix3D(expected, actual);
         }
      }
   }

   public static void thoroughAssertionsTuple2D(Tuple2DReadOnly expected, Tuple2DReadOnly actual)
   {
      assertObjectMethods(expected, actual);
      EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(actual, expected, EPSILON);
   }

   public static void thoroughAssertionsTuple3D(Tuple3DReadOnly expected, Tuple3DReadOnly actual)
   {
      assertObjectMethods(expected, actual);
      EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(actual, expected, EPSILON);
   }

   public static void thoroughAssertionsMatrix3D(Matrix3DReadOnly expected, Matrix3DReadOnly actual)
   {
      assertObjectMethods(expected, actual);
      EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      EuclidCoreTestTools.assertMatrix3DEquals(actual, expected, EPSILON);
   }

   public static void assertObjectMethods(Object expected, Object actual)
   {
      assertEquals(expected.hashCode(), actual.hashCode());
      assertEquals(expected, actual);
      assertEquals(actual, expected);
      assertEquals(expected.toString(), actual.toString());
   }
}
