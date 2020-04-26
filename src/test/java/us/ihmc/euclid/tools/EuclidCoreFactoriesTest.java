package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import java.util.function.Consumer;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.UnitVector2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
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

   @Test
   public void testNewObservablePoint2DReadOnly()
   {
      Random random = new Random(45);

      { // Test simple update operation.
         Point2D expected = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D source = new Point2D();
         Consumer<Axis2D> valueAccessedListener = axis -> source.setElement(axis, expected.getElement(axis));
         Point2DReadOnly observable = EuclidCoreFactories.newObservablePoint2DReadOnly(valueAccessedListener, source);

         EuclidCoreTestTools.assertTuple2DIsSetToZero(source);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         thoroughAssertionsTuple2D(expected, observable);
      }

      { // Test transform operation.
         Point2D expected = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D source = new Point2D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis2D> valueAccessedListener = axis -> transform.transform(source, false);
         Point2DReadOnly observable = EuclidCoreFactories.newObservablePoint2DReadOnly(valueAccessedListener, source);

         transform.transform(expected, false);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected, false);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
      }
   }

   @Test
   public void testNewObservableVector2DReadOnly()
   {
      Random random = new Random(45);

      { // Test simple update operation.
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D source = new Vector2D();
         Consumer<Axis2D> valueAccessedListener = axis -> source.setElement(axis, expected.getElement(axis));
         Vector2DReadOnly observable = EuclidCoreFactories.newObservableVector2DReadOnly(valueAccessedListener, source);

         EuclidCoreTestTools.assertTuple2DIsSetToZero(source);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         thoroughAssertionsTuple2D(expected, observable);
      }

      { // Test transform operation.
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D source = new Vector2D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis2D> valueAccessedListener = axis -> transform.transform(source, false);
         Vector2DReadOnly observable = EuclidCoreFactories.newObservableVector2DReadOnly(valueAccessedListener, source);

         transform.transform(expected, false);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected, false);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
      }
   }

   @Test
   public void testNewObservablePoint3DReadOnly()
   {
      Random random = new Random(45);

      { // Test simple update operation.
         Point3D expected = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D source = new Point3D();
         Consumer<Axis3D> valueAccessedListener = axis -> source.setElement(axis, expected.getElement(axis));
         Point3DReadOnly observable = EuclidCoreFactories.newObservablePoint3DReadOnly(valueAccessedListener, source);

         EuclidCoreTestTools.assertTuple3DIsSetToZero(source);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         assertEquals(expected.getZ(), observable.getZ());
         assertEquals(expected.getZ(), source.getZ());
         thoroughAssertionsTuple3D(expected, observable);
      }

      { // Test transform operation.
         Point3D expected = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D source = new Point3D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis3D> valueAccessedListener = axis -> transform.transform(source);
         Point3DReadOnly observable = EuclidCoreFactories.newObservablePoint3DReadOnly(valueAccessedListener, source);

         transform.transform(expected);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         transform.transform(expected);
         assertEquals(expected.getZ(), observable.getZ());
         assertEquals(expected.getZ(), source.getZ());
      }
   }

   @Test
   public void testNewObservableUnitVector2DReadOnly()
   {
      Random random = new Random(45);

      { // Test simple update operation.
         UnitVector2D expected = EuclidCoreRandomTools.nextUnitVector2D(random);
         UnitVector2D source = new UnitVector2D();
         Consumer<Axis2D> valueAccessedListener = axis -> source.set(expected);
         UnitVector2DReadOnly observable = EuclidCoreFactories.newObservableUnitVector2DReadOnly(valueAccessedListener, source);

         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         thoroughAssertionsTuple2D(expected, observable);
      }

      { // Test transform operation.
         UnitVector2D expected = EuclidCoreRandomTools.nextUnitVector2D(random);
         UnitVector2D source = new UnitVector2D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis2D> valueAccessedListener = axis -> transform.transform(source, false);
         UnitVector2DReadOnly observable = EuclidCoreFactories.newObservableUnitVector2DReadOnly(valueAccessedListener, source);

         transform.transform(expected, false);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected, false);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
      }
   }

   @Test
   public void testNewObservableVector3DReadOnly()
   {
      Random random = new Random(45);

      { // Test simple update operation.
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D source = new Vector3D();
         Consumer<Axis3D> valueAccessedListener = axis -> source.setElement(axis, expected.getElement(axis));
         Vector3DReadOnly observable = EuclidCoreFactories.newObservableVector3DReadOnly(valueAccessedListener, source);

         EuclidCoreTestTools.assertTuple3DIsSetToZero(source);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         assertEquals(expected.getZ(), observable.getZ());
         assertEquals(expected.getZ(), source.getZ());
         thoroughAssertionsTuple3D(expected, observable);
      }

      { // Test transform operation.
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D source = new Vector3D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis3D> valueAccessedListener = axis -> transform.transform(source);
         Vector3DReadOnly observable = EuclidCoreFactories.newObservableVector3DReadOnly(valueAccessedListener, source);

         transform.transform(expected);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         transform.transform(expected);
         assertEquals(expected.getZ(), observable.getZ());
         assertEquals(expected.getZ(), source.getZ());
      }
   }

   @Test
   public void testNewObservableUnitVector3DReadOnly()
   {
      Random random = new Random(45);

      { // Test simple update operation.
         UnitVector3D expected = EuclidCoreRandomTools.nextUnitVector3D(random);
         UnitVector3D source = new UnitVector3D();
         Consumer<Axis3D> valueAccessedListener = axis -> source.set(expected);
         UnitVector3DReadOnly observable = EuclidCoreFactories.newObservableUnitVector3DReadOnly(valueAccessedListener, source);

         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         assertEquals(expected.getZ(), observable.getZ());
         assertEquals(expected.getZ(), source.getZ());
         thoroughAssertionsTuple3D(expected, observable);
      }

      { // Test transform operation.
         UnitVector3D expected = EuclidCoreRandomTools.nextUnitVector3D(random);
         UnitVector3D source = new UnitVector3D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis3D> valueAccessedListener = axis -> transform.transform(source);
         UnitVector3DReadOnly observable = EuclidCoreFactories.newObservableUnitVector3DReadOnly(valueAccessedListener, source);

         transform.transform(expected);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         transform.transform(expected);
         assertEquals(expected.getZ(), observable.getZ());
         assertEquals(expected.getZ(), source.getZ());
      }
   }

   @Test
   public void testNewObservablePoint2DBasics()
   {
      Random random = new Random(4367);

      { // Test the link property with the source
         Point2D expected = new Point2D();
         Point2DBasics actual = EuclidCoreFactories.newObservablePoint2DBasics(null, null, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidCoreRandomTools.nextPoint2D(random));
            thoroughAssertionsTuple2D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextPoint2D(random));
            thoroughAssertionsTuple2D(expected, actual);
         }
      }

      { // Test with simple notification flags
         boolean[] changeTrace = {false, false};
         boolean[] accessTrace = {false, false};
         Point2DBasics source = new Point2D();
         Point2DBasics observable = EuclidCoreFactories.newObservablePoint2DBasics((axis, newValue) -> changeTrace[axis.ordinal()] = true,
                                                                                   axis -> accessTrace[axis.ordinal()] = true,
                                                                                   source);

         assertAllFalses(changeTrace);
         assertAllFalses(accessTrace);

         for (int i = 0; i < 2; i++)
         {
            double value = random.nextDouble();
            observable.setElement(i, value);
            assertTrue(changeTrace[i]);
            changeTrace[i] = false;
            observable.setElement(i, value);
            assertFalse(changeTrace[i]);
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);

            observable.getElement(i);
            assertTrue(accessTrace[i]);
            accessTrace[i] = false;
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);
         }
      }

      { // Test transform operation.
         Point2D expected = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D source = new Point2D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis2D> valueAccessedListener = axis -> transform.transform(source, false);
         Point2DBasics observable = EuclidCoreFactories.newObservablePoint2DBasics(null, valueAccessedListener, source);

         transform.transform(expected, false);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected, false);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
      }
   }

   @Test
   public void testNewObservableVector2DBasics()
   {
      Random random = new Random(4367);

      { // Test the link property with the source
         Vector2D expected = new Vector2D();
         Vector2DBasics actual = EuclidCoreFactories.newObservableVector2DBasics(null, null, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidCoreRandomTools.nextVector2D(random));
            thoroughAssertionsTuple2D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextVector2D(random));
            thoroughAssertionsTuple2D(expected, actual);
         }
      }

      { // Test with simple notification flags
         boolean[] changeTrace = {false, false};
         boolean[] accessTrace = {false, false};
         Vector2DBasics source = new Vector2D();
         Vector2DBasics observable = EuclidCoreFactories.newObservableVector2DBasics((axis, newValue) -> changeTrace[axis.ordinal()] = true,
                                                                                     axis -> accessTrace[axis.ordinal()] = true,
                                                                                     source);

         assertAllFalses(changeTrace);
         assertAllFalses(accessTrace);

         for (int i = 0; i < 2; i++)
         {
            double value = random.nextDouble();
            observable.setElement(i, value);
            assertTrue(changeTrace[i]);
            changeTrace[i] = false;
            observable.setElement(i, value);
            assertFalse(changeTrace[i]);
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);

            observable.getElement(i);
            assertTrue(accessTrace[i]);
            accessTrace[i] = false;
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);
         }
      }

      { // Test transform operation.
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D source = new Vector2D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis2D> valueAccessedListener = axis -> transform.transform(source, false);
         Vector2DBasics observable = EuclidCoreFactories.newObservableVector2DBasics(null, valueAccessedListener, source);

         transform.transform(expected, false);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected, false);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
      }
   }

   @Test
   public void testNewObservablePoint3DBasics()
   {
      Random random = new Random(4367);

      { // Test the link property with the source
         Point3D expected = new Point3D();
         Point3DBasics actual = EuclidCoreFactories.newObservablePoint3DBasics(null, null, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidCoreRandomTools.nextPoint3D(random));
            thoroughAssertionsTuple3D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextPoint3D(random));
            thoroughAssertionsTuple3D(expected, actual);
         }
      }

      { // Test with simple notification flags
         boolean[] changeTrace = {false, false, false};
         boolean[] accessTrace = {false, false, false};
         Point3DBasics source = new Point3D();
         Point3DBasics observable = EuclidCoreFactories.newObservablePoint3DBasics((axis, newValue) -> changeTrace[axis.ordinal()] = true,
                                                                                   axis -> accessTrace[axis.ordinal()] = true,
                                                                                   source);

         assertAllFalses(changeTrace);
         assertAllFalses(accessTrace);

         for (int i = 0; i < 3; i++)
         {
            double value = random.nextDouble();
            observable.setElement(i, value);
            assertTrue(changeTrace[i]);
            changeTrace[i] = false;
            observable.setElement(i, value);
            assertFalse(changeTrace[i]);
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);

            observable.getElement(i);
            assertTrue(accessTrace[i]);
            accessTrace[i] = false;
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);
         }
      }

      { // Test transform operation.
         Point3D expected = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D source = new Point3D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis3D> valueAccessedListener = axis -> transform.transform(source);
         Point3DBasics observable = EuclidCoreFactories.newObservablePoint3DBasics(null, valueAccessedListener, source);

         transform.transform(expected);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         transform.transform(expected);
         assertEquals(expected.getZ(), observable.getZ());
         assertEquals(expected.getZ(), source.getZ());
      }
   }

   @Test
   public void testNewObservableVector3DBasics()
   {
      Random random = new Random(4367);

      { // Test the link property with the source
         Vector3D expected = new Vector3D();
         Vector3DBasics actual = EuclidCoreFactories.newObservableVector3DBasics(null, null, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidCoreRandomTools.nextVector3D(random));
            thoroughAssertionsTuple3D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextVector3D(random));
            thoroughAssertionsTuple3D(expected, actual);
         }
      }

      { // Test with simple notification flags
         boolean[] changeTrace = {false, false, false};
         boolean[] accessTrace = {false, false, false};
         Vector3DBasics source = new Vector3D();
         Vector3DBasics observable = EuclidCoreFactories.newObservableVector3DBasics((axis, newValue) -> changeTrace[axis.ordinal()] = true,
                                                                                     axis -> accessTrace[axis.ordinal()] = true,
                                                                                     source);

         assertAllFalses(changeTrace);
         assertAllFalses(accessTrace);

         for (int i = 0; i < 3; i++)
         {
            double value = random.nextDouble();
            observable.setElement(i, value);
            assertTrue(changeTrace[i]);
            changeTrace[i] = false;
            observable.setElement(i, value);
            assertFalse(changeTrace[i]);
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);

            observable.getElement(i);
            assertTrue(accessTrace[i]);
            accessTrace[i] = false;
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);
         }
      }

      { // Test transform operation.
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D source = new Vector3D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis3D> valueAccessedListener = axis -> transform.transform(source);
         Vector3DBasics observable = EuclidCoreFactories.newObservableVector3DBasics(null, valueAccessedListener, source);

         transform.transform(expected);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         transform.transform(expected);
         assertEquals(expected.getZ(), observable.getZ());
         assertEquals(expected.getZ(), source.getZ());
      }
   }

   @Test
   public void testNewObservableUnitVector2DBasics()
   {
      Random random = new Random(4367);

      { // Test the link property with the source
         UnitVector2D expected = new UnitVector2D();
         UnitVector2DBasics actual = EuclidCoreFactories.newObservableUnitVector2DBasics(null, null, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidCoreRandomTools.nextUnitVector2D(random));
            thoroughAssertionsTuple2D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextUnitVector2D(random));
            thoroughAssertionsTuple2D(expected, actual);
         }
      }

      { // Test with simple notification flags
         boolean[] changeTrace = {false, false};
         boolean[] accessTrace = {false, false};
         UnitVector2DBasics source = new UnitVector2D();
         UnitVector2DBasics observable = EuclidCoreFactories.newObservableUnitVector2DBasics((axis, newValue) -> changeTrace[axis.ordinal()] = true,
                                                                                             axis -> accessTrace[axis.ordinal()] = true,
                                                                                             source);

         assertAllFalses(changeTrace);
         assertAllFalses(accessTrace);

         for (int i = 0; i < 2; i++)
         {
            double value = random.nextDouble();
            observable.setElement(i, value);
            assertTrue(changeTrace[i]);
            changeTrace[i] = false;
            observable.setElement(i, value);
            assertFalse(changeTrace[i]);
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);

            observable.getElement(i);
            assertTrue(accessTrace[i]);
            accessTrace[i] = false;
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);
         }
      }

      { // Test transform operation.
         UnitVector2D expected = EuclidCoreRandomTools.nextUnitVector2D(random);
         UnitVector2D source = new UnitVector2D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis2D> valueAccessedListener = axis -> transform.transform(source, false);
         UnitVector2DBasics observable = EuclidCoreFactories.newObservableUnitVector2DBasics(null, valueAccessedListener, source);

         transform.transform(expected, false);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected, false);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
      }
   }

   @Test
   public void testNewObservableUnitVector3DBasics()
   {
      Random random = new Random(4367);

      { // Test the link property with the source
         UnitVector3D expected = new UnitVector3D();
         UnitVector3DBasics actual = EuclidCoreFactories.newObservableUnitVector3DBasics(null, null, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidCoreRandomTools.nextUnitVector3D(random));
            thoroughAssertionsTuple3D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextUnitVector3D(random));
            thoroughAssertionsTuple3D(expected, actual);
         }
      }

      { // Test with simple notification flags
         boolean[] changeTrace = {false, false, false};
         boolean[] accessTrace = {false, false, false};
         UnitVector3DBasics source = new UnitVector3D();
         UnitVector3DBasics observable = EuclidCoreFactories.newObservableUnitVector3DBasics((axis, newValue) -> changeTrace[axis.ordinal()] = true,
                                                                                             axis -> accessTrace[axis.ordinal()] = true,
                                                                                             source);

         assertAllFalses(changeTrace);
         assertAllFalses(accessTrace);

         for (int i = 0; i < 3; i++)
         {
            double value = random.nextDouble();
            observable.setElement(i, value);
            assertTrue(changeTrace[i]);
            changeTrace[i] = false;
            observable.setElement(i, value);
            assertFalse(changeTrace[i]);
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);

            observable.getElement(i);
            assertTrue(accessTrace[i]);
            accessTrace[i] = false;
            assertAllFalses(changeTrace);
            assertAllFalses(accessTrace);
         }
      }

      { // Test transform operation.
         UnitVector3D expected = EuclidCoreRandomTools.nextUnitVector3D(random);
         UnitVector3D source = new UnitVector3D(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Consumer<Axis3D> valueAccessedListener = axis -> transform.transform(source);
         UnitVector3DBasics observable = EuclidCoreFactories.newObservableUnitVector3DBasics(null, valueAccessedListener, source);

         transform.transform(expected);
         assertEquals(expected.getX(), observable.getX());
         assertEquals(expected.getX(), source.getX());
         transform.transform(expected);
         assertEquals(expected.getY(), observable.getY());
         assertEquals(expected.getY(), source.getY());
         transform.transform(expected);
         assertEquals(expected.getZ(), observable.getZ());
         assertEquals(expected.getZ(), source.getZ());
      }
   }

   public static void assertAllFalses(boolean[] array)
   {
      for (int i = 0; i < array.length; i++)
         assertFalse(array[i]);
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
