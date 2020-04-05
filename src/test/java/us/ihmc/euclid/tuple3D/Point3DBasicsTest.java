package us.ihmc.euclid.tuple3D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public abstract class Point3DBasicsTest<T extends Point3DBasics> extends Tuple3DBasicsTest<T>
{
   // Read-only part
   @Test
   public void testDistance()
   {
      Random random = new Random(654135L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(expectedDistance);
         T p1 = createRandomTuple(random);
         T p2 = createTuple(p1.getX() + translation.getX(), p1.getY() + translation.getY(), p1.getZ() + translation.getZ());
         double actualDistance = p1.distance(p2);
         assertEquals(expectedDistance, actualDistance, getEpsilon());
      }
   }

   @Test
   public void testDistanceSquared()
   {
      Random random = new Random(654135L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double expectedDistanceSquared = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(EuclidCoreTools.squareRoot(expectedDistanceSquared));
         T p1 = createRandomTuple(random);
         T p2 = createTuple(p1.getX() + translation.getX(), p1.getY() + translation.getY(), p1.getZ() + translation.getZ());
         double actualDistanceSquared = p1.distanceSquared(p2);
         assertEquals(expectedDistanceSquared, actualDistanceSquared, getEpsilon());
      }
   }

   @Test
   public void testDistanceXY()
   {
      Random random = new Random(654135L);

      for (int i = 0; i < ITERATIONS; i++)
      { // With other point 3D
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         translation.setZ(0.0);
         translation.normalize();
         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(expectedDistance);
         T p1 = createRandomTuple(random);
         T p2 = createTuple(p1.getX() + translation.getX(), p1.getY() + translation.getY(), p1.getZ() + random.nextDouble());
         double actualDistance = p1.distanceXY(p2);
         assertEquals(expectedDistance, actualDistance, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // With point 2D
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         translation.setZ(0.0);
         translation.normalize();
         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(expectedDistance);
         T p1 = createRandomTuple(random);
         Point2D p2 = new Point2D(p1.getX() + translation.getX(), p1.getY() + translation.getY());
         double actualDistance = p1.distanceXY(p2);
         assertEquals(expectedDistance, actualDistance, getEpsilon());
      }
   }

   @Test
   public void testDistanceXYSquared()
   {
      Random random = new Random(65415L);

      for (int i = 0; i < ITERATIONS; i++)
      { // With other point 3D
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random, 1.0, 2.0);
         translation.setZ(0.0);
         translation.normalize();
         double expectedDistanceSquared = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
         translation.scale(EuclidCoreTools.squareRoot(expectedDistanceSquared));
         T p1 = createRandomTuple(random);
         T p2 = createTuple(p1.getX() + translation.getX(), p1.getY() + translation.getY(), p1.getZ() + random.nextDouble());
         double actualDistance = p1.distanceXYSquared(p2);
         assertEquals(expectedDistanceSquared, actualDistance, 10.0 * getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // With point 2D
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random, 1.0, 2.0);
         translation.setZ(0.0);
         translation.normalize();
         double expectedDistanceSquared = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
         translation.scale(EuclidCoreTools.squareRoot(expectedDistanceSquared));
         T p1 = createRandomTuple(random);
         Point2D p2 = new Point2D(p1.getX() + translation.getX(), p1.getY() + translation.getY());
         double actualDistance = p1.distanceXYSquared(p2);
         assertEquals(expectedDistanceSquared, actualDistance, getEpsilon());
      }
   }

   @Test
   public void testDistanceFromOrigin() throws Exception
   {
      Random random = new Random(654135L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(expectedDistance);
         T p = createTuple(translation.getX(), translation.getY(), translation.getZ());
         double actualDistance = p.distanceFromOrigin();
         assertEquals(expectedDistance, actualDistance, getEpsilon());
      }
   }

   @Test
   public void testDistanceFromOriginSquared() throws Exception
   {
      Random random = new Random(654135L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double expectedDistanceSquared = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(EuclidCoreTools.squareRoot(expectedDistanceSquared));
         T p = createTuple(translation.getX(), translation.getY(), translation.getZ());
         double actualDistance = p.distanceFromOriginSquared();
         assertEquals(expectedDistanceSquared, actualDistance, getEpsilon());
      }
   }

   // Basics part
   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testApplyInverseTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = new AffineTransform(new RotationScaleMatrix(EuclidCoreRandomTools.nextRotationMatrix(random),
                                                                                 EuclidCoreRandomTools.nextPoint3D(random, 1.0, 10.0)),
                                                         EuclidCoreRandomTools.nextPoint3D(random, 10.0));

         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Point3DBasics pointA;
      Point3DBasics pointB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         double epsilon = random.nextDouble();
         pointA = EuclidCoreRandomTools.nextPoint3D(random);
         pointB = EuclidCoreRandomTools.nextPoint3D(random);

         if (pointA.epsilonEquals(pointB, getEpsilon()))
         {
            assertTrue(pointA.geometricallyEquals(pointB, EuclidCoreTools.squareRoot(3) * getEpsilon()));
         }
         else
         {
            if (EuclidCoreTools.norm(pointA.getX() - pointB.getX(), pointA.getY() - pointB.getY(), pointA.getZ() - pointB.getZ()) <= getEpsilon())
            {
               assertTrue(pointA.geometricallyEquals(pointB, getEpsilon()));
            }
            else
            {
               assertFalse(pointA.geometricallyEquals(pointB, getEpsilon()));
            }
         }

         pointA = EuclidCoreRandomTools.nextPoint3D(random);

         pointB = new Point3D(pointA);
         Vector3D perturb = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon);
         pointB.add(perturb);

         assertTrue(pointA.geometricallyEquals(pointB, epsilon));

         pointB = new Point3D(pointA);
         perturb = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon);
         pointB.add(perturb);

         assertFalse(pointA.geometricallyEquals(pointB, epsilon));
      }
   }
}
