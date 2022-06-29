package us.ihmc.euclid.shape.primitives;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

class Capsule3DTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   void testConstructors() throws Exception
   {
      Random random = new Random(67542);

      { // Empty constructor
         Capsule3D capsule3D = new Capsule3D();

         EuclidCoreTestTools.assertTuple3DIsSetToZero(capsule3D.getPosition());
         EuclidCoreTestTools.assertEquals(Axis3D.Z, capsule3D.getAxis(), EPSILON);
         assertEquals(1.0, capsule3D.getLength());
         assertEquals(0.5, capsule3D.getHalfLength());
         assertEquals(0.5, capsule3D.getRadius());
         EuclidCoreTestTools.assertEquals(new Point3D(0.0, 0.0, 0.5), capsule3D.getTopCenter(), EPSILON);
         EuclidCoreTestTools.assertEquals(new Point3D(0.0, 0.0, -0.5), capsule3D.getBottomCenter(), EPSILON);
      }

      { // Capsule3D(double length, double radius)
         for (int i = 0; i < ITERATIONS; i++)
         {
            double length = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
            double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
            Capsule3D capsule3D = new Capsule3D(length, radius);

            EuclidCoreTestTools.assertTuple3DIsSetToZero(capsule3D.getPosition());
            EuclidCoreTestTools.assertEquals(Axis3D.Z, capsule3D.getAxis(), EPSILON);
            assertEquals(length, capsule3D.getLength());
            assertEquals(0.5 * length, capsule3D.getHalfLength());
            assertEquals(radius, capsule3D.getRadius());
            EuclidCoreTestTools.assertEquals(new Point3D(0.0, 0.0, 0.5 * length), capsule3D.getTopCenter(), EPSILON);
            EuclidCoreTestTools.assertEquals(new Point3D(0.0, 0.0, -0.5 * length), capsule3D.getBottomCenter(), EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Capsule3D(-0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Capsule3D(1.0, -0.1));
      }

      { // Capsule3D(Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Point3D expectedTopCenter = EuclidCoreRandomTools.nextPoint3D(random);
            Point3D expectedBottomCenter = EuclidCoreRandomTools.nextPoint3D(random);
            double length = expectedTopCenter.distance(expectedBottomCenter);
            double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
            Point3D position = EuclidGeometryTools.averagePoint3Ds(expectedTopCenter, expectedBottomCenter);
            Vector3D axis = new Vector3D();
            axis.sub(expectedTopCenter, expectedBottomCenter);
            Capsule3D capsule3D = new Capsule3D(position, axis, length, radius);
            axis.normalize();

            EuclidCoreTestTools.assertEquals(position, capsule3D.getPosition(), EPSILON);
            EuclidCoreTestTools.assertEquals(axis, capsule3D.getAxis(), EPSILON);
            assertEquals(length, capsule3D.getLength());
            assertEquals(0.5 * length, capsule3D.getHalfLength());
            assertEquals(radius, capsule3D.getRadius());
            EuclidCoreTestTools.assertEquals(expectedTopCenter, capsule3D.getTopCenter(), EPSILON);
            EuclidCoreTestTools.assertEquals(expectedBottomCenter, capsule3D.getBottomCenter(), EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Capsule3D(new Point3D(), Axis3D.Z, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Capsule3D(new Point3D(), Axis3D.Z, 1.0, -0.1));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Capsule3D(Capsule3DReadOnly other)
         Capsule3D original = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D copy = new Capsule3D(original);

         EuclidCoreTestTools.assertEquals(original, copy, EPSILON);
      }
   }

   @Test
   void testSetToNaN() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         assertFalse(capsule3D.containsNaN());
         assertFalse(capsule3D.getPosition().containsNaN());
         assertFalse(capsule3D.getAxis().containsNaN());
         assertFalse(capsule3D.getTopCenter().containsNaN());
         assertFalse(capsule3D.getBottomCenter().containsNaN());
         assertFalse(Double.isNaN(capsule3D.getLength()));
         assertFalse(Double.isNaN(capsule3D.getHalfLength()));
         assertFalse(Double.isNaN(capsule3D.getRadius()));

         capsule3D.setToNaN();

         assertTrue(capsule3D.containsNaN());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(capsule3D.getPosition());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(capsule3D.getAxis());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(capsule3D.getTopCenter());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(capsule3D.getBottomCenter());
         assertTrue(Double.isNaN(capsule3D.getLength()));
         assertTrue(Double.isNaN(capsule3D.getHalfLength()));
         assertTrue(Double.isNaN(capsule3D.getRadius()));
      }
   }

   @Test
   void testSetToZero() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         assertFalse(new Point3D().epsilonEquals(capsule3D.getPosition(), EPSILON));
         assertFalse(new Point3D().epsilonEquals(capsule3D.getAxis(), EPSILON));
         assertFalse(new Point3D().epsilonEquals(capsule3D.getTopCenter(), EPSILON));
         assertFalse(new Point3D().epsilonEquals(capsule3D.getBottomCenter(), EPSILON));
         assertNotEquals(0.0, capsule3D.getLength());
         assertNotEquals(0.0, capsule3D.getHalfLength());
         assertNotEquals(0.0, capsule3D.getRadius());

         capsule3D.setToZero();

         EuclidCoreTestTools.assertTuple3DIsSetToZero(capsule3D.getPosition());
         EuclidCoreTestTools.assertEquals(Axis3D.Z, capsule3D.getAxis(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(capsule3D.getTopCenter());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(capsule3D.getBottomCenter());
         assertEquals(0.0, capsule3D.getLength());
         assertEquals(0.0, capsule3D.getHalfLength());
         assertEquals(0.0, capsule3D.getRadius());
      }
   }

   @Test
   void testSetters() throws Exception
   {
      Random random = new Random(5467457);

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Capsule3D other)
         Capsule3D expected = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D actual = EuclidShapeRandomTools.nextCapsule3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set(expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Capsule3DReadOnly other)
         Capsule3D expected = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D actual = EuclidShapeRandomTools.nextCapsule3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set((Capsule3DReadOnly) expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      { // set(Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Capsule3D expected = EuclidShapeRandomTools.nextCapsule3D(random);
            Capsule3D actual = EuclidShapeRandomTools.nextCapsule3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(expected.getPosition(), expected.getAxis(), expected.getLength(), expected.getRadius());
            EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Capsule3D().set(new Point3D(), Axis3D.Z, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Capsule3D().set(new Point3D(), Axis3D.Z, 1.0, -0.1));
      }
   }

   @Test
   void testSetSize() throws Exception
   {
      Random random = new Random(43905783);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double length = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         assertNotEquals(length, capsule3D.getLength());
         assertNotEquals(radius, capsule3D.getRadius());
         capsule3D.setSize(length, radius);
         assertEquals(length, capsule3D.getLength());
         assertEquals(radius, capsule3D.getRadius());
      }

      assertThrows(IllegalArgumentException.class, () -> new Capsule3D().setSize(-0.1, 1.0));
      assertThrows(IllegalArgumentException.class, () -> new Capsule3D().setSize(1.0, -0.1));
   }

   @Test
   void testIsPointInside() throws Exception
   {
      Random random = new Random(3465463);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside generated to be within a certain radius of the axis
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(capsule3D.getTopCenter(), capsule3D.getBottomCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 0.0, capsule3D.getRadius());
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);

         assertTrue(capsule3D.isPointInside(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside generated to be within a certain radius of the axis
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(capsule3D.getTopCenter(), capsule3D.getBottomCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 1.0, 3.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);

         assertFalse(capsule3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside contained is the top sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         double distanceOffTopCenter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0) * capsule3D.getRadius();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, distanceOffTopCenter);
         Point3D pointInside = new Point3D();
         pointInside.add(capsule3D.getTopCenter(), translation);

         assertTrue(capsule3D.isPointInside(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside contained is the bottom sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         double distanceOffBottomCenter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0) * capsule3D.getRadius();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, distanceOffBottomCenter);
         Point3D pointInside = new Point3D();
         pointInside.add(capsule3D.getBottomCenter(), translation);

         assertTrue(capsule3D.isPointInside(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest is the top sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         /*
          * We build the direction of the translation such that it points away from the center of the
          * capsule, so the resulting point is guaranteed to be closest to the top sphere.
          */
         Vector3D direction = new Vector3D();
         direction.interpolate(capsule3D.getAxis(), orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffTopCenter = EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopCenter, direction, capsule3D.getTopCenter());

         assertFalse(capsule3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest is the bottom sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         /*
          * We build the direction of the translation such that it points away from the center of the
          * capsule, so the resulting point is guaranteed to be closest to the bottom sphere.
          */
         Vector3D direction = new Vector3D(capsule3D.getAxis());
         direction.negate();
         direction.interpolate(orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffBottomCenter = EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffBottomCenter, direction, capsule3D.getBottomCenter());

         assertFalse(capsule3D.isPointInside(pointOutside));
      }
   }

   @Test
   void testEvaluatePoint3DCollision() throws Exception
   {
      Random random = new Random(675654);

      Point3D actualClosestPoint = new Point3D();
      Vector3D actualNormal = new Vector3D();
      Point3D expectedClosestPoint = new Point3D();
      Vector3D expectedNormal = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      { // Edge-case: Query is on the axis
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(capsule3D.getTopCenter(), capsule3D.getBottomCenter(), random.nextDouble());

         assertTrue(capsule3D.evaluatePoint3DCollision(pointOnAxis, actualClosestPoint, actualNormal));
         assertFalse(actualClosestPoint.containsNaN());
         assertFalse(actualNormal.containsNaN());

         capsule3D.evaluatePoint3DCollision(actualClosestPoint, expectedClosestPoint, expectedNormal);
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Edge-case: Query is at the top center
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         assertTrue(capsule3D.evaluatePoint3DCollision(capsule3D.getTopCenter(), actualClosestPoint, actualNormal));
         assertFalse(actualClosestPoint.containsNaN());
         assertFalse(actualNormal.containsNaN());

         capsule3D.evaluatePoint3DCollision(actualClosestPoint, expectedClosestPoint, expectedNormal);
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Edge-case: Query is at the bottom center
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         assertTrue(capsule3D.evaluatePoint3DCollision(capsule3D.getBottomCenter(), actualClosestPoint, actualNormal));
         assertFalse(actualClosestPoint.containsNaN());
         assertFalse(actualNormal.containsNaN());

         capsule3D.evaluatePoint3DCollision(actualClosestPoint, expectedClosestPoint, expectedNormal);
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside generated to be within a certain radius of the axis
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(capsule3D.getTopCenter(), capsule3D.getBottomCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 0.0, capsule3D.getRadius());
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         expectedClosestPoint.scaleAdd(capsule3D.getRadius(), orthogonalToAxis, pointOnAxis);
         expectedNormal.setAndNormalize(orthogonalToAxis);

         assertTrue(capsule3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside generated to be within a certain radius of the axis
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(capsule3D.getTopCenter(), capsule3D.getBottomCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 1.0, 3.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         expectedClosestPoint.scaleAdd(capsule3D.getRadius(), orthogonalToAxis, pointOnAxis);
         expectedNormal.setAndNormalize(orthogonalToAxis);

         assertFalse(capsule3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside contained in the top half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D();
         direction.interpolate(capsule3D.getAxis(), orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffTopCenter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0) * capsule3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffTopCenter, direction, capsule3D.getTopCenter());
         expectedClosestPoint.scaleAdd(capsule3D.getRadius(), direction, capsule3D.getTopCenter());
         expectedNormal.set(direction);

         assertTrue(capsule3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside contained in the bottom half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D(capsule3D.getAxis());
         direction.negate();
         direction.interpolate(orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffBottomCenter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0) * capsule3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffBottomCenter, direction, capsule3D.getBottomCenter());
         expectedClosestPoint.scaleAdd(capsule3D.getRadius(), direction, capsule3D.getBottomCenter());
         expectedNormal.set(direction);

         assertTrue(capsule3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to the top half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D();
         direction.interpolate(capsule3D.getAxis(), orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffTopCenter = EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopCenter, direction, capsule3D.getTopCenter());
         expectedClosestPoint.scaleAdd(capsule3D.getRadius(), direction, capsule3D.getTopCenter());
         expectedNormal.set(direction);

         assertFalse(capsule3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to the bottom half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D(capsule3D.getAxis());
         direction.negate();
         direction.interpolate(orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffBottomCenter = EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffBottomCenter, direction, capsule3D.getBottomCenter());
         expectedClosestPoint.scaleAdd(capsule3D.getRadius(), direction, capsule3D.getBottomCenter());
         expectedNormal.set(direction);

         assertFalse(capsule3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }
   }

   @Test
   void testApplyTransform()
   {
      Random random = new Random(346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3D actual = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D expected = new Capsule3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPosition().applyTransform(transform);
         expected.getAxis().applyTransform(transform);
         actual.applyTransform(transform);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3D actual = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D expected = new Capsule3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPosition().applyTransform(transform);
         expected.getAxis().applyTransform(transform);
         expected.getAxis().normalize();
         actual.applyTransform(transform);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   void testApplyInverseTransform()
   {
      Random random = new Random(346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3D actual = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D original = new Capsule3D(actual);
         Capsule3D expected = new Capsule3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPosition().applyInverseTransform(transform);
         expected.getAxis().applyInverseTransform(transform);
         expected.getAxis().normalize();
         actual.applyInverseTransform(transform);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertEquals(original, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3D actual = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D original = new Capsule3D(actual);
         Capsule3D expected = new Capsule3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPosition().applyInverseTransform(transform);
         expected.getAxis().applyInverseTransform(transform);
         expected.getAxis().normalize();
         actual.applyInverseTransform(transform);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertEquals(original, actual, EPSILON);
      }
   }

   @Test
   void testDistance() throws Exception
   {
      Random random = new Random(4305973);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside generated to be within a certain radius of the axis
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(capsule3D.getTopCenter(), capsule3D.getTopCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 0.0, capsule3D.getRadius());
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         assertEquals(0.0, capsule3D.distance(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside generated to be within a certain radius of the axis
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(capsule3D.getTopCenter(), capsule3D.getTopCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 1.0, 3.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         assertEquals(distanceOffAxis - capsule3D.getRadius(), capsule3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside contained in the top half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D();
         direction.interpolate(capsule3D.getAxis(), orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffTopCenter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0) * capsule3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffTopCenter, direction, capsule3D.getTopCenter());
         assertEquals(0.0, capsule3D.distance(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside contained in the bottom half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D(capsule3D.getAxis());
         direction.negate();
         direction.interpolate(orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffBottomCenter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0) * capsule3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffBottomCenter, direction, capsule3D.getBottomCenter());
         assertEquals(0.0, capsule3D.distance(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to the top half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D();
         direction.interpolate(capsule3D.getAxis(), orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffTopCenter = EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopCenter, direction, capsule3D.getTopCenter());
         assertEquals(distanceOffTopCenter - capsule3D.getRadius(), capsule3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to the bottom half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D(capsule3D.getAxis());
         direction.negate();
         direction.interpolate(orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffBottomCenter = EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffBottomCenter, direction, capsule3D.getBottomCenter());
         assertEquals(distanceOffBottomCenter - capsule3D.getRadius(), capsule3D.distance(pointOutside), EPSILON);
      }
   }

   @Test
   void testSignedDistance() throws Exception
   {
      Random random = new Random(94753);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside generated to be within a certain radius of the axis
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(capsule3D.getTopCenter(), capsule3D.getTopCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 0.0, capsule3D.getRadius());
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         assertEquals(distanceOffAxis - capsule3D.getRadius(), capsule3D.signedDistance(pointInside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside generated to be within a certain radius of the axis
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(capsule3D.getTopCenter(), capsule3D.getTopCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 1.0, 3.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         assertEquals(distanceOffAxis - capsule3D.getRadius(), capsule3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside contained in the top half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D();
         direction.interpolate(capsule3D.getAxis(), orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffTopCenter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0) * capsule3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffTopCenter, direction, capsule3D.getTopCenter());
         assertEquals(distanceOffTopCenter - capsule3D.getRadius(), capsule3D.signedDistance(pointInside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside contained in the bottom half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D(capsule3D.getAxis());
         direction.negate();
         direction.interpolate(orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffBottomCenter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0) * capsule3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffBottomCenter, direction, capsule3D.getBottomCenter());
         assertEquals(distanceOffBottomCenter - capsule3D.getRadius(), capsule3D.signedDistance(pointInside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to the top half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D();
         direction.interpolate(capsule3D.getAxis(), orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffTopCenter = EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopCenter, direction, capsule3D.getTopCenter());
         assertEquals(distanceOffTopCenter - capsule3D.getRadius(), capsule3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to the bottom half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D(capsule3D.getAxis());
         direction.negate();
         direction.interpolate(orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffBottomCenter = EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffBottomCenter, direction, capsule3D.getBottomCenter());
         assertEquals(distanceOffBottomCenter - capsule3D.getRadius(), capsule3D.signedDistance(pointOutside), EPSILON);
      }
   }

   @Test
   void testOrthogonalProjection() throws Exception
   {
      Random random = new Random(4867);

      Point3D expectedProjection = new Point3D();

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside generated to be within a certain radius of the axis
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(capsule3D.getTopCenter(), capsule3D.getTopCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 0.0, capsule3D.getRadius());
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         assertNull(capsule3D.orthogonalProjectionCopy(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside generated to be within a certain radius of the axis
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(capsule3D.getTopCenter(), capsule3D.getTopCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 1.0, 3.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         expectedProjection.scaleAdd(capsule3D.getRadius(), orthogonalToAxis, pointOnAxis);

         EuclidCoreTestTools.assertEquals(expectedProjection, capsule3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside contained in the top half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D();
         direction.interpolate(capsule3D.getAxis(), orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffTopCenter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0) * capsule3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffTopCenter, direction, capsule3D.getTopCenter());
         assertNull(capsule3D.orthogonalProjectionCopy(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside contained in the bottom half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D(capsule3D.getAxis());
         direction.negate();
         direction.interpolate(orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffBottomCenter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0) * capsule3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffBottomCenter, direction, capsule3D.getBottomCenter());
         assertNull(capsule3D.orthogonalProjectionCopy(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to the top half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D();
         direction.interpolate(capsule3D.getAxis(), orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffTopCenter = EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopCenter, direction, capsule3D.getTopCenter());
         expectedProjection.scaleAdd(capsule3D.getRadius(), direction, capsule3D.getTopCenter());

         EuclidCoreTestTools.assertEquals(expectedProjection, capsule3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to the bottom half-sphere
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsule3D.getAxis(), true);
         Vector3D direction = new Vector3D(capsule3D.getAxis());
         direction.negate();
         direction.interpolate(orthogonalToAxis, random.nextDouble());
         direction.normalize();

         double distanceOffBottomCenter = EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0) * capsule3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffBottomCenter, direction, capsule3D.getBottomCenter());
         expectedProjection.scaleAdd(capsule3D.getRadius(), direction, capsule3D.getBottomCenter());

         EuclidCoreTestTools.assertEquals(expectedProjection, capsule3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }
   }

   @Test
   void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(546161);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3D capsule = EuclidShapeRandomTools.nextCapsule3D(random);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly supportingVertex = capsule.getSupportingVertex(supportDirection);
         assertTrue(capsule.isPointInside(supportingVertex, EPSILON));

         Point3D supportingVertexTranslated = new Point3D();
         supportDirection.normalize();
         supportingVertexTranslated.scaleAdd(1.0e-6, supportDirection, supportingVertex);
         assertFalse(capsule.isPointInside(supportingVertexTranslated, EPSILON));

         Vector3D actualNormal = new Vector3D();
         capsule.evaluatePoint3DCollision(supportingVertexTranslated, new Point3D(), actualNormal);
         EuclidCoreTestTools.assertEquals(supportDirection, actualNormal, EPSILON);
      }
   }

   @Test
   void testGetBoundingBox() throws Exception
   {
      Random random = new Random(36342);

      for (int i = 0; i < ITERATIONS; i++)
      { // Using getSupportingVertex
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         BoundingBox3D expectedBoundingBox = new BoundingBox3D();
         expectedBoundingBox.setToNaN();
         Vector3D supportDirection = new Vector3D(Axis3D.X);
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis3D.Y);
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis3D.Z);
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));

         BoundingBox3DReadOnly actualBoundingBox = capsule3D.getBoundingBox();
         EuclidCoreTestTools.assertEquals(expectedBoundingBox, actualBoundingBox, EPSILON);
      }
   }
}
