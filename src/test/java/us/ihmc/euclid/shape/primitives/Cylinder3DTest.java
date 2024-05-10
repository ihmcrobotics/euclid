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
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public class Cylinder3DTest
{
   private static final double EPSILON = 1e-12;

   @Test
   void testConstructors() throws Exception
   {
      Random random = new Random(67542);

      { // Empty constructor
         Cylinder3D cylinder3D = new Cylinder3D();

         EuclidCoreTestTools.assertTuple3DIsSetToZero(cylinder3D.getPosition());
         EuclidCoreTestTools.assertEquals(Axis3D.Z, cylinder3D.getAxis(), EPSILON);
         assertEquals(1.0, cylinder3D.getLength());
         assertEquals(0.5, cylinder3D.getHalfLength());
         assertEquals(0.5, cylinder3D.getRadius());
         EuclidCoreTestTools.assertEquals(new Point3D(0.0, 0.0, 0.5), cylinder3D.getTopCenter(), EPSILON);
         EuclidCoreTestTools.assertEquals(new Point3D(0.0, 0.0, -0.5), cylinder3D.getBottomCenter(), EPSILON);
      }

      { // Cylinder3D(double length, double radius)
         for (int i = 0; i < ITERATIONS; i++)
         {
            double length = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
            double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
            Cylinder3D cylinder3D = new Cylinder3D(length, radius);

            EuclidCoreTestTools.assertTuple3DIsSetToZero(cylinder3D.getPosition());
            EuclidCoreTestTools.assertEquals(Axis3D.Z, cylinder3D.getAxis(), EPSILON);
            assertEquals(length, cylinder3D.getLength());
            assertEquals(0.5 * length, cylinder3D.getHalfLength());
            assertEquals(radius, cylinder3D.getRadius());
            EuclidCoreTestTools.assertEquals(new Point3D(0.0, 0.0, 0.5 * length), cylinder3D.getTopCenter(), EPSILON);
            EuclidCoreTestTools.assertEquals(new Point3D(0.0, 0.0, -0.5 * length), cylinder3D.getBottomCenter(), EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Cylinder3D(-0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Cylinder3D(1.0, -0.1));
      }

      { // Cylinder3D(Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Point3D expectedTopCenter = EuclidCoreRandomTools.nextPoint3D(random);
            Point3D expectedBottomCenter = EuclidCoreRandomTools.nextPoint3D(random);
            double length = expectedTopCenter.distance(expectedBottomCenter);
            double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
            Point3D position = EuclidGeometryTools.averagePoint3Ds(expectedTopCenter, expectedBottomCenter);
            Vector3D axis = new Vector3D();
            axis.sub(expectedTopCenter, expectedBottomCenter);
            Cylinder3D cylinder3D = new Cylinder3D(position, axis, length, radius);
            axis.normalize();

            EuclidCoreTestTools.assertEquals(position, cylinder3D.getPosition(), EPSILON);
            EuclidCoreTestTools.assertEquals(axis, cylinder3D.getAxis(), EPSILON);
            assertEquals(length, cylinder3D.getLength());
            assertEquals(0.5 * length, cylinder3D.getHalfLength());
            assertEquals(radius, cylinder3D.getRadius());
            EuclidCoreTestTools.assertEquals(expectedTopCenter, cylinder3D.getTopCenter(), EPSILON);
            EuclidCoreTestTools.assertEquals(expectedBottomCenter, cylinder3D.getBottomCenter(), EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Cylinder3D(new Point3D(), Axis3D.Z, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Cylinder3D(new Point3D(), Axis3D.Z, 1.0, -0.1));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Cylinder3D(Cylinder3DReadOnly other)
         Cylinder3D original = EuclidShapeRandomTools.nextCylinder3D(random);
         Cylinder3D copy = new Cylinder3D(original);

         EuclidCoreTestTools.assertEquals(original, copy, EPSILON);
      }
   }

   @Test
   void testSetToNaN() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         assertFalse(cylinder3D.containsNaN());
         assertFalse(cylinder3D.getPosition().containsNaN());
         assertFalse(cylinder3D.getAxis().containsNaN());
         assertFalse(cylinder3D.getTopCenter().containsNaN());
         assertFalse(cylinder3D.getBottomCenter().containsNaN());
         assertFalse(Double.isNaN(cylinder3D.getLength()));
         assertFalse(Double.isNaN(cylinder3D.getHalfLength()));
         assertFalse(Double.isNaN(cylinder3D.getRadius()));

         cylinder3D.setToNaN();

         assertTrue(cylinder3D.containsNaN());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(cylinder3D.getPosition());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(cylinder3D.getAxis());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(cylinder3D.getTopCenter());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(cylinder3D.getBottomCenter());
         assertTrue(Double.isNaN(cylinder3D.getLength()));
         assertTrue(Double.isNaN(cylinder3D.getHalfLength()));
         assertTrue(Double.isNaN(cylinder3D.getRadius()));
      }
   }

   @Test
   void testSetToZero() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         assertFalse(new Point3D().epsilonEquals(cylinder3D.getPosition(), EPSILON));
         assertFalse(new Point3D().epsilonEquals(cylinder3D.getAxis(), EPSILON));
         assertFalse(new Point3D().epsilonEquals(cylinder3D.getTopCenter(), EPSILON));
         assertFalse(new Point3D().epsilonEquals(cylinder3D.getBottomCenter(), EPSILON));
         assertNotEquals(0.0, cylinder3D.getLength());
         assertNotEquals(0.0, cylinder3D.getHalfLength());
         assertNotEquals(0.0, cylinder3D.getRadius());

         cylinder3D.setToZero();

         EuclidCoreTestTools.assertTuple3DIsSetToZero(cylinder3D.getPosition());
         EuclidCoreTestTools.assertEquals(Axis3D.Z, cylinder3D.getAxis(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(cylinder3D.getTopCenter());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(cylinder3D.getBottomCenter());
         assertEquals(0.0, cylinder3D.getLength());
         assertEquals(0.0, cylinder3D.getHalfLength());
         assertEquals(0.0, cylinder3D.getRadius());
      }
   }

   @Test
   void testSetters() throws Exception
   {
      Random random = new Random(5467457);

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Cylinder3D other)
         Cylinder3D expected = EuclidShapeRandomTools.nextCylinder3D(random);
         Cylinder3D actual = EuclidShapeRandomTools.nextCylinder3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set(expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Cylinder3DReadOnly other)
         Cylinder3D expected = EuclidShapeRandomTools.nextCylinder3D(random);
         Cylinder3D actual = EuclidShapeRandomTools.nextCylinder3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set((Cylinder3DReadOnly) expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      { // set(Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Cylinder3D expected = EuclidShapeRandomTools.nextCylinder3D(random);
            Cylinder3D actual = EuclidShapeRandomTools.nextCylinder3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(expected.getPosition(), expected.getAxis(), expected.getLength(), expected.getRadius());
            EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Cylinder3D().set(new Point3D(), Axis3D.Z, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Cylinder3D().set(new Point3D(), Axis3D.Z, 1.0, -0.1));
      }
   }

   @Test
   void testSize() throws Exception
   {
      Random random = new Random(43905783);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double length = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         assertNotEquals(length, cylinder3D.getLength());
         assertNotEquals(radius, cylinder3D.getRadius());
         cylinder3D.setSize(length, radius);
         assertEquals(length, cylinder3D.getLength());
         assertEquals(radius, cylinder3D.getRadius());
      }

      assertThrows(IllegalArgumentException.class, () -> new Cylinder3D().setSize(-0.1, 1.0));
      assertThrows(IllegalArgumentException.class, () -> new Cylinder3D().setSize(1.0, -0.1));
   }

   @Test
   void testIsPointInside() throws Exception
   {
      Random random = new Random(3465463);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside generated to be within a certain radius of the axis
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D pointOnAxis = new Point3D();
         double alpha = random.nextDouble();
         pointOnAxis.interpolate(cylinder3D.getTopCenter(), cylinder3D.getBottomCenter(), alpha);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 0.0, cylinder3D.getRadius());
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);

         assertTrue(cylinder3D.isPointInside(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside generated to be within a certain radius of the axis
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(cylinder3D.getTopCenter(), cylinder3D.getBottomCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 1.0, 3.0) * cylinder3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);

         assertFalse(cylinder3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside hovering above the top cap
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnTopCap = new Point3D();
         pointOnTopCap.scaleAdd(random.nextDouble() * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getTopCenter());

         double distanceOffTopCap = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopCap, cylinder3D.getAxis(), pointOnTopCap);

         assertFalse(cylinder3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside hovering above the bottom cap
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnBottomCap = new Point3D();
         pointOnBottomCap.scaleAdd(random.nextDouble() * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getBottomCenter());

         double distanceOffBottomCap = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(-distanceOffBottomCap, cylinder3D.getAxis(), pointOnBottomCap);

         assertFalse(cylinder3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside the infinite cylinder AND beyond the top cap plane
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnTopPlane = new Point3D();
         pointOnTopPlane.scaleAdd((1.0 + random.nextDouble()) * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getTopCenter());

         double distanceOffTopPlane = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopPlane, cylinder3D.getAxis(), pointOnTopPlane);

         assertFalse(cylinder3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside the infinite cylinder AND beyond the bottom cap plane
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnBottomPlane = new Point3D();
         pointOnBottomPlane.scaleAdd((1.0 + random.nextDouble()) * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getBottomCenter());

         double distanceOffBottomPlane = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(-distanceOffBottomPlane, cylinder3D.getAxis(), pointOnBottomPlane);

         assertFalse(cylinder3D.isPointInside(pointOutside));
      }
   }

   @Test
   void testEvaluatePoint3DCollision() throws Exception
   {
      Random random = new Random(4444);

      Point3D actualClosestPoint = new Point3D();
      Vector3D actualNormal = new Vector3D();
      Point3D expectedClosestPoint = new Point3D();
      Vector3D expectedNormal = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      { // Edge-case: Query is on the axis
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(cylinder3D.getTopCenter(), cylinder3D.getBottomCenter(), random.nextDouble());

         assertTrue(cylinder3D.evaluatePoint3DCollision(pointOnAxis, actualClosestPoint, actualNormal));
         assertFalse(actualClosestPoint.containsNaN());
         assertFalse(actualNormal.containsNaN());

         cylinder3D.evaluatePoint3DCollision(actualClosestPoint, expectedClosestPoint, expectedNormal);
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside generated to be within a certain radius of the axis
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D pointOnAxis = new Point3D();
         double alpha = random.nextDouble();
         pointOnAxis.interpolate(cylinder3D.getTopCenter(), cylinder3D.getBottomCenter(), alpha);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 0.0, cylinder3D.getRadius());
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);

         // Need to figure out which cylinder's feature the point is the closest to
         double distanceFromInfiniteCylinder = cylinder3D.getRadius() - distanceOffAxis;
         double distanceFromTopCap = alpha * cylinder3D.getLength();
         double distanceFromBottomCap = (1.0 - alpha) * cylinder3D.getLength();

         if (distanceFromInfiniteCylinder <= distanceFromTopCap && distanceFromInfiniteCylinder <= distanceFromBottomCap)
         { // Closest to the infinite cylinder
            expectedClosestPoint.scaleAdd(cylinder3D.getRadius(), orthogonalToAxis, pointOnAxis);
            expectedNormal.setAndNormalize(orthogonalToAxis);
            assertTrue(cylinder3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal));
            EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, distanceOffAxis < 1.0e-3 ? 2.0 * EPSILON : EPSILON);
         }
         else
         {
            if (distanceFromTopCap <= distanceFromBottomCap)
            { // Closest to the top cap
               expectedClosestPoint.set(EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointInside, cylinder3D.getTopCenter(), cylinder3D.getAxis()));
               expectedNormal.set(cylinder3D.getAxis());
            }
            else
            { // Closest to the bottom cap
               expectedClosestPoint.set(EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointInside, cylinder3D.getBottomCenter(), cylinder3D.getAxis()));
               expectedNormal.setAndNegate(cylinder3D.getAxis());
            }
            assertTrue(cylinder3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal));
            EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside generated to be within a certain radius of the axis
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(cylinder3D.getTopCenter(), cylinder3D.getBottomCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 1.0, 3.0) * cylinder3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         expectedClosestPoint.scaleAdd(cylinder3D.getRadius(), orthogonalToAxis, pointOnAxis);
         expectedNormal.setAndNormalize(orthogonalToAxis);

         assertFalse(cylinder3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside hovering above the top cap
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnTopCap = new Point3D();
         pointOnTopCap.scaleAdd(random.nextDouble() * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getTopCenter());

         double distanceOffTopCap = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopCap, cylinder3D.getAxis(), pointOnTopCap);
         expectedClosestPoint.set(pointOnTopCap);
         expectedNormal.set(cylinder3D.getAxis());

         assertFalse(cylinder3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside hovering above the bottom cap
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnBottomCap = new Point3D();
         pointOnBottomCap.scaleAdd(random.nextDouble() * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getBottomCenter());

         double distanceOffBottomCap = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(-distanceOffBottomCap, cylinder3D.getAxis(), pointOnBottomCap);
         expectedClosestPoint.set(pointOnBottomCap);
         expectedNormal.setAndNegate(cylinder3D.getAxis());

         assertFalse(cylinder3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside the infinite cylinder AND beyond the top cap plane
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnTopPlane = new Point3D();
         pointOnTopPlane.scaleAdd((1.0 + random.nextDouble()) * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getTopCenter());

         double distanceOffTopPlane = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopPlane, cylinder3D.getAxis(), pointOnTopPlane);
         expectedClosestPoint.scaleAdd(cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getTopCenter());
         expectedNormal.sub(pointOutside, expectedClosestPoint);
         expectedNormal.normalize();

         assertFalse(cylinder3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside the infinite cylinder AND beyond the bottom cap plane
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnBottomPlane = new Point3D();
         pointOnBottomPlane.scaleAdd((1.0 + random.nextDouble()) * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getBottomCenter());

         double distanceOffBottomPlane = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(-distanceOffBottomPlane, cylinder3D.getAxis(), pointOnBottomPlane);
         expectedClosestPoint.scaleAdd(cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getBottomCenter());
         expectedNormal.sub(pointOutside, expectedClosestPoint);
         expectedNormal.normalize();

         assertFalse(cylinder3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
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
         Cylinder3D actual = EuclidShapeRandomTools.nextCylinder3D(random);
         Cylinder3D expected = new Cylinder3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPosition().applyTransform(transform);
         expected.getAxis().applyTransform(transform);
         actual.applyTransform(transform);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cylinder3D actual = EuclidShapeRandomTools.nextCylinder3D(random);
         Cylinder3D expected = new Cylinder3D(actual);

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
         Cylinder3D actual = EuclidShapeRandomTools.nextCylinder3D(random);
         Cylinder3D original = new Cylinder3D(actual);
         Cylinder3D expected = new Cylinder3D(actual);

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
         Cylinder3D actual = EuclidShapeRandomTools.nextCylinder3D(random);
         Cylinder3D original = new Cylinder3D(actual);
         Cylinder3D expected = new Cylinder3D(actual);

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
      Random random = new Random(987346);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside generated to be within a certain radius of the axis
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D pointOnAxis = new Point3D();
         double alpha = random.nextDouble();
         pointOnAxis.interpolate(cylinder3D.getTopCenter(), cylinder3D.getBottomCenter(), alpha);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 0.0, cylinder3D.getRadius());
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         assertEquals(0.0, cylinder3D.distance(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside generated to be within a certain radius of the axis
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(cylinder3D.getTopCenter(), cylinder3D.getBottomCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 1.0, 3.0) * cylinder3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         assertEquals(distanceOffAxis - cylinder3D.getRadius(), cylinder3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside hovering above the top cap
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnTopCap = new Point3D();
         pointOnTopCap.scaleAdd(random.nextDouble() * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getTopCenter());

         double distanceOffTopCap = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopCap, cylinder3D.getAxis(), pointOnTopCap);
         assertEquals(distanceOffTopCap, cylinder3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside hovering above the bottom cap
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnBottomCap = new Point3D();
         pointOnBottomCap.scaleAdd(random.nextDouble() * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getBottomCenter());

         double distanceOffBottomCap = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(-distanceOffBottomCap, cylinder3D.getAxis(), pointOnBottomCap);
         assertEquals(distanceOffBottomCap, cylinder3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside the infinite cylinder AND beyond the top cap plane
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnTopPlane = new Point3D();
         double distanceOrthogonalToAxis = (1.0 + random.nextDouble()) * cylinder3D.getRadius();
         pointOnTopPlane.scaleAdd(distanceOrthogonalToAxis, orthogonalToAxis, cylinder3D.getTopCenter());

         double distanceOffTopPlane = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopPlane, cylinder3D.getAxis(), pointOnTopPlane);
         double expectedDistance = EuclidCoreTools.norm(distanceOrthogonalToAxis - cylinder3D.getRadius(), distanceOffTopPlane);
         assertEquals(expectedDistance, cylinder3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside the infinite cylinder AND beyond the bottom cap plane
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnBottomPlane = new Point3D();
         double distanceOrthogonalToAxis = (1.0 + random.nextDouble()) * cylinder3D.getRadius();
         pointOnBottomPlane.scaleAdd(distanceOrthogonalToAxis, orthogonalToAxis, cylinder3D.getBottomCenter());

         double distanceOffBottomPlane = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(-distanceOffBottomPlane, cylinder3D.getAxis(), pointOnBottomPlane);
         double expectedDistance = EuclidCoreTools.norm(distanceOrthogonalToAxis - cylinder3D.getRadius(), distanceOffBottomPlane);
         assertEquals(expectedDistance, cylinder3D.distance(pointOutside), EPSILON);
      }
   }

   @Test
   void testSignedDistance() throws Exception
   {
      Random random = new Random(987);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside generated to be within a certain radius of the axis
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D pointOnAxis = new Point3D();
         double alpha = random.nextDouble();
         pointOnAxis.interpolate(cylinder3D.getTopCenter(), cylinder3D.getBottomCenter(), alpha);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 0.0, cylinder3D.getRadius());
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         double distanceFromInfiniteCylinder = cylinder3D.getRadius() - distanceOffAxis;
         double distanceFromTopCap = alpha * cylinder3D.getLength();
         double distanceFromBottomCap = (1.0 - alpha) * cylinder3D.getLength();
         assertEquals(-EuclidCoreTools.min(distanceFromInfiniteCylinder, distanceFromTopCap, distanceFromBottomCap),
                      cylinder3D.signedDistance(pointInside),
                      EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside generated to be within a certain radius of the axis
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(cylinder3D.getTopCenter(), cylinder3D.getBottomCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 1.0, 3.0) * cylinder3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         assertEquals(distanceOffAxis - cylinder3D.getRadius(), cylinder3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside hovering above the top cap
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnTopCap = new Point3D();
         pointOnTopCap.scaleAdd(random.nextDouble() * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getTopCenter());

         double distanceOffTopCap = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopCap, cylinder3D.getAxis(), pointOnTopCap);
         assertEquals(distanceOffTopCap, cylinder3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside hovering above the bottom cap
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnBottomCap = new Point3D();
         pointOnBottomCap.scaleAdd(random.nextDouble() * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getBottomCenter());

         double distanceOffBottomCap = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(-distanceOffBottomCap, cylinder3D.getAxis(), pointOnBottomCap);
         assertEquals(distanceOffBottomCap, cylinder3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside the infinite cylinder AND beyond the top cap plane
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnTopPlane = new Point3D();
         double distanceOrthogonalToAxis = (1.0 + random.nextDouble()) * cylinder3D.getRadius();
         pointOnTopPlane.scaleAdd(distanceOrthogonalToAxis, orthogonalToAxis, cylinder3D.getTopCenter());

         double distanceOffTopPlane = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopPlane, cylinder3D.getAxis(), pointOnTopPlane);
         double expectedDistance = EuclidCoreTools.norm(distanceOrthogonalToAxis - cylinder3D.getRadius(), distanceOffTopPlane);
         assertEquals(expectedDistance, cylinder3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside the infinite cylinder AND beyond the bottom cap plane
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnBottomPlane = new Point3D();
         double distanceOrthogonalToAxis = (1.0 + random.nextDouble()) * cylinder3D.getRadius();
         pointOnBottomPlane.scaleAdd(distanceOrthogonalToAxis, orthogonalToAxis, cylinder3D.getBottomCenter());

         double distanceOffBottomPlane = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(-distanceOffBottomPlane, cylinder3D.getAxis(), pointOnBottomPlane);
         double expectedDistance = EuclidCoreTools.norm(distanceOrthogonalToAxis - cylinder3D.getRadius(), distanceOffBottomPlane);
         assertEquals(expectedDistance, cylinder3D.signedDistance(pointOutside), EPSILON);
      }
   }

   @Test
   void testOrthogonalProjection() throws Exception
   {
      Random random = new Random(768);

      Point3D expectedProjection = new Point3D();

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside generated to be within a certain radius of the axis
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D pointOnAxis = new Point3D();
         double alpha = random.nextDouble();
         pointOnAxis.interpolate(cylinder3D.getTopCenter(), cylinder3D.getBottomCenter(), alpha);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 0.0, cylinder3D.getRadius());
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);

         assertNull(cylinder3D.orthogonalProjectionCopy(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside generated to be within a certain radius of the axis
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.interpolate(cylinder3D.getTopCenter(), cylinder3D.getBottomCenter(), random.nextDouble());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         double distanceOffAxis = EuclidCoreRandomTools.nextDouble(random, 1.0, 3.0) * cylinder3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffAxis, orthogonalToAxis, pointOnAxis);
         expectedProjection.scaleAdd(cylinder3D.getRadius(), orthogonalToAxis, pointOnAxis);
         EuclidCoreTestTools.assertEquals(expectedProjection, cylinder3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside hovering above the top cap
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnTopCap = new Point3D();
         pointOnTopCap.scaleAdd(random.nextDouble() * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getTopCenter());

         double distanceOffTopCap = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopCap, cylinder3D.getAxis(), pointOnTopCap);
         expectedProjection.set(pointOnTopCap);
         EuclidCoreTestTools.assertEquals(expectedProjection, cylinder3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside hovering above the bottom cap
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnBottomCap = new Point3D();
         pointOnBottomCap.scaleAdd(random.nextDouble() * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getBottomCenter());

         double distanceOffBottomCap = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(-distanceOffBottomCap, cylinder3D.getAxis(), pointOnBottomCap);
         expectedProjection.set(pointOnBottomCap);
         EuclidCoreTestTools.assertEquals(expectedProjection, cylinder3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside the infinite cylinder AND beyond the top cap plane
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnTopPlane = new Point3D();
         pointOnTopPlane.scaleAdd((1.0 + random.nextDouble()) * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getTopCenter());

         double distanceOffTopPlane = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceOffTopPlane, cylinder3D.getAxis(), pointOnTopPlane);
         expectedProjection.scaleAdd(cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getTopCenter());
         EuclidCoreTestTools.assertEquals(expectedProjection, cylinder3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside the infinite cylinder AND beyond the bottom cap plane
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinder3D.getAxis(), true);
         Point3D pointOnBottomPlane = new Point3D();
         pointOnBottomPlane.scaleAdd((1.0 + random.nextDouble()) * cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getBottomCenter());

         double distanceOffBottomPlane = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(-distanceOffBottomPlane, cylinder3D.getAxis(), pointOnBottomPlane);
         expectedProjection.scaleAdd(cylinder3D.getRadius(), orthogonalToAxis, cylinder3D.getBottomCenter());
         EuclidCoreTestTools.assertEquals(expectedProjection, cylinder3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(12653L);
      double epsilon = 1e-7;
      Cylinder3D firstCylinder, secondCylinder;
      Point3D position;
      Vector3D axis;
      double height, radius;
      Vector3D translation;

      height = random.nextDouble();
      radius = random.nextDouble();
      position = EuclidCoreRandomTools.nextPoint3D(random);
      axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

      firstCylinder = new Cylinder3D(position, axis, height, radius);
      secondCylinder = new Cylinder3D(position, axis, height, radius);

      assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      assertTrue(secondCylinder.geometricallyEquals(firstCylinder, epsilon));
      assertTrue(firstCylinder.geometricallyEquals(firstCylinder, epsilon));
      assertTrue(secondCylinder.geometricallyEquals(secondCylinder, epsilon));

      for (int i = 0; i < ITERATIONS; ++i)
      { // Cylinders are equal if heights are equal within +- epsilon and are otherwise the same
         height = random.nextDouble();
         radius = random.nextDouble();
         position = EuclidCoreRandomTools.nextPoint3D(random);
         axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         firstCylinder = new Cylinder3D(position, axis, height, radius);
         secondCylinder = new Cylinder3D(position, axis, height, radius);

         secondCylinder.setLength(height + 0.99 * epsilon);

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));

         secondCylinder.setLength(height + 1.01 * epsilon);

         assertFalse(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Cylinders are equal if radii are equal within +- epsilon and are otherwise the same
         height = random.nextDouble();
         radius = random.nextDouble();
         position = EuclidCoreRandomTools.nextPoint3D(random);
         axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         firstCylinder = new Cylinder3D(position, axis, height, radius);
         secondCylinder = new Cylinder3D(position, axis, height, radius);

         secondCylinder.setRadius(radius + 0.99 * epsilon);

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));

         secondCylinder.setRadius(radius + 1.01 * epsilon);

         assertFalse(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Cylinders are equal if aligned opposite on the same axis and are otherwise the same
         height = random.nextDouble();
         radius = random.nextDouble();
         position = EuclidCoreRandomTools.nextPoint3D(random);
         axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         firstCylinder = new Cylinder3D(position, axis, height, radius);
         secondCylinder = new Cylinder3D(position, axis, height, radius);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, axis, true);

         secondCylinder.getAxis().applyTransform(new RigidBodyTransform(new AxisAngle(orthogonalToAxis, Math.PI), new Vector3D()));

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Cylinders are equal if translations are equal within +- epsilon and are otherwise the same
         height = random.nextDouble();
         radius = random.nextDouble();
         position = EuclidCoreRandomTools.nextPoint3D(random);
         axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         firstCylinder = new Cylinder3D(position, axis, height, radius);
         secondCylinder = new Cylinder3D(position, axis, height, radius);

         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon);

         secondCylinder.applyTransform(new RigidBodyTransform(new Quaternion(), translation));

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));

         secondCylinder = new Cylinder3D(position, axis, height, radius);

         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon);

         secondCylinder.applyTransform(new RigidBodyTransform(new Quaternion(), translation));

         assertFalse(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      }
   }

   @Test
   void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(546161);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cylinder3D cylinder = EuclidShapeRandomTools.nextCylinder3D(random);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly supportingVertex = cylinder.getSupportingVertex(supportDirection);
         assertTrue(cylinder.isPointInside(supportingVertex, EPSILON));

         Point3D supportingVertexTranslated = new Point3D();
         supportDirection.normalize();
         supportingVertexTranslated.scaleAdd(1.0e-2, supportDirection, supportingVertex);
         assertFalse(cylinder.isPointInside(supportingVertexTranslated, EPSILON));

         Vector3D actualNormal = new Vector3D();
         cylinder.evaluatePoint3DCollision(supportingVertexTranslated, new Point3D(), actualNormal);
         EuclidCoreTestTools.assertEquals(supportDirection, actualNormal, EPSILON);
      }
   }

   @Test
   void testGetBoundingBox() throws Exception
   {
      Random random = new Random(36342);

      for (int i = 0; i < ITERATIONS; i++)
      { // Using getSupportingVertex
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);

         BoundingBox3D expectedBoundingBox = new BoundingBox3D();
         expectedBoundingBox.setToNaN();
         Vector3D supportDirection = new Vector3D(Axis3D.X);
         expectedBoundingBox.updateToIncludePoint(cylinder3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(cylinder3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis3D.Y);
         expectedBoundingBox.updateToIncludePoint(cylinder3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(cylinder3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis3D.Z);
         expectedBoundingBox.updateToIncludePoint(cylinder3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(cylinder3D.getSupportingVertex(supportDirection));

         BoundingBox3DReadOnly actualBoundingBox = cylinder3D.getBoundingBox();
         EuclidCoreTestTools.assertEquals(expectedBoundingBox, actualBoundingBox, EPSILON);
      }
   }
}
