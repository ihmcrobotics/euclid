package us.ihmc.euclid.shape.primitives;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Torus3DTest
{
   private static final double EPSILON = 1.0e-9;

   @Test
   void testConstructors() throws Exception
   {
      Random random = new Random(34563);

      { // Empty constructor
         Torus3D torus3D = new Torus3D();

         EuclidCoreTestTools.assertTuple3DIsSetToZero(torus3D.getPosition());
         EuclidCoreTestTools.assertTuple3DEquals(Axis.Z, torus3D.getAxis(), EPSILON);
         assertEquals(1.0, torus3D.getRadius());
         assertEquals(0.1, torus3D.getTubeRadius());
      }

      { // Torus3D(double radius, double tubeRadius)
         for (int i = 0; i < ITERATIONS; i++)
         {
            double radius = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
            double tubeRadius = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
            Torus3D torus3D = new Torus3D(radius, tubeRadius);

            EuclidCoreTestTools.assertTuple3DIsSetToZero(torus3D.getPosition());
            EuclidCoreTestTools.assertTuple3DEquals(Axis.Z, torus3D.getAxis(), EPSILON);
            assertEquals(radius, torus3D.getRadius());
            assertEquals(tubeRadius, torus3D.getTubeRadius());
         }

         assertThrows(IllegalArgumentException.class, () -> new Torus3D(-0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Torus3D(1.0, -0.1));
      }

      { // Torus3D(Point3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
         for (int i = 0; i < ITERATIONS; i++)
         {
            double radius = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
            double tubeRadius = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
            Point3DReadOnly position = EuclidCoreRandomTools.nextPoint3D(random);
            Vector3D axis = EuclidCoreRandomTools.nextVector3D(random);
            Torus3D torus3D = new Torus3D(position, axis, radius, tubeRadius);

            axis.normalize();
            EuclidCoreTestTools.assertTuple3DEquals(position, torus3D.getPosition(), EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(axis, torus3D.getAxis(), EPSILON);
            assertEquals(radius, torus3D.getRadius());
            assertEquals(tubeRadius, torus3D.getTubeRadius());
         }

         assertThrows(IllegalArgumentException.class, () -> new Torus3D(new Point3D(), Axis.Z, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Torus3D(new Point3D(), Axis.Z, 1.0, -0.1));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Torus3D(Torus3DReadOnly other)
         Torus3D original = EuclidShapeRandomTools.nextTorus3D(random);
         Torus3D copy = new Torus3D(original);

         EuclidShapeTestTools.assertTorus3DEquals(original, copy, EPSILON);
      }
   }

   @Test
   void testSetToNaN() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         assertFalse(torus3D.containsNaN());
         assertFalse(torus3D.getPosition().containsNaN());
         assertFalse(torus3D.getAxis().containsNaN());
         assertFalse(Double.isNaN(torus3D.getRadius()));
         assertFalse(Double.isNaN(torus3D.getTubeRadius()));

         torus3D.setToNaN();

         assertTrue(torus3D.containsNaN());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(torus3D.getPosition());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(torus3D.getAxis());
         assertTrue(Double.isNaN(torus3D.getRadius()));
         assertTrue(Double.isNaN(torus3D.getTubeRadius()));
      }
   }

   @Test
   void testSetToZero() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         assertFalse(new Point3D().epsilonEquals(torus3D.getPosition(), EPSILON));
         assertFalse(new Point3D().epsilonEquals(torus3D.getAxis(), EPSILON));
         assertNotEquals(0.0, torus3D.getRadius());
         assertNotEquals(0.0, torus3D.getTubeRadius());

         torus3D.setToZero();

         EuclidCoreTestTools.assertTuple3DIsSetToZero(torus3D.getPosition());
         EuclidCoreTestTools.assertTuple3DEquals(Axis.Z, torus3D.getAxis(), EPSILON);
         assertEquals(0.0, torus3D.getRadius());
         assertEquals(0.0, torus3D.getTubeRadius());
      }
   }

   @Test
   void testSetters() throws Exception
   {
      Random random = new Random(5467457);

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Torus3D other)
         Torus3D expected = EuclidShapeRandomTools.nextTorus3D(random);
         Torus3D actual = EuclidShapeRandomTools.nextTorus3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set(expected);
         EuclidShapeTestTools.assertTorus3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Torus3DReadOnly other)
         Torus3D expected = EuclidShapeRandomTools.nextTorus3D(random);
         Torus3D actual = EuclidShapeRandomTools.nextTorus3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set((Torus3DReadOnly) expected);
         EuclidShapeTestTools.assertTorus3DEquals(expected, actual, EPSILON);
      }

      { // set(Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Torus3D expected = EuclidShapeRandomTools.nextTorus3D(random);
            Torus3D actual = EuclidShapeRandomTools.nextTorus3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(expected.getPosition(), expected.getAxis(), expected.getRadius(), expected.getTubeRadius());
            EuclidShapeTestTools.assertTorus3DEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Torus3D().set(new Point3D(), Axis.Z, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Torus3D().set(new Point3D(), Axis.Z, 1.0, -0.1));
      }
   }

   @Test
   void testSetRadii() throws Exception
   {
      Random random = new Random(43905783);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double tubeRadius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);
         assertNotEquals(radius, torus3D.getRadius());
         assertNotEquals(tubeRadius, torus3D.getTubeRadius());
         torus3D.setRadii(radius, tubeRadius);
         assertEquals(radius, torus3D.getRadius());
         assertEquals(tubeRadius, torus3D.getTubeRadius());
      }

      assertThrows(IllegalArgumentException.class, () -> new Torus3D().setRadii(-0.1, 1.0));
      assertThrows(IllegalArgumentException.class, () -> new Torus3D().setRadii(1.0, -0.1));
   }

   @Test
   void testIsPointInside() throws Exception
   {
      Random random = new Random(17970997);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torus3D.getAxis(), true);
         Point3D pointOnTubeCenter = new Point3D();
         pointOnTubeCenter.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeAxis = new Vector3D();
         tubeAxis.cross(orthogonalToAxis, torus3D.getAxis());

         Vector3D orthogonalToTubeAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeAxis, true);
         double distanceFromTubeAxis = random.nextDouble() * torus3D.getTubeRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceFromTubeAxis, orthogonalToTubeAxis, pointOnTubeCenter);

         assertTrue(torus3D.isPointInside(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torus3D.getAxis(), true);
         Point3D pointOnTubeCenter = new Point3D();
         pointOnTubeCenter.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeAxis = new Vector3D();
         tubeAxis.cross(orthogonalToAxis, torus3D.getAxis());

         Vector3D orthogonalToTubeAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeAxis, true);
         double distanceFromTubeAxis = EuclidCoreRandomTools.nextDouble(random, torus3D.getTubeRadius(), torus3D.getRadius());
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceFromTubeAxis, orthogonalToTubeAxis, pointOnTubeCenter);

         assertFalse(torus3D.isPointInside(pointOutside));
      }
   }

   @Test
   void testEvaluatePoint3DCollision() throws Exception
   {
      Random random = new Random(16980501);

      Point3D actualClosestPoint = new Point3D();
      Vector3D actualNormal = new Vector3D();
      Point3D expectedClosestPoint = new Point3D();
      Vector3D expectedNormal = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      { // Edge-case: query is on torus' axis
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.scaleAdd(EuclidCoreRandomTools.nextDouble(random), torus3D.getAxis(), torus3D.getPosition());

         assertFalse(torus3D.evaluatePoint3DCollision(pointOnAxis, actualClosestPoint, actualNormal));
         assertFalse(actualClosestPoint.containsNaN());
         assertFalse(actualNormal.containsNaN());

         torus3D.evaluatePoint3DCollision(actualClosestPoint, expectedClosestPoint, expectedNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Edge-case: query is on torus' tube axis
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torus3D.getAxis(), true);
         Point3D pointOnTubeCenter = new Point3D();
         pointOnTubeCenter.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         assertTrue(torus3D.evaluatePoint3DCollision(pointOnTubeCenter, actualClosestPoint, actualNormal));
         assertFalse(actualClosestPoint.containsNaN());
         assertFalse(actualNormal.containsNaN());

         torus3D.evaluatePoint3DCollision(actualClosestPoint, expectedClosestPoint, expectedNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torus3D.getAxis(), true);
         Point3D pointOnTubeCenter = new Point3D();
         pointOnTubeCenter.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeAxis = new Vector3D();
         tubeAxis.cross(orthogonalToAxis, torus3D.getAxis());

         Vector3D orthogonalToTubeAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeAxis, true);
         double distanceFromTubeAxis = random.nextDouble() * torus3D.getTubeRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceFromTubeAxis, orthogonalToTubeAxis, pointOnTubeCenter);

         expectedClosestPoint.scaleAdd(torus3D.getTubeRadius(), orthogonalToTubeAxis, pointOnTubeCenter);
         expectedNormal.set(orthogonalToTubeAxis);

         assertTrue(torus3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torus3D.getAxis(), true);
         Point3D pointOnTubeCenter = new Point3D();
         pointOnTubeCenter.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeAxis = new Vector3D();
         tubeAxis.cross(orthogonalToAxis, torus3D.getAxis());

         Vector3D orthogonalToTubeAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeAxis, true);
         double distanceFromTubeAxis = EuclidCoreRandomTools.nextDouble(random, torus3D.getTubeRadius(), torus3D.getRadius());
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceFromTubeAxis, orthogonalToTubeAxis, pointOnTubeCenter);

         expectedClosestPoint.scaleAdd(torus3D.getTubeRadius(), orthogonalToTubeAxis, pointOnTubeCenter);
         expectedNormal.set(orthogonalToTubeAxis);

         assertFalse(torus3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }
   }

   @Test
   void testApplyTransform()
   {
      Random random = new Random(346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Torus3D actual = EuclidShapeRandomTools.nextTorus3D(random);
         Torus3D expected = new Torus3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPosition().applyTransform(transform);
         expected.getAxis().applyTransform(transform);
         actual.applyTransform(transform);

         EuclidShapeTestTools.assertTorus3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Torus3D actual = EuclidShapeRandomTools.nextTorus3D(random);
         Torus3D expected = new Torus3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPosition().applyTransform(transform);
         expected.getAxis().applyTransform(transform);
         expected.getAxis().normalize();
         actual.applyTransform(transform);

         EuclidShapeTestTools.assertTorus3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   void testApplyInverseTransform()
   {
      Random random = new Random(346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Torus3D actual = EuclidShapeRandomTools.nextTorus3D(random);
         Torus3D original = new Torus3D(actual);
         Torus3D expected = new Torus3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPosition().applyInverseTransform(transform);
         expected.getAxis().applyInverseTransform(transform);
         expected.getAxis().normalize();
         actual.applyInverseTransform(transform);

         EuclidShapeTestTools.assertTorus3DEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidShapeTestTools.assertTorus3DEquals(original, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Torus3D actual = EuclidShapeRandomTools.nextTorus3D(random);
         Torus3D original = new Torus3D(actual);
         Torus3D expected = new Torus3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPosition().applyInverseTransform(transform);
         expected.getAxis().applyInverseTransform(transform);
         expected.getAxis().normalize();
         actual.applyInverseTransform(transform);

         EuclidShapeTestTools.assertTorus3DEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidShapeTestTools.assertTorus3DEquals(original, actual, EPSILON);
      }
   }

   @Test
   void testDistance() throws Exception
   {
      Random random = new Random(20478904);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torus3D.getAxis(), true);
         Point3D pointOnTubeCenter = new Point3D();
         pointOnTubeCenter.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeAxis = new Vector3D();
         tubeAxis.cross(orthogonalToAxis, torus3D.getAxis());

         Vector3D orthogonalToTubeAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeAxis, true);
         double distanceFromTubeAxis = random.nextDouble() * torus3D.getTubeRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceFromTubeAxis, orthogonalToTubeAxis, pointOnTubeCenter);

         assertEquals(0.0, torus3D.distance(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torus3D.getAxis(), true);
         Point3D pointOnTubeCenter = new Point3D();
         pointOnTubeCenter.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeAxis = new Vector3D();
         tubeAxis.cross(orthogonalToAxis, torus3D.getAxis());

         Vector3D orthogonalToTubeAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeAxis, true);
         double distanceFromTubeAxis = EuclidCoreRandomTools.nextDouble(random, torus3D.getTubeRadius(), torus3D.getRadius());
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceFromTubeAxis, orthogonalToTubeAxis, pointOnTubeCenter);

         assertEquals(distanceFromTubeAxis - torus3D.getTubeRadius(), torus3D.distance(pointOutside), EPSILON);
      }
   }

   @Test
   void testSignedDistance() throws Exception
   {
      Random random = new Random(20478904);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torus3D.getAxis(), true);
         Point3D pointOnTubeCenter = new Point3D();
         pointOnTubeCenter.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeAxis = new Vector3D();
         tubeAxis.cross(orthogonalToAxis, torus3D.getAxis());

         Vector3D orthogonalToTubeAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeAxis, true);
         double distanceFromTubeAxis = random.nextDouble() * torus3D.getTubeRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceFromTubeAxis, orthogonalToTubeAxis, pointOnTubeCenter);

         assertEquals(distanceFromTubeAxis - torus3D.getTubeRadius(), torus3D.signedDistance(pointInside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torus3D.getAxis(), true);
         Point3D pointOnTubeCenter = new Point3D();
         pointOnTubeCenter.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeAxis = new Vector3D();
         tubeAxis.cross(orthogonalToAxis, torus3D.getAxis());

         Vector3D orthogonalToTubeAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeAxis, true);
         double distanceFromTubeAxis = EuclidCoreRandomTools.nextDouble(random, torus3D.getTubeRadius(), torus3D.getRadius());
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceFromTubeAxis, orthogonalToTubeAxis, pointOnTubeCenter);

         assertEquals(distanceFromTubeAxis - torus3D.getTubeRadius(), torus3D.signedDistance(pointOutside), EPSILON);
      }
   }

   @Test
   void testOrthogonalProjection() throws Exception
   {
      Random random = new Random(9387125);

      for (int i = 0; i < ITERATIONS; i++)
      { // Edge-case: query is on torus' axis
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);
         Point3D pointOnAxis = new Point3D();
         pointOnAxis.scaleAdd(EuclidCoreRandomTools.nextDouble(random), torus3D.getAxis(), torus3D.getPosition());

         Point3DBasics actualProjection = torus3D.orthogonalProjectionCopy(pointOnAxis);
         assertFalse(actualProjection.containsNaN());

         Point3D expectedProjection = new Point3D();
         torus3D.evaluatePoint3DCollision(actualProjection, expectedProjection, new Vector3D());
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torus3D.getAxis(), true);
         Point3D pointOnTubeCenter = new Point3D();
         pointOnTubeCenter.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeAxis = new Vector3D();
         tubeAxis.cross(orthogonalToAxis, torus3D.getAxis());

         Vector3D orthogonalToTubeAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeAxis, true);
         double distanceFromTubeAxis = random.nextDouble() * torus3D.getTubeRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceFromTubeAxis, orthogonalToTubeAxis, pointOnTubeCenter);

         assertNull(torus3D.orthogonalProjectionCopy(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torus3D.getAxis(), true);
         Point3D pointOnTubeCenter = new Point3D();
         pointOnTubeCenter.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeAxis = new Vector3D();
         tubeAxis.cross(orthogonalToAxis, torus3D.getAxis());

         Vector3D orthogonalToTubeAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeAxis, true);
         double distanceFromTubeAxis = EuclidCoreRandomTools.nextDouble(random, torus3D.getTubeRadius(), torus3D.getRadius());
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceFromTubeAxis, orthogonalToTubeAxis, pointOnTubeCenter);

         Point3D expectedProjection = new Point3D();
         expectedProjection.scaleAdd(torus3D.getTubeRadius(), orthogonalToTubeAxis, pointOnTubeCenter);

         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, torus3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }
   }

   @Test
   void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(546161);

      Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);
      Vector3D supportDirection = EuclidCoreRandomTools.nextVector3D(random);

      assertThrows(UnsupportedOperationException.class, () -> torus3D.getSupportingVertex(supportDirection));
   }

   @Test
   void testGetBoundingBox() throws Exception
   {
      Random random = new Random(36342);

      for (int i = 0; i < ITERATIONS; i++)
      { // Comparing to Cylinder3D, should be the same
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);
         Cylinder3D cylinder3D = new Cylinder3D(torus3D.getPosition(),
                                                torus3D.getAxis(),
                                                torus3D.getTubeRadius(),
                                                torus3D.getRadius() + torus3D.getTubeRadius());

         BoundingBox3DReadOnly expectedBoundingBox = cylinder3D.getBoundingBox();
         BoundingBox3DReadOnly actualBoundingBox = torus3D.getBoundingBox();
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expectedBoundingBox, actualBoundingBox, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(89725L);
      Torus3D firstTorus, secondTorus;
      Point3D position;
      Vector3D axis;
      double radius, tubeRadius;
      double epsilon = 1e-7;

      radius = 1.0 + random.nextDouble();
      tubeRadius = random.nextDouble();

      position = EuclidCoreRandomTools.nextPoint3D(random);
      axis = EuclidCoreRandomTools.nextVector3D(random);
      firstTorus = new Torus3D(position, axis, radius, tubeRadius);
      secondTorus = new Torus3D(position, axis, radius, tubeRadius);

      assertTrue(firstTorus.geometricallyEquals(secondTorus, epsilon));
      assertTrue(secondTorus.geometricallyEquals(firstTorus, epsilon));
      assertTrue(firstTorus.geometricallyEquals(firstTorus, epsilon));
      assertTrue(secondTorus.geometricallyEquals(secondTorus, epsilon));

      for (int i = 0; i < ITERATIONS; ++i)
      { // Torii do not represent the same geometry object
         position = EuclidCoreRandomTools.nextPoint3D(random);
         axis = EuclidCoreRandomTools.nextVector3D(random);
         radius = 1.0 + random.nextDouble();
         tubeRadius = random.nextDouble();

         firstTorus = new Torus3D(position, axis, radius, tubeRadius);
         secondTorus = new Torus3D(position, axis, radius, tubeRadius);

         secondTorus.applyTransform(EuclidCoreRandomTools.nextRigidBodyTransform(random));

         assertFalse(firstTorus.geometricallyEquals(secondTorus, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Torii within +- epsilon are equal
         position = EuclidCoreRandomTools.nextPoint3D(random);
         axis = EuclidCoreRandomTools.nextVector3D(random);
         radius = 1.0 + random.nextDouble();
         tubeRadius = random.nextDouble();

         firstTorus = new Torus3D(position, axis, radius, tubeRadius);
         secondTorus = new Torus3D(position, axis, radius + 0.99 * epsilon, tubeRadius);

         assertTrue(firstTorus.geometricallyEquals(secondTorus, epsilon));

         secondTorus = new Torus3D(position, axis, radius, tubeRadius + 0.99 * epsilon);

         assertTrue(firstTorus.geometricallyEquals(secondTorus, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Torii outside of +- epsilon are not equal
         position = EuclidCoreRandomTools.nextPoint3D(random);
         axis = EuclidCoreRandomTools.nextVector3D(random);
         radius = 1.0 + random.nextDouble();
         tubeRadius = random.nextDouble();

         firstTorus = new Torus3D(position, axis, radius, tubeRadius);
         secondTorus = new Torus3D(position, axis, radius + 1.01 * epsilon, tubeRadius);

         assertFalse(firstTorus.geometricallyEquals(secondTorus, epsilon));

         secondTorus = new Torus3D(position, axis, radius, tubeRadius + 1.01 * epsilon);

         assertFalse(firstTorus.geometricallyEquals(secondTorus, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Torii are equal if in the exact same position, but one is upside-down (w.r.t. reference frame)
         position = EuclidCoreRandomTools.nextPoint3D(random);
         axis = EuclidCoreRandomTools.nextVector3D(random);
         radius = 1.0 + random.nextDouble();
         tubeRadius = random.nextDouble();

         firstTorus = new Torus3D(position, axis, radius, tubeRadius);
         secondTorus = new Torus3D(position, axis, radius, tubeRadius);

         assertTrue(firstTorus.geometricallyEquals(secondTorus, epsilon));

         secondTorus.getAxis().negate();

         assertTrue(firstTorus.geometricallyEquals(secondTorus, epsilon));
      }
   }
}
