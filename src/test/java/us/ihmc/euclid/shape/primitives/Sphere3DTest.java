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
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Sphere3DTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   void testConstructors() throws Exception
   {
      Random random = new Random(19889566);

      { // Empty constructor
         Sphere3D sphere3D = new Sphere3D();

         EuclidCoreTestTools.assertTuple3DIsSetToZero(sphere3D.getPosition());
         assertEquals(1.0, sphere3D.getRadius());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Sphere3D(double radius)
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         Sphere3D sphere3D = new Sphere3D(radius);

         EuclidCoreTestTools.assertTuple3DIsSetToZero(sphere3D.getPosition());
         assertEquals(radius, sphere3D.getRadius());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Sphere3D(Point3DReadOnly center, double radius)
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         Point3D center = EuclidCoreRandomTools.nextPoint3D(random);
         Sphere3D sphere3D = new Sphere3D(center, radius);

         EuclidCoreTestTools.assertEquals(center, sphere3D.getPosition(), EPSILON);
         assertEquals(radius, sphere3D.getRadius());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Sphere3D(double centerX, double centerY, double centerZ, double radius)
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         Point3D center = EuclidCoreRandomTools.nextPoint3D(random);
         Sphere3D sphere3D = new Sphere3D(center.getX(), center.getY(), center.getZ(), radius);

         EuclidCoreTestTools.assertEquals(center, sphere3D.getPosition(), EPSILON);
         assertEquals(radius, sphere3D.getRadius());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Sphere3D(Sphere3DReadOnly other)
         Sphere3D original = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D copy = new Sphere3D(original);

         EuclidCoreTestTools.assertEquals(original, copy, EPSILON);
      }
   }

   @Test
   void testSetToNaN() throws Exception
   {
      Random random = new Random(16324321);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);

         assertFalse(sphere3D.containsNaN());
         assertFalse(sphere3D.getPosition().containsNaN());
         assertFalse(Double.isNaN(sphere3D.getRadius()));

         sphere3D.setToNaN();

         assertTrue(sphere3D.containsNaN());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(sphere3D.getPosition());
         assertTrue(Double.isNaN(sphere3D.getRadius()));
      }
   }

   @Test
   void testSetToZero() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);

         assertFalse(new Point3D().epsilonEquals(sphere3D.getPosition(), EPSILON));
         assertNotEquals(0.0, sphere3D.getRadius());

         sphere3D.setToZero();

         EuclidCoreTestTools.assertTuple3DIsSetToZero(sphere3D.getPosition());
         assertEquals(0.0, sphere3D.getRadius());
      }
   }

   @Test
   void testSetters() throws Exception
   {
      Random random = new Random(45837543);

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Sphere3D other)
         Sphere3D expected = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D actual = EuclidShapeRandomTools.nextSphere3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set(expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Sphere3DReadOnly other)
         Sphere3D expected = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D actual = EuclidShapeRandomTools.nextSphere3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set((Sphere3DReadOnly) expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      { // set(double centerX, double centerY, double centerZ, double radius)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Sphere3D expected = EuclidShapeRandomTools.nextSphere3D(random);
            Sphere3D actual = EuclidShapeRandomTools.nextSphere3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(expected.getPosition().getX(), expected.getPosition().getY(), expected.getPosition().getZ(), expected.getRadius());
            EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Sphere3D().set(1.0, 1.0, 1.0, -0.1));
      }

      { // set(Point3DReadOnly center, double radius)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Sphere3D expected = EuclidShapeRandomTools.nextSphere3D(random);
            Sphere3D actual = EuclidShapeRandomTools.nextSphere3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(expected.getPosition(), expected.getRadius());
            EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Sphere3D().set(1.0, 1.0, 1.0, -0.1));
      }
   }

   @Test
   void testIsPointInside() throws Exception
   {
      Random random = new Random(12116892);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double distanceFromCenter = random.nextDouble() * sphere3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceFromCenter, direction, sphere3D.getPosition());

         assertTrue(sphere3D.isPointInside(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double distanceFromCenter = random.nextDouble() + sphere3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceFromCenter, direction, sphere3D.getPosition());

         assertFalse(sphere3D.isPointInside(pointOutside));
      }
   }

   @Test
   void testEvaluatePoint3DCollision() throws Exception
   {
      Random random = new Random(2309819);

      Point3D actualClosestPoint = new Point3D();
      Vector3D actualNormal = new Vector3D();
      Point3D expectedClosestPoint = new Point3D();
      Vector3D expectedNormal = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double distanceFromCenter = random.nextDouble() * sphere3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceFromCenter, direction, sphere3D.getPosition());
         expectedClosestPoint.scaleAdd(sphere3D.getRadius(), direction, sphere3D.getPosition());
         expectedNormal.setAndNormalize(direction);

         assertTrue(sphere3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double distanceFromCenter = random.nextDouble() + sphere3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceFromCenter, direction, sphere3D.getPosition());
         expectedClosestPoint.scaleAdd(sphere3D.getRadius(), direction, sphere3D.getPosition());
         expectedNormal.setAndNormalize(direction);

         assertFalse(sphere3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }
   }

   @Test
   void testIntersectionWith() throws Exception
   {
      Random random = new Random(2309819);
      for (int i = 0; i < ITERATIONS; i++)
      {
         // Random sphere

         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);

         double radius = sphere3D.getRadius();
         double spherePositionX = sphere3D.getPosition().getX();
         double spherePositionY = sphere3D.getPosition().getY();
         double spherePositionZ = sphere3D.getPosition().getZ();

         Point3D firstIntersectionToPack = new Point3D();
         Point3D secondIntersectionToPack = new Point3D();

         // Intersecting or not 
         boolean intersecting = random.nextBoolean();
         //System.out.println(intersecting);

         if (intersecting)
         {

            // Generate a point on the sphere surface
            double x1 = spherePositionX + radius;
            double y1 = spherePositionY;
            double z1 = spherePositionZ;

            Point3D pointOnSurface = new Point3D(x1, y1, z1);

            //Generate a point inside the sphere
            double x2 = spherePositionX + (1 / 2) * radius;
            double y2 = spherePositionY;
            double z2 = spherePositionZ;

            Point3D pointInside = new Point3D(x2, y2, z2);

            double x = random.nextDouble();
            double y = random.nextDouble();
            double z = random.nextDouble();

            Vector3D randomDirection = new Vector3D(x, y, z);

            //System.out.println("point 1 : " + sphere3D.intersectionWith(point1, randomDirection, firstIntersectionToPack, secondIntersectionToPack));  

            //System.out.println("point 2 : " + sphere3D.intersectionWith(point2, randomDirection, firstIntersectionToPack, secondIntersectionToPack));
            //assertTrue(sphere3D.intersectionWith(pointOnSurface, randomDirection, firstIntersectionToPack, secondIntersectionToPack) == 1);
            //assertTrue(sphere3D.intersectionWith(pointInside, randomDirection, firstIntersectionToPack, secondIntersectionToPack) == 1);

            //Generate a point outside the sphere but the line is still intersecting

            double x3 = spherePositionX + 10 * radius;
            double y3 = spherePositionY;
            double z3 = spherePositionZ;

            Point3D pointOutside = new Point3D(x3, y3, z3);

            //Vector3D lineDirection = new Vector3D(spherePositionX - x3, spherePositionY - y3, spherePositionZ - z3);
            //System.out.println("point 3 : " + sphere3D.intersectionWith(point3, lineDirection, firstIntersectionToPack, secondIntersectionToPack));
            //assertTrue(sphere3D.intersectionWith(pointOutside, lineDirection, firstIntersectionToPack, secondIntersectionToPack) == 1);

         }
      }
      

      for (int i = 0; i < ITERATIONS; i++)
      { // No intersection
         
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);

         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         // Point outside on the line
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 1.1, 10.0) * sphere3D.getRadius(), direction, sphere3D.getCentroid());

         Vector3D lineDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, direction, false);
         
         
         Point3D actualFirstInstersection = new Point3D();
         Point3D actualSecondInstersection = new Point3D();

         // The line (pointOutside, lineDirection) is guaranteed to be outside by construction
         assertEquals(0, sphere3D.intersectionWith(pointOutside, lineDirection, null, null));
         assertEquals(0, sphere3D.intersectionWith(pointOutside, lineDirection, actualFirstInstersection, null));
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstInstersection);
         assertEquals(0, sphere3D.intersectionWith(pointOutside, lineDirection, null, actualSecondInstersection));
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondInstersection);
         actualFirstInstersection.setToZero();
         actualSecondInstersection.setToZero();
         assertEquals(0, sphere3D.intersectionWith(pointOutside, lineDirection, actualFirstInstersection, actualSecondInstersection));
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstInstersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondInstersection);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // 2 intersections
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);

         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         // Point outside on the line
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0, 0.99) * sphere3D.getRadius(), direction, sphere3D.getCentroid());

         Vector3D lineDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, direction, false);
         Point3D lineOrigin = new Point3D();
         lineOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, pointInside);

         Point3D actualFirstInstersection = new Point3D();
         Point3D actualSecondInstersection = new Point3D();

         // The line (pointOutside, lineDirection) is guaranteed to be outside by construction
         assertEquals(2, sphere3D.intersectionWith(lineOrigin, lineDirection, null, null));
         assertEquals(2, sphere3D.intersectionWith(lineOrigin, lineDirection, actualFirstInstersection, null));
         assertEquals(2, sphere3D.intersectionWith(lineOrigin, lineDirection, null, actualSecondInstersection));
         assertEquals(2, sphere3D.intersectionWith(lineOrigin, lineDirection, actualFirstInstersection, actualSecondInstersection));
         // We do not know the location of the 2 intersections
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // 2 intersections with expected intersection locations
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);

         Point3D expectedFirstInstersection = new Point3D();
         Point3D expectedSecondInstersection = new Point3D();

         Vector3D firstDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D secondDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         expectedFirstInstersection.scaleAdd(sphere3D.getRadius(), firstDirection, sphere3D.getCentroid());
         expectedSecondInstersection.scaleAdd(sphere3D.getRadius(), secondDirection, sphere3D.getCentroid());

         Point3D actualFirstInstersection = new Point3D();
         Point3D actualSecondInstersection = new Point3D();

         Point3D lineOrigin = new Point3D(expectedFirstInstersection);
         Vector3D lineDirection = new Vector3D();
         lineDirection.sub(expectedFirstInstersection, expectedSecondInstersection);
         if (random.nextBoolean())
            lineDirection.normalize();
         lineOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, lineOrigin);
         
         // The line (pointOutside, lineDirection) is guaranteed to be outside by construction
         assertEquals(2, sphere3D.intersectionWith(lineOrigin, lineDirection, null, null));
         assertEquals(2, sphere3D.intersectionWith(lineOrigin, lineDirection, actualFirstInstersection, null));
         assertEquals(2, sphere3D.intersectionWith(lineOrigin, lineDirection, null, actualSecondInstersection));
         assertEquals(2, sphere3D.intersectionWith(lineOrigin, lineDirection, actualFirstInstersection, actualSecondInstersection));
         // We do not know the location of the 2 intersections
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // 1 intersection
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);

         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         // Point outside on the line
         Point3D expectedFirstIntersection = new Point3D();
         expectedFirstIntersection.scaleAdd(sphere3D.getRadius(), direction, sphere3D.getCentroid());

         Vector3D lineDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, direction, false);
         Point3D lineOrigin = new Point3D();
         lineOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, expectedFirstIntersection);

         Point3D actualFirstInstersection = new Point3D();
         Point3D actualSecondInstersection = new Point3D();

         // The line (pointOutside, lineDirection) is guaranteed to be outside by construction
         assertEquals(1, sphere3D.intersectionWith(lineOrigin, lineDirection, null, null));
         assertEquals(1, sphere3D.intersectionWith(lineOrigin, lineDirection, actualFirstInstersection, null));
         EuclidCoreTestTools.assertEquals(expectedFirstIntersection, actualFirstInstersection, EPSILON);
         assertEquals(1, sphere3D.intersectionWith(lineOrigin, lineDirection, null, actualSecondInstersection));
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondInstersection);
         actualFirstInstersection.setToZero();
         actualSecondInstersection.setToZero();
         assertEquals(1, sphere3D.intersectionWith(lineOrigin, lineDirection, actualFirstInstersection, actualSecondInstersection));
         EuclidCoreTestTools.assertEquals(expectedFirstIntersection, actualFirstInstersection, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondInstersection);
      }
      
   }

   @Test
   void testApplyTransform()
   {
      Random random = new Random(346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Sphere3D actual = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D expected = new Sphere3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPosition().applyTransform(transform);
         actual.applyTransform(transform);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Sphere3D actual = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D expected = new Sphere3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPosition().applyTransform(transform);
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
         Sphere3D actual = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D original = new Sphere3D(actual);
         Sphere3D expected = new Sphere3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPosition().applyInverseTransform(transform);
         actual.applyInverseTransform(transform);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertEquals(original, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Sphere3D actual = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D original = new Sphere3D(actual);
         Sphere3D expected = new Sphere3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPosition().applyInverseTransform(transform);
         actual.applyInverseTransform(transform);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertEquals(original, actual, EPSILON);
      }
   }

   @Test
   void testDistance() throws Exception
   {
      Random random = new Random(12116892);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double distanceFromCenter = random.nextDouble() * sphere3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceFromCenter, direction, sphere3D.getPosition());

         assertEquals(0.0, sphere3D.distance(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double distanceFromCenter = random.nextDouble() + sphere3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceFromCenter, direction, sphere3D.getPosition());

         assertEquals(distanceFromCenter - sphere3D.getRadius(), sphere3D.distance(pointOutside), EPSILON);
      }
   }

   @Test
   void testSignedDistance() throws Exception
   {
      Random random = new Random(12116892);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double distanceFromCenter = random.nextDouble() * sphere3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceFromCenter, direction, sphere3D.getPosition());

         assertEquals(distanceFromCenter - sphere3D.getRadius(), sphere3D.signedDistance(pointInside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double distanceFromCenter = random.nextDouble() + sphere3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceFromCenter, direction, sphere3D.getPosition());

         assertEquals(distanceFromCenter - sphere3D.getRadius(), sphere3D.signedDistance(pointOutside), EPSILON);
      }
   }

   @Test
   void testOrthogonalProjection() throws Exception
   {
      Random random = new Random(9560362);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double distanceFromCenter = random.nextDouble() * sphere3D.getRadius();
         Point3D pointInside = new Point3D();
         pointInside.scaleAdd(distanceFromCenter, direction, sphere3D.getPosition());
         assertNull(sphere3D.orthogonalProjectionCopy(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double distanceFromCenter = random.nextDouble() + sphere3D.getRadius();
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distanceFromCenter, direction, sphere3D.getPosition());
         Point3D expectedProjection = new Point3D();
         expectedProjection.scaleAdd(sphere3D.getRadius(), direction, sphere3D.getPosition());

         EuclidCoreTestTools.assertEquals(expectedProjection, sphere3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }
   }

   /*
    * @Test void testIntersectionWith() throws Exception { Random random = new Random(10688467); for
    * (int i = 0; i < ITERATIONS; i++) { // Intersecting, generate the line from the two intersections
    * Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random); Point3D expectedIntersection1 =
    * new Point3D(); Point3D expectedIntersection2 = new Point3D();
    * expectedIntersection1.add(sphere3D.getPosition(),
    * EuclidCoreRandomTools.nextVector3DWithFixedLength(random, sphere3D.getRadius()));
    * expectedIntersection2.add(sphere3D.getPosition(),
    * EuclidCoreRandomTools.nextVector3DWithFixedLength(random, sphere3D.getRadius())); Line3D line =
    * new Line3D(expectedIntersection1, expectedIntersection2);
    * line.getPoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random), line.getDirection(),
    * line.getPoint()); Point3D actualIntersection1 = new Point3D(); Point3D actualIntersection2 = new
    * Point3D(); assertEquals(2, sphere3D.intersectionWith(line, actualIntersection1,
    * actualIntersection2)); EuclidCoreTestTools.assertEquals(expectedIntersection1,
    * actualIntersection1, EPSILON); EuclidCoreTestTools.assertEquals(expectedIntersection2,
    * actualIntersection2, EPSILON); } for (int i = 0; i < ITERATIONS; i++) { // Not intersecting,
    * generate the line using a point and normal on the sphere Sphere3D sphere3D =
    * EuclidShapeRandomTools.nextSphere3D(random); Vector3D normal =
    * EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0); Point3D pointOnSphere = new
    * Point3D(); pointOnSphere.setAndScale(sphere3D.getRadius(), normal);
    * pointOnSphere.add(sphere3D.getPosition()); Vector3D lineDirection =
    * EuclidCoreRandomTools.nextOrthogonalVector3D(random, normal, true); Point3D pointOnLine = new
    * Point3D(); pointOnLine.scaleAdd(random.nextDouble(), normal, pointOnSphere);
    * pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random), lineDirection, pointOnLine);
    * Line3D line = new Line3D(pointOnLine, lineDirection); assertEquals(0,
    * sphere3D.intersectionWith(line, null, null)); } }
    */
   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(34201L);
      Sphere3D firstSphere, secondSphere;
      Point3D center;
      double radius;
      double epsilon = 1e-7;

      center = EuclidCoreRandomTools.nextPoint3D(random);
      radius = random.nextDouble();

      firstSphere = new Sphere3D(center.getX(), center.getY(), center.getZ(), radius);
      secondSphere = new Sphere3D(center.getX(), center.getY(), center.getZ(), radius);

      assertTrue(firstSphere.geometricallyEquals(secondSphere, epsilon));
      assertTrue(secondSphere.geometricallyEquals(firstSphere, epsilon));
      assertTrue(firstSphere.geometricallyEquals(firstSphere, epsilon));
      assertTrue(secondSphere.geometricallyEquals(secondSphere, epsilon));

      for (int i = 0; i < ITERATIONS; ++i)
      { // Spheres are equal if radii are equal within +- epsilon and are otherwise the same
         center = EuclidCoreRandomTools.nextPoint3D(random);
         radius = random.nextDouble();

         firstSphere = new Sphere3D(center, radius);

         secondSphere = new Sphere3D(center, radius + epsilon * 0.99);

         assertTrue(firstSphere.geometricallyEquals(secondSphere, epsilon));

         secondSphere = new Sphere3D(center, radius - epsilon * 0.99);

         assertTrue(firstSphere.geometricallyEquals(secondSphere, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Spheres are not equal if radii outside of +- epsilon
         center = EuclidCoreRandomTools.nextPoint3D(random);
         radius = random.nextDouble();

         firstSphere = new Sphere3D(center, radius);

         secondSphere = new Sphere3D(center, radius + epsilon * 1.01);

         assertFalse(firstSphere.geometricallyEquals(secondSphere, epsilon));

         secondSphere = new Sphere3D(center, radius - epsilon * 1.01);

         assertFalse(firstSphere.geometricallyEquals(secondSphere, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Spheres are equal only if translations equal within +- epsilon and otherwise the same
         center = EuclidCoreRandomTools.nextPoint3D(random);
         radius = random.nextDouble();

         firstSphere = new Sphere3D(center, radius);
         secondSphere = new Sphere3D(center, radius);

         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon);

         secondSphere.getPosition().add(translation);

         assertTrue(firstSphere.geometricallyEquals(secondSphere, epsilon));

         secondSphere = new Sphere3D(center, radius);

         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon);

         secondSphere.getPosition().add(translation);

         assertFalse(firstSphere.geometricallyEquals(secondSphere, epsilon));
      }
   }

   @Test
   void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(546161);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Sphere3D sphere = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly supportingVertex = sphere.getSupportingVertex(supportDirection);
         Point3D supportingVertexTranslated = new Point3D();
         supportDirection.normalize();
         assertTrue(sphere.isPointInside(supportingVertex, EPSILON));
         supportingVertexTranslated.scaleAdd(1.0e-6, supportDirection, supportingVertex);
         assertFalse(sphere.isPointInside(supportingVertexTranslated, EPSILON));
         supportingVertexTranslated.scaleAdd(1.0e-2, supportDirection, supportingVertex);
         Vector3D expectedNormal = new Vector3D();
         expectedNormal.sub(supportingVertexTranslated, supportingVertex);
         expectedNormal.normalize();

         Vector3D actualNormal = new Vector3D();
         sphere.evaluatePoint3DCollision(supportingVertexTranslated, new Point3D(), actualNormal);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }
   }

   @Test
   void testGetBoundingBox() throws Exception
   {
      Random random = new Random(36342);

      for (int i = 0; i < ITERATIONS; i++)
      { // Using getSupportingVertex
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);

         BoundingBox3D expectedBoundingBox = new BoundingBox3D();
         expectedBoundingBox.setToNaN();
         Vector3D supportDirection = new Vector3D(Axis3D.X);
         expectedBoundingBox.updateToIncludePoint(sphere3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(sphere3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis3D.Y);
         expectedBoundingBox.updateToIncludePoint(sphere3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(sphere3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis3D.Z);
         expectedBoundingBox.updateToIncludePoint(sphere3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(sphere3D.getSupportingVertex(supportDirection));

         BoundingBox3DReadOnly actualBoundingBox = sphere3D.getBoundingBox();
         EuclidCoreTestTools.assertEquals(expectedBoundingBox, actualBoundingBox, EPSILON);
      }
   }
}
