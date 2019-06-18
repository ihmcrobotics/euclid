package us.ihmc.euclid.shape.primitives;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidEllipsoid3DTools;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public class Ellipsoid3DTest
{
   private static final double EPSILON = 1.0e-10;

   @Test
   void testConstructors() throws Exception
   {
      Random random = new Random(45675654);

      { // Empty constructor
         Ellipsoid3D ellipsoid3D = new Ellipsoid3D();

         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1, 1, 1), ellipsoid3D.getRadii(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(ellipsoid3D.getPosition());
         EuclidCoreTestTools.assertIdentity(ellipsoid3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ellipsoid3D(double radiusX, double radiusY, double radiusZ)
         double radiusX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double radiusY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double radiusZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Ellipsoid3D ellipsoid3D = new Ellipsoid3D(radiusX, radiusY, radiusZ);

         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(radiusX, radiusY, radiusZ), ellipsoid3D.getRadii(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(ellipsoid3D.getPosition());
         EuclidCoreTestTools.assertIdentity(ellipsoid3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ellipsoid3D(Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
         double radiusX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double radiusY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double radiusZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random);
         RotationMatrix orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
         Ellipsoid3D ellipsoid3D = new Ellipsoid3D(position, orientation, radiusX, radiusY, radiusZ);

         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(radiusX, radiusY, radiusZ), ellipsoid3D.getRadii(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(position, ellipsoid3D.getPosition(), EPSILON);
         EuclidCoreTestTools.assertMatrix3DEquals(orientation, ellipsoid3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ellipsoid3D(Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
         double radiusX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double radiusY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double radiusZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random);
         RotationMatrix orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
         Pose3D pose = new Pose3D(position, orientation);
         Ellipsoid3D ellipsoid3D = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);

         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(radiusX, radiusY, radiusZ), ellipsoid3D.getRadii(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(position, ellipsoid3D.getPosition(), EPSILON);
         EuclidCoreTestTools.assertMatrix3DEquals(orientation, ellipsoid3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ellipsoid3D(RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
         double sizeX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random);
         RotationMatrix orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RigidBodyTransform pose = new RigidBodyTransform(orientation, position);
         Ellipsoid3D ellipsoid3D = new Ellipsoid3D(pose, sizeX, sizeY, sizeZ);

         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(sizeX, sizeY, sizeZ), ellipsoid3D.getRadii(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(position, ellipsoid3D.getPosition(), EPSILON);
         EuclidCoreTestTools.assertMatrix3DEquals(orientation, ellipsoid3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ellipsoid3D(Ellipsoid3DReadOnly other)
         Ellipsoid3D original = EuclidShapeRandomTools.nextEllipsoid3D(random);
         Ellipsoid3D copy = new Ellipsoid3D(original);

         EuclidShapeTestTools.assertEllipsoid3DEquals(original, copy, EPSILON);
      }
   }

   @Test
   void testSetToNaN() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         assertFalse(ellipsoid3D.containsNaN());
         assertFalse(ellipsoid3D.getPose().containsNaN());
         assertFalse(ellipsoid3D.getPosition().containsNaN());
         assertFalse(ellipsoid3D.getOrientation().containsNaN());
         assertFalse(ellipsoid3D.getRadii().containsNaN());

         ellipsoid3D.setToNaN();

         assertTrue(ellipsoid3D.containsNaN());
         assertTrue(ellipsoid3D.getPose().containsNaN());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(ellipsoid3D.getPosition());
         EuclidCoreTestTools.assertMatrix3DContainsOnlyNaN(ellipsoid3D.getOrientation());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(ellipsoid3D.getRadii());
      }
   }

   @Test
   void testSetToZero() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         assertFalse(new Point3D().epsilonEquals(ellipsoid3D.getPosition(), EPSILON));
         assertFalse(new Point3D().epsilonEquals(ellipsoid3D.getRadii(), EPSILON));
         assertFalse(new RotationMatrix().epsilonEquals(ellipsoid3D.getOrientation(), EPSILON));

         ellipsoid3D.setToZero();

         EuclidCoreTestTools.assertTuple3DIsSetToZero(ellipsoid3D.getPosition());
         EuclidCoreTestTools.assertIdentity(ellipsoid3D.getOrientation(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(ellipsoid3D.getRadii());
      }
   }

   @Test
   void testSetters() throws Exception
   {
      Random random = new Random(45837543);

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Ellipsoid3D other)
         Ellipsoid3D expected = EuclidShapeRandomTools.nextEllipsoid3D(random);
         Ellipsoid3D actual = EuclidShapeRandomTools.nextEllipsoid3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set(expected);
         EuclidShapeTestTools.assertEllipsoid3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Ellipsoid3DReadOnly other)
         Ellipsoid3D expected = EuclidShapeRandomTools.nextEllipsoid3D(random);
         Ellipsoid3D actual = EuclidShapeRandomTools.nextEllipsoid3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set((Ellipsoid3DReadOnly) expected);
         EuclidShapeTestTools.assertEllipsoid3DEquals(expected, actual, EPSILON);
      }

      { // set(Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Ellipsoid3D expected = EuclidShapeRandomTools.nextEllipsoid3D(random);
            Ellipsoid3D actual = EuclidShapeRandomTools.nextEllipsoid3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(expected.getPosition(), expected.getOrientation(), expected.getRadiusX(), expected.getRadiusY(), expected.getRadiusZ());
            EuclidShapeTestTools.assertEllipsoid3DEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new Point3D(), new Quaternion(), -0.1, 1.0, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new Point3D(), new Quaternion(), 1.0, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new Point3D(), new Quaternion(), 1.0, 1.0, -0.1));
      }

      { // set(Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Ellipsoid3D expected = EuclidShapeRandomTools.nextEllipsoid3D(random);
            Ellipsoid3D actual = EuclidShapeRandomTools.nextEllipsoid3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(new Pose3D(expected.getPosition(), expected.getOrientation()), expected.getRadiusX(), expected.getRadiusY(), expected.getRadiusZ());
            EuclidShapeTestTools.assertEllipsoid3DEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new Pose3D(), -0.1, 1.0, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new Pose3D(), 1.0, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new Pose3D(), 1.0, 1.0, -0.1));
      }

      { // set(RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Ellipsoid3D expected = EuclidShapeRandomTools.nextEllipsoid3D(random);
            Ellipsoid3D actual = EuclidShapeRandomTools.nextEllipsoid3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(new RigidBodyTransform(expected.getOrientation(), expected.getPosition()),
                       expected.getRadiusX(),
                       expected.getRadiusY(),
                       expected.getRadiusZ());
            EuclidShapeTestTools.assertEllipsoid3DEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new RigidBodyTransform(), -0.1, 1.0, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new RigidBodyTransform(), 1.0, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new RigidBodyTransform(), 1.0, 1.0, -0.1));
      }

      { // set(RigidBodyTransformReadOnly pose, double[] size)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Ellipsoid3D expected = EuclidShapeRandomTools.nextEllipsoid3D(random);
            Ellipsoid3D actual = EuclidShapeRandomTools.nextEllipsoid3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(new RigidBodyTransform(expected.getOrientation(), expected.getPosition()),
                       new double[] {expected.getRadiusX(), expected.getRadiusY(), expected.getRadiusZ()});
            EuclidShapeTestTools.assertEllipsoid3DEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new RigidBodyTransform(), new double[] {-0.1, 1.0, 1.0}));
         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new RigidBodyTransform(), new double[] {1.0, -0.1, 1.0}));
         assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().set(new RigidBodyTransform(), new double[] {1.0, 1.0, -0.1}));
      }
   }

   @Test
   void testSetRadii() throws Exception
   {
      Random random = new Random(5465446);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double radiusX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double radiusY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double radiusZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);
         assertFalse(EuclidCoreTools.epsilonEquals(radiusX, ellipsoid3D.getRadiusX(), EPSILON));
         assertFalse(EuclidCoreTools.epsilonEquals(radiusY, ellipsoid3D.getRadiusY(), EPSILON));
         assertFalse(EuclidCoreTools.epsilonEquals(radiusZ, ellipsoid3D.getRadiusZ(), EPSILON));
         ellipsoid3D.setRadii(radiusX, radiusY, radiusZ);
         assertEquals(radiusX, ellipsoid3D.getRadiusX(), EPSILON);
         assertEquals(radiusY, ellipsoid3D.getRadiusY(), EPSILON);
         assertEquals(radiusZ, ellipsoid3D.getRadiusZ(), EPSILON);
      }

      assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().setRadii(-0.1, 1.0, 1.0));
      assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().setRadii(1.0, -0.1, 1.0));
      assertThrows(IllegalArgumentException.class, () -> new Ellipsoid3D().setRadii(1.0, 1.0, -0.1));
   }

   @Test
   void testIsPointInside() throws Exception
   {
      Random random = new Random(3656);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, generated to be inside unit-sphere and then scaled up by the radii
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         double radiusInUnitSphere = random.nextDouble();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, radiusInUnitSphere);
         translation.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());
         Point3D pointInside = new Point3D(translation);
         ellipsoid3D.transformToWorld(pointInside);
         assertTrue(ellipsoid3D.isPointInside(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, generated to be outside unit-sphere and then scaled up by the radii
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         double radiusInUnitSphere = 1.0 + random.nextDouble();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, radiusInUnitSphere);
         translation.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());
         Point3D pointOutside = new Point3D(translation);
         ellipsoid3D.transformToWorld(pointOutside);
         assertFalse(ellipsoid3D.isPointInside(pointOutside));
      }
   }

   @Test
   void testEvaluatePoint3DCollision() throws Exception
   {
      Random random = new Random(345345);

      Point3D actualClosestPointWorld = new Point3D();
      Point3D actualClosestPointLocal = new Point3D();
      Vector3D actualNormal = new Vector3D();
      Vector3D expectedNormal = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, generated to be inside unit-sphere and then scaled up by the radii
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random, 0.001, 2.0);

         double radiusInUnitSphere = random.nextDouble();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, radiusInUnitSphere);
         Point3D pointInside = new Point3D(translation);
         pointInside.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());

         ellipsoid3D.transformToWorld(pointInside);

         assertTrue(ellipsoid3D.evaluatePoint3DCollision(pointInside, actualClosestPointWorld, actualNormal));
         actualClosestPointLocal.set(actualClosestPointWorld);
         ellipsoid3D.transformToLocal(actualClosestPointLocal);
         assertEquals(0.0,
                      EuclidShapeTools.signedDistanceBetweenPoint3DAndEllipsoid3D(actualClosestPointLocal, ellipsoid3D.getRadii()),
                      EPSILON,
                      "Iteration: " + i);

         // The normal should be pointing from the query to the closest point (toward the outside).
         expectedNormal.sub(actualClosestPointWorld, pointInside);
         expectedNormal.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);

         // The normal should be orthogonal to the ellipsoid surface at the closest point.
         expectedNormal.set(actualClosestPointLocal);
         double rxSquare = ellipsoid3D.getRadiusX() * ellipsoid3D.getRadiusX();
         double rySquare = ellipsoid3D.getRadiusY() * ellipsoid3D.getRadiusY();
         double rzSquare = ellipsoid3D.getRadiusZ() * ellipsoid3D.getRadiusZ();
         expectedNormal.scale(1.0 / rxSquare, 1.0 / rySquare, 1.0 / rzSquare);
         expectedNormal.normalize();
         ellipsoid3D.transformToWorld(expectedNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, generated to be outside unit-sphere and then scaled up by the radii
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random, 0.001, 2.0);

         double radiusInUnitSphere = 1.0 + random.nextDouble();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, radiusInUnitSphere);
         Point3D pointOutside = new Point3D(translation);
         pointOutside.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());

         ellipsoid3D.transformToWorld(pointOutside);

         assertFalse(ellipsoid3D.evaluatePoint3DCollision(pointOutside, actualClosestPointWorld, actualNormal));
         actualClosestPointLocal.set(actualClosestPointWorld);
         ellipsoid3D.transformToLocal(actualClosestPointLocal);
         assertEquals(0.0,
                      EuclidShapeTools.signedDistanceBetweenPoint3DAndEllipsoid3D(actualClosestPointLocal, ellipsoid3D.getRadii()),
                      EPSILON,
                      "Iteration: " + i);

         // The normal should be pointing from the closest point to the query (toward the outside).
         expectedNormal.sub(pointOutside, actualClosestPointWorld);
         expectedNormal.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);

         // The normal should be orthogonal to the ellipsoid surface at the closest point.
         expectedNormal.set(actualClosestPointLocal);
         double rxSquare = ellipsoid3D.getRadiusX() * ellipsoid3D.getRadiusX();
         double rySquare = ellipsoid3D.getRadiusY() * ellipsoid3D.getRadiusY();
         double rzSquare = ellipsoid3D.getRadiusZ() * ellipsoid3D.getRadiusZ();
         expectedNormal.scale(1.0 / rxSquare, 1.0 / rySquare, 1.0 / rzSquare);
         expectedNormal.normalize();
         ellipsoid3D.transformToWorld(expectedNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point on surface
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random, 0.001, 2.0);

         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Point3D pointOnSurface = new Point3D(translation);
         pointOnSurface.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());

         ellipsoid3D.transformToWorld(pointOnSurface);

         assertTrue(ellipsoid3D.evaluatePoint3DCollision(pointOnSurface, actualClosestPointWorld, actualNormal));
         actualClosestPointLocal.set(actualClosestPointWorld);
         ellipsoid3D.transformToLocal(actualClosestPointLocal);
         assertEquals(0.0,
                      EuclidShapeTools.signedDistanceBetweenPoint3DAndEllipsoid3D(actualClosestPointLocal, ellipsoid3D.getRadii()),
                      EPSILON,
                      "Iteration: " + i);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnSurface, actualClosestPointWorld, EPSILON);
         // The normal should be orthogonal to the ellipsoid surface at the closest point.
         expectedNormal.set(actualClosestPointLocal);
         double rxSquare = ellipsoid3D.getRadiusX() * ellipsoid3D.getRadiusX();
         double rySquare = ellipsoid3D.getRadiusY() * ellipsoid3D.getRadiusY();
         double rzSquare = ellipsoid3D.getRadiusZ() * ellipsoid3D.getRadiusZ();
         expectedNormal.scale(1.0 / rxSquare, 1.0 / rySquare, 1.0 / rzSquare);
         expectedNormal.normalize();
         ellipsoid3D.transformToWorld(expectedNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point lying on 1, 2, or 3 of the ellipsoid axes
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random, 0.001, 2.0);
         Point3D point = new Point3D(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.5));
         if (random.nextBoolean())
            point.setX(0.0);
         if (random.nextBoolean())
            point.setY(0.0);
         if (random.nextBoolean())
            point.setZ(0.0);
         ellipsoid3D.transformToWorld(point);

         ellipsoid3D.evaluatePoint3DCollision(point, actualClosestPointWorld, actualNormal);
         actualClosestPointLocal.set(actualClosestPointWorld);
         ellipsoid3D.transformToLocal(actualClosestPointLocal);
         assertEquals(0.0,
                      EuclidShapeTools.signedDistanceBetweenPoint3DAndEllipsoid3D(actualClosestPointLocal, ellipsoid3D.getRadii()),
                      EPSILON,
                      "Iteration: " + i);
         // The normal should be orthogonal to the ellipsoid surface at the closest point.
         expectedNormal.set(actualClosestPointLocal);
         double rxSquare = ellipsoid3D.getRadiusX() * ellipsoid3D.getRadiusX();
         double rySquare = ellipsoid3D.getRadiusY() * ellipsoid3D.getRadiusY();
         double rzSquare = ellipsoid3D.getRadiusZ() * ellipsoid3D.getRadiusZ();
         expectedNormal.scale(1.0 / rxSquare, 1.0 / rySquare, 1.0 / rzSquare);
         expectedNormal.normalize();
         ellipsoid3D.transformToWorld(expectedNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }
   }

   @Test
   void testApplyTransform()
   {
      Random random = new Random(346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ellipsoid3D actual = EuclidShapeRandomTools.nextEllipsoid3D(random);
         Ellipsoid3D expected = new Ellipsoid3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPose().applyTransform(transform);
         actual.applyTransform(transform);

         EuclidShapeTestTools.assertEllipsoid3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ellipsoid3D actual = EuclidShapeRandomTools.nextEllipsoid3D(random);
         Ellipsoid3D expected = new Ellipsoid3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPose().applyTransform(transform);
         actual.applyTransform(transform);

         EuclidShapeTestTools.assertEllipsoid3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   void testApplyInverseTransform()
   {
      Random random = new Random(346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ellipsoid3D actual = EuclidShapeRandomTools.nextEllipsoid3D(random);
         Ellipsoid3D original = new Ellipsoid3D(actual);
         Ellipsoid3D expected = new Ellipsoid3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPose().applyInverseTransform(transform);
         actual.applyInverseTransform(transform);

         EuclidShapeTestTools.assertEllipsoid3DEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidShapeTestTools.assertEllipsoid3DEquals(original, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ellipsoid3D actual = EuclidShapeRandomTools.nextEllipsoid3D(random);
         Ellipsoid3D original = new Ellipsoid3D(actual);
         Ellipsoid3D expected = new Ellipsoid3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPose().applyInverseTransform(transform);
         actual.applyInverseTransform(transform);

         EuclidShapeTestTools.assertEllipsoid3DEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidShapeTestTools.assertEllipsoid3DEquals(original, actual, EPSILON);
      }
   }

   @Test
   void testDistance() throws Exception
   {
      Random random = new Random(2147385);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, generated to be inside unit-sphere and then scaled up by the radii
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         double radiusInUnitSphere = random.nextDouble();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, radiusInUnitSphere);
         Point3D pointInside = new Point3D(translation);
         pointInside.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());

         ellipsoid3D.transformToWorld(pointInside);
         assertEquals(0.0, ellipsoid3D.distance(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, generated to be outside unit-sphere and then scaled up by the radii
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         double radiusInUnitSphere = 1.0 + random.nextDouble();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, radiusInUnitSphere);
         Point3D pointOutsideLocal = new Point3D(translation);
         pointOutsideLocal.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());

         Point3D pointOutsideWorld = new Point3D(pointOutsideLocal);
         ellipsoid3D.transformToWorld(pointOutsideWorld);
         double expectedDistance = Math.max(0.0, EuclidEllipsoid3DTools.distancePoint3DEllipsoid3D(ellipsoid3D.getRadii(), pointOutsideLocal));
         assertEquals(expectedDistance, ellipsoid3D.distance(pointOutsideWorld), EPSILON, "Iteration: " + i);
      }
   }

   @Test
   void testSignedDistance() throws Exception
   {
      Random random = new Random(2147385);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, generated to be inside unit-sphere and then scaled up by the radii
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         double radiusInUnitSphere = random.nextDouble();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, radiusInUnitSphere);
         Point3D pointInsideLocal = new Point3D(translation);
         pointInsideLocal.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());

         Point3D pointInsideWorld = new Point3D(pointInsideLocal);
         ellipsoid3D.transformToWorld(pointInsideWorld);
         double expectedDistance = EuclidEllipsoid3DTools.distancePoint3DEllipsoid3D(ellipsoid3D.getRadii(), pointInsideLocal);
         assertEquals(expectedDistance, ellipsoid3D.signedDistance(pointInsideWorld), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, generated to be outside unit-sphere and then scaled up by the radii
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         double radiusInUnitSphere = 1.0 + random.nextDouble();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, radiusInUnitSphere);
         Point3D pointOutsideLocal = new Point3D(translation);
         pointOutsideLocal.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());

         Point3D pointOutsideWorld = new Point3D(pointOutsideLocal);
         ellipsoid3D.transformToWorld(pointOutsideWorld);
         double expectedDistance = EuclidEllipsoid3DTools.distancePoint3DEllipsoid3D(ellipsoid3D.getRadii(), pointOutsideLocal);
         assertEquals(expectedDistance, ellipsoid3D.signedDistance(pointOutsideWorld), EPSILON);
      }
   }

   @Test
   void testOrthogonalProjection() throws Exception
   {
      Random random = new Random(4157885);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, generated to be inside unit-sphere and then scaled up by the radii
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         double radiusInUnitSphere = random.nextDouble();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, radiusInUnitSphere);
         Point3D pointInside = new Point3D(translation);
         pointInside.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());

         ellipsoid3D.transformToWorld(pointInside);
         assertNull(ellipsoid3D.orthogonalProjectionCopy(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, we rely on doPoint3DCollisionTest to test orthogonalProjection
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         double radiusInUnitSphere = 1.0 + random.nextDouble();
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, radiusInUnitSphere);
         Point3D pointOutside = new Point3D(translation);
         pointOutside.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());

         ellipsoid3D.transformToWorld(pointOutside);

         Point3D expectedProjection = new Point3D();
         ellipsoid3D.evaluatePoint3DCollision(pointOutside, expectedProjection, new Vector3D());
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, ellipsoid3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }
   }

   @Test
   void testIntersectionWith() throws Exception
   {
      Random random = new Random(10688467);

      for (int i = 0; i < ITERATIONS; i++)
      { // Intersecting, generate the line from the two intersections
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         Point3D expectedIntersection1 = new Point3D(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0));
         expectedIntersection1.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());
         Point3D expectedIntersection2 = new Point3D(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0));
         expectedIntersection2.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());

         ellipsoid3D.transformToWorld(expectedIntersection1);
         ellipsoid3D.transformToWorld(expectedIntersection2);

         Line3D line = new Line3D(expectedIntersection1, expectedIntersection2);
         line.getPoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random), line.getDirection(), line.getPoint());
         Point3D actualIntersection1 = new Point3D();
         Point3D actualIntersection2 = new Point3D();
         assertEquals(2, ellipsoid3D.intersectionWith(line, actualIntersection1, actualIntersection2));
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Not intersecting, generate the line using a point and normal on the ellipsoid
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         Vector3D direction = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         Point3D pointOnEllipsoid = new Point3D(direction);
         Vector3D normal = new Vector3D(direction);
         pointOnEllipsoid.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());
         normal.scale(1.0 / ellipsoid3D.getRadiusX(), 1.0 / ellipsoid3D.getRadiusY(), 1.0 / ellipsoid3D.getRadiusZ());
         normal.normalize();

         Vector3D lineDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normal, true);
         Point3D pointOnLine = new Point3D();
         pointOnLine.scaleAdd(random.nextDouble(), normal, pointOnEllipsoid);
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random), lineDirection, pointOnLine);
         Line3D line = new Line3D(pointOnLine, lineDirection);

         ellipsoid3D.transformToWorld(line);
         assertEquals(0, ellipsoid3D.intersectionWith(line, null, null));
      }
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(89725L);

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test with identical ellipsoids
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();

         Ellipsoid3D firstEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);
         Ellipsoid3D secondEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(secondEllipsoid.geometricallyEquals(firstEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(firstEllipsoid.geometricallyEquals(firstEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(secondEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);

         firstEllipsoid = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);
         secondEllipsoid = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(secondEllipsoid.geometricallyEquals(firstEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(firstEllipsoid.geometricallyEquals(firstEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(secondEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test 2 ellipsoids with same pose and different radii - Method 1
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 0.1);
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         // Generating random radii while making sure we don't generate spheres
         double radiusX1 = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double radiusY1 = EuclidCoreRandomTools.nextDouble(random, radiusX1 + 2.0 * epsilon, radiusX1 + 1.0);
         double radiusZ1 = EuclidCoreRandomTools.nextDouble(random, radiusY1 + 2.0 * epsilon, radiusY1 + 1.0);
         double radiusX2 = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double radiusY2 = EuclidCoreRandomTools.nextDouble(random, radiusX2 + 2.0 * epsilon, radiusX2 + 1.0);
         double radiusZ2 = EuclidCoreRandomTools.nextDouble(random, radiusY2 + 2.0 * epsilon, radiusY2 + 1.0);
         Vector3D radii1 = new Vector3D(radiusX1, radiusY1, radiusZ1);
         Vector3D radii2 = new Vector3D(radiusX2, radiusY2, radiusZ2);
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusX1, radiusY1, radiusZ1);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusX2, radiusY2, radiusZ2);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON) == radii1.geometricallyEquals(radii2, EPSILON), "Iteration: " + i);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, epsilon) == radii1.geometricallyEquals(radii2, epsilon), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test 2 ellipsoids with same pose and different radii - Method 2
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 0.1);
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         // Generating random radii while making sure we don't generate spheres
         double radiusX1 = EuclidCoreRandomTools.nextDouble(random, 2.0 * epsilon, 10.0);
         double radiusY1 = EuclidCoreRandomTools.nextDouble(random, radiusX1 + 2.0 * epsilon, radiusX1 + 1.0);
         double radiusZ1 = EuclidCoreRandomTools.nextDouble(random, radiusY1 + 2.0 * epsilon, radiusY1 + 1.0);
         Vector3D radii1 = new Vector3D(radiusX1, radiusY1, radiusZ1);
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radii1.getX(), radii1.getY(), radii1.getZ());

         Vector3D radii2 = new Vector3D();
         radii2.add(radii1, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon));
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radii2.getX(), radii2.getY(), radii2.getZ());
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, epsilon) == radii1.geometricallyEquals(radii2, epsilon), "Iteration: " + i);

         radii2.add(radii1, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon));
         ellipsoid2 = new Ellipsoid3D(pose, radii2.getX(), radii2.getY(), radii2.getZ());
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, epsilon) == radii1.geometricallyEquals(radii2, epsilon), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Ellipsoids are equal if translations are equal within +- epsilon and are otherwise the same
         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();

         Ellipsoid3D firstEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);

         Ellipsoid3D secondEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * EPSILON);
         secondEllipsoid.getPose().appendTranslation(translation);

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);

         secondEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);
         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * EPSILON);
         secondEllipsoid.getPose().appendTranslation(translation);

         assertFalse(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Ellipsoids are equal if translations are equal within +- epsilon and are otherwise the same
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();

         Ellipsoid3D firstEllipsoid = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);

         Ellipsoid3D secondEllipsoid = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * EPSILON);
         secondEllipsoid.getPose().appendTranslation(translation);

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);

         secondEllipsoid = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);
         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * EPSILON);
         secondEllipsoid.getPose().appendTranslation(translation);

         assertFalse(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Show that if the three radii are equal, the rotation does not matter as we are dealing with spheres.
         double radius = random.nextDouble();
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         RigidBodyTransform pose1 = new RigidBodyTransform(EuclidCoreRandomTools.nextAxisAngle(random), position);
         RigidBodyTransform pose2 = new RigidBodyTransform(EuclidCoreRandomTools.nextAxisAngle(random), position);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radius, radius, radius);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radius, radius, radius);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.setRadii(radius + 0.99 * EPSILON, radius + 0.99 * EPSILON, radius + 0.99 * EPSILON);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.setRadii(radius + 1.19 * EPSILON, radius + 0.99 * EPSILON, radius + 0.79 * EPSILON);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.setRadii(radius + 0.99 * EPSILON, radius + 1.19 * EPSILON, radius + 0.79 * EPSILON);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.setRadii(radius + 1.01 * EPSILON, radius + 1.01 * EPSILON, radius + 1.01 * EPSILON);
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.setRadii(radius + 1.11 * EPSILON, radius + 1.01 * EPSILON, radius + 0.91 * EPSILON);
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // The first ellipsoid is a sphere, ensuring that the second is tested before assuming it is also a sphere.
         double radius = random.nextDouble();
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         RigidBodyTransform pose1 = new RigidBodyTransform(EuclidCoreRandomTools.nextAxisAngle(random), position);
         RigidBodyTransform pose2 = new RigidBodyTransform(EuclidCoreRandomTools.nextAxisAngle(random), position);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radius, radius, radius);

         Vector3D radii = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         radii.absolute();
         double average = (radii.getX() + radii.getY() + radii.getZ()) / 3.0;
         radii.scale(average / radii.length());

         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radii.getX(), radii.getY(), radii.getZ());

         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         radii = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         radii.setX(radii.getY());
         radii.absolute();
         average = (radii.getX() + radii.getY() + radii.getZ()) / 3.0;
         radii.scale(average / radii.length());

         ellipsoid2 = new Ellipsoid3D(pose2, radii.getX(), radii.getY(), radii.getZ());

         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Show that if two of the radii are equal, the rotation around the third axis is negligible.
        // Radii X and Y are equal, rotations around Z and 180 degree flip around any axis orthogonal to Z do not affect the method.
         double radiusXY = random.nextDouble();
         double radiusZ = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXY, radiusXY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusXY, radiusXY, radiusZ);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.getPose().appendYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 0.0, 1.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));
         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Show that if two of the radii are equal, the rotation around the third axis is negligible.
        // Radii X and Z are equal, rotations around Y and 180 degree flip around any axis orthogonal to Y do not affect the method.
         double radiusXZ = random.nextDouble();
         double radiusY = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXZ, radiusY, radiusXZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusXZ, radiusY, radiusXZ);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.getPose().appendPitchRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 1.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Show that if two of the radii are equal, the rotation around the third axis is negligible.
        // Radii Y and Z are equal, rotations around X and 180 degree flip around any axis orthogonal to X do not affect the method.
         double radiusX = random.nextDouble();
         double radiusYZ = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusX, radiusYZ, radiusYZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusX, radiusYZ, radiusYZ);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.getPose().appendRollRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(1.0, 0.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test 180 degree flips around the x, y, or z axis
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);

         int axis = random.nextInt(3);
         switch (axis)
         {
            case 0:
               ellipsoid2.getPose().appendRollRotation(Math.PI);
               break;
            case 1:
               ellipsoid2.getPose().appendPitchRotation(Math.PI);
               break;
            case 2:
               ellipsoid2.getPose().appendYawRotation(Math.PI);
               break;
            default:
               throw new RuntimeException("Unexpected axis value: " + axis);
         }

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         double angle = EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI / 2.0);
         switch (axis)
         {
            case 0:
               ellipsoid2.getPose().appendRollRotation(angle);
               break;
            case 1:
               ellipsoid2.getPose().appendPitchRotation(angle);
               break;
            case 2:
               ellipsoid2.getPose().appendYawRotation(angle);
               break;
            default:
               throw new RuntimeException("Unexpected axis value: " + axis);
         }

         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Swapping two of the radii components and adding a 90-degree rotation such that the two ellipsoids should represent the same geometry
        // Swapping X <-> Y and adding a 90-degree rotation around Z
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendYawRotation(Math.PI / 2.0);

         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double heightZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radiusX, radiusY, heightZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radiusY, radiusX, heightZ);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Swapping two of the radii components and adding a 90-degree rotation such that the two ellipsoids should represent the same geometry
        // Swapping X <-> Z and adding a 90-degree rotation around Y
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendPitchRotation(Math.PI / 2.0);

         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radiusX, radiusY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radiusZ, radiusY, radiusX);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Swapping two of the radii components and adding a 90-degree rotation such that the two ellipsoids should represent the same geometry
        // Swapping Y <-> Z and adding a 90-degree rotation around X
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendRollRotation(Math.PI / 2.0);

         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radiusX, radiusY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radiusX, radiusZ, radiusY);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Permuting the three radii components and adding a 90-degree rotations such that the two ellipsoids should represent the same geometry
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendYawRotation(Math.PI / 2.0);
         pose2.appendPitchRotation(Math.PI / 2.0);

         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radiusX, radiusY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radiusZ, radiusX, radiusY);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Permuting the three radii components and adding a 90-degree rotations such that the two ellipsoids should represent the same geometry
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendYawRotation(Math.PI / 2.0);
         pose2.appendRollRotation(Math.PI / 2.0);

         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radiusX, radiusY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radiusY, radiusZ, radiusX);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii X and Y are equal.
        // 2- Swap X <-> Z
         double radiusXY = random.nextDouble();
         double radiusZ = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXY, radiusXY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusZ, radiusXY, radiusXY);
         ellipsoid2.getPose().appendPitchRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.getPose().appendRollRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(1.0, 0.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii X and Y are equal.
        // 2- Swap Y <-> Z
         double radiusXY = random.nextDouble();
         double radiusZ = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXY, radiusXY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusXY, radiusZ, radiusXY);
         ellipsoid2.getPose().appendRollRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.getPose().appendPitchRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 1.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii X and Z are equal.
        // 2- Swap X <-> Y
         double radiusXZ = random.nextDouble();
         double radiusY = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXZ, radiusY, radiusXZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusY, radiusXZ, radiusXZ);
         ellipsoid2.getPose().appendYawRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.getPose().appendRollRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(1.0, 0.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii X and Z are equal.
        // 2- Swap Y <-> Z
         double radiusXZ = random.nextDouble();
         double radiusY = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXZ, radiusY, radiusXZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusXZ, radiusXZ, radiusY);
         ellipsoid2.getPose().appendRollRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.getPose().appendYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 0.0, 1.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii Y and Z are equal.
        // 2- Swap X <-> Y
         double radiusYZ = random.nextDouble();
         double radiusX = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusX, radiusYZ, radiusYZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusYZ, radiusX, radiusYZ);
         ellipsoid2.getPose().appendYawRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.getPose().appendPitchRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 1.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii Y and Z are equal.
        // 2- Swap X <-> Z
         double radiusYZ = random.nextDouble();
         double radiusX = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusX, radiusYZ, radiusYZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusYZ, radiusYZ, radiusX);
         ellipsoid2.getPose().appendPitchRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.getPose().appendYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 0.0, 1.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.getPose().multiply(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }
   }

   @Test
   void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(546161);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ellipsoid3D ellipsoid = EuclidShapeRandomTools.nextEllipsoid3D(random);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly supportingVertex = ellipsoid.getSupportingVertex(supportDirection);
         assertTrue(ellipsoid.isPointInside(supportingVertex, EPSILON));

         Point3D supportingVertexTranslated = new Point3D();
         supportDirection.normalize();
         supportingVertexTranslated.scaleAdd(1.0e-6, supportDirection, supportingVertex);
         assertFalse(ellipsoid.isPointInside(supportingVertexTranslated, EPSILON));

         for (int j = 0; j < 100; j++)
         {
            // Slightly translated the supporting vertex on plane orthogonal to the direction and asserting that the resulting point isn't inside.
            Vector3D orthogonalToSupportDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, supportDirection, true);
            Point3D pointSupposedlyOutside = new Point3D();
            pointSupposedlyOutside.scaleAdd(1.0e-4 * ellipsoid.getRadii().length(), orthogonalToSupportDirection, supportingVertex);
            assertFalse(ellipsoid.isPointInside(pointSupposedlyOutside, EPSILON));
         }

         supportingVertexTranslated.scaleAdd(1.0e-8, supportDirection, supportingVertex);
         assertFalse(ellipsoid.isPointInside(supportingVertexTranslated, EPSILON));

         Vector3D actualNormal = new Vector3D();
         ellipsoid.evaluatePoint3DCollision(supportingVertex, new Point3D(), actualNormal);
         EuclidCoreTestTools.assertTuple3DEquals(supportDirection, actualNormal, EPSILON);
      }
   }

   @Test
   void testGetBoundingBox() throws Exception
   {
      Random random = new Random(36342);

      for (int i = 0; i < ITERATIONS; i++)
      { // Using getSupportingVertex
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random);

         BoundingBox3D expectedBoundingBox = new BoundingBox3D();
         expectedBoundingBox.setToNaN();
         Vector3D supportDirection = new Vector3D(Axis.X);
         expectedBoundingBox.updateToIncludePoint(ellipsoid3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(ellipsoid3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis.Y);
         expectedBoundingBox.updateToIncludePoint(ellipsoid3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(ellipsoid3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis.Z);
         expectedBoundingBox.updateToIncludePoint(ellipsoid3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(ellipsoid3D.getSupportingVertex(supportDirection));

         BoundingBox3DReadOnly actualBoundingBox = ellipsoid3D.getBoundingBox();
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expectedBoundingBox, actualBoundingBox, EPSILON);
      }
   }
}
