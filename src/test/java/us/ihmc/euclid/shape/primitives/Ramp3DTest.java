package us.ihmc.euclid.shape.primitives;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Arrays;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
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
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public class Ramp3DTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   void testConstructors() throws Exception
   {
      Random random = new Random(10869);

      { // Empty constructor
         Ramp3D ramp3D = new Ramp3D();

         EuclidCoreTestTools.assertEquals(new Vector3D(1, 1, 1), ramp3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(ramp3D.getPosition());
         EuclidCoreTestTools.assertIdentity(ramp3D.getOrientation(), EPSILON);
      }

      { // Empty constructor
         Ramp3D ramp3D = new Ramp3D();

         EuclidCoreTestTools.assertEquals(new Vector3D(1, 1, 1), ramp3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(ramp3D.getPosition());
         EuclidCoreTestTools.assertIdentity(ramp3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ramp3D(double sizeX, double sizeY, double sizeZ)
         double sizeX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Ramp3D ramp3D = new Ramp3D(sizeX, sizeY, sizeZ);

         EuclidCoreTestTools.assertEquals(new Vector3D(sizeX, sizeY, sizeZ), ramp3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(ramp3D.getPosition());
         EuclidCoreTestTools.assertIdentity(ramp3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ramp3D(Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
         double sizeX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random);
         RotationMatrix orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
         Ramp3D ramp3D = new Ramp3D(position, orientation, sizeX, sizeY, sizeZ);

         EuclidCoreTestTools.assertEquals(new Vector3D(sizeX, sizeY, sizeZ), ramp3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertEquals(position, ramp3D.getPosition(), EPSILON);
         EuclidCoreTestTools.assertMatrix3DEquals(orientation, ramp3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ramp3D(Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
         double sizeX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random);
         RotationMatrix orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
         Pose3D pose = new Pose3D(position, orientation);
         Ramp3D ramp3D = new Ramp3D(pose, sizeX, sizeY, sizeZ);

         EuclidCoreTestTools.assertEquals(new Vector3D(sizeX, sizeY, sizeZ), ramp3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertEquals(position, ramp3D.getPosition(), EPSILON);
         EuclidCoreTestTools.assertMatrix3DEquals(orientation, ramp3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ramp3D(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
         double sizeX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random);
         RotationMatrix orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RigidBodyTransform pose = new RigidBodyTransform(orientation, position);
         Ramp3D ramp3D = new Ramp3D(pose, sizeX, sizeY, sizeZ);

         EuclidCoreTestTools.assertEquals(new Vector3D(sizeX, sizeY, sizeZ), ramp3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertEquals(position, ramp3D.getPosition(), EPSILON);
         EuclidCoreTestTools.assertMatrix3DEquals(orientation, ramp3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ramp3D(Ramp3DReadOnly other)
         Ramp3D original = EuclidShapeRandomTools.nextRamp3D(random);
         Ramp3D copy = new Ramp3D(original);

         EuclidCoreTestTools.assertEquals(original, copy, EPSILON);
      }
   }

   @Test
   void testSetToNaN() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         assertFalse(ramp3D.containsNaN());
         assertFalse(ramp3D.getPose().containsNaN());
         assertFalse(ramp3D.getPosition().containsNaN());
         assertFalse(ramp3D.getOrientation().containsNaN());
         assertFalse(ramp3D.getSize().containsNaN());

         ramp3D.setToNaN();

         assertTrue(ramp3D.containsNaN());
         assertTrue(ramp3D.getPose().containsNaN());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(ramp3D.getPosition());
         EuclidCoreTestTools.assertMatrix3DContainsOnlyNaN(ramp3D.getOrientation());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(ramp3D.getSize());
      }
   }

   @Test
   void testSetToZero() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         assertFalse(new Point3D().epsilonEquals(ramp3D.getPosition(), EPSILON));
         assertFalse(new Point3D().epsilonEquals(ramp3D.getSize(), EPSILON));
         assertFalse(new RotationMatrix().epsilonEquals(ramp3D.getOrientation(), EPSILON));

         ramp3D.setToZero();

         EuclidCoreTestTools.assertTuple3DIsSetToZero(ramp3D.getPosition());
         EuclidCoreTestTools.assertIdentity(ramp3D.getOrientation(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(ramp3D.getSize());
      }
   }

   @Test
   void testSetters() throws Exception
   {
      Random random = new Random(45837543);

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Ramp3D other)
         Ramp3D expected = EuclidShapeRandomTools.nextRamp3D(random);
         Ramp3D actual = EuclidShapeRandomTools.nextRamp3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set(expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Ramp3DReadOnly other)
         Ramp3D expected = EuclidShapeRandomTools.nextRamp3D(random);
         Ramp3D actual = EuclidShapeRandomTools.nextRamp3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set((Ramp3DReadOnly) expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      { // set(Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Ramp3D expected = EuclidShapeRandomTools.nextRamp3D(random);
            Ramp3D actual = EuclidShapeRandomTools.nextRamp3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(expected.getPosition(), expected.getOrientation(), expected.getSizeX(), expected.getSizeY(), expected.getSizeZ());
            EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new Point3D(), new Quaternion(), -0.1, 1.0, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new Point3D(), new Quaternion(), 1.0, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new Point3D(), new Quaternion(), 1.0, 1.0, -0.1));
      }

      { // set(Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Ramp3D expected = EuclidShapeRandomTools.nextRamp3D(random);
            Ramp3D actual = EuclidShapeRandomTools.nextRamp3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(new Pose3D(expected.getPosition(), expected.getOrientation()), expected.getSizeX(), expected.getSizeY(), expected.getSizeZ());
            EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new Pose3D(), -0.1, 1.0, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new Pose3D(), 1.0, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new Pose3D(), 1.0, 1.0, -0.1));
      }

      { // set(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Ramp3D expected = EuclidShapeRandomTools.nextRamp3D(random);
            Ramp3D actual = EuclidShapeRandomTools.nextRamp3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(new RigidBodyTransform(expected.getOrientation(), expected.getPosition()),
                       expected.getSizeX(),
                       expected.getSizeY(),
                       expected.getSizeZ());
            EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new RigidBodyTransform(), -0.1, 1.0, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new RigidBodyTransform(), 1.0, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new RigidBodyTransform(), 1.0, 1.0, -0.1));
      }

      { // set(RigidBodyTransformReadOnly pose, double[] size)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Ramp3D expected = EuclidShapeRandomTools.nextRamp3D(random);
            Ramp3D actual = EuclidShapeRandomTools.nextRamp3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(new RigidBodyTransform(expected.getOrientation(), expected.getPosition()),
                       new double[] {expected.getSizeX(), expected.getSizeY(), expected.getSizeZ()});
            EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new RigidBodyTransform(), new double[] {-0.1, 1.0, 1.0}));
         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new RigidBodyTransform(), new double[] {1.0, -0.1, 1.0}));
         assertThrows(IllegalArgumentException.class, () -> new Ramp3D().set(new RigidBodyTransform(), new double[] {1.0, 1.0, -0.1}));
      }
   }

   @Test
   void testSetSize() throws Exception
   {
      Random random = new Random(5465446);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double sizeX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         assertFalse(EuclidCoreTools.epsilonEquals(sizeX, ramp3D.getSizeX(), EPSILON));
         assertFalse(EuclidCoreTools.epsilonEquals(sizeY, ramp3D.getSizeY(), EPSILON));
         assertFalse(EuclidCoreTools.epsilonEquals(sizeZ, ramp3D.getSizeZ(), EPSILON));
         ramp3D.setSize(sizeX, sizeY, sizeZ);
         assertEquals(sizeX, ramp3D.getSizeX(), EPSILON);
         assertEquals(sizeY, ramp3D.getSizeY(), EPSILON);
         assertEquals(sizeZ, ramp3D.getSizeZ(), EPSILON);
      }

      assertThrows(IllegalArgumentException.class, () -> new Ramp3D().setSize(-0.1, 1.0, 1.0));
      assertThrows(IllegalArgumentException.class, () -> new Ramp3D().setSize(1.0, -0.1, 1.0));
      assertThrows(IllegalArgumentException.class, () -> new Ramp3D().setSize(1.0, 1.0, -0.1));
   }

   @Test
   void testIsPointInside() throws Exception
   {
      Random random = new Random(839161);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, below the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOutside = new Point3D();
         // Directly below
         pointOutside.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeX()));
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(-random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Beyond the ramp
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(-random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Before the ramp
         pointOutside.setX(-random.nextDouble());
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(-random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Left of the ramp
         pointOutside.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeX()));
         pointOutside.setY(0.5 * ramp3D.getSizeY() + random.nextDouble());
         pointOutside.setZ(-random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Right of the ramp
         pointOutside.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeX()));
         pointOutside.setY(-0.5 * ramp3D.getSizeY() - random.nextDouble());
         pointOutside.setZ(-random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Beyond & left of the ramp
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(0.5 * ramp3D.getSizeY() + random.nextDouble());
         pointOutside.setZ(-random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Beyond & right of the ramp
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(-0.5 * ramp3D.getSizeY() - random.nextDouble());
         pointOutside.setZ(-random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Before & left of the ramp
         pointOutside.setX(-random.nextDouble());
         pointOutside.setY(0.5 * ramp3D.getSizeY() + random.nextDouble());
         pointOutside.setZ(-random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Before & right of the ramp
         pointOutside.setX(-random.nextDouble());
         pointOutside.setY(-0.5 * ramp3D.getSizeY() - random.nextDouble());
         pointOutside.setZ(-random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, beyond the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOutside = new Point3D();
         // Directly beyond the ramp
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeZ()));
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Beyond & left of the ramp
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(0.5 * ramp3D.getSizeY() + random.nextDouble());
         pointOutside.setZ(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeZ()));
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Beyond & right of the ramp
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(-0.5 * ramp3D.getSizeY() - random.nextDouble());
         pointOutside.setZ(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeZ()));
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Beyond & above the ramp
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(ramp3D.getSizeZ() + random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Beyond & above & left of the ramp
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(0.5 * ramp3D.getSizeY() + random.nextDouble());
         pointOutside.setZ(ramp3D.getSizeZ() + random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         // Beyond & above & right of the ramp
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(-0.5 * ramp3D.getSizeY() - random.nextDouble());
         pointOutside.setZ(ramp3D.getSizeZ() + random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, to the left of the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOutside = new Point3D();
         pointOutside.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeX()));
         pointOutside.setY(0.5 * ramp3D.getSizeY() + random.nextDouble());
         pointOutside.setZ(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeZ()));
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         pointOutside.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeX()));
         pointOutside.setY(0.5 * ramp3D.getSizeY() + random.nextDouble());
         pointOutside.setZ(ramp3D.getSizeZ() + random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, to the right of the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOutside = new Point3D();
         pointOutside.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeX()));
         pointOutside.setY(-0.5 * ramp3D.getSizeY() - random.nextDouble());
         pointOutside.setZ(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeZ()));
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));

         pointOutside.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeX()));
         pointOutside.setY(-0.5 * ramp3D.getSizeY() - random.nextDouble());
         pointOutside.setZ(ramp3D.getSizeZ() + random.nextDouble());
         ramp3D.transformToWorld(pointOutside);
         assertFalse(ramp3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, using the random weighted average generator
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, ramp3D.getVertices());
         assertTrue(ramp3D.isPointInside(pointInside));
      }
   }

   @Test
   void testEvaluatePoint3DCollision() throws Exception
   {
      Random random = new Random(17792681);

      Point3D actualClosestPoint = new Point3D();
      Vector3D actualNormal = new Vector3D();
      Point3D expectedClosestPoint = new Point3D();
      Vector3D expectedNormal = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly below the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOutside = new Point3D();
         pointOutside.setX(random.nextDouble() * ramp3D.getSizeX());
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(-random.nextDouble());

         expectedClosestPoint.set(pointOutside.getX(), pointOutside.getY(), 0.0);
         expectedNormal.setAndNegate(Axis3D.Z);

         ramp3D.transformToWorld(pointOutside);
         ramp3D.transformToWorld(expectedClosestPoint);
         ramp3D.transformToWorld(expectedNormal);
         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly beyond the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOutside = new Point3D();
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(random.nextDouble() * ramp3D.getSizeZ());

         expectedClosestPoint.set(ramp3D.getSizeX(), pointOutside.getY(), pointOutside.getZ());
         expectedNormal.set(Axis3D.X);

         ramp3D.transformToWorld(pointOutside);
         ramp3D.transformToWorld(expectedClosestPoint);
         ramp3D.transformToWorld(expectedNormal);
         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly above the slope portion
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();

         Point3D pointOutside = new Point3D();
         pointOutside.setX(distanceOnRamp * EuclidCoreTools.cos(angle));
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(distanceOnRamp * EuclidCoreTools.sin(angle));
         expectedClosestPoint.set(pointOutside);

         expectedNormal.set(ramp3D.getRampSurfaceNormal());

         ramp3D.transformToWorld(pointOutside);
         pointOutside.scaleAdd(random.nextDouble(), ramp3D.getRampSurfaceNormal(), pointOutside);

         ramp3D.transformToWorld(expectedClosestPoint);
         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly on the left side
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();

         Point3D pointOutside = new Point3D();
         pointOutside.setX(distanceOnRamp * EuclidCoreTools.cos(angle));
         pointOutside.setY(0.5 * ramp3D.getSizeY() + random.nextDouble());
         pointOutside.setZ(distanceOnRamp * EuclidCoreTools.sin(angle) * random.nextDouble());

         expectedClosestPoint.set(pointOutside.getX(), 0.5 * ramp3D.getSizeY(), pointOutside.getZ());
         expectedNormal.set(Axis3D.Y);

         ramp3D.transformToWorld(pointOutside);
         ramp3D.transformToWorld(expectedClosestPoint);
         ramp3D.transformToWorld(expectedNormal);
         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly on the right side
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();

         Point3D pointOutside = new Point3D();
         pointOutside.setX(distanceOnRamp * EuclidCoreTools.cos(angle));
         pointOutside.setY(-0.5 * ramp3D.getSizeY() - random.nextDouble());
         pointOutside.setZ(distanceOnRamp * EuclidCoreTools.sin(angle) * random.nextDouble());

         expectedClosestPoint.set(pointOutside.getX(), -0.5 * ramp3D.getSizeY(), pointOutside.getZ());
         expectedNormal.setAndNegate(Axis3D.Y);

         ramp3D.transformToWorld(pointOutside);
         ramp3D.transformToWorld(expectedClosestPoint);
         ramp3D.transformToWorld(expectedNormal);
         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the top edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()), ramp3D.getSizeZ());
         ramp3D.transformToWorld(pointOnEdge);

         Vector3DReadOnly rampNormal = ramp3D.getRampSurfaceNormal();
         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rampNormal, backFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double distance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, towardOutside, pointOnEdge);

         expectedClosestPoint.set(pointOnEdge);
         expectedNormal.set(towardOutside);

         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the front edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(0.0, EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3DReadOnly rampNormal = ramp3D.getRampSurfaceNormal();
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rampNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double distance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, towardOutside, pointOnEdge);

         expectedClosestPoint.set(pointOnEdge);
         expectedNormal.set(towardOutside);

         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the bottom-back edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(backFaceNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double distance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, towardOutside, pointOnEdge);

         expectedClosestPoint.set(pointOnEdge);
         expectedNormal.set(towardOutside);

         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the bottom-left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(random.nextDouble() * ramp3D.getSizeX(), 0.5 * ramp3D.getSizeY(), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D leftFaceNormal = new Vector3D(ramp3D.getPose().getYAxis());
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(leftFaceNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double distance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, towardOutside, pointOnEdge);

         expectedClosestPoint.set(pointOnEdge);
         expectedNormal.set(towardOutside);

         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the bottom-right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(random.nextDouble() * ramp3D.getSizeX(), -0.5 * ramp3D.getSizeY(), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D rightFaceNormal = new Vector3D();
         rightFaceNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rightFaceNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double distance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, towardOutside, pointOnEdge);

         expectedClosestPoint.set(pointOnEdge);
         expectedNormal.set(towardOutside);

         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the back-left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), 0.5 * ramp3D.getSizeY(), random.nextDouble() * ramp3D.getSizeZ());
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D leftFaceNormal = new Vector3D(ramp3D.getPose().getYAxis());
         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(leftFaceNormal, backFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double distance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, towardOutside, pointOnEdge);

         expectedClosestPoint.set(pointOnEdge);
         expectedNormal.set(towardOutside);

         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the back-right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), -0.5 * ramp3D.getSizeY(), random.nextDouble() * ramp3D.getSizeZ());
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D rightFaceNormal = new Vector3D();
         rightFaceNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rightFaceNormal, backFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double distance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, towardOutside, pointOnEdge);

         expectedClosestPoint.set(pointOnEdge);
         expectedNormal.set(towardOutside);

         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the slope-left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();
         Point3D pointOnEdge = new Point3D(distanceOnRamp * EuclidCoreTools.cos(angle), 0.5 * ramp3D.getSizeY(), distanceOnRamp * EuclidCoreTools.sin(angle));
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D leftFaceNormal = new Vector3D(ramp3D.getPose().getYAxis());
         Vector3D slopeFaceNormal = new Vector3D(ramp3D.getRampSurfaceNormal());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(leftFaceNormal, slopeFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double distance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, towardOutside, pointOnEdge);

         expectedClosestPoint.set(pointOnEdge);
         expectedNormal.set(towardOutside);

         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the slope-right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();
         Point3D pointOnEdge = new Point3D(distanceOnRamp * EuclidCoreTools.cos(angle), -0.5 * ramp3D.getSizeY(), distanceOnRamp * EuclidCoreTools.sin(angle));
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D rightFaceNormal = new Vector3D();
         rightFaceNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D slopeFaceNormal = new Vector3D(ramp3D.getRampSurfaceNormal());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rightFaceNormal, slopeFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double distance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, towardOutside, pointOnEdge);

         expectedClosestPoint.set(pointOnEdge);
         expectedNormal.set(towardOutside);

         assertFalse(ramp3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, we use brute force to figure out the expected closest point and normal
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, ramp3D.getVertices());

         Plane3D backPlane = new Plane3D(new Point3D(ramp3D.getSizeX(), 0.0, 0.0), Axis3D.X);
         Plane3D bottomPlane = new Plane3D(new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, -1.0));
         Plane3D leftPlane = new Plane3D(new Point3D(0.0, 0.5 * ramp3D.getSizeY(), 0.0), Axis3D.Y);
         Plane3D rightPlane = new Plane3D(new Point3D(0.0, -0.5 * ramp3D.getSizeY(), 0.0), new Vector3D(0.0, -1.0, 0.0));
         Plane3D slopePlane = new Plane3D(ramp3D.getPosition(), ramp3D.getRampSurfaceNormal());
         ramp3D.transformToWorld(backPlane);
         ramp3D.transformToWorld(bottomPlane);
         ramp3D.transformToWorld(leftPlane);
         ramp3D.transformToWorld(rightPlane);

         double distanceToBack = backPlane.distance(pointInside);
         double distanceToBottom = bottomPlane.distance(pointInside);
         double distanceToLeft = leftPlane.distance(pointInside);
         double distanceToRight = rightPlane.distance(pointInside);
         double distanceToSlope = slopePlane.distance(pointInside);

         Plane3D closestPlane;
         if (EuclidShapeTools.isFirstValueMinimum(distanceToBack, distanceToBottom, distanceToLeft, distanceToRight, distanceToSlope))
         {
            closestPlane = backPlane;
         }
         else if (EuclidShapeTools.isFirstValueMinimum(distanceToBottom, distanceToLeft, distanceToRight, distanceToSlope))
         {
            closestPlane = bottomPlane;
         }
         else if (EuclidShapeTools.isFirstValueMinimum(distanceToLeft, distanceToRight, distanceToSlope))
         {
            closestPlane = leftPlane;
         }
         else if (distanceToRight <= distanceToSlope)
         {
            closestPlane = rightPlane;
         }
         else
         {
            closestPlane = slopePlane;
         }

         expectedClosestPoint.set(closestPlane.orthogonalProjectionCopy(pointInside));
         expectedNormal.set(closestPlane.getNormal());

         assertTrue(ramp3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }
   }

   @Test
   void testGetVertices() throws Exception
   {
      Random random = new Random(335436);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with pose set to zero
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         ramp3D.getPose().setToZero();
         double sizeX = ramp3D.getSizeX();
         double sizeY = 0.5 * ramp3D.getSizeY();
         double sizeZ = ramp3D.getSizeZ();

         Point3DBasics[] vertices = ramp3D.getVertices();
         assertEquals(6, vertices.length);
         int vertexIndex = 0;
         EuclidCoreTestTools.assertEquals(new Point3D(sizeX, sizeY, 0.0), vertices[vertexIndex++], EPSILON);
         EuclidCoreTestTools.assertEquals(new Point3D(sizeX, -sizeY, 0.0), vertices[vertexIndex++], EPSILON);
         EuclidCoreTestTools.assertEquals(new Point3D(0.0, sizeY, 0.0), vertices[vertexIndex++], EPSILON);
         EuclidCoreTestTools.assertEquals(new Point3D(0.0, -sizeY, 0.0), vertices[vertexIndex++], EPSILON);
         EuclidCoreTestTools.assertEquals(new Point3D(sizeX, sizeY, sizeZ), vertices[vertexIndex++], EPSILON);
         EuclidCoreTestTools.assertEquals(new Point3D(sizeX, -sizeY, sizeZ), vertices[vertexIndex++], EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Just testing that the vertices transformation
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3DBasics[] expectedVertices = ramp3D.getVertices();

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         ramp3D.applyTransform(transform);
         Arrays.asList(expectedVertices).forEach(transform::transform);

         Point3DBasics[] actualVertices = ramp3D.getVertices();

         for (int j = 0; j < 6; j++)
         {
            EuclidCoreTestTools.assertEquals(expectedVertices[j], actualVertices[j], EPSILON);
         }
      }
   }

   @Test
   void testApplyTransform()
   {
      Random random = new Random(346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ramp3D actual = EuclidShapeRandomTools.nextRamp3D(random);
         Ramp3D expected = new Ramp3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPose().applyTransform(transform);
         actual.applyTransform(transform);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ramp3D actual = EuclidShapeRandomTools.nextRamp3D(random);
         Ramp3D expected = new Ramp3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPose().applyTransform(transform);
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
         Ramp3D actual = EuclidShapeRandomTools.nextRamp3D(random);
         Ramp3D original = new Ramp3D(actual);
         Ramp3D expected = new Ramp3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPose().applyInverseTransform(transform);
         actual.applyInverseTransform(transform);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertEquals(original, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ramp3D actual = EuclidShapeRandomTools.nextRamp3D(random);
         Ramp3D original = new Ramp3D(actual);
         Ramp3D expected = new Ramp3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPose().applyInverseTransform(transform);
         actual.applyInverseTransform(transform);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertEquals(original, actual, EPSILON);
      }
   }

   @Test
   void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(546161);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ramp3D ramp = EuclidShapeRandomTools.nextRamp3D(random);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly supportingVertex = ramp.getSupportingVertex(supportDirection);
         Point3D supportingVertexTranslated = new Point3D();
         supportDirection.normalize();
         assertTrue(ramp.isPointInside(supportingVertex, EPSILON));
         supportingVertexTranslated.scaleAdd(1.0e-6, supportDirection, supportingVertex);
         assertFalse(ramp.isPointInside(supportingVertexTranslated, EPSILON));
         supportingVertexTranslated.scaleAdd(1.0e-2, supportDirection, supportingVertex);
         Vector3D expectedNormal = new Vector3D();
         expectedNormal.sub(supportingVertexTranslated, supportingVertex);
         expectedNormal.normalize();

         Vector3D actualNormal = new Vector3D();
         ramp.evaluatePoint3DCollision(supportingVertexTranslated, new Point3D(), actualNormal);
         EuclidCoreTestTools.assertEquals(expectedNormal, actualNormal, EPSILON);
      }
   }

   @Test
   void testDistance() throws Exception
   {
      Random random = new Random(11426096);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly below the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOutside = new Point3D();
         pointOutside.setX(random.nextDouble() * ramp3D.getSizeX());
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(-random.nextDouble());

         double expectedDistance = -pointOutside.getZ();

         ramp3D.transformToWorld(pointOutside);
         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly beyond the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOutside = new Point3D();
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(random.nextDouble() * ramp3D.getSizeZ());

         double expectedDistance = pointOutside.getX() - ramp3D.getSizeX();
         ramp3D.transformToWorld(pointOutside);
         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly above the slope portion
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();

         Point3D pointOutside = new Point3D();
         pointOutside.setX(distanceOnRamp * EuclidCoreTools.cos(angle));
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(distanceOnRamp * EuclidCoreTools.sin(angle));

         ramp3D.transformToWorld(pointOutside);
         double expectedDistance = random.nextDouble();
         pointOutside.scaleAdd(expectedDistance, ramp3D.getRampSurfaceNormal(), pointOutside);

         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly on the left side
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();

         Point3D pointOutside = new Point3D();
         pointOutside.setX(distanceOnRamp * EuclidCoreTools.cos(angle));
         pointOutside.setY(0.5 * ramp3D.getSizeY() + random.nextDouble());
         pointOutside.setZ(distanceOnRamp * EuclidCoreTools.sin(angle) * random.nextDouble());

         double expectedDistance = pointOutside.getY() - 0.5 * ramp3D.getSizeY();
         ramp3D.transformToWorld(pointOutside);
         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly on the right side
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();

         Point3D pointOutside = new Point3D();
         pointOutside.setX(distanceOnRamp * EuclidCoreTools.cos(angle));
         pointOutside.setY(-0.5 * ramp3D.getSizeY() - random.nextDouble());
         pointOutside.setZ(distanceOnRamp * EuclidCoreTools.sin(angle) * random.nextDouble());

         double expectedDistance = -pointOutside.getY() - 0.5 * ramp3D.getSizeY();
         ramp3D.transformToWorld(pointOutside);
         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the top edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()), ramp3D.getSizeZ());
         ramp3D.transformToWorld(pointOnEdge);

         Vector3DReadOnly rampNormal = ramp3D.getRampSurfaceNormal();
         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rampNormal, backFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the front edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(0.0, EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3DReadOnly rampNormal = ramp3D.getRampSurfaceNormal();
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rampNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the bottom-back edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(backFaceNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the bottom-left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(random.nextDouble() * ramp3D.getSizeX(), 0.5 * ramp3D.getSizeY(), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D leftFaceNormal = new Vector3D(ramp3D.getPose().getYAxis());
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(leftFaceNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the bottom-right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(random.nextDouble() * ramp3D.getSizeX(), -0.5 * ramp3D.getSizeY(), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D rightFaceNormal = new Vector3D();
         rightFaceNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rightFaceNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the back-left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), 0.5 * ramp3D.getSizeY(), random.nextDouble() * ramp3D.getSizeZ());
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D leftFaceNormal = new Vector3D(ramp3D.getPose().getYAxis());
         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(leftFaceNormal, backFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the back-right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), -0.5 * ramp3D.getSizeY(), random.nextDouble() * ramp3D.getSizeZ());
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D rightFaceNormal = new Vector3D();
         rightFaceNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rightFaceNormal, backFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the slope-left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();
         Point3D pointOnEdge = new Point3D(distanceOnRamp * EuclidCoreTools.cos(angle), 0.5 * ramp3D.getSizeY(), distanceOnRamp * EuclidCoreTools.sin(angle));
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D leftFaceNormal = new Vector3D(ramp3D.getPose().getYAxis());
         Vector3D slopeFaceNormal = new Vector3D(ramp3D.getRampSurfaceNormal());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(leftFaceNormal, slopeFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the slope-right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();
         Point3D pointOnEdge = new Point3D(distanceOnRamp * EuclidCoreTools.cos(angle), -0.5 * ramp3D.getSizeY(), distanceOnRamp * EuclidCoreTools.sin(angle));
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D rightFaceNormal = new Vector3D();
         rightFaceNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D slopeFaceNormal = new Vector3D(ramp3D.getRampSurfaceNormal());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rightFaceNormal, slopeFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, the distance should be 0.0
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, ramp3D.getVertices());
         assertEquals(0.0, ramp3D.distance(pointInside));
      }
   }

   @Test
   void testSignedDistance() throws Exception
   {
      Random random = new Random(13199357);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly below the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOutside = new Point3D();
         pointOutside.setX(random.nextDouble() * ramp3D.getSizeX());
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(-random.nextDouble());

         double expectedDistance = -pointOutside.getZ();

         ramp3D.transformToWorld(pointOutside);
         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly beyond the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOutside = new Point3D();
         pointOutside.setX(ramp3D.getSizeX() + random.nextDouble());
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(random.nextDouble() * ramp3D.getSizeZ());

         double expectedDistance = pointOutside.getX() - ramp3D.getSizeX();
         ramp3D.transformToWorld(pointOutside);
         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly above the slope portion
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();

         Point3D pointOutside = new Point3D();
         pointOutside.setX(distanceOnRamp * EuclidCoreTools.cos(angle));
         pointOutside.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOutside.setZ(distanceOnRamp * EuclidCoreTools.sin(angle));

         ramp3D.transformToWorld(pointOutside);
         double expectedDistance = random.nextDouble();
         pointOutside.scaleAdd(expectedDistance, ramp3D.getRampSurfaceNormal(), pointOutside);

         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly on the left side
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();

         Point3D pointOutside = new Point3D();
         pointOutside.setX(distanceOnRamp * EuclidCoreTools.cos(angle));
         pointOutside.setY(0.5 * ramp3D.getSizeY() + random.nextDouble());
         pointOutside.setZ(distanceOnRamp * EuclidCoreTools.sin(angle) * random.nextDouble());

         double expectedDistance = pointOutside.getY() - 0.5 * ramp3D.getSizeY();
         ramp3D.transformToWorld(pointOutside);
         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, directly on the right side
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();

         Point3D pointOutside = new Point3D();
         pointOutside.setX(distanceOnRamp * EuclidCoreTools.cos(angle));
         pointOutside.setY(-0.5 * ramp3D.getSizeY() - random.nextDouble());
         pointOutside.setZ(distanceOnRamp * EuclidCoreTools.sin(angle) * random.nextDouble());

         double expectedDistance = -pointOutside.getY() - 0.5 * ramp3D.getSizeY();
         ramp3D.transformToWorld(pointOutside);
         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the top edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()), ramp3D.getSizeZ());
         ramp3D.transformToWorld(pointOnEdge);

         Vector3DReadOnly rampNormal = ramp3D.getRampSurfaceNormal();
         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rampNormal, backFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the front edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(0.0, EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3DReadOnly rampNormal = ramp3D.getRampSurfaceNormal();
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rampNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the bottom-back edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(backFaceNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the bottom-left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(random.nextDouble() * ramp3D.getSizeX(), 0.5 * ramp3D.getSizeY(), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D leftFaceNormal = new Vector3D(ramp3D.getPose().getYAxis());
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(leftFaceNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the bottom-right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(random.nextDouble() * ramp3D.getSizeX(), -0.5 * ramp3D.getSizeY(), 0.0);
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D rightFaceNormal = new Vector3D();
         rightFaceNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D bottomFaceNormal = new Vector3D();
         bottomFaceNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rightFaceNormal, bottomFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the back-left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), 0.5 * ramp3D.getSizeY(), random.nextDouble() * ramp3D.getSizeZ());
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D leftFaceNormal = new Vector3D(ramp3D.getPose().getYAxis());
         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(leftFaceNormal, backFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the back-right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnEdge = new Point3D(ramp3D.getSizeX(), -0.5 * ramp3D.getSizeY(), random.nextDouble() * ramp3D.getSizeZ());
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D rightFaceNormal = new Vector3D();
         rightFaceNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D backFaceNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rightFaceNormal, backFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the slope-left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();
         Point3D pointOnEdge = new Point3D(distanceOnRamp * EuclidCoreTools.cos(angle), 0.5 * ramp3D.getSizeY(), distanceOnRamp * EuclidCoreTools.sin(angle));
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D leftFaceNormal = new Vector3D(ramp3D.getPose().getYAxis());
         Vector3D slopeFaceNormal = new Vector3D(ramp3D.getRampSurfaceNormal());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(leftFaceNormal, slopeFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to the slope-right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         double angle = ramp3D.getRampIncline();
         double distanceOnRamp = random.nextDouble() * ramp3D.getRampLength();
         Point3D pointOnEdge = new Point3D(distanceOnRamp * EuclidCoreTools.cos(angle), -0.5 * ramp3D.getSizeY(), distanceOnRamp * EuclidCoreTools.sin(angle));
         ramp3D.transformToWorld(pointOnEdge);

         Vector3D rightFaceNormal = new Vector3D();
         rightFaceNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D slopeFaceNormal = new Vector3D(ramp3D.getRampSurfaceNormal());
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(rightFaceNormal, slopeFaceNormal, random.nextDouble());
         towardOutside.normalize();
         double expectedDistance = random.nextDouble();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, towardOutside, pointOnEdge);

         assertEquals(expectedDistance, ramp3D.signedDistance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, the distance should be 0.0
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, ramp3D.getVertices());
         Plane3D backPlane = new Plane3D(new Point3D(ramp3D.getSizeX(), 0.0, 0.0), Axis3D.X);
         Plane3D bottomPlane = new Plane3D(new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, -1.0));
         Plane3D leftPlane = new Plane3D(new Point3D(0.0, 0.5 * ramp3D.getSizeY(), 0.0), Axis3D.Y);
         Plane3D rightPlane = new Plane3D(new Point3D(0.0, -0.5 * ramp3D.getSizeY(), 0.0), new Vector3D(0.0, -1.0, 0.0));
         Plane3D slopePlane = new Plane3D(ramp3D.getPosition(), ramp3D.getRampSurfaceNormal());
         ramp3D.transformToWorld(backPlane);
         ramp3D.transformToWorld(bottomPlane);
         ramp3D.transformToWorld(leftPlane);
         ramp3D.transformToWorld(rightPlane);

         double expectedDistance = Math.min(backPlane.distance(pointInside), bottomPlane.distance(pointInside));
         expectedDistance = Math.min(expectedDistance, leftPlane.distance(pointInside));
         expectedDistance = Math.min(expectedDistance, rightPlane.distance(pointInside));
         expectedDistance = Math.min(expectedDistance, slopePlane.distance(pointInside));
         assertEquals(-expectedDistance, ramp3D.signedDistance(pointInside), EPSILON);
      }
   }

   @Test
   void testOrthogonalProjection() throws Exception
   {
      Random random = new Random(10276897);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, projection should return null
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, ramp3D.getVertices());
         assertNull(ramp3D.orthogonalProjectionCopy(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // We use the already tested doPoint3DCollisionTest to verify the orthogonalProjection result
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D point = new Point3D();
         point.setX(EuclidCoreRandomTools.nextDouble(random, -2.0, 2.0 + ramp3D.getSizeX()));
         point.setY(EuclidCoreRandomTools.nextDouble(random, -1.0 - 0.5 * ramp3D.getSizeY(), 1.0 + 0.5 * ramp3D.getSizeY()));
         point.setX(EuclidCoreRandomTools.nextDouble(random, -2.0, 2.0 + ramp3D.getSizeZ()));

         ramp3D.transformToWorld(point);

         Point3D expectedProjection = new Point3D();
         boolean isInside = ramp3D.evaluatePoint3DCollision(point, expectedProjection, new Vector3D());
         if (isInside)
            assertNull(ramp3D.orthogonalProjectionCopy(point));
         else
            EuclidCoreTestTools.assertEquals(expectedProjection, ramp3D.orthogonalProjectionCopy(point), EPSILON);
      }
   }

   @Test
   void testGetBoundingBox3D() throws Exception
   {
      Random random = new Random(34563);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         BoundingBox3D boundingBox = EuclidGeometryRandomTools.nextBoundingBox3D(random);
         ramp3D.getBoundingBox(boundingBox);

         for (Point3DBasics vertex : ramp3D.getVertices())
            assertTrue(boundingBox.isInsideInclusive(vertex));
         for (int j = 0; j < 100; j++)
            assertTrue(boundingBox.isInsideExclusive(EuclidGeometryRandomTools.nextWeightedAverage(random, ramp3D.getVertices())));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Using getSupportingVertex
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         BoundingBox3D expectedBoundingBox = new BoundingBox3D();
         expectedBoundingBox.setToNaN();
         Vector3D supportDirection = new Vector3D(Axis3D.X);
         expectedBoundingBox.updateToIncludePoint(ramp3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(ramp3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis3D.Y);
         expectedBoundingBox.updateToIncludePoint(ramp3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(ramp3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis3D.Z);
         expectedBoundingBox.updateToIncludePoint(ramp3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(ramp3D.getSupportingVertex(supportDirection));

         BoundingBox3DReadOnly actualBoundingBox = ramp3D.getBoundingBox();
         EuclidCoreTestTools.assertEquals(expectedBoundingBox, actualBoundingBox, EPSILON);
      }
   }

   @Test
   public void testSurfaceNormal()
   {
      Ramp3D ramp = new Ramp3D(1.0, 1.0, 1.0);
      Vector3D surfaceNormal = new Vector3D();
      ramp.getRampSurfaceNormal(surfaceNormal);
      assertEquals(surfaceNormal.getX(), -1.0 / EuclidCoreTools.squareRoot(2.0), 1e-14, "not equal");
      assertEquals(surfaceNormal.getY(), 0.0, 1e-14, "not equal");
      assertEquals(surfaceNormal.getZ(), 1.0 / EuclidCoreTools.squareRoot(2.0), 1e-14, "not equal");
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(34201L);
      Ramp3D firstRamp, secondRamp;
      double lengthX, widthY, heightZ;
      double epsilon = 1e-7;

      lengthX = random.nextDouble();
      widthY = random.nextDouble();
      heightZ = random.nextDouble();

      firstRamp = new Ramp3D(lengthX, widthY, heightZ);
      secondRamp = new Ramp3D(lengthX, widthY, heightZ);

      assertTrue(firstRamp.geometricallyEquals(secondRamp, epsilon));
      assertTrue(secondRamp.geometricallyEquals(firstRamp, epsilon));
      assertTrue(firstRamp.geometricallyEquals(firstRamp, epsilon));
      assertTrue(secondRamp.geometricallyEquals(secondRamp, epsilon));

      secondRamp = new Ramp3D(lengthX + epsilon * 0.99, widthY, heightZ);
      assertTrue(firstRamp.geometricallyEquals(secondRamp, epsilon));
      secondRamp = new Ramp3D(lengthX, widthY + epsilon * 0.99, heightZ);
      assertTrue(firstRamp.geometricallyEquals(secondRamp, epsilon));
      secondRamp = new Ramp3D(lengthX, widthY, heightZ + epsilon * 0.99);
      assertTrue(firstRamp.geometricallyEquals(secondRamp, epsilon));
      secondRamp = new Ramp3D(lengthX - epsilon * 0.99, widthY, heightZ);
      assertTrue(firstRamp.geometricallyEquals(secondRamp, epsilon));
      secondRamp = new Ramp3D(lengthX, widthY - epsilon * 0.99, heightZ);
      assertTrue(firstRamp.geometricallyEquals(secondRamp, epsilon));
      secondRamp = new Ramp3D(lengthX, widthY, heightZ - epsilon * 0.99);
      assertTrue(firstRamp.geometricallyEquals(secondRamp, epsilon));

      secondRamp = new Ramp3D(lengthX + epsilon * 1.01, widthY, heightZ);
      assertFalse(firstRamp.geometricallyEquals(secondRamp, epsilon));
      secondRamp = new Ramp3D(lengthX, widthY + epsilon * 1.01, heightZ);
      assertFalse(firstRamp.geometricallyEquals(secondRamp, epsilon));
      secondRamp = new Ramp3D(lengthX, widthY, heightZ + epsilon * 1.01);
      assertFalse(firstRamp.geometricallyEquals(secondRamp, epsilon));
      secondRamp = new Ramp3D(lengthX - epsilon * 1.01, widthY, heightZ);
      assertFalse(firstRamp.geometricallyEquals(secondRamp, epsilon));
      secondRamp = new Ramp3D(lengthX, widthY - epsilon * 1.01, heightZ);
      assertFalse(firstRamp.geometricallyEquals(secondRamp, epsilon));
      secondRamp = new Ramp3D(lengthX, widthY, heightZ - epsilon * 1.01);
      assertFalse(firstRamp.geometricallyEquals(secondRamp, epsilon));

      for (int i = 0; i < ITERATIONS; ++i)
      {
         Vector3D translationVector = EuclidCoreRandomTools.nextRotationVector(random);
         firstRamp = new Ramp3D(new RigidBodyTransform(new RotationMatrix(), translationVector), lengthX, widthY, heightZ);
         secondRamp = new Ramp3D(firstRamp);

         translationVector = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon);
         secondRamp.getPose().appendTranslation(translationVector);

         assertTrue(firstRamp.geometricallyEquals(secondRamp, epsilon));

         secondRamp = new Ramp3D(firstRamp);

         translationVector = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon);
         secondRamp.getPose().appendTranslation(translationVector);

         assertFalse(firstRamp.geometricallyEquals(secondRamp, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         Quaternion rotation = EuclidCoreRandomTools.nextQuaternion(random);
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Pose3D pose = new Pose3D(position, rotation);
         firstRamp = new Ramp3D(pose, lengthX, widthY, heightZ);
         secondRamp = new Ramp3D(firstRamp);

         RigidBodyTransform transform = new RigidBodyTransform();
         AxisAngle axisAngle = new AxisAngle();

         axisAngle.set(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), 0.99 * epsilon);
         transform.getRotation().set(axisAngle);
         secondRamp.getPose().multiply(transform);
         assertTrue(firstRamp.geometricallyEquals(secondRamp, epsilon));

         secondRamp = new Ramp3D(firstRamp);

         axisAngle.set(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), 1.01 * epsilon);
         transform.getRotation().set(axisAngle);
         secondRamp.getPose().multiply(transform);
         assertFalse(firstRamp.geometricallyEquals(secondRamp, epsilon));
      }
   }
}
