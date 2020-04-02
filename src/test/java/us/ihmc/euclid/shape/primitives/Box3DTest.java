package us.ihmc.euclid.shape.primitives;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public class Box3DTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   void testConstructors() throws Exception
   {
      Random random = new Random(34590376);

      { // Empty constructor
         Box3D box3D = new Box3D();

         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1, 1, 1), box3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(box3D.getPosition());
         EuclidCoreTestTools.assertIdentity(box3D.getOrientation(), EPSILON);
      }

      { // Empty constructor
         Box3D box3D = new Box3D();

         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1, 1, 1), box3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(box3D.getPosition());
         EuclidCoreTestTools.assertIdentity(box3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Box3D(double sizeX, double sizeY, double sizeZ)
         double sizeX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Box3D box3D = new Box3D(sizeX, sizeY, sizeZ);

         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(sizeX, sizeY, sizeZ), box3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(box3D.getPosition());
         EuclidCoreTestTools.assertIdentity(box3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Box3D(Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
         double sizeX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random);
         RotationMatrix orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
         Box3D box3D = new Box3D(position, orientation, sizeX, sizeY, sizeZ);

         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(sizeX, sizeY, sizeZ), box3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(position, box3D.getPosition(), EPSILON);
         EuclidCoreTestTools.assertMatrix3DEquals(orientation, box3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Box3D(Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
         double sizeX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random);
         RotationMatrix orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
         Pose3D pose = new Pose3D(position, orientation);
         Box3D box3D = new Box3D(pose, sizeX, sizeY, sizeZ);

         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(sizeX, sizeY, sizeZ), box3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(position, box3D.getPosition(), EPSILON);
         EuclidCoreTestTools.assertMatrix3DEquals(orientation, box3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Box3D(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
         double sizeX = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeY = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         double sizeZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 5.0);
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random);
         RotationMatrix orientation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RigidBodyTransform pose = new RigidBodyTransform(orientation, position);
         Box3D box3D = new Box3D(pose, sizeX, sizeY, sizeZ);

         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(sizeX, sizeY, sizeZ), box3D.getSize(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(position, box3D.getPosition(), EPSILON);
         EuclidCoreTestTools.assertMatrix3DEquals(orientation, box3D.getOrientation(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Box3D(Box3DReadOnly other)
         Box3D original = EuclidShapeRandomTools.nextBox3D(random);
         Box3D copy = new Box3D(original);

         EuclidShapeTestTools.assertBox3DEquals(original, copy, EPSILON);
      }
   }

   @Test
   void testSetToNaN() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);

         assertFalse(box3D.containsNaN());
         assertFalse(box3D.getPose().containsNaN());
         assertFalse(box3D.getPosition().containsNaN());
         assertFalse(box3D.getOrientation().containsNaN());
         assertFalse(box3D.getSize().containsNaN());

         box3D.setToNaN();

         assertTrue(box3D.containsNaN());
         assertTrue(box3D.getPose().containsNaN());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(box3D.getPosition());
         EuclidCoreTestTools.assertMatrix3DContainsOnlyNaN(box3D.getOrientation());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(box3D.getSize());
      }
   }

   @Test
   void testSetToZero() throws Exception
   {
      Random random = new Random(34575754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);

         assertFalse(new Point3D().epsilonEquals(box3D.getPosition(), EPSILON));
         assertFalse(new Point3D().epsilonEquals(box3D.getSize(), EPSILON));
         assertFalse(new RotationMatrix().epsilonEquals(box3D.getOrientation(), EPSILON));

         box3D.setToZero();

         EuclidCoreTestTools.assertTuple3DIsSetToZero(box3D.getPosition());
         EuclidCoreTestTools.assertIdentity(box3D.getOrientation(), EPSILON);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(box3D.getSize());
      }
   }

   @Test
   void testSetters() throws Exception
   {
      Random random = new Random(45837543);

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Box3D other)
         Box3D expected = EuclidShapeRandomTools.nextBox3D(random);
         Box3D actual = EuclidShapeRandomTools.nextBox3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set(expected);
         EuclidShapeTestTools.assertBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Box3DReadOnly other)
         Box3D expected = EuclidShapeRandomTools.nextBox3D(random);
         Box3D actual = EuclidShapeRandomTools.nextBox3D(random);
         assertFalse(expected.epsilonEquals(actual, EPSILON));
         actual.set((Box3DReadOnly) expected);
         EuclidShapeTestTools.assertBox3DEquals(expected, actual, EPSILON);
      }

      { // set(Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Box3D expected = EuclidShapeRandomTools.nextBox3D(random);
            Box3D actual = EuclidShapeRandomTools.nextBox3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(expected.getPosition(), expected.getOrientation(), expected.getSizeX(), expected.getSizeY(), expected.getSizeZ());
            EuclidShapeTestTools.assertBox3DEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new Point3D(), new Quaternion(), -0.1, 1.0, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new Point3D(), new Quaternion(), 1.0, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new Point3D(), new Quaternion(), 1.0, 1.0, -0.1));
      }

      { // set(Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Box3D expected = EuclidShapeRandomTools.nextBox3D(random);
            Box3D actual = EuclidShapeRandomTools.nextBox3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(new Pose3D(expected.getPosition(), expected.getOrientation()), expected.getSizeX(), expected.getSizeY(), expected.getSizeZ());
            EuclidShapeTestTools.assertBox3DEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new Pose3D(), -0.1, 1.0, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new Pose3D(), 1.0, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new Pose3D(), 1.0, 1.0, -0.1));
      }

      { // set(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Box3D expected = EuclidShapeRandomTools.nextBox3D(random);
            Box3D actual = EuclidShapeRandomTools.nextBox3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(new RigidBodyTransform(expected.getOrientation(), expected.getPosition()),
                       expected.getSizeX(),
                       expected.getSizeY(),
                       expected.getSizeZ());
            EuclidShapeTestTools.assertBox3DEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new RigidBodyTransform(), -0.1, 1.0, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new RigidBodyTransform(), 1.0, -0.1, 1.0));
         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new RigidBodyTransform(), 1.0, 1.0, -0.1));
      }

      { // set(RigidBodyTransformReadOnly pose, double[] size)
         for (int i = 0; i < ITERATIONS; i++)
         {
            Box3D expected = EuclidShapeRandomTools.nextBox3D(random);
            Box3D actual = EuclidShapeRandomTools.nextBox3D(random);
            assertFalse(expected.epsilonEquals(actual, EPSILON));
            actual.set(new RigidBodyTransform(expected.getOrientation(), expected.getPosition()),
                       new double[] {expected.getSizeX(), expected.getSizeY(), expected.getSizeZ()});
            EuclidShapeTestTools.assertBox3DEquals(expected, actual, EPSILON);
         }

         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new RigidBodyTransform(), new double[] {-0.1, 1.0, 1.0}));
         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new RigidBodyTransform(), new double[] {1.0, -0.1, 1.0}));
         assertThrows(IllegalArgumentException.class, () -> new Box3D().set(new RigidBodyTransform(), new double[] {1.0, 1.0, -0.1}));
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
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         assertFalse(EuclidCoreTools.epsilonEquals(sizeX, box3D.getSizeX(), EPSILON));
         assertFalse(EuclidCoreTools.epsilonEquals(sizeY, box3D.getSizeY(), EPSILON));
         assertFalse(EuclidCoreTools.epsilonEquals(sizeZ, box3D.getSizeZ(), EPSILON));
         box3D.getSize().set(sizeX, sizeY, sizeZ);
         assertEquals(sizeX, box3D.getSizeX(), EPSILON);
         assertEquals(sizeY, box3D.getSizeY(), EPSILON);
         assertEquals(sizeZ, box3D.getSizeZ(), EPSILON);
      }

      assertThrows(IllegalArgumentException.class, () -> new Box3D().getSize().set(-0.1, 1.0, 1.0));
      assertThrows(IllegalArgumentException.class, () -> new Box3D().getSize().set(1.0, -0.1, 1.0));
      assertThrows(IllegalArgumentException.class, () -> new Box3D().getSize().set(1.0, 1.0, -0.1));
   }

   @Test
   void testScale() throws Exception
   {
      Random random = new Random(405743);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D actual = EuclidShapeRandomTools.nextBox3D(random);
         Box3D expected = new Box3D(actual);
         Vector3D size = new Vector3D(actual.getSize());
         double scale = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         actual.scale(scale);
         size.scale(scale);
         expected.getSize().set(size);

         EuclidShapeTestTools.assertBox3DEquals(expected, actual, EPSILON);
      }

      assertThrows(IllegalArgumentException.class, () -> new Box3D().scale(-0.1));
   }

   @Test
   void testIsPointInside() throws Exception
   {
      Random random = new Random(435635675);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with point inside using the weighted random generator
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);

         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, box3D.getVertices());
         assertTrue(box3D.isPointInside(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Another pass with the point inside
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);

         Point3D pointInside = EuclidCoreRandomTools.nextPoint3D(random, 0.5 * box3D.getSizeX(), 0.5 * box3D.getSizeY(), 0.5 * box3D.getSizeZ());
         box3D.transformToWorld(pointInside);
         assertTrue(box3D.isPointInside(pointInside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point hovering over each face
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double sizeX = box3D.getSizeX();
         double sizeY = box3D.getSizeY();
         double sizeZ = box3D.getSizeZ();

         Point3D pointOutside = new Point3D();

         // Beyond the X+ face
         pointOutside.set(EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeX,
                          EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeY),
                          EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeZ));
         box3D.transformToWorld(pointOutside);
         assertFalse(box3D.isPointInside(pointOutside));

         // Beyond the X- face
         pointOutside.set(-EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeX,
                          EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeY),
                          EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeZ));
         box3D.transformToWorld(pointOutside);
         assertFalse(box3D.isPointInside(pointOutside));

         // Beyond the Y+ face
         pointOutside.set(EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeX),
                          EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeY,
                          EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeZ));
         box3D.transformToWorld(pointOutside);
         assertFalse(box3D.isPointInside(pointOutside));

         // Beyond the Y- face
         pointOutside.set(EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeX),
                          -EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeY,
                          -EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeZ));
         box3D.transformToWorld(pointOutside);
         assertFalse(box3D.isPointInside(pointOutside));

         // Beyond the Z+ face
         pointOutside.set(EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeX),
                          EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeY),
                          EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeZ);
         box3D.transformToWorld(pointOutside);
         assertFalse(box3D.isPointInside(pointOutside));

         // Beyond the Z- face
         pointOutside.set(EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeX),
                          EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeY),
                          -EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeZ);
         box3D.transformToWorld(pointOutside);
         assertFalse(box3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to an edge
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double sizeX = box3D.getSizeX();
         double sizeY = box3D.getSizeY();
         double sizeZ = box3D.getSizeZ();

         Point3D pointOutside = new Point3D();
         double xSign, ySign, zSign;

         // Beyond edge collinear to X-axis
         ySign = random.nextBoolean() ? -1.0 : 1.0;
         zSign = random.nextBoolean() ? -1.0 : 1.0;
         pointOutside.set(EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeX),
                          ySign * EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeY,
                          zSign * EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeZ);
         box3D.transformToWorld(pointOutside);
         assertFalse(box3D.isPointInside(pointOutside));

         // Beyond edge collinear to Y-axis
         xSign = random.nextBoolean() ? -1.0 : 1.0;
         zSign = random.nextBoolean() ? -1.0 : 1.0;
         pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeX,
                          EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeY),
                          zSign * EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeZ);
         box3D.transformToWorld(pointOutside);
         assertFalse(box3D.isPointInside(pointOutside));

         // Beyond edge collinear to Z-axis
         xSign = random.nextBoolean() ? -1.0 : 1.0;
         ySign = random.nextBoolean() ? -1.0 : 1.0;
         pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeX,
                          ySign * EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeY,
                          EuclidCoreRandomTools.nextDouble(random, 0.5 * sizeZ));
         box3D.transformToWorld(pointOutside);
         assertFalse(box3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to a vertex
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double sizeX = box3D.getSizeX();
         double sizeY = box3D.getSizeY();
         double sizeZ = box3D.getSizeZ();

         Point3D pointOutside = new Point3D();

         double xSign = random.nextBoolean() ? -1.0 : 1.0;
         double ySign = random.nextBoolean() ? -1.0 : 1.0;
         double zSign = random.nextBoolean() ? -1.0 : 1.0;
         pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeX,
                          ySign * EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeY,
                          zSign * EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0) * sizeZ);
         box3D.transformToWorld(pointOutside);
         assertFalse(box3D.isPointInside(pointOutside));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test that the vertices are inside within an epsilon
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         for (Point3DBasics vertex : box3D.getVertices())
         {
            assertTrue(box3D.isPointInside(vertex, EPSILON));
         }
      }
   }

   @Test
   void testEvaluatePoint3DCollision() throws Exception
   {
      Random random = new Random(354534665);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point not colliding, closest to a face
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         Point3D pointOutside = new Point3D();
         Point3D actualClosestPoint = new Point3D();
         Vector3D actualNormal = new Vector3D();
         Point3D expectedClosestPoint = new Point3D();
         Vector3D expectedNormal = new Vector3D();

         { // X faces
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeY),
                             EuclidCoreRandomTools.nextDouble(random, halfSizeZ));
            expectedClosestPoint.set(xSign * halfSizeX, pointOutside.getY(), pointOutside.getZ());
            expectedNormal.set(xSign, 0.0, 0.0);
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedClosestPoint);
            box3D.transformToWorld(expectedNormal);
            assertFalse(box3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
            EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
         }

         { // Y faces
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(EuclidCoreRandomTools.nextDouble(random, halfSizeX),
                             ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeZ));
            expectedClosestPoint.set(pointOutside.getX(), ySign * halfSizeY, pointOutside.getZ());
            expectedNormal.set(0.0, ySign, 0.0);
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedClosestPoint);
            box3D.transformToWorld(expectedNormal);
            assertFalse(box3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
            EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
         }

         { // Z faces
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(EuclidCoreRandomTools.nextDouble(random, halfSizeX),
                             EuclidCoreRandomTools.nextDouble(random, halfSizeY),
                             zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);
            expectedClosestPoint.set(pointOutside.getX(), pointOutside.getY(), zSign * halfSizeZ);
            expectedNormal.set(0.0, 0.0, zSign);
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedClosestPoint);
            box3D.transformToWorld(expectedNormal);
            assertFalse(box3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
            EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point not colliding, closest to an edge
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         Point3D pointOutside = new Point3D();
         Point3D actualClosestPoint = new Point3D();
         Vector3D actualNormal = new Vector3D();
         Point3D expectedClosestPoint = new Point3D();
         Vector3D expectedNormal = new Vector3D();

         { // Collinear to X-axis
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(EuclidCoreRandomTools.nextDouble(random, halfSizeX),
                             ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                             zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);
            expectedClosestPoint.set(pointOutside.getX(), ySign * halfSizeY, zSign * halfSizeZ);
            expectedNormal.sub(pointOutside, expectedClosestPoint);
            expectedNormal.normalize();
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedClosestPoint);
            box3D.transformToWorld(expectedNormal);
            assertFalse(box3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
            EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
         }

         { // Collinear to Y-axis
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeY),
                             zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);
            expectedClosestPoint.set(xSign * halfSizeX, pointOutside.getY(), zSign * halfSizeZ);
            expectedNormal.sub(pointOutside, expectedClosestPoint);
            expectedNormal.normalize();
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedClosestPoint);
            box3D.transformToWorld(expectedNormal);
            assertFalse(box3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
            EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
         }

         { // Collinear to Z-axis
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                             ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeZ));
            expectedClosestPoint.set(xSign * halfSizeX, ySign * halfSizeY, pointOutside.getZ());
            expectedNormal.sub(pointOutside, expectedClosestPoint);
            expectedNormal.normalize();
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedClosestPoint);
            box3D.transformToWorld(expectedNormal);
            assertFalse(box3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
            EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point not colliding, closest to a vertex
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         Point3D pointOutside = new Point3D();
         Point3D actualClosestPoint = new Point3D();
         Vector3D actualNormal = new Vector3D();
         Point3D expectedClosestPoint = new Point3D();
         Vector3D expectedNormal = new Vector3D();

         double xSign = random.nextBoolean() ? -1.0 : 1.0;
         double ySign = random.nextBoolean() ? -1.0 : 1.0;
         double zSign = random.nextBoolean() ? -1.0 : 1.0;
         pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                          ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                          zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);
         expectedClosestPoint.set(xSign * halfSizeX, ySign * halfSizeY, zSign * halfSizeZ);
         expectedNormal.sub(pointOutside, expectedClosestPoint);
         expectedNormal.normalize();
         box3D.transformToWorld(pointOutside);
         box3D.transformToWorld(expectedClosestPoint);
         box3D.transformToWorld(expectedNormal);
         assertFalse(box3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal));
         EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside generated by choosing the closest face
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         // Going for a cube setup so it is easier to predict which face is closest.
         double size = EuclidCoreRandomTools.nextDouble(random, 0.2, 10.0);
         box3D.getSize().set(size, size, size);
         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         Point3D pointInside = new Point3D();
         Point3D actualClosestPoint = new Point3D();
         Vector3D actualNormal = new Vector3D();
         Point3D expectedClosestPoint = new Point3D();
         Vector3D expectedNormal = new Vector3D();

         { // X faces
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            Point3D[] faceVertices = {new Point3D(xSign * halfSizeX, halfSizeY, halfSizeZ), new Point3D(xSign * halfSizeX, -halfSizeY, halfSizeZ),
                  new Point3D(xSign * halfSizeX, -halfSizeY, -halfSizeZ), new Point3D(xSign * halfSizeX, halfSizeY, -halfSizeZ)};

            if (random.nextBoolean()) // Can only generate a point from the box's center and 3 vertices. Randomly pick a triangle of 2 possible triangles for the face
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[1], faceVertices[2]));
            else
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[2], faceVertices[3]));

            expectedClosestPoint.set(xSign * halfSizeX, pointInside.getY(), pointInside.getZ());
            expectedNormal.set(xSign, 0.0, 0.0);
            box3D.transformToWorld(pointInside);
            box3D.transformToWorld(expectedClosestPoint);
            box3D.transformToWorld(expectedNormal);
            assertTrue(box3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal));
            EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
         }

         { // Y faces
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            Point3D[] faceVertices = {new Point3D(halfSizeX, ySign * halfSizeY, halfSizeZ), new Point3D(-halfSizeX, ySign * halfSizeY, halfSizeZ),
                  new Point3D(-halfSizeX, ySign * halfSizeY, -halfSizeZ), new Point3D(halfSizeX, ySign * halfSizeY, -halfSizeZ)};

            if (random.nextBoolean()) // Can only generate a point from the box's center and 3 vertices. Randomly pick a triangle of 2 possible triangles for the face
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[1], faceVertices[2]));
            else
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[2], faceVertices[3]));

            expectedClosestPoint.set(pointInside.getX(), ySign * halfSizeY, pointInside.getZ());
            expectedNormal.set(0.0, ySign, 0.0);
            box3D.transformToWorld(pointInside);
            box3D.transformToWorld(expectedClosestPoint);
            box3D.transformToWorld(expectedNormal);
            assertTrue(box3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal));
            EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
         }

         { // Z faces
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            Point3D[] faceVertices = {new Point3D(halfSizeX, halfSizeY, zSign * halfSizeZ), new Point3D(-halfSizeX, halfSizeY, zSign * halfSizeZ),
                  new Point3D(-halfSizeX, -halfSizeY, zSign * halfSizeZ), new Point3D(halfSizeX, -halfSizeY, zSign * halfSizeZ)};

            if (random.nextBoolean()) // Can only generate a point from the box's center and 3 vertices. Randomly pick a triangle of 2 possible triangles for the face
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[1], faceVertices[2]));
            else
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[2], faceVertices[3]));

            expectedClosestPoint.set(pointInside.getX(), pointInside.getY(), zSign * halfSizeZ);
            expectedNormal.set(0.0, 0.0, zSign);
            box3D.transformToWorld(pointInside);
            box3D.transformToWorld(expectedClosestPoint);
            box3D.transformToWorld(expectedNormal);
            assertTrue(box3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal));
            EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
         }
      }
   }

   @Test
   void testGetVertices() throws Exception
   {
      Random random = new Random(3465);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial test with the pose set to zero
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         box3D.getPose().setToZero();

         Point3D expectedAbsoluteVertex = new Point3D(box3D.getSize());
         expectedAbsoluteVertex.scale(0.5);

         assertEquals(8, box3D.getVertices().length);

         Point3DBasics[] vertices = box3D.getVertices();

         for (int j = 0; j < vertices.length; j++)
         {
            Point3DBasics vertex = vertices[j];
            EuclidCoreTestTools.assertTuple3DEquals(vertex, box3D.getVertex(j), EPSILON);
            vertex.absolute();
            EuclidCoreTestTools.assertTuple3DEquals(expectedAbsoluteVertex, vertex, EPSILON);
         }

         double x = 0.5 * box3D.getSizeX();
         double y = 0.5 * box3D.getSizeY();
         double z = 0.5 * box3D.getSizeZ();
         int vertexIndex = 0;

         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(x, y, z), box3D.getVertex(vertexIndex++), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(-x, y, z), box3D.getVertex(vertexIndex++), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(x, -y, z), box3D.getVertex(vertexIndex++), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(-x, -y, z), box3D.getVertex(vertexIndex++), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(x, y, -z), box3D.getVertex(vertexIndex++), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(-x, y, -z), box3D.getVertex(vertexIndex++), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(x, -y, -z), box3D.getVertex(vertexIndex++), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(-x, -y, -z), box3D.getVertex(vertexIndex++), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Just testing that the vertices transformation
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);

         Point3DBasics[] expectedVertices = box3D.getVertices();

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         box3D.applyTransform(transform);
         Arrays.asList(expectedVertices).forEach(transform::transform);

         Point3DBasics[] actualVertices = box3D.getVertices();

         for (int j = 0; j < 8; j++)
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedVertices[j], actualVertices[j], EPSILON);
         }
      }
   }

   @Test
   void testApplyTransform()
   {
      Random random = new Random(346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D actual = EuclidShapeRandomTools.nextBox3D(random);
         Box3D expected = new Box3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPose().applyTransform(transform);
         actual.applyTransform(transform);

         EuclidShapeTestTools.assertBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D actual = EuclidShapeRandomTools.nextBox3D(random);
         Box3D expected = new Box3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPose().applyTransform(transform);
         actual.applyTransform(transform);

         EuclidShapeTestTools.assertBox3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   void testApplyInverseTransform()
   {
      Random random = new Random(346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D actual = EuclidShapeRandomTools.nextBox3D(random);
         Box3D original = new Box3D(actual);
         Box3D expected = new Box3D(actual);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.getPose().applyInverseTransform(transform);
         actual.applyInverseTransform(transform);

         EuclidShapeTestTools.assertBox3DEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidShapeTestTools.assertBox3DEquals(original, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D actual = EuclidShapeRandomTools.nextBox3D(random);
         Box3D original = new Box3D(actual);
         Box3D expected = new Box3D(actual);

         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);

         expected.getPose().applyInverseTransform(transform);
         actual.applyInverseTransform(transform);

         EuclidShapeTestTools.assertBox3DEquals(expected, actual, EPSILON);
         actual.applyTransform(transform);
         EuclidShapeTestTools.assertBox3DEquals(original, actual, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(89725L);

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test with identical boxes
         double lengthX = random.nextDouble();
         double widthY = random.nextDouble();
         double heightZ = random.nextDouble();

         Box3D box1 = new Box3D(lengthX, widthY, heightZ);
         Box3D box2 = new Box3D(lengthX, widthY, heightZ);

         assertTrue(box1.geometricallyEquals(box2, EPSILON), "Iteration: " + i);
         assertTrue(box2.geometricallyEquals(box1, EPSILON), "Iteration: " + i);
         assertTrue(box1.geometricallyEquals(box1, EPSILON), "Iteration: " + i);
         assertTrue(box2.geometricallyEquals(box2, EPSILON), "Iteration: " + i);

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         box1 = new Box3D(pose, lengthX, widthY, heightZ);
         box2 = new Box3D(pose, lengthX, widthY, heightZ);

         assertTrue(box1.geometricallyEquals(box2, EPSILON), "Iteration: " + i);
         assertTrue(box2.geometricallyEquals(box1, EPSILON), "Iteration: " + i);
         assertTrue(box1.geometricallyEquals(box1, EPSILON), "Iteration: " + i);
         assertTrue(box2.geometricallyEquals(box2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test 2 boxes with same pose and different size - Method 1
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double length1 = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double width1 = EuclidCoreRandomTools.nextDouble(random, 0.0, length1);
         double height1 = EuclidCoreRandomTools.nextDouble(random, 0.0, width1);
         double length2 = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double width2 = EuclidCoreRandomTools.nextDouble(random, 0.0, length2);
         double height2 = EuclidCoreRandomTools.nextDouble(random, 0.0, width2);
         Vector3D size1 = new Vector3D(length1, width1, height1);
         Vector3D size2 = new Vector3D(length2, width2, height2);
         Box3D box1 = new Box3D(pose, length1, width1, height1);
         Box3D box2 = new Box3D(pose, length2, width2, height2);

         assertTrue(box1.geometricallyEquals(box2, EPSILON) == size1.geometricallyEquals(size2, EPSILON), "Iteration: " + i);
         double epsilon = random.nextDouble();
         assertTrue(box1.geometricallyEquals(box2, epsilon) == size1.geometricallyEquals(size2, epsilon), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test 2 boxes with same pose and different size - Method 2
         double epsilon = random.nextDouble();
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D size1 = EuclidCoreRandomTools.nextVector3D(random, 2.0 * epsilon, 10.0);
         Box3D box1 = new Box3D(pose, size1.getX(), size1.getY(), size1.getZ());

         Vector3D size2 = new Vector3D();
         size2.add(size1, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon));
         Box3D box2 = new Box3D(pose, size2.getX(), size2.getY(), size2.getZ());
         assertTrue(box1.geometricallyEquals(box2, epsilon) == size1.geometricallyEquals(size2, epsilon), "Iteration: " + i);

         size2.add(size1, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon));
         box2 = new Box3D(pose, size2.getX(), size2.getY(), size2.getZ());
         assertTrue(box1.geometricallyEquals(box2, epsilon) == size1.geometricallyEquals(size2, epsilon), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test 2 boxes with same orientation, same size, but different position
         double epsilon = random.nextDouble();
         Vector3D size = EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0);

         Quaternion orientation = EuclidCoreRandomTools.nextQuaternion(random);
         Point3D position1 = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         RigidBodyTransform pose1 = new RigidBodyTransform(orientation, position1);
         Box3D box1 = new Box3D(pose1, size.getX(), size.getY(), size.getZ());

         Point3D position2 = new Point3D();
         position2.add(position1, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon));
         RigidBodyTransform pose2 = new RigidBodyTransform(orientation, position2);
         Box3D box2 = new Box3D(pose2, size.getX(), size.getY(), size.getZ());
         assertTrue(box1.geometricallyEquals(box2, epsilon), "Iteration: " + i);

         position2.add(position1, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon));
         pose2 = new RigidBodyTransform(orientation, position2);
         box2 = new Box3D(pose2, size.getX(), size.getY(), size.getZ());
         assertFalse(box1.geometricallyEquals(box2, epsilon), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test 2 boxes with same position, same size, but different orientation
         Quaternion orientation1 = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion orientation2 = EuclidCoreRandomTools.nextQuaternion(random);
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         RigidBodyTransform pose1 = new RigidBodyTransform(orientation1, position);
         RigidBodyTransform pose2 = new RigidBodyTransform(orientation2, position);
         Vector3D size = EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0);
         Box3D box1 = new Box3D(pose1, size.getX(), size.getY(), size.getZ());
         Box3D box2 = new Box3D(pose2, size.getX(), size.getY(), size.getZ());

         assertTrue(box1.geometricallyEquals(box2, EPSILON) == orientation1.geometricallyEquals(orientation2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test 180 degree flips around the x, y, or z axis
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double lengthX = random.nextDouble();
         double widthY = random.nextDouble();
         double heightZ = random.nextDouble();
         Box3D box1 = new Box3D(pose, lengthX, widthY, heightZ);

         int axis = random.nextInt(3);
         switch (axis)
         {
            case 0:
               pose.appendRollRotation(Math.PI);
               break;
            case 1:
               pose.appendPitchRotation(Math.PI);
               break;
            case 2:
               pose.appendYawRotation(Math.PI);
               break;
            default:
               throw new RuntimeException("Unexpected axis value: " + axis);
         }

         Box3D box2 = new Box3D(pose, lengthX, widthY, heightZ);
         assertTrue(box1.geometricallyEquals(box2, EPSILON), "Iteration: " + i);

         double angle = EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI / 2.0);
         switch (axis)
         {
            case 0:
               pose.appendRollRotation(angle);
               break;
            case 1:
               pose.appendPitchRotation(angle);
               break;
            case 2:
               pose.appendYawRotation(angle);
               break;
            default:
               throw new RuntimeException("Unexpected axis value: " + axis);
         }

         box2 = new Box3D(pose, lengthX, widthY, heightZ);
         assertFalse(box1.geometricallyEquals(box2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Swapping two of the size components and adding a 90-degree rotation such that the two boxes should represent the same geometry
        // Swapping X <-> Y and adding a 90-degree rotation around Z
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendYawRotation(Math.PI / 2.0);

         double lengthX = random.nextDouble();
         double widthY = random.nextDouble();
         double heightZ = random.nextDouble();
         Box3D box1 = new Box3D(pose1, lengthX, widthY, heightZ);
         Box3D box2 = new Box3D(pose2, widthY, lengthX, heightZ);

         assertTrue(box1.geometricallyEquals(box2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Swapping two of the size components and adding a 90-degree rotation such that the two boxes should represent the same geometry
        // Swapping X <-> Z and adding a 90-degree rotation around Y
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendPitchRotation(Math.PI / 2.0);

         double lengthX = random.nextDouble();
         double widthY = random.nextDouble();
         double heightZ = random.nextDouble();
         Box3D box1 = new Box3D(pose1, lengthX, widthY, heightZ);
         Box3D box2 = new Box3D(pose2, heightZ, widthY, lengthX);

         assertTrue(box1.geometricallyEquals(box2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Swapping two of the size components and adding a 90-degree rotation such that the two boxes should represent the same geometry
        // Swapping Y <-> Z and adding a 90-degree rotation around X
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendRollRotation(Math.PI / 2.0);

         double lengthX = random.nextDouble();
         double widthY = random.nextDouble();
         double heightZ = random.nextDouble();
         Box3D box1 = new Box3D(pose1, lengthX, widthY, heightZ);
         Box3D box2 = new Box3D(pose2, lengthX, heightZ, widthY);

         assertTrue(box1.geometricallyEquals(box2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Permuting the three size components and adding a 90-degree rotations such that the two boxes should represent the same geometry
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendYawRotation(Math.PI / 2.0);
         pose2.appendPitchRotation(Math.PI / 2.0);

         double lengthX = random.nextDouble();
         double widthY = random.nextDouble();
         double heightZ = random.nextDouble();
         Box3D box1 = new Box3D(pose1, lengthX, widthY, heightZ);
         Box3D box2 = new Box3D(pose2, heightZ, lengthX, widthY);

         assertTrue(box1.geometricallyEquals(box2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Permuting the three size components and adding a 90-degree rotations such that the two boxes should represent the same geometry
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendYawRotation(Math.PI / 2.0);
         pose2.appendRollRotation(Math.PI / 2.0);

         double lengthX = random.nextDouble();
         double widthY = random.nextDouble();
         double heightZ = random.nextDouble();
         Box3D box1 = new Box3D(pose1, lengthX, widthY, heightZ);
         Box3D box2 = new Box3D(pose2, widthY, heightZ, lengthX);

         assertTrue(box1.geometricallyEquals(box2, EPSILON), "Iteration: " + i);
      }
   }

   @Test
   void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(546161);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D box = EuclidShapeRandomTools.nextBox3D(random);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3D(random);
         Point3DBasics expectedSupportingVertex = Stream.of(box.getVertices()).sorted((v1,
                                                                                       v2) -> -Double.compare(TupleTools.dot(supportDirection, v1),
                                                                                                              TupleTools.dot(supportDirection, v2)))
                                                        .findFirst().get();
         Point3DReadOnly actualSupportingVertex = box.getSupportingVertex(supportDirection);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSupportingVertex, actualSupportingVertex, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D box = EuclidShapeRandomTools.nextBox3D(random);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly supportingVertex = box.getSupportingVertex(supportDirection);
         Point3D supportingVertexTranslated = new Point3D();
         supportDirection.normalize();
         assertTrue(box.isPointInside(supportingVertex, EPSILON));
         supportingVertexTranslated.scaleAdd(1.0e-6, supportDirection, supportingVertex);
         assertFalse(box.isPointInside(supportingVertexTranslated));
         supportingVertexTranslated.scaleAdd(1.0e-2, supportDirection, supportingVertex);
         Vector3D expectedNormal = new Vector3D();
         expectedNormal.sub(supportingVertexTranslated, supportingVertex);
         expectedNormal.normalize();

         Vector3D actualNormal = new Vector3D();
         box.evaluatePoint3DCollision(supportingVertexTranslated, new Point3D(), actualNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }
   }

   @Test
   void testDistance() throws Exception
   {
      Random random = new Random(986);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to a face
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);

         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         Point3D pointOutside = new Point3D();

         { // X faces
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeY),
                             EuclidCoreRandomTools.nextDouble(random, halfSizeZ));

            double expectedDistance = Math.abs(pointOutside.getX() - xSign * halfSizeX);
            box3D.transformToWorld(pointOutside);
            assertEquals(expectedDistance, box3D.distance(pointOutside), EPSILON);
         }

         { // Y faces
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(EuclidCoreRandomTools.nextDouble(random, halfSizeX),
                             ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeZ));

            double expectedDistance = Math.abs(pointOutside.getY() - ySign * halfSizeY);
            box3D.transformToWorld(pointOutside);
            assertEquals(expectedDistance, box3D.distance(pointOutside), EPSILON);
         }

         { // Z faces
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(EuclidCoreRandomTools.nextDouble(random, halfSizeX),
                             EuclidCoreRandomTools.nextDouble(random, halfSizeY),
                             zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);

            double expectedDistance = Math.abs(pointOutside.getZ() - zSign * halfSizeZ);
            box3D.transformToWorld(pointOutside);
            assertEquals(expectedDistance, box3D.distance(pointOutside), EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point not colliding, closest to an edge
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         Point3D pointOutside = new Point3D();
         Point3D closestPoint = new Point3D();

         { // Collinear to X-axis
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(EuclidCoreRandomTools.nextDouble(random, halfSizeX),
                             ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                             zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);
            closestPoint.set(pointOutside.getX(), ySign * halfSizeY, zSign * halfSizeZ);
            double expectedDistance = closestPoint.distance(pointOutside);
            box3D.transformToWorld(pointOutside);
            assertEquals(expectedDistance, box3D.distance(pointOutside), EPSILON);
         }

         { // Collinear to Y-axis
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeY),
                             zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);
            closestPoint.set(xSign * halfSizeX, pointOutside.getY(), zSign * halfSizeZ);
            double expectedDistance = closestPoint.distance(pointOutside);
            box3D.transformToWorld(pointOutside);
            assertEquals(expectedDistance, box3D.distance(pointOutside), EPSILON);
         }

         { // Collinear to Z-axis
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                             ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeZ));
            closestPoint.set(xSign * halfSizeX, ySign * halfSizeY, pointOutside.getZ());
            double expectedDistance = closestPoint.distance(pointOutside);
            box3D.transformToWorld(pointOutside);
            assertEquals(expectedDistance, box3D.distance(pointOutside), EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point not colliding, closest to a vertex
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         Point3D pointOutside = new Point3D();
         Point3D closestVertex = new Point3D();

         double xSign = random.nextBoolean() ? -1.0 : 1.0;
         double ySign = random.nextBoolean() ? -1.0 : 1.0;
         double zSign = random.nextBoolean() ? -1.0 : 1.0;
         pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                          ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                          zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);
         closestVertex.set(xSign * halfSizeX, ySign * halfSizeY, zSign * halfSizeZ);
         double expectedDistance = closestVertex.distance(pointOutside);
         box3D.transformToWorld(pointOutside);
         assertEquals(expectedDistance, box3D.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, the returned distance should be zero
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);

         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, box3D.getVertices());
         assertEquals(0.0, box3D.distance(pointInside));
      }
   }

   @Test
   void testSignedDistance() throws Exception
   {
      Random random = new Random(345346);

      for (int i = 0; i < ITERATIONS; i++)
      { // Simple tests against Box3DReadOnly.distance(Point3DReadOnly)
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         Point3DReadOnly point = EuclidCoreRandomTools.nextPoint3D(random, box3D.getSizeX(), box3D.getSizeY(), box3D.getSizeZ());
         double distance = box3D.distance(point);
         double signedDistance = box3D.signedDistance(point);

         if (!box3D.isPointInside(point))
            assertEquals(distance, signedDistance, EPSILON);
         else
            assertTrue(signedDistance < 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Using a cube making it easier to tests with queries inside
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double size = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
         box3D.getSize().set(size, size, size);
         double halfSize = 0.5 * size;

         Point3D pointInside = new Point3D();

         { // X faces
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            Point3D[] faceVertices = {new Point3D(xSign * halfSize, halfSize, halfSize), new Point3D(xSign * halfSize, -halfSize, halfSize),
                  new Point3D(xSign * halfSize, -halfSize, -halfSize), new Point3D(xSign * halfSize, halfSize, -halfSize)};

            if (random.nextBoolean()) // Can only generate a point from the box's center and 3 vertices. Randomly pick a triangle of 2 possible triangles for the face
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[1], faceVertices[2]));
            else
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[2], faceVertices[3]));

            double expectedDistance = -Math.abs(pointInside.getX() - xSign * halfSize);

            box3D.transformToWorld(pointInside);
            assertEquals(expectedDistance, box3D.signedDistance(pointInside), EPSILON);
         }

         { // Y faces
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            Point3D[] faceVertices = {new Point3D(halfSize, ySign * halfSize, halfSize), new Point3D(-halfSize, ySign * halfSize, halfSize),
                  new Point3D(-halfSize, ySign * halfSize, -halfSize), new Point3D(halfSize, ySign * halfSize, -halfSize)};

            if (random.nextBoolean()) // Can only generate a point from the box's center and 3 vertices. Randomly pick a triangle of 2 possible triangles for the face
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[1], faceVertices[2]));
            else
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[2], faceVertices[3]));

            double expectedDistance = -Math.abs(pointInside.getY() - ySign * halfSize);
            box3D.transformToWorld(pointInside);
            assertEquals(expectedDistance, box3D.signedDistance(pointInside), EPSILON);
         }

         { // Z faces
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            Point3D[] faceVertices = {new Point3D(halfSize, halfSize, zSign * halfSize), new Point3D(-halfSize, halfSize, zSign * halfSize),
                  new Point3D(-halfSize, -halfSize, zSign * halfSize), new Point3D(halfSize, -halfSize, zSign * halfSize)};

            if (random.nextBoolean()) // Can only generate a point from the box's center and 3 vertices. Randomly pick a triangle of 2 possible triangles for the face
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[1], faceVertices[2]));
            else
               pointInside.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, new Point3D(), faceVertices[0], faceVertices[2], faceVertices[3]));

            double expectedDistance = -Math.abs(pointInside.getZ() - zSign * halfSize);
            box3D.transformToWorld(pointInside);
            assertEquals(expectedDistance, box3D.signedDistance(pointInside), EPSILON);
         }
      }
   }

   @Test
   void testOrthogonalProjection() throws Exception
   {
      Random random = new Random(984);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to a face
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         Point3D pointOutside = new Point3D();
         Point3D expectedProjection = new Point3D();

         { // X faces
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeY),
                             EuclidCoreRandomTools.nextDouble(random, halfSizeZ));
            expectedProjection.set(xSign * halfSizeX, pointOutside.getY(), pointOutside.getZ());
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedProjection);
            EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, box3D.orthogonalProjectionCopy(pointOutside), EPSILON);
         }

         { // Y faces
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(EuclidCoreRandomTools.nextDouble(random, halfSizeX),
                             ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeZ));
            expectedProjection.set(pointOutside.getX(), ySign * halfSizeY, pointOutside.getZ());
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedProjection);
            EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, box3D.orthogonalProjectionCopy(pointOutside), EPSILON);
         }

         { // Z faces
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(EuclidCoreRandomTools.nextDouble(random, halfSizeX),
                             EuclidCoreRandomTools.nextDouble(random, halfSizeY),
                             zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);
            expectedProjection.set(pointOutside.getX(), pointOutside.getY(), zSign * halfSizeZ);
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedProjection);
            EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, box3D.orthogonalProjectionCopy(pointOutside), EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to an edge
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         Point3D pointOutside = new Point3D();
         Point3D expectedProjection = new Point3D();

         { // Collinear to X-axis
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(EuclidCoreRandomTools.nextDouble(random, halfSizeX),
                             ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                             zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);
            expectedProjection.set(pointOutside.getX(), ySign * halfSizeY, zSign * halfSizeZ);
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedProjection);
            EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, box3D.orthogonalProjectionCopy(pointOutside), EPSILON);
         }

         { // Collinear to Y-axis
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeY),
                             zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);
            expectedProjection.set(xSign * halfSizeX, pointOutside.getY(), zSign * halfSizeZ);
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedProjection);
            EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, box3D.orthogonalProjectionCopy(pointOutside), EPSILON);
         }

         { // Collinear to Z-axis
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                             ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                             EuclidCoreRandomTools.nextDouble(random, halfSizeZ));
            expectedProjection.set(xSign * halfSizeX, ySign * halfSizeY, pointOutside.getZ());
            box3D.transformToWorld(pointOutside);
            box3D.transformToWorld(expectedProjection);
            EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, box3D.orthogonalProjectionCopy(pointOutside), EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside, closest to a vertex
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         Point3D pointOutside = new Point3D();
         Point3D expectedProjection = new Point3D();

         double xSign = random.nextBoolean() ? -1.0 : 1.0;
         double ySign = random.nextBoolean() ? -1.0 : 1.0;
         double zSign = random.nextBoolean() ? -1.0 : 1.0;
         pointOutside.set(xSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeX,
                          ySign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeY,
                          zSign * EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * halfSizeZ);
         expectedProjection.set(xSign * halfSizeX, ySign * halfSizeY, zSign * halfSizeZ);
         box3D.transformToWorld(pointOutside);
         box3D.transformToWorld(expectedProjection);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, box3D.orthogonalProjectionCopy(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside, orthogonalProjection should return null
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, box3D.getVertices());
         assertNull(box3D.orthogonalProjectionCopy(pointInside));
      }
   }

   @Test
   void testIntersectionWith() throws Exception
   {
      Random random = new Random(865);

      for (int i = 0; i < ITERATIONS; i++)
      { // Line not intersecting, hovering over a face
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         { // X faces
            double xSign = random.nextBoolean() ? -1.0 : 1.0;
            List<LineSegment3D> edges = new ArrayList<>();
            edges.add(new LineSegment3D(xSign * halfSizeX, halfSizeY, halfSizeZ, xSign * halfSizeX, halfSizeY, -halfSizeZ));
            edges.add(new LineSegment3D(xSign * halfSizeX, halfSizeY, -halfSizeZ, xSign * halfSizeX, -halfSizeY, -halfSizeZ));
            edges.add(new LineSegment3D(xSign * halfSizeX, -halfSizeY, -halfSizeZ, xSign * halfSizeX, -halfSizeY, halfSizeZ));
            edges.add(new LineSegment3D(xSign * halfSizeX, -halfSizeY, halfSizeZ, xSign * halfSizeX, halfSizeY, halfSizeZ));

            LineSegment3D edge1 = edges.remove(random.nextInt(4));
            LineSegment3D edge2 = edges.remove(random.nextInt(3));

            Point3D pointOnEdge1 = new Point3D();
            Point3D pointOnEdge2 = new Point3D();
            pointOnEdge1.interpolate(edge1.getFirstEndpoint(), edge1.getSecondEndpoint(), random.nextDouble());
            pointOnEdge2.interpolate(edge2.getFirstEndpoint(), edge2.getSecondEndpoint(), random.nextDouble());

            pointOnEdge1.addX(xSign * EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            pointOnEdge2.addX(xSign * EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

            Line3D line = new Line3D(pointOnEdge1, pointOnEdge2);

            box3D.transformToWorld(line);
            assertEquals(0, box3D.intersectionWith(line, null, null));
         }

         { // Y faces
            double ySign = random.nextBoolean() ? -1.0 : 1.0;
            List<LineSegment3D> edges = new ArrayList<>();
            edges.add(new LineSegment3D(halfSizeX, ySign * halfSizeY, halfSizeZ, halfSizeX, ySign * halfSizeY, -halfSizeZ));
            edges.add(new LineSegment3D(halfSizeX, ySign * halfSizeY, -halfSizeZ, -halfSizeX, ySign * halfSizeY, -halfSizeZ));
            edges.add(new LineSegment3D(-halfSizeX, ySign * halfSizeY, -halfSizeZ, -halfSizeX, ySign * halfSizeY, halfSizeZ));
            edges.add(new LineSegment3D(-halfSizeX, ySign * halfSizeY, halfSizeZ, halfSizeX, ySign * halfSizeY, halfSizeZ));

            LineSegment3D edge1 = edges.remove(random.nextInt(4));
            LineSegment3D edge2 = edges.remove(random.nextInt(3));

            Point3D pointOnEdge1 = new Point3D();
            Point3D pointOnEdge2 = new Point3D();
            pointOnEdge1.interpolate(edge1.getFirstEndpoint(), edge1.getSecondEndpoint(), random.nextDouble());
            pointOnEdge2.interpolate(edge2.getFirstEndpoint(), edge2.getSecondEndpoint(), random.nextDouble());

            pointOnEdge1.addY(ySign * EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            pointOnEdge2.addY(ySign * EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

            Line3D line = new Line3D(pointOnEdge1, pointOnEdge2);

            box3D.transformToWorld(line);
            assertEquals(0, box3D.intersectionWith(line, null, null));
         }

         { // Z faces
            double zSign = random.nextBoolean() ? -1.0 : 1.0;
            List<LineSegment3D> edges = new ArrayList<>();
            edges.add(new LineSegment3D(halfSizeX, halfSizeY, zSign * halfSizeZ, halfSizeX, -halfSizeY, zSign * halfSizeZ));
            edges.add(new LineSegment3D(halfSizeX, -halfSizeY, zSign * halfSizeZ, -halfSizeX, -halfSizeY, zSign * halfSizeZ));
            edges.add(new LineSegment3D(-halfSizeX, -halfSizeY, zSign * halfSizeZ, -halfSizeX, halfSizeY, zSign * halfSizeZ));
            edges.add(new LineSegment3D(-halfSizeX, halfSizeY, zSign * halfSizeZ, halfSizeX, halfSizeY, zSign * halfSizeZ));

            LineSegment3D edge1 = edges.remove(random.nextInt(4));
            LineSegment3D edge2 = edges.remove(random.nextInt(3));

            Point3D pointOnEdge1 = new Point3D();
            Point3D pointOnEdge2 = new Point3D();
            pointOnEdge1.interpolate(edge1.getFirstEndpoint(), edge1.getSecondEndpoint(), random.nextDouble());
            pointOnEdge2.interpolate(edge2.getFirstEndpoint(), edge2.getSecondEndpoint(), random.nextDouble());

            pointOnEdge1.addZ(zSign * EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            pointOnEdge2.addZ(zSign * EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

            Line3D line = new Line3D(pointOnEdge1, pointOnEdge2);

            box3D.transformToWorld(line);
            assertEquals(0, box3D.intersectionWith(line, null, null));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Line intersecting, we generate the line from the intersections
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         double halfSizeX = 0.5 * box3D.getSizeX();
         double halfSizeY = 0.5 * box3D.getSizeY();
         double halfSizeZ = 0.5 * box3D.getSizeZ();

         Point3D expectedIntersection1 = EuclidCoreRandomTools.nextPoint3D(random, halfSizeX, halfSizeY, halfSizeZ);
         Point3D expectedIntersection2 = EuclidCoreRandomTools.nextPoint3D(random, halfSizeX, halfSizeY, halfSizeZ);
         Point3D actualIntersection1 = new Point3D();
         Point3D actualIntersection2 = new Point3D();

         double sign1 = random.nextBoolean() ? -1.0 : 1.0;
         double sign2 = random.nextBoolean() ? -1.0 : 1.0;
         int index1 = random.nextInt(3);
         int index2 = random.nextInt(3);
         if (sign1 == sign2 && index2 == index1)
            index2 = (index2 + 1) % 3; // Prevent to generate the two points on the same face.

         expectedIntersection1.setElement(index1, sign1 * 0.5 * box3D.getSize().getElement(index1));
         expectedIntersection2.setElement(index2, sign2 * 0.5 * box3D.getSize().getElement(index2));

         Line3D line = new Line3D(expectedIntersection1, expectedIntersection2);
         box3D.transformToWorld(line);
         box3D.transformToWorld(expectedIntersection1);
         box3D.transformToWorld(expectedIntersection2);
         assertEquals(2, box3D.intersectionWith(line, actualIntersection1, actualIntersection2));
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, EPSILON);
      }
   }

   @Test
   void testGetBoundingBox3D() throws Exception
   {
      Random random = new Random(34563);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         BoundingBox3D boundingBox = EuclidGeometryRandomTools.nextBoundingBox3D(random);
         box3D.getBoundingBox(boundingBox);

         for (Point3DBasics vertex : box3D.getVertices())
            assertTrue(boundingBox.isInsideInclusive(vertex));
         for (int j = 0; j < 100; j++)
            assertTrue(boundingBox.isInsideExclusive(EuclidGeometryRandomTools.nextWeightedAverage(random, box3D.getVertices())));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Using getSupportingVertex
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);

         BoundingBox3D expectedBoundingBox = new BoundingBox3D();
         expectedBoundingBox.setToNaN();
         Vector3D supportDirection = new Vector3D(Axis.X);
         expectedBoundingBox.updateToIncludePoint(box3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(box3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis.Y);
         expectedBoundingBox.updateToIncludePoint(box3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(box3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis.Z);
         expectedBoundingBox.updateToIncludePoint(box3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(box3D.getSupportingVertex(supportDirection));

         BoundingBox3DReadOnly actualBoundingBox = box3D.getBoundingBox();
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expectedBoundingBox, actualBoundingBox, EPSILON);
      }
   }
}
