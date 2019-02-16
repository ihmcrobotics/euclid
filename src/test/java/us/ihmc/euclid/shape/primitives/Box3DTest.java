package us.ihmc.euclid.shape.primitives;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Random;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public class Box3DTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testCommonShape3dFunctionality()
   {
      Shape3DTestHelper testHelper = new Shape3DTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         testHelper.runSimpleTests(EuclidShapeRandomTools.nextBox3D(random), random, numberOfPoints);
      }
   }

   @Test
   public void testCopyConstructor()
   {
      Random random = new Random(12434L);
      Box3D box1 = EuclidShapeRandomTools.nextBox3D(random);
      Box3D box2 = new Box3D(box1);

      EuclidShapeTestTools.assertBox3DEquals(box1, box2, 1e-14);

      // make sure we're not copying references:
      box1.getPose().set(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      box1.setSize(random.nextDouble(), random.nextDouble(), random.nextDouble());
      assertEverythingDifferent(box1, box2, 1e-14);
   }

   @Test
   public void testConstructors()
   {
      Random random = new Random(2345L);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      double length = random.nextDouble();
      double width = random.nextDouble();
      double height = random.nextDouble();

      Box3D box1 = new Box3D(transform, length, width, height);
      Box3D box2 = new Box3D(transform, new double[] {length, width, height});
      Box3D box3 = new Box3D(length, width, height);
      box3.getPose().set(transform);

      EuclidShapeTestTools.assertBox3DEquals(box1, box2, 0.0);
      EuclidShapeTestTools.assertBox3DEquals(box1, box3, 0.0);
   }

   @Test
   public void testSetTransform3DAndGetters()
   {
      Random random = new Random(351235L);
      Box3D box = new Box3D();
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      box.getPose().set(transform);

      RigidBodyTransform transformBack = new RigidBodyTransform();
      transformBack.set(box.getPose());
      EuclidCoreTestTools.assertRigidBodyTransformEquals(transform, transformBack, EPSILON);

      RotationMatrix matrix = new RotationMatrix();
      Vector3D vector = new Vector3D();
      transform.get(matrix, vector);

      RotationMatrix matrixBack = new RotationMatrix();
      Point3D pointBack = new Point3D();

      matrixBack.set(box.getPose().getShapeOrientation());
      assertTrue(matrix.epsilonEquals(matrixBack, EPSILON));

      box.getCenter(pointBack);
      assertTrue(vector.epsilonEquals(pointBack, EPSILON));
      assertTrue(vector.epsilonEquals(box.getPosition(), EPSILON));
   }

   @Test
   public void testIsInsideOrOnSurfaceConvexCombinationOfVertices()
   {
      Random random = new Random(123234L);
      Box3D box = EuclidShapeRandomTools.nextBox3D(random);

      Point3D[] vertices = new Point3D[8];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      box.getVertices(vertices);

      int nTests = 300;
      for (int i = 0; i < nTests; i++)
      {
         Point3D pointToTest = EuclidShapeRandomTools.nextWeightedAverage(random, vertices);

         assertTrue(box.isPointInside(pointToTest));
      }
   }

   @Test
   public void testVerticesProjection()
   {
      Random random = new Random(123234L);
      Box3D box = EuclidShapeRandomTools.nextBox3D(random);

      Point3D[] vertices = new Point3D[8];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      box.getVertices(vertices);

      for (int vertexIndex = 0; vertexIndex < vertices.length; vertexIndex++)
      {
         Point3D vertex = vertices[vertexIndex];
         Point3D projectedVertex = new Point3D(vertex);
         box.orthogonalProjection(projectedVertex);
         EuclidCoreTestTools.assertTuple3DEquals("Vertex index: " + vertexIndex, vertex, projectedVertex, 1e-14);
      }
   }

   @Test
   public void testIsInsideOrOnSurfaceVertices()
   {
      Random random = new Random(123234L);
      Box3D box = EuclidShapeRandomTools.nextBox3D(random);

      Point3D[] vertices = new Point3D[8];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      box.getVertices(vertices);

      for (Point3D vertex : vertices)
      {
         assertTrue(box.isPointInside(vertex, 1e-14));
      }
   }

   @Test
   public void testProjectionToVertices()
   {
      Random random = new Random(123234L);

      int nTests = 300;
      for (int testIndex = 0; testIndex < nTests; testIndex++)
      {
         Box3D box = EuclidShapeRandomTools.nextBox3D(random);

         Point3D[] vertices = new Point3D[8];
         for (int i = 0; i < vertices.length; i++)
         {
            vertices[i] = new Point3D();
         }

         double epsilon = EPSILON;
         double maxScale = 2.0;
         box.getVertices(vertices);
         Tuple3DReadOnly center = box.getPosition();
         for (Point3D vertex : vertices)
         {
            Vector3D offset = new Vector3D(vertex);
            offset.sub(center);
            offset.normalize();
            offset.scale(EuclidCoreRandomTools.nextDouble(random, epsilon, maxScale));
            Point3D testPoint = new Point3D(vertex);
            testPoint.add(offset);
            assertFalse(box.isPointInside(testPoint, epsilon));

            Point3D projection = new Point3D(testPoint);
            box.orthogonalProjection(projection);
            EuclidCoreTestTools.assertTuple3DEquals(vertex, projection, epsilon);
            assertTrue(box.isPointInside(projection, epsilon));
         }
      }
   }

   @Test
   public void testComputeVertices()
   {
      Random random = new Random(6234L);
      double length = random.nextDouble();
      double width = random.nextDouble();
      double height = random.nextDouble();
      RigidBodyTransform configuration = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      Box3D box3d = new Box3D(configuration, length, width, height);

      ArrayList<Point3D> expectedVertices = new ArrayList<>(8);
      expectedVertices.add(new Point3D(length / 2.0, width / 2.0, height / 2.0));
      expectedVertices.add(new Point3D(length / 2.0, width / 2.0, -height / 2.0));
      expectedVertices.add(new Point3D(length / 2.0, -width / 2.0, height / 2.0));
      expectedVertices.add(new Point3D(length / 2.0, -width / 2.0, -height / 2.0));
      expectedVertices.add(new Point3D(-length / 2.0, width / 2.0, height / 2.0));
      expectedVertices.add(new Point3D(-length / 2.0, width / 2.0, -height / 2.0));
      expectedVertices.add(new Point3D(-length / 2.0, -width / 2.0, height / 2.0));
      expectedVertices.add(new Point3D(-length / 2.0, -width / 2.0, -height / 2.0));

      for (Point3D point3d : expectedVertices)
      {
         configuration.transform(point3d);
      }

      Point3D[] vertices = new Point3D[8];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      box3d.getVertices(vertices);

      double epsilon = EPSILON;
      for (Point3D vertex : vertices)
      {
         Iterator<Point3D> iterator = expectedVertices.iterator();
         while (iterator.hasNext())
         {
            Point3D expectedVertex = iterator.next();
            if (expectedVertex.epsilonEquals(vertex, epsilon))
               iterator.remove();
         }
      }

      assertEquals(0, expectedVertices.size());
   }

   @Test
   public void testDistanceProjectionAndClosestPointAndNormal()
   {
      Random random = new Random(123415L);
      int nBoxes = 10;
      int nTestsPerBox = 1000;
      for (int boxNumber = 0; boxNumber < nBoxes; boxNumber++)
      {
         Box3D box = EuclidShapeRandomTools.nextBox3D(random);
         Box3D biggerBox = new Box3D(box);
         biggerBox.scale(2.0);

         Point3D[] vertices = new Point3D[8];
         for (int i = 0; i < vertices.length; i++)
         {
            vertices[i] = new Point3D();
         }
         biggerBox.getVertices(vertices);

         for (int testNumber = 0; testNumber < nTestsPerBox; testNumber++)
         {
            Point3D point = EuclidShapeRandomTools.nextWeightedAverage(random, vertices);
            Point3D projectedPoint = new Point3D(point);
            box.orthogonalProjection(projectedPoint);

            Point3D closestPoint = new Point3D();
            Vector3D normal = new Vector3D();
            box.doPoint3DCollisionTest(point, closestPoint, normal);

            // check distance stuff:
            double epsilon = 1e-14;
            assertEquals(box.distance(closestPoint), 0.0, epsilon);
            assertTrue(box.isPointInside(closestPoint, epsilon));
            assertTrue(box.isPointInside(projectedPoint, epsilon));
            if (box.isPointInside(point))
            {
               EuclidCoreTestTools.assertTuple3DEquals(point, projectedPoint, 1e-10);
            }
            else
            {
               assertEquals(point.distance(projectedPoint), Math.abs(box.distance(point)), epsilon);
               EuclidCoreTestTools.assertTuple3DEquals(projectedPoint, closestPoint, epsilon);
            }

            // create 3 points that are close to point and check whether a numerical normal to a plane through those points equals the returned normal:
            double delta = 1e-5;

            Point3D point2 = new Point3D(point);
            point2.add(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, delta));
            Point3D closestPoint2 = new Point3D();
            Vector3D normal2 = new Vector3D();
            box.doPoint3DCollisionTest(point2, closestPoint2, normal2);

            Point3D point3 = new Point3D(point);
            point3.add(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, delta));
            Point3D closestPoint3 = new Point3D();
            Vector3D normal3 = new Vector3D();
            box.doPoint3DCollisionTest(point3, closestPoint3, normal3);

            boolean pointsDistinct = closestPoint2.distance(closestPoint) > epsilon && closestPoint3.distance(closestPoint) > epsilon
                  && closestPoint3.distance(closestPoint2) > epsilon;
            if (pointsDistinct)
            {
               boolean normalsEqual = normal.epsilonEquals(normal2, epsilon) && normal.epsilonEquals(normal3, epsilon);
               if (normalsEqual)
               {
                  Vector3D vector1 = new Vector3D();
                  vector1.sub(closestPoint2, closestPoint);

                  Vector3D vector2 = new Vector3D();
                  vector2.sub(closestPoint3, closestPoint);

                  Vector3D normalFromCrossProduct = new Vector3D();
                  normalFromCrossProduct.cross(vector1, vector2);
                  normalFromCrossProduct.normalize();

                  assertEquals(1.0, Math.abs(normalFromCrossProduct.dot(normal)), 1e-8);
               }
            }
         }
      }
   }

   @Test
   public void testApplyTransform()
   {
      Random random = new Random(562346L);
      int nBoxes = 100;
      int nTestsPerBox = 1000;
      for (int boxNumber = 0; boxNumber < nBoxes; boxNumber++)
      {
         Box3D box = EuclidShapeRandomTools.nextBox3D(random);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Box3D boxTransformed = new Box3D(box);
         boxTransformed.applyTransform(transform);
         Box3D biggerBox = new Box3D(box);
         biggerBox.scale(2.0);

         Point3D[] vertices = new Point3D[8];
         for (int i = 0; i < vertices.length; i++)
         {
            vertices[i] = new Point3D();
         }
         biggerBox.getVertices(vertices);

         for (int testNumber = 0; testNumber < nTestsPerBox; testNumber++)
         {
            Point3D point = EuclidShapeRandomTools.nextWeightedAverage(random, vertices);
            Point3D pointTransformed = new Point3D(point);
            box.transformToLocal(pointTransformed);
            boxTransformed.transformToWorld(pointTransformed);

            assertEquals(box.isPointInside(point), boxTransformed.isPointInside(pointTransformed));
         }
      }
   }

   @Test
   public void testApplyTransform2()
   {
      Box3D box3d = new Box3D();
      RigidBodyTransform transform = new RigidBodyTransform();
      Point3D point = new Point3D();
      point.set(1.0, 1.0, 1.0);
      transform.setTranslation(point);
      box3d.applyTransform(transform);

      box3d.getCenter(point);
      Point3D expectedPosition = new Point3D(1.0, 1.0, 1.0);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPosition, point, EPSILON);

      Quaternion quat = new Quaternion();
      quat.setYawPitchRoll(1.0, 1.0, 1.0);
      Quaternion quat2 = new Quaternion(quat);
      transform.setRotationAndZeroTranslation(quat);

      box3d.applyTransform(transform);

      point.set(box3d.getPose().getShapePosition());
      quat.set(box3d.getPose().getShapeOrientation());
      expectedPosition.applyTransform(transform);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPosition, point, EPSILON);
      EuclidCoreTestTools.assertQuaternionEquals(quat2, quat, EPSILON);
   }

   @Test
   public void testSetYawPitchRoll()
   {
      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         Random random = new Random(562346L);
         Box3D box = new Box3D();

         double yaw = random.nextDouble();
         double pitch = random.nextDouble();
         double roll = random.nextDouble();
         box.getPose().setRotationYawPitchRoll(yaw, pitch, roll);

         RotationMatrix rotation = new RotationMatrix();
         rotation.set(box.getPose().getShapeOrientation());

         double epsilon = 1e-14;
         assertEquals(yaw, rotation.getYaw(), epsilon);
         assertEquals(pitch, rotation.getPitch(), epsilon);
         assertEquals(roll, rotation.getRoll(), epsilon);
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
         Point3D expectedSupportingVertex = Stream.of(box.getVertices())
                                                  .sorted((v1,
                                                           v2) -> -Double.compare(TupleTools.dot(supportDirection, v1), TupleTools.dot(supportDirection, v2)))
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
         assertTrue(box.isPointInside(supportingVertex));
         supportingVertexTranslated.scaleAdd(1.0e-6, supportDirection, supportingVertex);
         assertFalse(box.isPointInside(supportingVertexTranslated));
         supportingVertexTranslated.scaleAdd(1.0e-2, supportDirection, supportingVertex);
         Vector3D expectedNormal = new Vector3D();
         expectedNormal.sub(supportingVertexTranslated, supportingVertex);
         expectedNormal.normalize();

         Vector3D actualNormal = new Vector3D();
         box.doPoint3DCollisionTest(supportingVertexTranslated, new Point3D(), actualNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }
   }

   private static void assertEverythingDifferent(Box3D box1, Box3D box2, double epsilon)
   {
      assertFalse(box1.getOrientation().epsilonEquals(box2.getOrientation(), epsilon));
      assertFalse(box1.getPosition().epsilonEquals(box2.getPosition(), epsilon));

      assertFalse(EuclidCoreTools.epsilonEquals(box1.getSizeX(), box2.getSizeX(), epsilon));
      assertFalse(EuclidCoreTools.epsilonEquals(box1.getSizeY(), box2.getSizeY(), epsilon));
      assertFalse(EuclidCoreTools.epsilonEquals(box1.getSizeZ(), box2.getSizeZ(), epsilon));
   }
}
