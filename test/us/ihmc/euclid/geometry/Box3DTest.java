package us.ihmc.euclid.geometry;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public class Box3DTest
{

   private static final double EPSILON = 1.0e-12;
   private static final int NUM_ITERATIONS = 100;

   @Test
   public void testCommonShape3dFunctionality()
   {
      Shape3DTestHelper testHelper = new Shape3DTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         double length = EuclidCoreRandomTools.generateRandomDouble(random, 0.01, 10.0);
         double width = EuclidCoreRandomTools.generateRandomDouble(random, 0.01, 10.0);
         double height = EuclidCoreRandomTools.generateRandomDouble(random, 0.01, 10.0);
         Box3D box3d = new Box3D(transform, length, width, height);

         testHelper.runSimpleTests(box3d, random, numberOfPoints);
      }

   }

   @Test
   public void testCopyConstructor()
   {
      Random random = new Random(12434L);
      Box3D box1 = createRandomBox(random);
      Box3D box2 = new Box3D(box1);

      assertBoxEquals(box1, box2, 1e-14);

      // make sure we're not copying references:
      box1.setPose(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));
      box1.setSize(random.nextDouble(), random.nextDouble(), random.nextDouble());
      assertEverythingDifferent(box1, box2, 1e-14);
   }

   @Test
   public void testConstructors()
   {
      Random random = new Random(2345L);
      RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

      double length = random.nextDouble();
      double width = random.nextDouble();
      double height = random.nextDouble();

      Box3D box1 = new Box3D(transform, length, width, height);
      Box3D box2 = new Box3D(transform, new double[] {length, width, height});
      Box3D box3 = new Box3D(length, width, height);
      box3.setPose(transform);

      assertBoxEquals(box1, box2, 0.0);
      assertBoxEquals(box1, box3, 0.0);
   }

   @Test
   public void testSetTransform3DAndGetters()
   {
      Random random = new Random(351235L);
      Box3D box = new Box3D();
      RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      box.setPose(transform);

      RigidBodyTransform transformBack = new RigidBodyTransform();
      box.getPose(transformBack);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(transform, transformBack, EPSILON);

      RotationMatrix matrix = new RotationMatrix();
      Vector3D vector = new Vector3D();
      transform.get(matrix, vector);

      RotationMatrix matrixBack = new RotationMatrix();
      Point3D pointBack = new Point3D();

      box.getOrientation(matrixBack);
      assertTrue(matrix.epsilonEquals(matrixBack, EPSILON));

      box.getCenter(pointBack);
      assertTrue(vector.epsilonEquals(pointBack, EPSILON));
      assertTrue(vector.epsilonEquals(box.getPosition(), EPSILON));
   }

   @Test
   public void testIsInsideOrOnSurfaceConvexCombinationOfVertices()
   {
      Random random = new Random(123234L);
      Box3D box = createRandomBox(random);

      Point3D[] vertices = new Point3D[8];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      box.getVertices(vertices);

      int nTests = 300;
      for (int i = 0; i < nTests; i++)
      {
         Point3D pointToTest = getRandomConvexCombination(random, vertices);

         assertTrue(box.isInsideOrOnSurface(pointToTest));
      }
   }

   @Test
   public void testVerticesProjection()
   {
      Random random = new Random(123234L);
      Box3D box = createRandomBox(random);

      Point3D[] vertices = new Point3D[8];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      box.getVertices(vertices);

      for (Point3D vertex : vertices)
      {
         Point3D projectedVertex = new Point3D(vertex);
         box.orthogonalProjection(projectedVertex);
         EuclidCoreTestTools.assertTuple3DEquals(vertex, projectedVertex, 1e-14);
      }
   }

   @Test
   public void testIsInsideOrOnSurfaceVertices()
   {
      Random random = new Random(123234L);
      Box3D box = createRandomBox(random);

      Point3D[] vertices = new Point3D[8];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      box.getVertices(vertices);

      for (Point3D vertex : vertices)
      {
         assertTrue(box.isInsideEpsilon(vertex, 1e-14));
      }
   }

   @Test
   public void testProjectionToVertices()
   {
      Random random = new Random(123234L);

      int nTests = 300;
      for (int testIndex = 0; testIndex < nTests; testIndex++)
      {
         Box3D box = createRandomBox(random);

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
            offset.scale(EuclidCoreRandomTools.generateRandomDouble(random, epsilon, maxScale));
            Point3D testPoint = new Point3D(vertex);
            testPoint.add(offset);
            assertFalse(box.isInsideEpsilon(testPoint, epsilon));

            Point3D projection = new Point3D(testPoint);
            box.orthogonalProjection(projection);
            EuclidCoreTestTools.assertTuple3DEquals(vertex, projection, epsilon);
            assertTrue(box.isInsideEpsilon(projection, epsilon));
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
      RigidBodyTransform configuration = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      Box3D box3d = new Box3D(configuration, length, width, height);

      ArrayList<Point3D> expectedVertices = new ArrayList<Point3D>(8);
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
         Box3D box = createRandomBox(random);
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
            Point3D point = getRandomConvexCombination(random, vertices);
            Point3D projectedPoint = new Point3D(point);
            box.orthogonalProjection(projectedPoint);

            Point3D closestPoint = new Point3D();
            Vector3D normal = new Vector3D();
            box.checkIfInside(point, closestPoint, normal);

            // check distance stuff:
            double epsilon = 1e-14;
            assertEquals(box.distance(closestPoint), 0.0, epsilon);
            assertTrue(box.isInsideEpsilon(closestPoint, epsilon));
            assertTrue(box.isInsideEpsilon(projectedPoint, epsilon));
            if (box.isInsideOrOnSurface(point))
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
            point2.add(EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, delta));
            Point3D closestPoint2 = new Point3D();
            Vector3D normal2 = new Vector3D();
            box.checkIfInside(point2, closestPoint2, normal2);

            Point3D point3 = new Point3D(point);
            point3.add(EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, delta));
            Point3D closestPoint3 = new Point3D();
            Vector3D normal3 = new Vector3D();
            box.checkIfInside(point3, closestPoint3, normal3);

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
         Box3D box = createRandomBox(random);
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
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
            Point3D point = getRandomConvexCombination(random, vertices);
            Point3D pointTransformed = new Point3D(point);
            box.transformToLocal(pointTransformed);
            boxTransformed.transformToWorld(pointTransformed);

            assertEquals(box.isInsideOrOnSurface(point), boxTransformed.isInsideOrOnSurface(pointTransformed));
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

      box3d.getPosition(point);
      box3d.getOrientation(quat);
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
         box.setOrientationYawPitchRoll(yaw, pitch, roll);

         RotationMatrix rotation = new RotationMatrix();
         box.getOrientation(rotation);

         double epsilon = 1e-14;
         assertEquals(yaw, rotation.getYaw(), epsilon);
         assertEquals(pitch, rotation.getPitch(), epsilon);
         assertEquals(roll, rotation.getRoll(), epsilon);
      }
   }
   
   @Test
   public void testGeometricallyEquals() {
      Random random = new Random(89725L);
      Box3D firstBox, secondBox;
      double lengthX, widthY, heightZ;
      double epsilon = 1e-7;
      
      for (int i = 0; i < NUM_ITERATIONS; ++i) {
         lengthX = random.nextDouble();
         widthY = random.nextDouble();
         heightZ = random.nextDouble();
         
         firstBox = new Box3D(lengthX, widthY, heightZ);
         
         secondBox = new Box3D(lengthX, widthY, heightZ);
         
         assertTrue(firstBox.geometricallyEquals(secondBox, epsilon));

         // SecondBox = (LX, HZ, WY)
         secondBox = new Box3D(lengthX, heightZ, widthY);
         
         assertFalse(firstBox.geometricallyEquals(secondBox, epsilon));

         secondBox.appendTransform(new RigidBodyTransform(new AxisAngle(1.0, 0.0, 0.0, Math.PI / 2.0), new Vector3D()));

         assertTrue(firstBox.geometricallyEquals(secondBox, epsilon));
         
         // SecondBox = (WY, LX, HZ)
         secondBox = new Box3D(widthY, lengthX, heightZ);
         
         assertFalse(firstBox.geometricallyEquals(secondBox, epsilon));

         secondBox.appendTransform(new RigidBodyTransform(new AxisAngle(0.0, 0.0, 1.0, Math.PI / 2.0), new Vector3D()));

         assertTrue(firstBox.geometricallyEquals(secondBox, epsilon));

         // SecondBox = (HZ, WY, LX)
         secondBox = new Box3D(heightZ, widthY, lengthX);
         
         assertFalse(firstBox.geometricallyEquals(secondBox, epsilon));

         secondBox.appendTransform(new RigidBodyTransform(new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), new Vector3D()));

         assertTrue(firstBox.geometricallyEquals(secondBox, epsilon));

         // SecondBox = (HZ, LX, WY)
         secondBox = new Box3D(heightZ, lengthX, widthY);
         
         assertFalse(firstBox.geometricallyEquals(secondBox, epsilon));

         secondBox.appendTransform(new RigidBodyTransform(0,1,0,0,0,0,1,0,1,0,0,0));

         assertTrue(firstBox.geometricallyEquals(secondBox, epsilon));

         // SecondBox = (WY, HZ, LX)
         secondBox = new Box3D(widthY, heightZ, lengthX);
         
         assertFalse(firstBox.geometricallyEquals(secondBox, epsilon));

         secondBox.appendTransform(new RigidBodyTransform(0,0,1,0,1,0,0,0,0,1,0,0));
         
         assertTrue(firstBox.geometricallyEquals(secondBox, epsilon));
      }
   }

   private static Box3D createRandomBox(Random random)
   {
      RigidBodyTransform configuration = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      double lengthX = random.nextDouble();
      double widthY = random.nextDouble();
      double heightZ = random.nextDouble();

      return new Box3D(configuration, lengthX, widthY, heightZ);
   }

   private static void assertEverythingDifferent(Box3D box1, Box3D box2, double epsilon)
   {
      assertFalse(box1.getOrientation().epsilonEquals(box2.getOrientation(), epsilon));
      assertFalse(box1.getPosition().epsilonEquals(box2.getPosition(), epsilon));

      assertFalse(EuclidCoreTools.epsilonEquals(box1.getSizeX(), box2.getSizeX(), epsilon));
      assertFalse(EuclidCoreTools.epsilonEquals(box1.getSizeY(), box2.getSizeY(), epsilon));
      assertFalse(EuclidCoreTools.epsilonEquals(box1.getSizeZ(), box2.getSizeZ(), epsilon));
   }

   private static void assertBoxEquals(Box3D box1, Box3D box2, double epsilon)
   {
      assertTrue(box1.getOrientation().epsilonEquals(box2.getOrientation(), epsilon));
      assertTrue(box1.getPosition().epsilonEquals(box2.getPosition(), epsilon));

      assertEquals(box1.getSizeX(), box2.getSizeX(), epsilon);
      assertEquals(box1.getSizeY(), box2.getSizeY(), epsilon);
      assertEquals(box1.getSizeZ(), box2.getSizeZ(), epsilon);
   }

   private static Point3D getRandomConvexCombination(Random random, Point3D[] vertices)
   {
      double[] weightings = new double[vertices.length];
      double sum = 0.0;

      for (int j = 0; j < weightings.length; j++)
      {
         sum += weightings[j] = random.nextDouble();
      }

      for (int j = 0; j < weightings.length; j++)
      {
         weightings[j] /= sum;
      }

      Point3D pointToTest = new Point3D();
      for (int j = 0; j < weightings.length; j++)
      {
         Point3D tempPoint = new Point3D(vertices[j]);
         tempPoint.scale(weightings[j]);
         pointToTest.add(tempPoint);
      }
      return pointToTest;
   }
}
