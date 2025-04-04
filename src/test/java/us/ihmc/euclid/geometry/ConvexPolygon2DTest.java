package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class ConvexPolygon2DTest extends ConvexPolygon2DBasicsTest<ConvexPolygon2D>
{
   @Override
   public ConvexPolygon2D createEmptyConvexPolygon2D()
   {
      return new ConvexPolygon2D();
   }

   @Override
   public ConvexPolygon2D createRandomConvexPolygon2D(Random random)
   {
      return EuclidGeometryRandomTools.nextConvexPolygon2D(random, 2.0, 50);
   }

   @Override
   public ConvexPolygon2D createConvexPolygon2D(Vertex2DSupplier supplier)
   {
      return new ConvexPolygon2D(supplier);
   }

   @Test
   public void testIssueWithGiftWrappingFromFootstepSnapping()
   {
      Random random = new Random(1738L);
      ConvexPolygon2D basePolygon = new ConvexPolygon2D();
      basePolygon.addVertex(-0.108, 0.048);
      basePolygon.addVertex(0.108, 0.030);
      basePolygon.addVertex(-0.108, -0.030);
      basePolygon.addVertex(-0.108, -0.048);
      basePolygon.update();

      double gridSizeXY = 0.02;
      int yawDivisions = 36;
      double gridSizeYaw = 2.0 * Math.PI / yawDivisions;

      double snapAreaResolution = 0.2;

      // get a polygon transformed to a random location.
      RigidBodyTransform transform = new RigidBodyTransform();
      int gridXIndex = -50;
      int gridYIndex = -31;
      int yawIndex = 19;
      transform.getTranslation().set(gridSizeXY * gridXIndex, gridSizeXY * gridYIndex, 0.0);
      transform.getRotation().appendYawRotation(yawIndex * gridSizeYaw);

      ConvexPolygon2D transformedPolygon = new ConvexPolygon2D(basePolygon);
      transformedPolygon.applyTransform(transform);

      // Here we want to collect all the points  under the foothold similar to what is done during snapping, but slightly different by assuming they're all
      // valid
      List<Point3D> footPointsInEnvironment = new ArrayList<>();
      Point2DReadOnly corner0 = transformedPolygon.getVertex(0);
      Point2DReadOnly corner1 = transformedPolygon.getVertex(1);
      Point2DReadOnly corner2 = transformedPolygon.getVertex(2);
      Point2DReadOnly corner3 = transformedPolygon.getVertex(3);

      Point2D pointOnEdge1 = new Point2D();
      Point2D pointOnEdge2 = new Point2D();
      Point2D footPointToSnap = new Point2D();

      double height = RandomNumbers.nextDouble(random, 1.0);

      for (double edgeAlpha = 0.0; edgeAlpha <= 1.0; edgeAlpha += snapAreaResolution)
      {
         pointOnEdge1.interpolate(corner0, corner1, edgeAlpha);
         pointOnEdge2.interpolate(corner3, corner2, edgeAlpha);

         for (double interiorAlpha = 0.0; interiorAlpha <= 1.0; interiorAlpha += snapAreaResolution)
         {
            footPointToSnap.interpolate(pointOnEdge1, pointOnEdge2, interiorAlpha);

            Point3D point = new Point3D(footPointToSnap.getX(), footPointToSnap.getY(), height);
            footPointsInEnvironment.add(point);
         }
      }

      // Now get the wrapped hull of this both before and after transform, and assert all the points are inside.
      ConvexPolygon2D croppedFootholdOf3D = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(footPointsInEnvironment));
      List<Point2D> sortedPoints = footPointsInEnvironment.stream().map(Point2D::new).collect(Collectors.toList());
      ConvexPolygon2D croppedFootholdOf2D = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(sortedPoints));
      assertEquals(croppedFootholdOf2D.getNumberOfVertices(), croppedFootholdOf3D.getNumberOfVertices());
      assertPointsInside(croppedFootholdOf3D, footPointsInEnvironment, 1e-7);

      List<Point2D> footPointsInFoot = footPointsInEnvironment.stream().map(point ->
                                                                            {
                                                                               Point3D transformedPoint = new Point3D(point);
                                                                               transformedPoint.applyInverseTransform(transform);
                                                                               return new Point2D(transformedPoint);
                                                                            }).toList();

      // Transform crop, and make sure all the transformed points are inside
      //         croppedFoothold.applyInverseTransform(transform, false);
      //         assertPoint2DsInside(croppedFoothold, footPointsInFoot);

      // Create a polygon around all the transformed points, and make sure they're inside
      ConvexPolygon2D polygon2D = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPointsInFoot));
      assertPoint2DsInside(polygon2D, footPointsInFoot, 1e-7);

   }

   @Test
   public void testIssueWithGiftWrappingFromFootstepSnappingWithManyAttempts()
   {
      Random random = new Random(1738L);
      ConvexPolygon2D basePolygon = new ConvexPolygon2D();
      basePolygon.addVertex(-0.108, 0.048);
      basePolygon.addVertex(0.108, 0.030);
      basePolygon.addVertex(-0.108, -0.030);
      basePolygon.addVertex(-0.108, -0.048);
      basePolygon.update();

      double gridSizeXY = 0.02;
      int yawDivisions = 36;
      double gridSizeYaw = 2.0 * Math.PI / yawDivisions;

      double snapAreaResolution = 0.2;
      int iters = 100000;
      for (int iter = 0; iter < iters; iter++)
      {
         // get a polygon transformed to a random location.
         RigidBodyTransform transform = new RigidBodyTransform();
         int gridXIndex = RandomNumbers.nextInt(random, -100, 100);
         int gridYIndex = RandomNumbers.nextInt(random, -100, 100);
         int yawIndex = RandomNumbers.nextInt(random, 0, yawDivisions);
         transform.getTranslation().set(gridSizeXY * gridXIndex, gridSizeXY * gridYIndex, 0.0);
         transform.getRotation().appendYawRotation(yawIndex * gridSizeYaw);

         ConvexPolygon2D transformedPolygon = new ConvexPolygon2D(basePolygon);
         transformedPolygon.applyTransform(transform);

         // Here we want to collect all the points  under the foothold similar to what is done during snapping, but slightly different by assuming they're all
         // valid
         List<Point3D> footPointsInEnvironment = new ArrayList<>();
         Point2DReadOnly corner0 = transformedPolygon.getVertex(0);
         Point2DReadOnly corner1 = transformedPolygon.getVertex(1);
         Point2DReadOnly corner2 = transformedPolygon.getVertex(2);
         Point2DReadOnly corner3 = transformedPolygon.getVertex(3);

         Point2D pointOnEdge1 = new Point2D();
         Point2D pointOnEdge2 = new Point2D();
         Point2D footPointToSnap = new Point2D();

         double height = RandomNumbers.nextDouble(random, 1.0);

         for (double edgeAlpha = 0.0; edgeAlpha <= 1.0; edgeAlpha += snapAreaResolution)
         {
            pointOnEdge1.interpolate(corner0, corner1, edgeAlpha);
            pointOnEdge2.interpolate(corner3, corner2, edgeAlpha);

            for (double interiorAlpha = 0.0; interiorAlpha <= 1.0; interiorAlpha += snapAreaResolution)
            {
               footPointToSnap.interpolate(pointOnEdge1, pointOnEdge2, interiorAlpha);

               Point3D point = new Point3D(footPointToSnap.getX(), footPointToSnap.getY(), height);
               footPointsInEnvironment.add(point);
            }
         }

         List<Point2D> footPointsInFoot = footPointsInEnvironment.stream().map(point ->
                                                                               {
                                                                                  Point3D transformedPoint = new Point3D(point);
                                                                                  transformedPoint.applyInverseTransform(transform);
                                                                                  return new Point2D(transformedPoint);
                                                                               }).toList();

         // Now get the wrapped hull of this both before and after transform, and assert all the points are inside.
         ConvexPolygon2D croppedFoothold = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(footPointsInEnvironment));
         assertPointsInside(croppedFoothold, footPointsInEnvironment, 1e-7);


         // Create a polygon around all teh transformed points, and make sure they're inside
         ConvexPolygon2D polygon2D = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPointsInFoot));
         assertPoint2DsInside(polygon2D, footPointsInFoot, 1e-7);
      }
   }

   private static void assertPointsInside(ConvexPolygon2DReadOnly polygonToCheck, List<Point3D> points, double epsilon)
   {
      for (Point3D point : points)
      {
         double distance = polygonToCheck.signedDistance(new Point2D(point));
         assertTrue(distance < epsilon, "Point " + point + " is not inside the polygon. Distance was " + distance);
         assertTrue(polygonToCheck.isPointInside(new Point2D(point), epsilon), "Point " + point + " is not inside the polygon. Distance was " + distance);
      }
   }

   private static void assertPoint2DsInside(ConvexPolygon2DReadOnly polygonToCheck, List<Point2D> points, double epsilon)
   {
      for (Point2D point : points)
      {
         double distance = polygonToCheck.signedDistance(point);
         assertTrue(polygonToCheck.isPointInside(point, epsilon), "Point " + point + " is not inside the polygon. Distance was " + distance);
         assertTrue(distance < epsilon, "Point " + point + " is not inside the polygon. Distance was " + distance);
      }
   }

   @Test
   public void testIssue17() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointA = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D pointB = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(pointA);
         polygon.addVertex(pointA);
         polygon.addVertex(pointB);
         polygon.addVertex(pointB);

         assertFalse(pointA.epsilonEquals(pointB, 1.0e-7));

         polygon.update();

         assertEquals(2, polygon.getNumberOfVertices());
         if (polygon.getVertex(0).equals(pointA))
         {
            assertEquals(polygon.getVertex(1), pointB);
         }
         else
         {
            assertEquals(polygon.getVertex(0), pointB);
            assertEquals(polygon.getVertex(1), pointA);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointA = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D pointB = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(pointA);
         polygon.addVertex(pointB);
         polygon.addVertex(pointA);
         polygon.addVertex(pointB);

         assertFalse(pointA.epsilonEquals(pointB, 1.0e-7));

         polygon.update();

         assertEquals(2, polygon.getNumberOfVertices());
         if (polygon.getVertex(0).equals(pointA))
         {
            assertEquals(polygon.getVertex(1), pointB);
         }
         else
         {
            assertEquals(polygon.getVertex(0), pointB);
            assertEquals(polygon.getVertex(1), pointA);
         }
      }
   }

   @Test
   public void testConstructors()
   {
      ConvexPolygon2D defaultConstructor = new ConvexPolygon2D();
      assertEquals(0.0, defaultConstructor.getNumberOfVertices(), EPSILON, "Number of vertices should be zero");
      assertTrue(defaultConstructor.isUpToDate());

      int numberOfVertices = 4;
      ArrayList<Point2D> verticesList = new ArrayList<>();
      verticesList.add(new Point2D(0.0, 0.0));
      verticesList.add(new Point2D(0.0, 1.0));
      verticesList.add(new Point2D(1.0, 0.0));
      verticesList.add(new Point2D(1.0, 1.0));

      ConvexPolygon2D listInt = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesList, numberOfVertices));
      assertEquals(4.0, listInt.getNumberOfVertices(), EPSILON, "Number of vertices should be 4");

      ConvexPolygon2D list = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesList));
      assertEquals(4.0, list.getNumberOfVertices(), EPSILON, "Number of vertices should be 4");

      double[][] verticesArray = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {1.0, 1.0}};

      ConvexPolygon2D doubleInt = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesArray, numberOfVertices));
      assertEquals(4.0, doubleInt.getNumberOfVertices(), EPSILON, "Number of vertices should be four");
      assertTrue(doubleInt.isUpToDate());

      ConvexPolygon2D doubles = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesArray));
      assertEquals(4.0, doubles.getNumberOfVertices(), EPSILON, "Number of vertices should be four");
      assertTrue(doubles.isUpToDate());

      ConvexPolygon2D polygon = new ConvexPolygon2D(doubles);
      assertEquals(4.0, polygon.getNumberOfVertices(), EPSILON, "Number of vertices should be four");
      assertTrue(polygon.isUpToDate());

      ConvexPolygon2D polygonPolygon = new ConvexPolygon2D(doubleInt, doubles);
      assertEquals(4.0, polygonPolygon.getNumberOfVertices(), EPSILON, "Number of vertices should be four");
      assertTrue(polygonPolygon.isUpToDate());
   }

   @Test
   
   
   /* If two polygons built in different ways (vertices in different order), they are still equivalent
              
                                      */
   public void geometricallyEquals() 
   {

      int numberOfVertices = 4;
      ArrayList<Point2D> verticesList = new ArrayList<>();
      verticesList.add(new Point2D(0.0, 0.0));
      verticesList.add(new Point2D(0.0, 1.0));
      verticesList.add(new Point2D(1.0, 0.0));
      verticesList.add(new Point2D(1.0, 1.0));

      int numberOfVertices2 = 4;
      ArrayList<Point2D> verticesList2 = new ArrayList<>();

      verticesList2.add(new Point2D(1.0, 0.0));
      verticesList2.add(new Point2D(1.0, 1.0));
      verticesList2.add(new Point2D(0.0, 1.0));
      verticesList2.add(new Point2D(0.0, 0.0));

      ConvexPolygon2D polygonA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesList, numberOfVertices));
      ConvexPolygon2D polygonB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesList2, numberOfVertices2));

      assertEquals(polygonA.getNumberOfVertices(), polygonB.getNumberOfVertices());

      for (int i = 0; i < numberOfVertices; i++)
      {
         boolean foundMatchingVertex = false;
         for (int j = 0; j < numberOfVertices2; j++)
         {
            if (polygonA.getVertex(i).equals(polygonB.getVertex(j)))
            {
               assertEquals(polygonA.getVertex(i), polygonB.getVertex(j));
               foundMatchingVertex = true;
               break;
            } // Once we find the corresponding vertex, we stop
         }
         assertTrue(foundMatchingVertex, "No matching vertex found in polygonB for vertex" + polygonA.getVertex(i));
      }
   }
}
