package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;

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
}
