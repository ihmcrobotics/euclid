package us.ihmc.euclid.shape.convexPolytope.tools;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory.GeometryMesh3D;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory.Triangle3D;
import us.ihmc.euclid.tuple3D.Point3D;

public class IcoSphereFactoryTest
{
   private static final double ROUNDING = 1.0e-6;
   public static final double EPSILON = 1.0e-12;

   @Test
   void testIcosahedron() throws Exception
   {
      int recursionLevel = 0;
      GeometryMesh3D icoSphere = IcoSphereFactory.newIcoSphere(recursionLevel);

      List<Point3D> vertices = icoSphere.getVertices();

      assertEquals(12, vertices.size());
      assertEquals(20, icoSphere.getNumberOfTriangles());
      int numerOfEdges = icoSphere.getNumberOfTriangles() * 3 / 2;
      assertEquals(vertices.size(), numerOfEdges - icoSphere.getNumberOfTriangles() + 2);

      assertVerticesAreUnique(vertices, ROUNDING);
      assertVerticesOnUnitSphere(vertices);
      checkTriangleProperties(icoSphere.getAllTriangles(), recursionLevel);
   }

   @Test
   void testIcosphereRecursion1() throws Exception
   {
      int recursionLevel = 1;
      GeometryMesh3D icoSphere = IcoSphereFactory.newIcoSphere(recursionLevel);

      List<Point3D> vertices = icoSphere.getVertices();

      assertEquals(42, vertices.size());
      assertEquals(80, icoSphere.getNumberOfTriangles());
      int numerOfEdges = icoSphere.getNumberOfTriangles() * 3 / 2;
      assertEquals(vertices.size(), EuclidPolytopeTools.computeConvexPolytopeNumberOfVertices(icoSphere.getNumberOfTriangles(), numerOfEdges));

      assertVerticesAreUnique(vertices, ROUNDING);
      assertVerticesOnUnitSphere(vertices);
      checkTriangleProperties(icoSphere.getAllTriangles(), recursionLevel);
   }

   @Test
   void testIcosphereRecursion2() throws Exception
   {
      int recursionLevel = 2;
      GeometryMesh3D icoSphere = IcoSphereFactory.newIcoSphere(recursionLevel);

      List<Point3D> vertices = icoSphere.getVertices();

      assertEquals(4 * 80, icoSphere.getNumberOfTriangles());
      int numerOfEdges = icoSphere.getNumberOfTriangles() * 3 / 2;
      assertEquals(vertices.size(), EuclidPolytopeTools.computeConvexPolytopeNumberOfVertices(icoSphere.getNumberOfTriangles(), numerOfEdges));

      assertVerticesAreUnique(vertices, ROUNDING);
      assertVerticesOnUnitSphere(vertices);
      checkTriangleProperties(icoSphere.getAllTriangles(), recursionLevel);
   }

   @Test
   void testIcosphereRecursion3() throws Exception
   {
      int recursionLevel = 3;
      GeometryMesh3D icoSphere = IcoSphereFactory.newIcoSphere(recursionLevel);

      List<Point3D> vertices = icoSphere.getVertices();

      assertEquals(16 * 80, icoSphere.getNumberOfTriangles());
      int numerOfEdges = icoSphere.getNumberOfTriangles() * 3 / 2;
      assertEquals(vertices.size(), EuclidPolytopeTools.computeConvexPolytopeNumberOfVertices(icoSphere.getNumberOfTriangles(), numerOfEdges));

      assertVerticesAreUnique(vertices, ROUNDING);
      assertVerticesOnUnitSphere(vertices);
      checkTriangleProperties(icoSphere.getAllTriangles(), recursionLevel);
   }

   @Test
   void testIcosphereRecursion4() throws Exception
   {
      int recursionLevel = 4;
      GeometryMesh3D icoSphere = IcoSphereFactory.newIcoSphere(recursionLevel);

      List<Point3D> vertices = icoSphere.getVertices();

      assertEquals(2562, vertices.size());
      assertEquals(5120, icoSphere.getNumberOfTriangles());
      int numerOfEdges = icoSphere.getNumberOfTriangles() * 3 / 2;
      assertEquals(vertices.size(), EuclidPolytopeTools.computeConvexPolytopeNumberOfVertices(icoSphere.getNumberOfTriangles(), numerOfEdges));

      assertVerticesAreUnique(vertices, 1.0e-6);
      assertVerticesOnUnitSphere(vertices);
      checkTriangleProperties(icoSphere.getAllTriangles(), recursionLevel);
   }

   private void assertVerticesOnUnitSphere(List<Point3D> vertices)
   {
      vertices.forEach(vertex -> assertEquals(1.0, vertex.distanceFromOrigin(), EPSILON, "Vertex not on unit-sphere."));
   }

   private void assertVerticesAreUnique(List<Point3D> vertices, double epsilon)
   {
      long numberOfDistinctVertices = vertices.stream().map(vertex -> round(vertex, epsilon)).distinct().count();
      assertEquals(vertices.size(), numberOfDistinctVertices);
   }

   private Point3D round(Point3D value, double precision)
   {
      return new Point3D(round(value.getX(), precision), round(value.getY(), precision), round(value.getZ(), precision));
   }

   private double round(double value, double precision)
   {
      return ((long) (value / precision)) * precision;
   }

   private static void checkTriangleProperties(List<Triangle3D> icoSphereTrianges, int recursionLevel)
   {
      if (recursionLevel == 0)
      { // The triangles should form a regular icosahedron
        // Assert the triangles are all equilateral
         icoSphereTrianges.forEach(triangle -> assertTrue(triangle.isEquilateral(EPSILON),
                                                          "Not equilateral: " + triangle.getAB() + ", " + triangle.getBC() + ", " + triangle.getCA()));

         // Assert that the edges are of equal length (since all the triangles are equilateral only need to check one edge per triangle)
         Triangle3D firstTriangle = icoSphereTrianges.get(0);
         double expectedEdgeLength = firstTriangle.getAB();

         icoSphereTrianges.stream().mapToDouble(Triangle3D::getAB)
                          .forEach(actualEdgeLength -> assertEquals(expectedEdgeLength, actualEdgeLength, EPSILON, "Edges are not all of same length."));

         // Assert that the triangles have the same area
         double expectedArea = firstTriangle.getArea();

         for (int i = 1; i < icoSphereTrianges.size(); i++)
         {
            assertEquals(expectedArea, icoSphereTrianges.get(i).getArea(), EPSILON);
         }
      }
      else
      { // Only 20 triangles will be equilateral
         int expectedNumberOfEquilateralTriangles = 20;
         int actualNumberOfEquilateralTriangles = (int) icoSphereTrianges.stream().filter(triangle -> triangle.isEquilateral(EPSILON)).count();
         assertEquals(expectedNumberOfEquilateralTriangles, actualNumberOfEquilateralTriangles);

         // That's about all we can test.
      }
   }
}
