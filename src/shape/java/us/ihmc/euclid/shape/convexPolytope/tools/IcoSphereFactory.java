package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * From http://blog.andreaskahler.com/2009/06/creating-icosphere-mesh-in-code.html
 *
 */
public class IcoSphereFactory
{
   public static class TriangleIndices
   {
      private final int indexA;
      private final int indexB;
      private final int indexC;

      public TriangleIndices(int v1, int v2, int v3)
      {
         this.indexA = v1;
         this.indexB = v2;
         this.indexC = v3;
      }
   }

   public static class Triangle3D
   {
      private final Point3D a = new Point3D();
      private final Point3D b = new Point3D();
      private final Point3D c = new Point3D();

      public Triangle3D()
      {
      }

      public Triangle3D(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
      {
         set(a, b, c);
      }

      public Triangle3D(Triangle3D other)
      {
         set(other);
      }

      public void set(Triangle3D other)
      {
         set(other.getA(), other.getB(), other.getC());
      }

      public void set(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
      {
         this.a.set(a);
         this.b.set(b);
         this.c.set(c);
      }

      public Point3D getA()
      {
         return a;
      }

      public Point3D getB()
      {
         return b;
      }

      public Point3D getC()
      {
         return c;
      }

      public double getAB()
      {
         return a.distance(b);
      }

      public double getBC()
      {
         return b.distance(c);
      }

      public double getCA()
      {
         return c.distance(a);
      }

      public double getArea()
      {
         return EuclidGeometryTools.triangleArea(a, b, c);
      }

      public boolean isEquilateral(double epsilon)
      {
         double abSquare = a.distanceSquared(b);
         double acSquare = a.distanceSquared(c);

         if (!EuclidCoreTools.epsilonEquals(abSquare, acSquare, epsilon))
            return false;

         double bcSquare = b.distanceSquared(c);

         return EuclidCoreTools.epsilonEquals(acSquare, bcSquare, epsilon);
      }

      public boolean geometryEquals(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c, double epsilon)
      {
         if (this.a.geometricallyEquals(a, epsilon))
         {
            if (this.b.geometricallyEquals(b, epsilon))
               return this.c.geometricallyEquals(c, epsilon);
            else
               return this.b.geometricallyEquals(c, epsilon) && this.c.geometricallyEquals(b, epsilon);
         }
         else if (this.a.geometricallyEquals(b, epsilon))
         {
            if (this.b.geometricallyEquals(a, epsilon))
               return this.c.geometricallyEquals(c, epsilon);
            else
               return this.b.geometricallyEquals(c, epsilon) && this.c.geometricallyEquals(a, epsilon);
         }
         else if (this.a.geometricallyEquals(c, epsilon))
         {
            if (this.b.geometricallyEquals(a, epsilon))
               return this.c.geometricallyEquals(b, epsilon);
            else
               return this.b.geometricallyEquals(b, epsilon) && this.c.geometricallyEquals(a, epsilon);
         }
         else
         {
            return false;
         }
      }
   }

   public static class GeometryMesh3D
   {
      private final List<Point3D> vertices = new ArrayList<>();
      private final List<TriangleIndices> faces = new ArrayList<>();

      public int addVertex(Point3D vertex)
      {
         vertices.add(vertex);
         return vertices.size() - 1;
      }

      public Point3D getVertex(int index)
      {
         return vertices.get(index);
      }

      public List<Point3D> getVertices()
      {
         return vertices;
      }

      public int getNumberOfVertices()
      {
         return vertices.size();
      }

      public int getNumberOfTriangles()
      {
         return faces.size();
      }

      public Triangle3D getTriangle(int index)
      {
         TriangleIndices triangleIndices = faces.get(index);
         Point3D a = vertices.get(triangleIndices.indexA);
         Point3D b = vertices.get(triangleIndices.indexB);
         Point3D c = vertices.get(triangleIndices.indexC);
         return new Triangle3D(a, b, c);
      }

      public List<Triangle3D> getAllTriangles()
      {
         List<Triangle3D> triangles = new ArrayList<>();

         for (int index = 0; index < getNumberOfTriangles(); index++)
            triangles.add(getTriangle(index));

         return triangles;
      }
   }

   public static GeometryMesh3D newIcoSphere(int recursionLevel)
   {
      GeometryMesh3D geometry = new GeometryMesh3D();
      Map<Long, Integer> midVertexIndexCache = new HashMap<>();

      // create 12 vertices of a icosahedron
      double t = (1.0 + Math.sqrt(5.0)) / 2.0;

      geometry.addVertex(new Point3D(-1, t, 0));
      geometry.addVertex(new Point3D(1, t, 0));
      geometry.addVertex(new Point3D(-1, -t, 0));
      geometry.addVertex(new Point3D(1, -t, 0));
      geometry.addVertex(new Point3D(0, -1, t));
      geometry.addVertex(new Point3D(0, 1, t));
      geometry.addVertex(new Point3D(0, -1, -t));
      geometry.addVertex(new Point3D(0, 1, -t));
      geometry.addVertex(new Point3D(t, 0, -1));
      geometry.addVertex(new Point3D(t, 0, 1));
      geometry.addVertex(new Point3D(-t, 0, -1));
      geometry.addVertex(new Point3D(-t, 0, 1));

      // create 20 triangles of the icosahedron
      List<TriangleIndices> faces = new ArrayList<>();

      // 5 faces around point 0
      faces.add(new TriangleIndices(0, 11, 5));
      faces.add(new TriangleIndices(0, 5, 1));
      faces.add(new TriangleIndices(0, 1, 7));
      faces.add(new TriangleIndices(0, 7, 10));
      faces.add(new TriangleIndices(0, 10, 11));

      // 5 adjacent faces
      faces.add(new TriangleIndices(1, 5, 9));
      faces.add(new TriangleIndices(5, 11, 4));
      faces.add(new TriangleIndices(11, 10, 2));
      faces.add(new TriangleIndices(10, 7, 6));
      faces.add(new TriangleIndices(7, 1, 8));

      // 5 faces around point 3
      faces.add(new TriangleIndices(3, 9, 4));
      faces.add(new TriangleIndices(3, 4, 2));
      faces.add(new TriangleIndices(3, 2, 6));
      faces.add(new TriangleIndices(3, 6, 8));
      faces.add(new TriangleIndices(3, 8, 9));

      // 5 adjacent faces
      faces.add(new TriangleIndices(4, 9, 5));
      faces.add(new TriangleIndices(2, 4, 11));
      faces.add(new TriangleIndices(6, 2, 10));
      faces.add(new TriangleIndices(8, 6, 7));
      faces.add(new TriangleIndices(9, 8, 1));

      // refine triangles
      for (int i = 0; i < recursionLevel; i++)
      {
         List<TriangleIndices> newFaces = new ArrayList<>();

         for (TriangleIndices triangleIndices : faces)
         {
            // replace triangle by 4 triangles
            int midAB = addMidVertex(geometry, midVertexIndexCache, triangleIndices.indexA, triangleIndices.indexB);
            int midBC = addMidVertex(geometry, midVertexIndexCache, triangleIndices.indexB, triangleIndices.indexC);
            int midCA = addMidVertex(geometry, midVertexIndexCache, triangleIndices.indexC, triangleIndices.indexA);

            newFaces.add(new TriangleIndices(triangleIndices.indexA, midAB, midCA));
            newFaces.add(new TriangleIndices(triangleIndices.indexB, midBC, midAB));
            newFaces.add(new TriangleIndices(triangleIndices.indexC, midCA, midBC));
            newFaces.add(new TriangleIndices(midAB, midBC, midCA));
         }
         faces = newFaces;
      }

      // done, now add triangles to mesh
      geometry.faces.addAll(faces);
      // Move all vertices such that they lie on the unit-sphere
      geometry.vertices.forEach(vertex -> vertex.scale(1.0 / vertex.distanceFromOrigin()));

      return geometry;
   }

   private static int addMidVertex(GeometryMesh3D geometry, Map<Long, Integer> midVertexIndexCache, int firstVertexIndex, int secondVertexIndex)
   {
      // first check if we have it already
      boolean firstIsSmaller = firstVertexIndex < secondVertexIndex;
      long smallerIndex = firstIsSmaller ? firstVertexIndex : secondVertexIndex;
      long greaterIndex = firstIsSmaller ? secondVertexIndex : firstVertexIndex;
      long key = (smallerIndex << 32) + greaterIndex;

      Integer midVertexIndex = midVertexIndexCache.get(key);

      if (midVertexIndex != null)
         return midVertexIndex.intValue();

      // not in cache, calculate it
      Point3D vertex1 = geometry.getVertex(firstVertexIndex);
      Point3D vertex2 = geometry.getVertex(secondVertexIndex);

      int newVertexIndex = geometry.addVertex(EuclidGeometryTools.averagePoint3Ds(vertex1, vertex2));

      // store it, return index
      midVertexIndexCache.put(key, newVertexIndex);

      return newVertexIndex;
   }
}
