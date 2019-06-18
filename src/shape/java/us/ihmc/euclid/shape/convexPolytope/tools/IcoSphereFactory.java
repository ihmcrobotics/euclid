package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.geometry.Triangle3D;
import us.ihmc.euclid.geometry.interfaces.Triangle3DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * Factory for creating icosahedra and ico-spheres.
 * <p>
 * The algorithm is from
 * <a href="http://blog.andreaskahler.com/2009/06/creating-icosphere-mesh-in-code.html">link</a>.
 * </p>
 *
 * @author Jerry Pratt
 * @author Sylvain Bertrand
 */
public class IcoSphereFactory
{
   /**
    * Holds onto the indices referencing to the vertices of a triangle.
    */
   public static class TriangleIndices
   {
      private final int indexA;
      private final int indexB;
      private final int indexC;

      /**
       * Creates a new triangle index map.
       * 
       * @param v1 the index to the first triangle vertex.
       * @param v2 the index to the second triangle vertex.
       * @param v3 the index to the third triangle vertex.
       */
      public TriangleIndices(int v1, int v2, int v3)
      {
         this.indexA = v1;
         this.indexB = v2;
         this.indexC = v3;
      }
   }

   /**
    * Represents a 3D mesh made of 3D triangles.
    */
   public static class TriangleMesh3D implements Transformable
   {
      /** The mesh unordered vertices. */
      private final List<Point3D> vertices = new ArrayList<>();
      /** The indices to the mesh triangles. */
      private final List<TriangleIndices> faces = new ArrayList<>();

      /**
       * Adds a vertex to this mesh.
       * 
       * @param vertex the new vertex.
       */
      public void addVertex(Point3D vertex)
      {
         vertices.add(vertex);
      }

      /**
       * Gets the i<sup>th</sup> vertex of this mesh.
       * 
       * @param index the vertex index.
       * @return the vertex.
       */
      public Point3D getVertex(int index)
      {
         return vertices.get(index);
      }

      /**
       * Gets the unordered list of vertices for this mesh.
       * 
       * @return the unordered vertices.
       */
      public List<Point3D> getVertices()
      {
         return vertices;
      }

      /**
       * Gets the number of vertices this mesh is composed of.
       * 
       * @return the number of vertices.
       */
      public int getNumberOfVertices()
      {
         return vertices.size();
      }

      /**
       * Gets the number of triangles this mesh is composed of.
       * 
       * @return the number of triangles.
       */
      public int getNumberOfTriangles()
      {
         return faces.size();
      }

      /**
       * Gets the i<sup>th</sup> triangle of this mesh.
       * <p>
       * WARNING: This method generates garbage.
       * </p>
       * 
       * @param index the triangle index.
       * @return the triangle.
       */
      public Triangle3D getTriangle(int index)
      {
         TriangleIndices triangleIndices = faces.get(index);
         Point3D a = vertices.get(triangleIndices.indexA);
         Point3D b = vertices.get(triangleIndices.indexB);
         Point3D c = vertices.get(triangleIndices.indexC);
         return new Triangle3D(a, b, c);
      }

      /**
       * Gets the i<sup>th</sup> triangle of this mesh.
       * 
       * @param index          the triangle index.
       * @param triangleToPack the triangle used to pack mesh triangle. Modified.
       */
      public void getTriangle(int index, Triangle3DBasics triangleToPack)
      {
         TriangleIndices triangleIndices = faces.get(index);
         Point3D a = vertices.get(triangleIndices.indexA);
         Point3D b = vertices.get(triangleIndices.indexB);
         Point3D c = vertices.get(triangleIndices.indexC);
         triangleToPack.set(a, b, c);
      }

      /**
       * Gets the of all the triangles composing this triangle mesh.
       * 
       * @return the list of this mesh triangles.
       */
      public List<Triangle3D> getAllTriangles()
      {
         List<Triangle3D> triangles = new ArrayList<>();

         for (int index = 0; index < getNumberOfTriangles(); index++)
            triangles.add(getTriangle(index));

         return triangles;
      }

      /** {@inheritDoc} */
      @Override
      public void applyTransform(Transform transform)
      {
         vertices.forEach(transform::transform);
      }

      /** {@inheritDoc} */
      @Override
      public void applyInverseTransform(Transform transform)
      {
         vertices.forEach(transform::inverseTransform);
      }
   }

   private IcoSphereFactory()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Creates a new ico-sphere given the recursion level.
    * <p>
    * The ico-sphere is centered at the origin and the radius of its circumscribed sphere is equal to
    * 1.0.
    * </p>
    * <p>
    * A recursion level of 0 generates an icosahedron which has 12 vertices. For each additional
    * recursion level, the number of triangles is multiplied by 4. Such that for a recursion level of 5
    * the number of triangles is 5120.
    * </p>
    * 
    * @param recursionLevel the resolution of the generated mesh, preferably below 5.
    * @return the triangle mesh of the ico-sphere.
    */
   public static TriangleMesh3D newIcoSphere(int recursionLevel)
   {
      TriangleMesh3D geometry = new TriangleMesh3D();
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

   private static int addMidVertex(TriangleMesh3D geometry, Map<Long, Integer> midVertexIndexCache, int firstVertexIndex, int secondVertexIndex)
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

      int newVertexIndex = geometry.getNumberOfVertices();
      geometry.addVertex(EuclidGeometryTools.averagePoint3Ds(vertex1, vertex2));

      // store it, return index
      midVertexIndexCache.put(key, newVertexIndex);

      return newVertexIndex;
   }
}
