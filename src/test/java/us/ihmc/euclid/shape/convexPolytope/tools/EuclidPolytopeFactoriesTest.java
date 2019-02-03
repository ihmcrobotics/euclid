package us.ihmc.euclid.shape.convexPolytope.tools;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

class EuclidPolytopeFactoriesTest
{
   public static final int ITERATIONS = 1000;
   public static final double EPSILON = 1.0e-12;

   @Test
   void testNewIcosahedron()
   {
      Random random = new Random(23049723);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         ConvexPolytope3D icosahedron = EuclidPolytopeFactories.newIcosahedron(radius);

         assertEquals(12, icosahedron.getNumberOfVertices());
         assertEquals(20, icosahedron.getNumberOfFaces());
         assertEquals(30, icosahedron.getNumberOfEdges());

         icosahedron.getVertices().forEach(vertex -> assertEquals(radius, vertex.distanceFromOrigin(), EPSILON));
         icosahedron.getHalfEdges()
                    .forEach(edge -> assertEquals(EuclidPolytopeTools.icosahedronEdgeLength(radius), edge.getDirection(false).length(), EPSILON));
         assertEquals(EuclidPolytopeTools.icosahedronVolume(EuclidPolytopeTools.icosahedronEdgeLength(radius)), icosahedron.getVolume(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(), icosahedron.getCentroid(), EPSILON);
      }
   }

   @Test
   void testNewIcoSphere() throws Exception
   {
      Random random = new Random(3463634);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         int recursionLevel = random.nextInt(3);
         ConvexPolytope3D icoSphere = EuclidPolytopeFactories.newIcoSphere(radius, recursionLevel);

         icoSphere.getVertices().forEach(vertex -> assertEquals(radius, vertex.distanceFromOrigin(), EPSILON));
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(), icoSphere.getCentroid(), EPSILON);
      }
   }

   @Test
   void testNewCone() throws Exception
   {
      Random random = new Random(8956057);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double height = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         int numberOfDivisions = random.nextInt(100) + 3;
         ConvexPolytope3D cone = EuclidPolytopeFactories.newCone(height, radius, numberOfDivisions);

         assertEquals(numberOfDivisions + 1, cone.getNumberOfVertices());
         assertEquals(numberOfDivisions + 1, cone.getNumberOfFaces());
         assertEquals(2 * numberOfDivisions, cone.getNumberOfEdges());

         Point3D topVertex = new Point3D(0, 0, height);
         assertEquals(1, cone.getVertices().stream().filter(vertex -> vertex.epsilonEquals(topVertex, EPSILON)).count());
         cone.getVertices().stream().filter(vertex -> !vertex.epsilonEquals(topVertex, EPSILON))
             .peek(baseVertex -> assertEquals(0.0, baseVertex.getZ(), EPSILON))
             .forEach(baseVertex -> assertEquals(radius, baseVertex.distanceFromOrigin(), EPSILON));

         Vector3D baseNormal = new Vector3D(0, 0, -1);
         assertEquals(1, cone.getFaces().stream().filter(face -> face.getNormal().epsilonEquals(baseNormal, EPSILON)).count());
      }
   }

   @Test
   void testNewCube() throws Exception
   {
      Random random = new Random(34563575);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double edgeLength = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         ConvexPolytope3D cube = EuclidPolytopeFactories.newCube(edgeLength);

         assertEquals(8, cube.getNumberOfVertices());
         assertEquals(12, cube.getNumberOfEdges());
         assertEquals(6, cube.getNumberOfFaces());

         Point3D absoluteVertex = new Point3D(edgeLength, edgeLength, edgeLength);
         absoluteVertex.scale(0.5);
         cube.getVertices().stream().map(Point3D::new).peek(Point3D::absolute)
             .forEach(vertex -> EuclidCoreTestTools.assertTuple3DEquals(vertex, absoluteVertex, EPSILON));
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(), cube.getCentroid(), EPSILON);
      }
   }
}
