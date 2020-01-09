package us.ihmc.euclid.shape.convexPolytope.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
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
         icosahedron.getHalfEdges().forEach(edge -> assertEquals(EuclidShapeTools.icosahedronEdgeLength(radius), edge.getDirection(false).length(), EPSILON));
         assertEquals(EuclidShapeTools.icosahedronVolume(EuclidShapeTools.icosahedronEdgeLength(radius)), icosahedron.getVolume(), EPSILON);
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

         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, 0.25 * height), cone.getCentroid(), EPSILON);
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
         assertEquals(edgeLength * edgeLength * edgeLength, cube.getVolume(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(), cube.getCentroid(), EPSILON);

         for (Face3D face : cube.getFaces())
         {
            if (EuclidCoreTools.epsilonEquals(0.5 * edgeLength, Math.abs(face.getCentroid().getX()), EPSILON))
            {
               Vector3D expectedNormal = new Vector3D(Axis.X);
               if (face.getCentroid().getX() < 0.0)
                  expectedNormal.negate();
               EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, face.getNormal(), EPSILON);
            }
            else if (EuclidCoreTools.epsilonEquals(0.5 * edgeLength, Math.abs(face.getCentroid().getY()), EPSILON))
            {
               Vector3D expectedNormal = new Vector3D(Axis.Y);
               if (face.getCentroid().getY() < 0.0)
                  expectedNormal.negate();
               EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, face.getNormal(), EPSILON);
            }
            else if (EuclidCoreTools.epsilonEquals(0.5 * edgeLength, Math.abs(face.getCentroid().getZ()), EPSILON))
            {
               Vector3D expectedNormal = new Vector3D(Axis.Z);
               if (face.getCentroid().getZ() < 0.0)
                  expectedNormal.negate();
               EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, face.getNormal(), EPSILON);
            }
            else
            {
               fail("Unexpected face's centroid: " + face.getCentroid() + ", edge length: " + edgeLength);
            }
         }
      }
   }

   @Test
   void testNewCylinder() throws Exception
   {
      Random random = new Random(345343);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double length = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         int numberOfDivisions = random.nextInt(100) + 3;
         ConvexPolytope3D cylinder = EuclidPolytopeFactories.newCylinder(length, radius, numberOfDivisions);

         assertEquals(2 * numberOfDivisions, cylinder.getNumberOfVertices());
         assertEquals(3 * numberOfDivisions, cylinder.getNumberOfEdges());
         assertEquals(numberOfDivisions + 2, cylinder.getNumberOfFaces());

         cylinder.getVertices().stream().map(Point2D::new).forEach(vertex -> assertEquals(radius, vertex.distanceFromOrigin(), EPSILON));
         cylinder.getVertices().stream().forEach(vertex -> assertEquals(0.5 * length, Math.abs(vertex.getZ()), EPSILON));
         for (Face3D face : cylinder.getFaces())
         {
            // The expected distance of the face's centroid from the cylinder's axis is less than the radius because of the discretization.
            if (EuclidCoreTools.epsilonEquals(radius * Math.cos(Math.PI / numberOfDivisions), face.getCentroid().distanceXY(new Point2D()), EPSILON))
            {
               assertEquals(0.0, face.getNormal().getZ(), EPSILON);
               Vector3D expectedNormal = new Vector3D(face.getCentroid());
               expectedNormal.normalize();
               EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, face.getNormal(), EPSILON);
            }
            else if (EuclidCoreTools.epsilonEquals(0.0, face.getCentroid().distanceXY(new Point2D()), EPSILON))
            {
               if (face.getCentroid().getZ() < 0.0)
                  assertEquals(-1.0, face.getNormal().getZ(), EPSILON);
               else
                  assertEquals(1.0, face.getNormal().getZ(), EPSILON);
            }
            else
            {
               fail("Unexpected face's centroid: " + face.getCentroid());
            }
         }

         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(), cylinder.getCentroid(), EPSILON);
      }
   }

   @Test
   void testNewPyramid() throws Exception
   {
      Random random = new Random(354355);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double height = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double baseLength = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double baseWidth = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         ConvexPolytope3D pyramid = EuclidPolytopeFactories.newPyramid(height, baseLength, baseWidth);

         assertEquals(5, pyramid.getNumberOfVertices());
         assertEquals(5, pyramid.getNumberOfFaces());
         assertEquals(8, pyramid.getNumberOfEdges());

         Point3D topVertex = new Point3D(0, 0, height);
         assertEquals(1, pyramid.getVertices().stream().filter(vertex -> vertex.epsilonEquals(topVertex, EPSILON)).count());

         pyramid.getVertices().stream().filter(vertex -> !vertex.epsilonEquals(topVertex, EPSILON))
                .peek(baseVertex -> assertEquals(0.5 * baseLength, Math.abs(baseVertex.getX()), EPSILON))
                .peek(baseVertex -> assertEquals(0.5 * baseWidth, Math.abs(baseVertex.getY()), EPSILON))
                .forEach(baseVertex -> assertEquals(0.0, baseVertex.getZ(), EPSILON));

         assertEquals(EuclidShapeTools.pyramidVolume(height, baseLength, baseWidth), pyramid.getVolume(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, 0.25 * height), pyramid.getCentroid(), EPSILON);

      }
   }
}
