package us.ihmc.euclid.shape.convexPolytope;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.testSuite.EuclidTestSuite;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ConvexPolytope3DTest
{
   private static final int ITERATIONS = EuclidTestSuite.ITERATIONS;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testConstrustingAPolytope()
   {
      Random random = new Random(4533543);

      ConvexPolytope3D polytope = new ConvexPolytope3D();

      // Testing properties for empty polytope.
      assertEquals(0, polytope.getNumberOfVertices());
      assertEquals(0, polytope.getNumberOfEdges());
      assertEquals(0, polytope.getNumberOfFaces());

      List<Point3D> pointsAdded = new ArrayList<>();

      // Testing properties for single vertex polytope.
      Point3D firstVertex = EuclidCoreRandomTools.nextPoint3D(random);
      polytope.addVertex(firstVertex, 0.0);
      pointsAdded.add(firstVertex);

      assertEquals(1, polytope.getNumberOfVertices());
      assertEquals(0, polytope.getNumberOfEdges());
      //      assertEquals(0, polytope.getNumberOfFaces()); // FIXME
      EuclidCoreTestTools.assertTuple3DEquals(firstVertex, polytope.getCentroid(), EPSILON);

      for (int vertexIndex = 0; vertexIndex < pointsAdded.size(); vertexIndex++)
      {
         EuclidCoreTestTools.assertTuple3DEquals(pointsAdded.get(vertexIndex), polytope.getVertex(vertexIndex), EPSILON);
      }

      // Assert that adding the same point twice does not change anything
      polytope.addVertex(firstVertex, 0.0);
      assertEquals(1, polytope.getNumberOfVertices());
      assertEquals(0, polytope.getNumberOfEdges());
      //      assertEquals(0, polytope.getNumberOfFaces()); // FIXME

      // Testing properties for single edge polytope.
      Point3D secondVertex = EuclidCoreRandomTools.nextPoint3D(random);
      polytope.addVertex(secondVertex, 0.0);
      pointsAdded.add(secondVertex);

      assertEquals(2, polytope.getNumberOfVertices());
      assertEquals(1, polytope.getNumberOfEdges());
      //      assertEquals(0, polytope.getNumberOfFaces()); // FIXME
      EuclidCoreTestTools.assertTuple3DEquals(EuclidGeometryTools.averagePoint3Ds(pointsAdded), polytope.getCentroid(), EPSILON);

      for (int vertexIndex = 0; vertexIndex < pointsAdded.size(); vertexIndex++)
      {
         EuclidCoreTestTools.assertTuple3DEquals(pointsAdded.get(vertexIndex), polytope.getVertex(vertexIndex), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Assert that adding a point that is on the edge does not change anything.
         Point3D pointInside = EuclidShapeRandomTools.nextWeightedAverage(random, pointsAdded);

         //         polytope.addVertex(pointInside, 0.0); FIXME

         assertEquals(2, polytope.getNumberOfVertices());
         assertEquals(1, polytope.getNumberOfEdges());
         //      assertEquals(0, polytope.getNumberOfFaces()); // FIXME
      }

      // Testing properties for single triangle face polytope
      Point3D thirdVertex = EuclidCoreRandomTools.nextPoint3D(random);
      polytope.addVertex(thirdVertex, 0.0);
      pointsAdded.add(thirdVertex);

      assertEquals(3, polytope.getNumberOfVertices());
      //      assertEquals(3, polytope.getNumberOfEdges()); // FIXME
      assertEquals(1, polytope.getNumberOfFaces());
      Point3D expectedCentroid = EuclidGeometryTools.averagePoint3Ds(pointsAdded);
      EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid, polytope.getCentroid(), EPSILON);

      for (int vertexIndex = 0; vertexIndex < pointsAdded.size(); vertexIndex++)
      {
         EuclidCoreTestTools.assertTuple3DEquals(pointsAdded.get(vertexIndex), polytope.getVertex(vertexIndex), EPSILON);
      }

      EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid, polytope.getFace(0).getFaceCentroid(), EPSILON);
      Vector3D expectedNormal = EuclidGeometryTools.normal3DFromThreePoint3Ds(firstVertex, secondVertex, thirdVertex);
      if (expectedNormal.dot(polytope.getFace(0).getFaceNormal()) < 0.0)
         expectedNormal.negate();
      EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, polytope.getFace(0).getFaceNormal(), EPSILON);

      for (int i = 0; i < ITERATIONS; i++)
      { // Assert that adding a point that is on the face does not change anything.
         Point3D pointInside = EuclidShapeRandomTools.nextWeightedAverage(random, pointsAdded);

         //         polytope.addVertex(pointInside, 0.0);

         assertEquals(3, polytope.getNumberOfVertices());
         //      assertEquals(3, polytope.getNumberOfEdges()); // FIXME
         assertEquals(1, polytope.getNumberOfFaces());
      }

      // We finally have an usual polytope: a tetrahedron
      Point3D fourthVertex = EuclidCoreRandomTools.nextPoint3D(random);
      polytope.addVertex(fourthVertex, 0.0);
      pointsAdded.add(fourthVertex);

      assertEquals(4, polytope.getNumberOfVertices());
      assertEquals(6, polytope.getNumberOfEdges());
      assertEquals(4, polytope.getNumberOfFaces());
      expectedCentroid = EuclidGeometryTools.averagePoint3Ds(pointsAdded);
      EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid, polytope.getCentroid(), EPSILON);

      for (int vertexIndex = 0; vertexIndex < pointsAdded.size(); vertexIndex++)
      {
         Vertex3D vertex = polytope.getVertex(vertexIndex);
         assertTrue(pointsAdded.stream().anyMatch(point -> point.epsilonEquals(vertex, EPSILON)));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Assert that adding a point that is inside the polytope does not change anything.
         Point3D pointInside = EuclidShapeRandomTools.nextWeightedAverage(random, pointsAdded);

         assertTrue(polytope.isInteriorPoint(pointInside, EPSILON));

         polytope.addVertex(pointInside, 0.0);

         assertEquals(4, polytope.getNumberOfVertices());
         assertEquals(6, polytope.getNumberOfEdges());
         assertEquals(4, polytope.getNumberOfFaces());
      }
   }
}
