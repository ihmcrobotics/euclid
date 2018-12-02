package us.ihmc.euclid.shape.convexPolytope;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTestTools;
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

      EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid, polytope.getFace(0).getCentroid(), EPSILON);
      Vector3D expectedNormal = EuclidGeometryTools.normal3DFromThreePoint3Ds(firstVertex, secondVertex, thirdVertex);
      if (expectedNormal.dot(polytope.getFace(0).getNormal()) < 0.0)
         expectedNormal.negate();
      EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, polytope.getFace(0).getNormal(), EPSILON);

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

      for (int vertexIndex = 0; vertexIndex < polytope.getVertices().size(); vertexIndex++)
      {
         Vertex3D vertex = polytope.getVertex(vertexIndex);
         // Assert that each vertex is unique
         assertTrue(polytope.getVertices().stream().noneMatch(otherVertex -> (otherVertex != vertex && otherVertex.epsilonEquals(vertex, EPSILON))));
         // Assert that all vertex are from the points added
         assertTrue(pointsAdded.stream().anyMatch(point -> point.epsilonEquals(vertex, EPSILON)));

         for (HalfEdge3D edge : vertex.getAssociatedEdges())
            assertTrue(edge.getOrigin() == vertex);
      }

      for (int edgeIndex = 0; edgeIndex < polytope.getEdges().size(); edgeIndex++)
      {
         HalfEdge3D edge = polytope.getEdge(edgeIndex);
         // Assert that each vertex is unique
         assertTrue(polytope.getEdges().stream().noneMatch(otherEdge -> (otherEdge != edge && otherEdge.epsilonEquals(edge, EPSILON))));
         // Assert that all vertex are from the points added
         assertTrue(pointsAdded.stream().anyMatch(point -> point.epsilonEquals(edge.getOrigin(), EPSILON)));
         assertTrue(pointsAdded.stream().anyMatch(point -> point.epsilonEquals(edge.getDestination(), EPSILON)));

         EuclidPolytopeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getTwinEdge().getDestination(), EPSILON);
         EuclidPolytopeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getTwinEdge().getOrigin(), EPSILON);

         EuclidPolytopeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getPreviousEdge().getDestination(), EPSILON);
         EuclidPolytopeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getNextEdge().getOrigin(), EPSILON);

         EuclidPolytopeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getPreviousEdge().getTwinEdge().getOrigin(), EPSILON);
         EuclidPolytopeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getNextEdge().getTwinEdge().getDestination(), EPSILON);

         EuclidPolytopeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getTwinEdge().getNextEdge().getOrigin(), EPSILON);
         EuclidPolytopeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getTwinEdge().getPreviousEdge().getDestination(),
                                                      EPSILON);

         assertTrue(edge.getTwinEdge().getTwinEdge() == edge);
         assertTrue(edge.getNextEdge().getPreviousEdge() == edge);
         assertTrue(edge.getPreviousEdge().getNextEdge() == edge);
      }

      for (int faceIndex = 0; faceIndex < polytope.getFaces().size(); faceIndex++)
      {
         Face3D face = polytope.getFace(faceIndex);
         assertTrue(polytope.getFaces().stream().noneMatch(otherFace -> (otherFace != face && otherFace.epsilonEquals(face, EPSILON))));
         // Assert that all vertex are from the points added
         for (HalfEdge3D edge : face.getEdges())
         {
            assertTrue(pointsAdded.stream().anyMatch(point -> point.epsilonEquals(edge.getOrigin(), EPSILON)));
            assertTrue(pointsAdded.stream().anyMatch(point -> point.epsilonEquals(edge.getDestination(), EPSILON)));
         }
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
