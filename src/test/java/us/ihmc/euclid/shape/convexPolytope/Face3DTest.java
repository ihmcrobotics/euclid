package us.ihmc.euclid.shape.convexPolytope;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.testSuite.EuclidTestSuite;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class Face3DTest
{
   private static final int ITERATIONS = EuclidTestSuite.ITERATIONS;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testSimpleConstruction2D() throws Exception
   {
      Point3D p0 = new Point3D(0, 0, 0);
      Point3D p1 = new Point3D(1, 0, 0);
      Point3D p2 = new Point3D(1, 1, 0);
      Point3D p3 = new Point3D(0, 1, 0);

      Face3D face = new Face3D(Axis.Z);

      face.addVertex(new Vertex3D(p0), 0);
      assertEquals(p0, face.getVertices().get(0));

      face.addVertex(new Vertex3D(p1), 0);
      assertEquals(p0, face.getVertices().get(0));
      assertEquals(p1, face.getVertices().get(1));

      face.addVertex(new Vertex3D(p2), 0);
      assertEquals(p1, face.getVertices().get(0));
      assertEquals(p0, face.getVertices().get(1));
      assertEquals(p2, face.getVertices().get(2));

      face.addVertex(new Vertex3D(p3), 0);
      assertEquals(p1, face.getVertices().get(0));
      assertEquals(p0, face.getVertices().get(1));
      assertEquals(p3, face.getVertices().get(2));
      assertEquals(p2, face.getVertices().get(3));
      assertEquals(4, face.getNumberOfEdges());
   }

   @Test
   public void testCircleBasedConstruction2D() throws Exception
   {
      double radius = 1.0;
      double numberOfPoints = 10;

      List<Point3D> points = new ArrayList<>();

      for (int i = 0; i < numberOfPoints; i++)
      {
         double theta = i * 2.0 * Math.PI / numberOfPoints;
         double x = -radius * Math.cos(theta);
         double y = radius * Math.sin(theta);
         points.add(new Point3D(x, y, 0));
      }

      Face3D face = new Face3D(Axis.Z);
      for (int i = 0; i < numberOfPoints; i++)
      {
         face.addVertex(new Vertex3D(points.get(i)), 0);
         assertEquals(i + 1, face.getNumberOfEdges());
      }

      Random random = new Random(3453);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      points.forEach(transform::transform);
      face = new Face3D(Axis.Z);
      for (int i = 0; i < numberOfPoints; i++)
      {
         face.addVertex(new Vertex3D(points.get(i)), 0);
         assertEquals(i + 1, face.getNumberOfEdges());
      }
   }

   @Test
   public void testHalfEdgeAreSetProperly() throws Exception
   {
      Random random = new Random(2342);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfVertices = 3;
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 1.0, 1.0, numberOfVertices, Axis.Z);
         assertEquals(numberOfVertices, face.getNumberOfEdges());

         List<HalfEdge3D> edges = face.getEdges();
         for (int edgeIndex = 0; edgeIndex < numberOfVertices; edgeIndex++)
         {
            int previousIndex = EuclidGeometryPolygonTools.previous(edgeIndex, edges.size());
            int nextIndex = EuclidGeometryPolygonTools.next(edgeIndex, edges.size());
            HalfEdge3D prevEdge = edges.get(previousIndex);
            HalfEdge3D nextEdge = edges.get(nextIndex);
            HalfEdge3D edge = edges.get(edgeIndex);

            assertTrue(edge.getPreviousEdge() == prevEdge);
            assertTrue(edge.getNextEdge() == nextEdge);
            assertTrue(edge.getFace() == face);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfVertices = 10;
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 1.0, 1.0, numberOfVertices, Axis.Z);
         assertEquals(numberOfVertices, face.getNumberOfEdges());

         List<HalfEdge3D> edges = face.getEdges();
         for (int edgeIndex = 0; edgeIndex < numberOfVertices; edgeIndex++)
         {
            int previousIndex = EuclidGeometryPolygonTools.previous(edgeIndex, edges.size());
            int nextIndex = EuclidGeometryPolygonTools.next(edgeIndex, edges.size());
            HalfEdge3D prevEdge = edges.get(previousIndex);
            HalfEdge3D nextEdge = edges.get(nextIndex);
            HalfEdge3D edge = edges.get(edgeIndex);

            assertTrue(edge.getPreviousEdge() == prevEdge);
            assertTrue(edge.getNextEdge() == nextEdge);
            assertTrue(edge.getFace() == face);
         }
      }
   }

   @Test
   public void testAddVertexWithConvexVertices()
   {
      Random random = new Random(366);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D faceNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         List<Point3D> points = EuclidPolytopeRandomTools.nextCircleBasedConvexPolygon3D(random, 5.0, 1.0, 6, faceNormal);

         Face3D face3D = new Face3D(Axis.Z);
         for (int j = 0; j < points.size(); j++)
         {
            Point3D point = points.get(j);
            face3D.addVertex(new Vertex3D(point), 0.0);
         }

         assertEquals("Iteration: " + i, points.size(), face3D.getNumberOfEdges());
      }
   }

   @Test
   public void testNormalConsistencyWithEdgeOrdering() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D expectedNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15, expectedNormal);

         assertTrue(face3D.getNumberOfEdges() >= 3);

         Vector3D actualNormal = face3D.getFaceNormal();
         String errorMessage = "Iteration: " + i + ", angle between the vectors: " + expectedNormal.angle(actualNormal);
         assertTrue(errorMessage, EuclidGeometryTools.areVector3DsParallel(expectedNormal, actualNormal, 1.0e-5));
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(expectedNormal, actualNormal, EPSILON);

      }
   }

   @Test
   public void testIsPointOnInteriorSideOfEdgeInternal() throws Exception
   {
      Random random = new Random(34534);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with the face centroid
         Vector3D faceNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15, faceNormal);

         for (int edgeIndex = 0; edgeIndex < face3D.getNumberOfEdges(); edgeIndex++)
         {
            assertTrue(face3D.canObserverSeeEdge(face3D.getFaceCentroid(), edgeIndex));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with point inside the face
         Vector3D faceNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15, faceNormal);
         List<? extends Vertex3DReadOnly> vertices = face3D.getVertices();
         Point3D pointInside = EuclidShapeRandomTools.nextWeightedAverage(random, vertices);

         for (int edgeIndex = 0; edgeIndex < face3D.getNumberOfEdges(); edgeIndex++)
         {
            assertTrue(face3D.canObserverSeeEdge(pointInside, edgeIndex));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with point on exterior/interior side of one edge
         Vector3D faceNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15, faceNormal);

         int edgeIndex = random.nextInt(face.getNumberOfEdges());
         HalfEdge3D edge = face.getEdge(edgeIndex);

         assertTrue(edge == face.getEdges().get(edgeIndex));
         assertTrue(face.getNumberOfEdges() >= 3);

         Point3D centroid = face.getFaceCentroid();

         Vector3D towardOutside = new Vector3D();
         Vector3DBasics edgeDirection = edge.getDirection(true);
         towardOutside.sub(EuclidGeometryTools.orthogonalProjectionOnLine3D(centroid, edge.getOrigin(), edgeDirection), centroid);
         towardOutside.normalize();
         assertEquals(0.0, towardOutside.dot(faceNormal), EPSILON);
         assertEquals(0.0, towardOutside.dot(edgeDirection), EPSILON);

         Point3D pointInside = new Point3D(edge.midpoint());
         pointInside.scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), towardOutside, pointInside);
         pointInside.scaleAdd(EuclidCoreRandomTools.nextDouble(random), faceNormal, pointInside);
         pointInside.scaleAdd(EuclidCoreRandomTools.nextDouble(random), edgeDirection, pointInside);
         assertTrue("Iteration: " + i, face.canObserverSeeEdge(pointInside, edgeIndex));

         assertEquals(0.0, towardOutside.dot(faceNormal), EPSILON);
         assertEquals(0.0, towardOutside.dot(edgeDirection), EPSILON);

         Point3D pointOutside = new Point3D(edge.midpoint());
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), towardOutside, pointOutside);
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random), faceNormal, pointOutside);
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random), edgeDirection, pointOutside);
         assertFalse("Iteration: " + i, face.canObserverSeeEdge(pointOutside, edgeIndex));
      }
   }

   @Test
   public void testDistance() throws Exception
   {
      Random random = new Random(3453456);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with a point inside the face
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15);
         Point3D pointOnFace = EuclidShapeRandomTools.nextWeightedAverage(random, face.getVertices());

         assertEquals(0.0, face.distance(pointOnFace), EPSILON);

         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         Point3D pointDirectlyAbove = new Point3D();
         pointDirectlyAbove.scaleAdd(expectedDistance, face.getFaceNormal(), pointOnFace);
         assertEquals(expectedDistance, face.distance(pointDirectlyAbove), EPSILON);

         Point3D pointDirectlyBelow = new Point3D();
         pointDirectlyBelow.scaleAdd(-expectedDistance, face.getFaceNormal(), pointOnFace);
         assertEquals(expectedDistance, face.distance(pointDirectlyBelow), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to an edge
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15);

         HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));

         Vector3D outsideDirection = new Vector3D();
         outsideDirection.cross(face.getFaceNormal(), edge.getDirection(false));
         outsideDirection.normalize();

         Point3D pointOnEdge = new Point3D();
         pointOnEdge.interpolate(edge.getOrigin(), edge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, outsideDirection, pointOnEdge);

         assertEquals(expectedDistance, face.distance(pointOutside), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to one of the face's vertices
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15);

         HalfEdge3D prevEdge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
         HalfEdge3D nextEdge = prevEdge.getNextEdge();

         Vertex3D closestVertex = prevEdge.getDestination();

         Vector3D prevOutsideDirection = new Vector3D();
         prevOutsideDirection.cross(face.getFaceNormal(), prevEdge.getDirection(false));
         prevOutsideDirection.normalize();

         Vector3D nextOutsideDirection = new Vector3D();
         nextOutsideDirection.cross(face.getFaceNormal(), nextEdge.getDirection(false));
         nextOutsideDirection.normalize();

         Vector3D outsideDirection = new Vector3D();
         outsideDirection.interpolate(prevOutsideDirection, nextOutsideDirection, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         outsideDirection.normalize();

         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, outsideDirection, closestVertex);

         //         assertEquals(expectedDistance, face.distance(pointOutside), EPSILON); TODO
      }
   }

   @Test
   public void testGetClosestEdgeTo() throws Exception
   {
      Random random = new Random(43654);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with triangle to get edge-cases
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 3);

         Point3D pointOutside = new Point3D();
         {
            HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));

            Vector3D outsideDirection = new Vector3D();
            outsideDirection.cross(face.getFaceNormal(), edge.getDirection(false));
            outsideDirection.normalize();

            Point3D pointOnEdge = new Point3D();
            pointOnEdge.interpolate(edge.getOrigin(), edge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

            pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), outsideDirection, pointOnEdge);
         }

         HalfEdge3D expectedEdge = null;
         int expectedEdgeIndex = -1;
         double distance = Double.POSITIVE_INFINITY;

         for (int edgeIndex = 0; edgeIndex < face.getNumberOfEdges(); edgeIndex++)
         {
            HalfEdge3D candidate = face.getEdge(edgeIndex);
            double candidateDistance = candidate.distance(pointOutside);
            if (candidateDistance < distance)
            {
               expectedEdge = candidate;
               expectedEdgeIndex = edgeIndex;
               distance = candidateDistance;
            }
         }

         HalfEdge3D actualEdge = face.getEdgeClosestTo(pointOutside);
         int actualEdgeIndex = face.getEdges().indexOf(actualEdge);
         String errorMessage = "Iteration: " + i + ", distance from: expected: " + expectedEdge.distance(pointOutside) + ", actual: "
               + actualEdge.distance(pointOutside);
         assertEquals(errorMessage, expectedEdgeIndex, actualEdgeIndex);
         assertTrue(errorMessage, expectedEdge == actualEdge);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against naive method with point outside closest to an edge
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15);

         Point3D pointOutside = new Point3D();
         {
            HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));

            Vector3D outsideDirection = new Vector3D();
            outsideDirection.cross(face.getFaceNormal(), edge.getDirection(false));
            outsideDirection.normalize();

            Point3D pointOnEdge = new Point3D();
            pointOnEdge.interpolate(edge.getOrigin(), edge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

            pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), outsideDirection, pointOnEdge);
         }

         HalfEdge3D expectedEdge = null;
         int expectedEdgeIndex = -1;
         double distance = Double.POSITIVE_INFINITY;

         for (int edgeIndex = 0; edgeIndex < face.getNumberOfEdges(); edgeIndex++)
         {
            HalfEdge3D candidate = face.getEdge(edgeIndex);
            double candidateDistance = candidate.distance(pointOutside);
            if (candidateDistance < distance)
            {
               expectedEdge = candidate;
               expectedEdgeIndex = edgeIndex;
               distance = candidateDistance;
            }
         }

         HalfEdge3D actualEdge = face.getEdgeClosestTo(pointOutside);
         int actualEdgeIndex = face.getEdges().indexOf(actualEdge);
         String errorMessage = "Iteration: " + i + ", distance from: expected: " + expectedEdge.distance(pointOutside) + ", actual: "
               + actualEdge.distance(pointOutside);
         assertEquals(errorMessage, expectedEdgeIndex, actualEdgeIndex);
         assertTrue(errorMessage, expectedEdge == actualEdge);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15);

         int expectedEdgeIndex = random.nextInt(face.getNumberOfEdges());
         HalfEdge3D expectedEdge = face.getEdge(expectedEdgeIndex);

         Vector3D outsideDirection = new Vector3D();
         outsideDirection.cross(face.getFaceNormal(), expectedEdge.getDirection(false));
         outsideDirection.normalize();

         Point3D pointOnEdge = new Point3D();
         pointOnEdge.interpolate(expectedEdge.getOrigin(), expectedEdge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), outsideDirection, pointOnEdge);

         HalfEdge3D actualEdge = face.getEdgeClosestTo(pointOutside);
         int actualEdgeIndex = face.getEdges().indexOf(actualEdge);

         assertEquals("Iteration: " + i, expectedEdgeIndex, actualEdgeIndex);
         assertTrue("Iteration: " + i, expectedEdge == actualEdge);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against naive method with point inside
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15);

         Point3D point = EuclidShapeRandomTools.nextWeightedAverage(random, face.getVertices());

         HalfEdge3D expectedEdge = null;
         int expectedEdgeIndex = -1;
         double distance = Double.POSITIVE_INFINITY;

         for (int edgeIndex = 0; edgeIndex < face.getNumberOfEdges(); edgeIndex++)
         {
            HalfEdge3D candidate = face.getEdge(edgeIndex);
            double candidateDistance = candidate.distance(point);
            if (candidateDistance < distance)
            {
               expectedEdge = candidate;
               expectedEdgeIndex = edgeIndex;
               distance = candidateDistance;
            }
         }

         HalfEdge3D actualEdge = face.getEdgeClosestTo(point);
         int actualEdgeIndex = face.getEdges().indexOf(actualEdge);
         String errorMessage = "Iteration: " + i + ", distance from: expected: " + expectedEdge.distance(point) + ", actual: " + actualEdge.distance(point);
         assertEquals(errorMessage, expectedEdgeIndex, actualEdgeIndex);
         assertTrue(errorMessage, expectedEdge == actualEdge);
      }
   }
}
