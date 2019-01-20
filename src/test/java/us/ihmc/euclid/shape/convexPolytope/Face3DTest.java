package us.ihmc.euclid.shape.convexPolytope;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.testSuite.EuclidTestSuite;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
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
   public void testGetFirstVisibleEdge() throws Exception
   {
      { // Simple queries using a 
        // TODO
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

         assertEquals(points.size(), face3D.getNumberOfEdges(), "Iteration: " + i);
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

         Vector3D actualNormal = face3D.getNormal();
         String errorMessage = "Iteration: " + i + ", angle between the vectors: " + expectedNormal.angle(actualNormal);
         assertTrue(EuclidGeometryTools.areVector3DsParallel(expectedNormal, actualNormal, 1.0e-5), errorMessage);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(expectedNormal, actualNormal, EPSILON);

      }
   }

   @Test
   public void testCanObserverSeeEdge() throws Exception
   {
      Random random = new Random(34534);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with the face centroid
         Vector3D faceNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15, faceNormal);

         for (int edgeIndex = 0; edgeIndex < face3D.getNumberOfEdges(); edgeIndex++)
         {
            assertFalse(face3D.canObserverSeeEdge(face3D.getCentroid(), edgeIndex));
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
            assertFalse(face3D.canObserverSeeEdge(pointInside, edgeIndex));
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

         Point3D centroid = face.getCentroid();

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
         assertFalse(face.canObserverSeeEdge(pointInside, edgeIndex), "Iteration: " + i);

         assertEquals(0.0, towardOutside.dot(faceNormal), EPSILON);
         assertEquals(0.0, towardOutside.dot(edgeDirection), EPSILON);

         Point3D pointOutside = new Point3D(edge.midpoint());
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), towardOutside, pointOutside);
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random), faceNormal, pointOutside);
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random), edgeDirection, pointOutside);
         assertTrue(face.canObserverSeeEdge(pointOutside, edgeIndex), "Iteration: " + i);
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
         pointDirectlyAbove.scaleAdd(expectedDistance, face.getNormal(), pointOnFace);
         assertEquals(expectedDistance, face.distance(pointDirectlyAbove), EPSILON);

         Point3D pointDirectlyBelow = new Point3D();
         pointDirectlyBelow.scaleAdd(-expectedDistance, face.getNormal(), pointOnFace);
         assertEquals(expectedDistance, face.distance(pointDirectlyBelow), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to an edge
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15);

         HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));

         Vector3D outsideDirection = new Vector3D();
         outsideDirection.cross(face.getNormal(), edge.getDirection(false));
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
         prevOutsideDirection.cross(face.getNormal(), prevEdge.getDirection(false));
         prevOutsideDirection.normalize();

         Vector3D nextOutsideDirection = new Vector3D();
         nextOutsideDirection.cross(face.getNormal(), nextEdge.getDirection(false));
         nextOutsideDirection.normalize();

         Vector3D outsideDirection = new Vector3D();
         outsideDirection.interpolate(prevOutsideDirection, nextOutsideDirection, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         outsideDirection.normalize();

         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(expectedDistance, outsideDirection, closestVertex);

         assertEquals(expectedDistance, face.distance(pointOutside), EPSILON);
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
            outsideDirection.cross(face.getNormal(), edge.getDirection(false));
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

         HalfEdge3D actualEdge = face.getClosestEdge(pointOutside);
         int actualEdgeIndex = face.getEdges().indexOf(actualEdge);
         String errorMessage = "Iteration: " + i + ", distance from: expected: " + expectedEdge.distance(pointOutside) + ", actual: "
               + actualEdge.distance(pointOutside);
         assertEquals(expectedEdgeIndex, actualEdgeIndex, errorMessage);
         assertTrue(expectedEdge == actualEdge, errorMessage);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against naive method with point outside closest to an edge
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15);

         Point3D pointOutside = new Point3D();
         {
            HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));

            Vector3D outsideDirection = new Vector3D();
            outsideDirection.cross(face.getNormal(), edge.getDirection(false));
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

         HalfEdge3D actualEdge = face.getClosestEdge(pointOutside);
         int actualEdgeIndex = face.getEdges().indexOf(actualEdge);
         String errorMessage = "Iteration: " + i + ", distance from: expected: " + expectedEdge.distance(pointOutside) + ", actual: "
               + actualEdge.distance(pointOutside);
         assertEquals(expectedEdgeIndex, actualEdgeIndex, errorMessage);
         assertTrue(expectedEdge == actualEdge, errorMessage);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15);

         int expectedEdgeIndex = random.nextInt(face.getNumberOfEdges());
         HalfEdge3D expectedEdge = face.getEdge(expectedEdgeIndex);

         Vector3D outsideDirection = new Vector3D();
         outsideDirection.cross(face.getNormal(), expectedEdge.getDirection(false));
         outsideDirection.normalize();

         Point3D pointOnEdge = new Point3D();
         pointOnEdge.interpolate(expectedEdge.getOrigin(), expectedEdge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), outsideDirection, pointOnEdge);

         HalfEdge3D actualEdge = face.getClosestEdge(pointOutside);
         int actualEdgeIndex = face.getEdges().indexOf(actualEdge);

         assertEquals(expectedEdgeIndex, actualEdgeIndex, "Iteration: " + i);
         assertTrue(expectedEdge == actualEdge, "Iteration: " + i);
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

         HalfEdge3D actualEdge = face.getClosestEdge(point);
         int actualEdgeIndex = face.getEdges().indexOf(actualEdge);
         String errorMessage = "Iteration: " + i + ", distance from: expected: " + expectedEdge.distance(point) + ", actual: " + actualEdge.distance(point);
         assertEquals(expectedEdgeIndex, actualEdgeIndex, errorMessage);
         assertTrue(expectedEdge == actualEdge, errorMessage);
      }
   }

   @Test
   public void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial 2D tests:
         List<Point2D> vertices2D = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 5.0, 1.0, 15);
         Face3D face = new Face3D(Axis.Z);
         vertices2D.forEach(vertex2D -> face.addVertex(new Vertex3D(new Point3D(vertex2D)), 0.0));

         Vector3D supportVector = new Vector3D();

         // supportVector = +Axis.X
         supportVector.set(Axis.X);
         Vertex3DReadOnly expected = face.getVertices().stream().sorted((v1, v2) -> -Double.compare(v1.getX(), v2.getX())).findFirst().get();
         Point3DReadOnly actual = face.getSupportingVertex(supportVector);
         assertEquals(expected, actual, "Iteration: " + i);

         // supportVector = -Axis.X
         supportVector.setAndNegate(Axis.X);
         expected = face.getVertices().stream().sorted((v1, v2) -> Double.compare(v1.getX(), v2.getX())).findFirst().get();
         actual = face.getSupportingVertex(supportVector);
         assertEquals(expected, actual, "Iteration: " + i);

         // supportVector = +Axis.Y
         supportVector.set(Axis.Y);
         expected = face.getVertices().stream().sorted((v1, v2) -> -Double.compare(v1.getY(), v2.getY())).findFirst().get();
         actual = face.getSupportingVertex(supportVector);
         assertEquals(expected, actual, "Iteration: " + i);

         // supportVector = -Axis.Y
         supportVector.setAndNegate(Axis.Y);
         expected = face.getVertices().stream().sorted((v1, v2) -> Double.compare(v1.getY(), v2.getY())).findFirst().get();
         actual = face.getSupportingVertex(supportVector);
         assertEquals(expected, actual, "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Using the projection on a line to predict which vertex is the farthest.
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 5.0, 1.0, 15);

         Vector3D supportVector = EuclidCoreRandomTools.nextVector3D(random);
         LinePercentageComparator comparator = new LinePercentageComparator(new Line3D(new Point3D(), supportVector));
         comparator.flipDirection();

         Vertex3DReadOnly expected = face.getVertices().stream().sorted(comparator::compare).findFirst().get();
         Point3DReadOnly actual = face.getSupportingVertex(supportVector);

         assertEquals(expected, actual, "Iteration: " + i);
      }
   }

   private static class LinePercentageComparator implements Comparator<Point3DReadOnly>
   {
      private final Line3D line;

      public LinePercentageComparator(Line3D line)
      {
         this.line = line;
      }

      public void flipDirection()
      {
         line.negateDirection();
      }

      @Override
      public int compare(Point3DReadOnly o1, Point3DReadOnly o2)
      {
         double p1 = EuclidGeometryTools.percentageAlongLine3D(o1, line.getPoint(), line.getDirection());
         double p2 = EuclidGeometryTools.percentageAlongLine3D(o2, line.getPoint(), line.getDirection());
         return Double.compare(p1, p2);
      }
   }
}
