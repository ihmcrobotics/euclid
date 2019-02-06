package us.ihmc.euclid.shape.convexPolytope;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
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
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Face3DTest
{
   private static final int ITERATIONS = EuclidTestSuite.ITERATIONS;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testSimpleConstruction2D() throws Exception
   {
      // p0 -> p3 forms a unit-square which vertices are counter-clockwise ordered.
      Point3D p0 = new Point3D(0, 0, 0);
      Point3D p1 = new Point3D(1, 0, 0);
      Point3D p2 = new Point3D(1, 1, 0);
      Point3D p3 = new Point3D(0, 1, 0);

      Face3D face = new Face3D(Axis.Z);

      face.addVertex(new Vertex3D(p0));
      assertEquals(p0, face.getVertices().get(0));
      assertEquals(1, face.getNumberOfEdges()); // The first edge is initialized to start and end at the first vertex added.
      EuclidCoreTestTools.assertTuple3DEquals(Axis.Z, face.getNormal(), EPSILON);

      face.addVertex(new Vertex3D(p1));
      assertEquals(p0, face.getVertices().get(0));
      assertEquals(p1, face.getVertices().get(1));
      assertEquals(2, face.getNumberOfEdges());
      EuclidCoreTestTools.assertTuple3DEquals(Axis.Z, face.getNormal(), EPSILON);

      // From here we're verifying that the face is re-ordering the vertices so they are clockwise ordered.
      face.addVertex(new Vertex3D(p2));
      assertEquals(p1, face.getVertices().get(0));
      assertEquals(p0, face.getVertices().get(1));
      assertEquals(p2, face.getVertices().get(2));
      assertEquals(3, face.getNumberOfEdges());
      EuclidCoreTestTools.assertTuple3DEquals(Axis.Z, face.getNormal(), EPSILON);

      face.addVertex(new Vertex3D(p3));
      assertEquals(p1, face.getVertices().get(0));
      assertEquals(p0, face.getVertices().get(1));
      assertEquals(p3, face.getVertices().get(2));
      assertEquals(p2, face.getVertices().get(3));
      assertEquals(4, face.getNumberOfEdges());
      EuclidCoreTestTools.assertTuple3DEquals(Axis.Z, face.getNormal(), EPSILON);
   }

   @Test
   public void testCircleBasedConstruction2D() throws Exception
   {
      double radius = 1.0;
      double numberOfPoints = 10;

      // points form a polygonized circle which vertices are clockwise ordered.
      List<Point3D> points = new ArrayList<>();

      for (int i = 0; i < numberOfPoints; i++)
      {
         double theta = i * 2.0 * Math.PI / numberOfPoints;
         double x = -radius * Math.cos(theta); // Negating the sign here to make the points be clockwise ordered.
         double y = radius * Math.sin(theta);
         points.add(new Point3D(x, y, 0));
      }

      Face3D face = new Face3D(Axis.Z);

      for (int i = 0; i < numberOfPoints; i++)
      {
         face.addVertex(new Vertex3D(points.get(i)));
         assertEquals(i + 1, face.getNumberOfEdges()); // Ensuring that all vertices are added.
         EuclidCoreTestTools.assertTuple3DEquals(points.get(i), face.getVertices().get(i), EPSILON);
      }

      EuclidCoreTestTools.assertTuple3DEquals(Axis.Z, face.getNormal(), EPSILON); // Check that the normal has not changed.

      Random random = new Random(3453);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      points.forEach(transform::transform);

      face = new Face3D(Axis.Z);

      for (int i = 0; i < numberOfPoints; i++)
      {
         face.addVertex(new Vertex3D(points.get(i)));
         assertEquals(i + 1, face.getNumberOfEdges());
      }

      // Check the normal
      Vector3D expectedNormal = new Vector3D();
      transform.getRotation().getColumn(2, expectedNormal);
      // Because the initial is +Z, if the rotation is too large, the estimated normal will be flipped such that its z-component is positive.
      if (expectedNormal.getZ() < 0.0)
         expectedNormal.negate();
      EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, face.getNormal(), EPSILON);
   }

   @Test
   public void testLineOfSight() throws Exception
   {
      Random random = new Random(1951);

      for (int i = 0; i < ITERATIONS; i++)
      { // We'll use the line-of-sight calculation from ConvexPolygon2D, compute everything in 2D and then go to 3D (no transform).
         ConvexPolygon2D convexPolygon2D = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 5.0, random.nextInt(25) + 3);
         Point2D centroid2D = new Point2D(convexPolygon2D.getCentroid());
         int numberOfVertices = convexPolygon2D.getNumberOfVertices();

         int edgeIndex = random.nextInt(numberOfVertices);
         Point2DReadOnly vertex2D = convexPolygon2D.getVertex(edgeIndex);
         Point2DReadOnly nextVertex2D = convexPolygon2D.getNextVertex(edgeIndex);
         Point2D pointOnEdge2D = new Point2D();
         pointOnEdge2D.interpolate(vertex2D, nextVertex2D, random.nextDouble());
         assertEquals(0.0, convexPolygon2D.distance(pointOnEdge2D), EPSILON);
         // Create a 2D observer that is outside the polygon:
         Point2D observer2D = new Point2D();
         observer2D.interpolate(centroid2D, pointOnEdge2D, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         assertFalse(convexPolygon2D.isPointInside(observer2D));

         int[] expectedLineOfSightIndices = convexPolygon2D.lineOfSightIndices(observer2D);
         Point2DBasics[] lineOfSightVertices2D = convexPolygon2D.lineOfSightVertices(observer2D);
         assertNotNull(lineOfSightVertices2D);
         assertEquals(2, lineOfSightVertices2D.length);
         assertNotNull(lineOfSightVertices2D[0]);
         assertNotNull(lineOfSightVertices2D[1]);

         // Now let's go to 3D
         Face3D face3D = new Face3D(Axis.Z);
         convexPolygon2D.getPolygonVerticesView().stream().map(Point3D::new).map(Vertex3D::new).forEach(v -> face3D.addVertex(v));
         assertEquals(numberOfVertices, face3D.getNumberOfEdges());
         EuclidCoreTestTools.assertTuple3DEquals(Axis.Z, face3D.getNormal(), EPSILON);

         Point3D[] expectedLineOfSightVertices3D = Stream.of(lineOfSightVertices2D).map(Point3D::new).toArray(Point3D[]::new);

         Point3D observer3D = new Point3D(observer2D);
         assertFalse(face3D.isPointInside(observer3D, 0.0));
         assertTrue(face3D.isPointInFacePlane(observer3D, EPSILON));

         Vertex3DReadOnly expectedLineOfSightStartVertex3D = face3D.getVertices().get(expectedLineOfSightIndices[0]);
         Vertex3DReadOnly expectedLineOfSightEndVertex3D = face3D.getVertices().get(expectedLineOfSightIndices[1]);
         HalfEdge3D expectedLineOfSightStartEdge3D = face3D.getEdge(expectedLineOfSightIndices[0]);
         HalfEdge3D expectedLineOfSightEndEdge3D = face3D.getEdge(expectedLineOfSightIndices[1]).getPrevious();
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[0], expectedLineOfSightStartVertex3D, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[0], expectedLineOfSightStartEdge3D.getOrigin(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[1], expectedLineOfSightEndVertex3D, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[1], expectedLineOfSightEndEdge3D.getDestination(), EPSILON);

         { // Testing Face3DReadOnly.canObserverSeeEdge(...) which is used for the line-of-sight calculation.
            { // Go through the edges that are in the line of sight of the observer and assert that they are visible.
               int visibleEdgeIndex = expectedLineOfSightIndices[0];
               HalfEdge3D visibleEdge = expectedLineOfSightStartEdge3D;

               while (visibleEdge != expectedLineOfSightEndEdge3D.getNext())
               {
                  assertTrue(face3D.canObserverSeeEdge(observer3D, visibleEdgeIndex));
                  assertTrue(face3D.canObserverSeeEdge(observer3D, visibleEdge));
                  visibleEdgeIndex = (visibleEdgeIndex + 1) % face3D.getNumberOfEdges();
                  visibleEdge = visibleEdge.getNext();
               }
            }

            { // Go through the hidden edges.
               int hiddenEdgeIndex = expectedLineOfSightIndices[1];
               HalfEdge3D hiddenEdge = expectedLineOfSightEndEdge3D.getNext();

               while (hiddenEdge != expectedLineOfSightStartEdge3D)
               {
                  assertFalse(face3D.canObserverSeeEdge(observer3D, hiddenEdgeIndex));
                  assertFalse(face3D.canObserverSeeEdge(observer3D, hiddenEdge));
                  hiddenEdgeIndex = (hiddenEdgeIndex + 1) % face3D.getNumberOfEdges();
                  hiddenEdge = hiddenEdge.getNext();
               }
            }
         }

         String errorMessage = "Iteration: " + i;
         assertNotNull(face3D.lineOfSightStart(observer3D), errorMessage);
         EuclidCoreTestTools.assertTuple3DEquals(errorMessage, expectedLineOfSightVertices3D[0], face3D.lineOfSightStart(observer3D).getOrigin(), EPSILON);

         assertNotNull(face3D.lineOfSightEnd(observer3D), errorMessage);
         EuclidCoreTestTools.assertTuple3DEquals(errorMessage, expectedLineOfSightVertices3D[1], face3D.lineOfSightEnd(observer3D).getDestination(), EPSILON);

         List<HalfEdge3D> lineOfSight3D = face3D.lineOfSight(observer3D);
         assertFalse(lineOfSight3D.isEmpty(), errorMessage);
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[0], lineOfSight3D.get(0).getOrigin(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[1], lineOfSight3D.get(lineOfSight3D.size() - 1).getDestination(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Same test as before, but when switching to 3D we apply a transform
         ConvexPolygon2D convexPolygon2D = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 5.0, random.nextInt(25) + 3);
         Point2D centroid2D = new Point2D(convexPolygon2D.getCentroid());
         int numberOfVertices = convexPolygon2D.getNumberOfVertices();

         int edgeIndex = random.nextInt(numberOfVertices);
         Point2DReadOnly vertex2D = convexPolygon2D.getVertex(edgeIndex);
         Point2DReadOnly nextVertex2D = convexPolygon2D.getNextVertex(edgeIndex);
         Point2D pointOnEdge2D = new Point2D();
         pointOnEdge2D.interpolate(vertex2D, nextVertex2D, random.nextDouble());
         assertEquals(0.0, convexPolygon2D.distance(pointOnEdge2D), EPSILON);
         // Create a 2D observer that is outside the polygon:
         Point2D observer2D = new Point2D();
         observer2D.interpolate(centroid2D, pointOnEdge2D, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         assertFalse(convexPolygon2D.isPointInside(observer2D));

         int[] expectedLineOfSightIndices = convexPolygon2D.lineOfSightIndices(observer2D);
         Point2DBasics[] lineOfSightVertices2D = convexPolygon2D.lineOfSightVertices(observer2D);
         assertNotNull(lineOfSightVertices2D);
         assertEquals(2, lineOfSightVertices2D.length);
         assertNotNull(lineOfSightVertices2D[0]);
         assertNotNull(lineOfSightVertices2D[1]);

         // Now let's go to 3D and apply a random transform as we are switching.
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D faceNormal = new Vector3D();
         transform.getRotation().getColumn(2, faceNormal);

         Face3D face3D = new Face3D(faceNormal); // Feeding the expected normal to prevent any flip between expected and actual normal.
         convexPolygon2D.getPolygonVerticesView().stream().map(Point3D::new).map(Vertex3D::new).peek(transform::transform).forEach(v -> face3D.addVertex(v));
         assertEquals(numberOfVertices, face3D.getNumberOfEdges());
         EuclidCoreTestTools.assertTuple3DEquals(faceNormal, face3D.getNormal(), EPSILON);

         Point3D[] expectedLineOfSightVertices3D = Stream.of(lineOfSightVertices2D).map(Point3D::new).peek(transform::transform).toArray(Point3D[]::new);

         Point3D observer3D = new Point3D(observer2D);
         observer3D.applyTransform(transform);
         assertFalse(face3D.isPointInside(observer3D, 0.0));
         assertTrue(face3D.isPointInFacePlane(observer3D, EPSILON));

         Vertex3DReadOnly expectedLineOfSightStartVertex3D = face3D.getVertices().get(expectedLineOfSightIndices[0]);
         Vertex3DReadOnly expectedLineOfSightEndVertex3D = face3D.getVertices().get(expectedLineOfSightIndices[1]);
         HalfEdge3D expectedLineOfSightStartEdge3D = face3D.getEdge(expectedLineOfSightIndices[0]);
         HalfEdge3D expectedLineOfSightEndEdge3D = face3D.getEdge(expectedLineOfSightIndices[1]).getPrevious();
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[0], expectedLineOfSightStartVertex3D, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[0], expectedLineOfSightStartEdge3D.getOrigin(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[1], expectedLineOfSightEndVertex3D, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[1], expectedLineOfSightEndEdge3D.getDestination(), EPSILON);

         { // Testing Face3DReadOnly.canObserverSeeEdge(...) which is used for the line-of-sight calculation.
            { // Go through the edges that are in the line of sight of the observer and assert that they are visible.
               int visibleEdgeIndex = expectedLineOfSightIndices[0];
               HalfEdge3D visibleEdge = expectedLineOfSightStartEdge3D;

               while (visibleEdge != expectedLineOfSightEndEdge3D.getNext())
               {
                  assertTrue(face3D.canObserverSeeEdge(observer3D, visibleEdgeIndex));
                  assertTrue(face3D.canObserverSeeEdge(observer3D, visibleEdge));
                  visibleEdgeIndex = (visibleEdgeIndex + 1) % face3D.getNumberOfEdges();
                  visibleEdge = visibleEdge.getNext();
               }
            }

            { // Go through the hidden edges.
               int hiddenEdgeIndex = expectedLineOfSightIndices[1];
               HalfEdge3D hiddenEdge = expectedLineOfSightEndEdge3D.getNext();

               while (hiddenEdge != expectedLineOfSightStartEdge3D)
               {
                  assertFalse(face3D.canObserverSeeEdge(observer3D, hiddenEdgeIndex));
                  assertFalse(face3D.canObserverSeeEdge(observer3D, hiddenEdge));
                  hiddenEdgeIndex = (hiddenEdgeIndex + 1) % face3D.getNumberOfEdges();
                  hiddenEdge = hiddenEdge.getNext();
               }
            }
         }

         String errorMessage = "Iteration: " + i;
         assertNotNull(face3D.lineOfSightStart(observer3D), errorMessage);
         EuclidCoreTestTools.assertTuple3DEquals(errorMessage, expectedLineOfSightVertices3D[0], face3D.lineOfSightStart(observer3D).getOrigin(), EPSILON);

         assertNotNull(face3D.lineOfSightEnd(observer3D), errorMessage);
         EuclidCoreTestTools.assertTuple3DEquals(errorMessage, expectedLineOfSightVertices3D[1], face3D.lineOfSightEnd(observer3D).getDestination(), EPSILON);

         List<HalfEdge3D> lineOfSight3D = face3D.lineOfSight(observer3D);
         assertFalse(lineOfSight3D.isEmpty(), errorMessage);
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[0], lineOfSight3D.get(0).getOrigin(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedLineOfSightVertices3D[1], lineOfSight3D.get(lineOfSight3D.size() - 1).getDestination(), EPSILON);
      }
   }

   @Test
   public void testHalfEdgeAreSetProperly() throws Exception
   {
      Random random = new Random(2342);

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

            assertTrue(edge.getPrevious() == prevEdge);
            assertTrue(edge.getNext() == nextEdge);
            assertTrue(edge.getFace() == face);

            assertTrue(prevEdge.getDestination() == edge.getOrigin());
            assertTrue(nextEdge.getOrigin() == edge.getDestination());
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Asserts that the flip feature preserve a proper edge setup.
         int numberOfVertices = 10;
         Face3D face = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random, 1.0, 1.0, numberOfVertices, Axis.Z);
         assertEquals(numberOfVertices, face.getNumberOfEdges());

         face.flip();

         List<HalfEdge3D> edges = face.getEdges();

         for (int edgeIndex = 0; edgeIndex < numberOfVertices; edgeIndex++)
         {
            int previousIndex = EuclidGeometryPolygonTools.previous(edgeIndex, edges.size());
            int nextIndex = EuclidGeometryPolygonTools.next(edgeIndex, edges.size());

            HalfEdge3D prevEdge = edges.get(previousIndex);
            HalfEdge3D nextEdge = edges.get(nextIndex);
            HalfEdge3D edge = edges.get(edgeIndex);

            assertTrue(edge.getPrevious() == prevEdge);
            assertTrue(edge.getNext() == nextEdge);
            assertTrue(edge.getFace() == face);

            assertTrue(prevEdge.getDestination() == edge.getOrigin());
            assertTrue(nextEdge.getOrigin() == edge.getDestination());
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Now the vertices are shuffled so they're not nicely ordered.
         int numberOfVertices = 10;
         Vector3DReadOnly faceNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         List<Point3D> vertices = EuclidPolytopeRandomTools.nextCircleBasedConvexPolygon3D(random, 1.0, 1.0, numberOfVertices, faceNormal);
         Collections.shuffle(vertices, random);
         Face3D face = new Face3D(faceNormal);
         vertices.forEach(vertex -> face.addVertex(new Vertex3D(vertex)));
         assertEquals(numberOfVertices, face.getNumberOfEdges());

         List<HalfEdge3D> edges = face.getEdges();

         for (int edgeIndex = 0; edgeIndex < numberOfVertices; edgeIndex++)
         {
            int previousIndex = EuclidGeometryPolygonTools.previous(edgeIndex, edges.size());
            int nextIndex = EuclidGeometryPolygonTools.next(edgeIndex, edges.size());

            HalfEdge3D prevEdge = edges.get(previousIndex);
            HalfEdge3D nextEdge = edges.get(nextIndex);
            HalfEdge3D edge = edges.get(edgeIndex);

            assertTrue(edge.getPrevious() == prevEdge);
            assertTrue(edge.getNext() == nextEdge);
            assertTrue(edge.getFace() == face);

            assertTrue(prevEdge.getDestination() == edge.getOrigin());
            assertTrue(nextEdge.getOrigin() == edge.getDestination());
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
            face3D.addVertex(new Vertex3D(point));
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
         HalfEdge3D nextEdge = prevEdge.getNext();

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
         vertices2D.forEach(vertex2D -> face.addVertex(new Vertex3D(new Point3D(vertex2D))));

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

   @Test
   void testOrhtogonalProjection() throws Exception
   {
      Random random = new Random(34534);

      for (int i = 0; i < ITERATIONS; i++)
      {// Point to project is directly above the face
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random);

         HalfEdge3D edge = face3D.getEdge(random.nextInt(face3D.getNumberOfEdges()));
         Point3D pointAbove = EuclidShapeRandomTools.nextPoint3DInTriangle(random, face3D.getCentroid(), edge.getOrigin(), edge.getDestination());
         pointAbove.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), face3D.getNormal(), pointAbove);

         Point3D expectedProjection = EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointAbove, face3D.getCentroid(), face3D.getNormal());
         Point3DBasics actualProjection = face3D.orthogonalProjection(pointAbove);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {// Point to project is directly below the face
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random);

         HalfEdge3D edge = face3D.getEdge(random.nextInt(face3D.getNumberOfEdges()));
         Point3D pointBelow = EuclidShapeRandomTools.nextPoint3DInTriangle(random, face3D.getCentroid(), edge.getOrigin(), edge.getDestination());
         pointBelow.scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), face3D.getNormal(), pointBelow);

         Point3D expectedProjection = EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointBelow, face3D.getCentroid(), face3D.getNormal());
         Point3DBasics actualProjection = face3D.orthogonalProjection(pointBelow);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {// Point to project is outside closest to an edge
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random);

         HalfEdge3D edge = face3D.getEdge(random.nextInt(face3D.getNumberOfEdges()));
         Point3D pointOnEdge = new Point3D();
         pointOnEdge.interpolate(edge.getOrigin(), edge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Vector3D towardOutside = new Vector3D();
         towardOutside.cross(face3D.getNormal(), edge.getDirection(true));
         towardOutside.normalize();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), towardOutside, pointOnEdge);
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), face3D.getNormal(), pointOutside);

         Point3D expectedProjection = EuclidGeometryTools.orthogonalProjectionOnLine3D(pointOutside, edge.getOrigin(), edge.getDirection(true));
         Point3DBasics actualProjection = face3D.orthogonalProjection(pointOutside);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {// Point to project is outside closest to a vertex
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random);

         HalfEdge3D edge = face3D.getEdge(random.nextInt(face3D.getNumberOfEdges()));
         HalfEdge3D next = edge.getNext();
         Vertex3D closestVertex = edge.getDestination();

         Vector3D edgeOut = new Vector3D();
         edgeOut.cross(face3D.getNormal(), edge.getDirection(true));
         edgeOut.normalize();
         Vector3D nextOut = new Vector3D();
         nextOut.cross(face3D.getNormal(), next.getDirection(true));
         nextOut.normalize();
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(edgeOut, nextOut, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         towardOutside.normalize();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), nextOut, closestVertex);
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), face3D.getNormal(), pointOutside);

         Point3DReadOnly expectedProjection = closestVertex;
         Point3DBasics actualProjection = face3D.orthogonalProjection(pointOutside);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
      }
   }

   @Test
   void testGetSupportVectorDirectionTo() throws Exception
   {
      Random random = new Random(4589342);
      // We will use the already tested Face3DReadOnly.orthogonalProjection() to test this method.

      for (int i = 0; i < ITERATIONS; i++)
      {// Point to project is directly above the face
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random);

         HalfEdge3D edge = face3D.getEdge(random.nextInt(face3D.getNumberOfEdges()));
         Point3D pointAbove = EuclidShapeRandomTools.nextPoint3DInTriangle(random, face3D.getCentroid(), edge.getOrigin(), edge.getDestination());
         pointAbove.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), face3D.getNormal(), pointAbove);

         Vector3D expectedSupportVector = new Vector3D();
         expectedSupportVector.sub(pointAbove, face3D.orthogonalProjection(pointAbove));
         expectedSupportVector.normalize();
         Vector3DBasics actualSupportVector = face3D.getSupportVectorDirectionTo(pointAbove);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {// Point to project is directly below the face
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random);

         HalfEdge3D edge = face3D.getEdge(random.nextInt(face3D.getNumberOfEdges()));
         Point3D pointBelow = EuclidShapeRandomTools.nextPoint3DInTriangle(random, face3D.getCentroid(), edge.getOrigin(), edge.getDestination());
         pointBelow.scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), face3D.getNormal(), pointBelow);

         Vector3D expectedSupportVector = new Vector3D();
         expectedSupportVector.sub(pointBelow, face3D.orthogonalProjection(pointBelow));
         expectedSupportVector.normalize();
         Vector3DBasics actualSupportVector = face3D.getSupportVectorDirectionTo(pointBelow);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {// Point to project is outside closest to an edge
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random);

         HalfEdge3D edge = face3D.getEdge(random.nextInt(face3D.getNumberOfEdges()));
         Point3D pointOnEdge = new Point3D();
         pointOnEdge.interpolate(edge.getOrigin(), edge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Vector3D towardOutside = new Vector3D();
         towardOutside.cross(face3D.getNormal(), edge.getDirection(true));
         towardOutside.normalize();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), towardOutside, pointOnEdge);
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), face3D.getNormal(), pointOutside);

         Vector3D expectedSupportVector = new Vector3D();
         expectedSupportVector.sub(pointOutside, face3D.orthogonalProjection(pointOutside));
//         expectedSupportVector.normalize();
         Vector3DBasics actualSupportVector = face3D.getSupportVectorDirectionTo(pointOutside);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {// Point to project is outside closest to a vertex
         Face3D face3D = EuclidPolytopeRandomTools.nextCircleBasedFace3D(random);

         HalfEdge3D edge = face3D.getEdge(random.nextInt(face3D.getNumberOfEdges()));
         HalfEdge3D next = edge.getNext();
         Vertex3D closestVertex = edge.getDestination();

         Vector3D edgeOut = new Vector3D();
         edgeOut.cross(face3D.getNormal(), edge.getDirection(true));
         edgeOut.normalize();
         Vector3D nextOut = new Vector3D();
         nextOut.cross(face3D.getNormal(), next.getDirection(true));
         nextOut.normalize();
         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(edgeOut, nextOut, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         towardOutside.normalize();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), nextOut, closestVertex);
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), face3D.getNormal(), pointOutside);

         Vector3D expectedSupportVector = new Vector3D();
         expectedSupportVector.sub(pointOutside, face3D.orthogonalProjection(pointOutside));
//         expectedSupportVector.normalize(); TODO So sometimes it is normalized sometimes not, not sure that is accepetable
         Vector3DBasics actualSupportVector = face3D.getSupportVectorDirectionTo(pointOutside);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
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
