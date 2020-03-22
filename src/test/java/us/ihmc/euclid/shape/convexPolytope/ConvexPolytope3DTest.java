package us.ihmc.euclid.shape.convexPolytope;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.percentageAlongLineSegment3D;

import java.util.*;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3DTroublesomeDatasetLibrary.*;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory.TriangleMesh3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class ConvexPolytope3DTest
{
   private static final int ITERATIONS = EuclidTestConstants.ITERATIONS;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testConstrustingAPolytope()
   {
      Random random = new Random(4533543);

      { // Series of basic queries and assertions.
         ConvexPolytope3D polytope = new ConvexPolytope3D();

         // Testing properties for empty polytope.
         assertEquals(0, polytope.getNumberOfVertices());
         assertEquals(0, polytope.getNumberOfEdges());
         assertEquals(0, polytope.getNumberOfFaces());

         List<Point3D> pointsAdded = new ArrayList<>();

         // Testing properties for single vertex polytope.
         Point3D firstVertex = EuclidCoreRandomTools.nextPoint3D(random);
         polytope.addVertex(firstVertex);
         pointsAdded.add(firstVertex);

         assertEquals(1, polytope.getNumberOfVertices());
         assertEquals(1, polytope.getNumberOfEdges());
         assertEquals(1, polytope.getNumberOfFaces());
         EuclidCoreTestTools.assertTuple3DEquals(firstVertex, polytope.getCentroid(), EPSILON);

         for (int vertexIndex = 0; vertexIndex < pointsAdded.size(); vertexIndex++)
         {
            EuclidCoreTestTools.assertTuple3DEquals(pointsAdded.get(vertexIndex), polytope.getVertex(vertexIndex), EPSILON);
         }

         // Assert that adding the same point twice does not change anything
         polytope.addVertex(firstVertex);
         assertEquals(1, polytope.getNumberOfVertices());
         assertEquals(1, polytope.getNumberOfEdges());
         assertEquals(1, polytope.getNumberOfFaces());

         // Testing properties for single edge polytope.
         Point3D secondVertex = EuclidCoreRandomTools.nextPoint3D(random);
         polytope.addVertex(secondVertex);
         pointsAdded.add(secondVertex);

         assertEquals(2, polytope.getNumberOfVertices());
         assertEquals(2, polytope.getNumberOfEdges());
         assertEquals(1, polytope.getNumberOfFaces());
         EuclidCoreTestTools.assertTuple3DEquals(EuclidGeometryTools.averagePoint3Ds(pointsAdded), polytope.getCentroid(), EPSILON);

         for (int vertexIndex = 0; vertexIndex < pointsAdded.size(); vertexIndex++)
         {
            EuclidCoreTestTools.assertTuple3DEquals(pointsAdded.get(vertexIndex), polytope.getVertex(vertexIndex), EPSILON);
         }

         for (int i = 0; i < ITERATIONS; i++)
         { // Assert that adding a point that is on the edge does not change anything.
            Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, pointsAdded);
            polytope.addVertex(pointInside);

            assertEquals(2, polytope.getNumberOfVertices());
            assertEquals(2, polytope.getNumberOfEdges());
            assertEquals(1, polytope.getNumberOfFaces());
         }

         // Testing properties for single triangle face polytope
         Point3D thirdVertex = EuclidCoreRandomTools.nextPoint3D(random);
         polytope.addVertex(thirdVertex);
         pointsAdded.add(thirdVertex);

         assertEquals(3, polytope.getNumberOfVertices());
         assertEquals(3, polytope.getNumberOfEdges());
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
            Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, pointsAdded);
            polytope.addVertex(pointInside);

            assertEquals(3, polytope.getNumberOfVertices());
            assertEquals(3, polytope.getNumberOfEdges());
            assertEquals(1, polytope.getNumberOfFaces());
         }

         // We finally have an usual polytope: a tetrahedron
         Point3D fourthVertex = EuclidCoreRandomTools.nextPoint3D(random);
         polytope.addVertex(fourthVertex);
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

         for (int edgeIndex = 0; edgeIndex < polytope.getHalfEdges().size(); edgeIndex++)
         {
            HalfEdge3D edge = polytope.getHalfEdge(edgeIndex);
            // Assert that each vertex is unique
            assertTrue(polytope.getHalfEdges().stream().noneMatch(otherEdge -> (otherEdge != edge && otherEdge.epsilonEquals(edge, EPSILON))));
            // Assert that all vertex are from the points added
            assertTrue(pointsAdded.stream().anyMatch(point -> point.epsilonEquals(edge.getOrigin(), EPSILON)));
            assertTrue(pointsAdded.stream().anyMatch(point -> point.epsilonEquals(edge.getDestination(), EPSILON)));

            EuclidCoreTestTools.assertTuple3DEquals(edge.getOrigin(), edge.getTwin().getDestination(), EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(edge.getDestination(), edge.getTwin().getOrigin(), EPSILON);

            EuclidCoreTestTools.assertTuple3DEquals(edge.getOrigin(), edge.getPrevious().getDestination(), EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(edge.getDestination(), edge.getNext().getOrigin(), EPSILON);

            EuclidCoreTestTools.assertTuple3DEquals(edge.getOrigin(), edge.getPrevious().getTwin().getOrigin(), EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(edge.getDestination(), edge.getNext().getTwin().getDestination(), EPSILON);

            EuclidCoreTestTools.assertTuple3DEquals(edge.getOrigin(), edge.getTwin().getNext().getOrigin(), EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(edge.getDestination(), edge.getTwin().getPrevious().getDestination(), EPSILON);

            assertTrue(edge.getTwin().getTwin() == edge);
            assertTrue(edge.getNext().getPrevious() == edge);
            assertTrue(edge.getPrevious().getNext() == edge);
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
            Point3D pointInside = EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, firstVertex, secondVertex, thirdVertex, fourthVertex);

            assertTrue(polytope.isPointInside(pointInside, EPSILON));

            polytope.addVertex(pointInside);

            assertEquals(4, polytope.getNumberOfVertices());
            assertEquals(6, polytope.getNumberOfEdges());
            assertEquals(4, polytope.getNumberOfFaces());
         }
      }
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   void testConstructingTetrahedron() throws Exception
   {
      Random random = new Random(34636);

      { // Trivial tests.
         Point3D top = new Point3D(0.0, 0.0, 1.0);
         Point3D bottomP0 = new Point3D(-0.5, -0.5, 0.0);
         Point3D bottomP1 = new Point3D(0.5, -0.5, 0.0);
         Point3D bottomP2 = new Point3D(0.0, 0.5, 0.0);

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         convexPolytope3D.addVertex(bottomP0);
         convexPolytope3D.addVertex(bottomP1);
         convexPolytope3D.addVertex(bottomP2);
         convexPolytope3D.addVertex(top);

         assertTrue(convexPolytope3D.getVertices().contains(top));
         assertTrue(convexPolytope3D.getVertices().contains(bottomP0));
         assertTrue(convexPolytope3D.getVertices().contains(bottomP1));
         assertTrue(convexPolytope3D.getVertices().contains(bottomP2));

         assertEquals(4, convexPolytope3D.getNumberOfVertices());
         assertEquals(6, convexPolytope3D.getNumberOfEdges());
         assertEquals(4, convexPolytope3D.getNumberOfFaces());

         Vector3D bottomNormal = new Vector3D(0.0, 0.0, -1.0);
         Vector3D sideYMinusNormal = EuclidGeometryTools.normal3DFromThreePoint3Ds(top, bottomP0, bottomP1);
         if (sideYMinusNormal.getY() > 0.0)
            sideYMinusNormal.negate();
         Vector3D sideXMinusNormal = EuclidGeometryTools.normal3DFromThreePoint3Ds(top, bottomP0, bottomP2);
         if (sideXMinusNormal.getX() > 0.0)
            sideXMinusNormal.negate();
         Vector3D sideXPlusNormal = new Vector3D(sideXMinusNormal);
         sideXPlusNormal.setX(-sideXPlusNormal.getX());

         for (int faceIndex = 0; faceIndex < 4; faceIndex++)
         {
            Face3D face = convexPolytope3D.getFace(faceIndex);
            Vector3DBasics normal = face.getNormal();

            if (normal.epsilonEquals(bottomNormal, EPSILON))
            {
               assertTrue(convexPolytope3D.getVertices().contains(bottomP0));
               assertTrue(convexPolytope3D.getVertices().contains(bottomP1));
               assertTrue(convexPolytope3D.getVertices().contains(bottomP2));
            }
            else if (normal.epsilonEquals(sideYMinusNormal, EPSILON))
            {
               assertTrue(convexPolytope3D.getVertices().contains(top));
               assertTrue(convexPolytope3D.getVertices().contains(bottomP0));
               assertTrue(convexPolytope3D.getVertices().contains(bottomP1));
            }
            else if (normal.epsilonEquals(sideXMinusNormal, EPSILON))
            {
               assertTrue(convexPolytope3D.getVertices().contains(top));
               assertTrue(convexPolytope3D.getVertices().contains(bottomP0));
               assertTrue(convexPolytope3D.getVertices().contains(bottomP2));
            }
            else if (normal.epsilonEquals(sideXPlusNormal, EPSILON))
            {
               assertTrue(convexPolytope3D.getVertices().contains(top));
               assertTrue(convexPolytope3D.getVertices().contains(bottomP0));
               assertTrue(convexPolytope3D.getVertices().contains(bottomP1));
            }
            else
            {
               fail("Unexpected face normal: " + normal);
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         /*
          * We start from a tetrahedron, then add a vertex that is outside but lies on the support line of
          * one the edges. So the result should be a bigger tetrahedron.
          */

         List<Point3D> vertices = new ArrayList<>();
         vertices.add(EuclidCoreRandomTools.nextPoint3D(random));
         vertices.add(EuclidCoreRandomTools.nextPoint3D(random));
         vertices.add(EuclidCoreRandomTools.nextPoint3D(random));
         vertices.add(EuclidCoreRandomTools.nextPoint3D(random));

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         vertices.forEach(vertex -> convexPolytope3D.addVertex(vertex));

         assertEquals(4, convexPolytope3D.getNumberOfVertices());
         assertEquals(6, convexPolytope3D.getNumberOfEdges());
         assertEquals(4, convexPolytope3D.getNumberOfFaces());

         HalfEdge3D edge = convexPolytope3D.getHalfEdge(random.nextInt(convexPolytope3D.getNumberOfEdges()));
         Point3DBasics newVertex = edge.pointOnLineGivenPercentage(1.0 + random.nextDouble());
         Vertex3D expectedVertexRemoved = edge.getDestination();

         convexPolytope3D.addVertex(newVertex);

         String errorMessage = "Iteration: " + i;
         assertEquals(4, convexPolytope3D.getNumberOfVertices(), errorMessage);
         assertEquals(6, convexPolytope3D.getNumberOfEdges(), errorMessage);
         assertEquals(4, convexPolytope3D.getNumberOfFaces(), errorMessage);
         assertTrue(convexPolytope3D.getVertices().stream().anyMatch(vertex -> vertex.epsilonEquals(newVertex, EPSILON)), errorMessage);
         assertFalse(convexPolytope3D.getVertices().contains(expectedVertexRemoved), errorMessage);
      }
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   void testConstructingCube() throws Exception
   {
      Random random = new Random(5464566);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial test with a unit-length side cube
         Point3D bottomP0 = new Point3D(-0.5, -0.5, 0.0);
         Point3D bottomP1 = new Point3D(-0.5, 0.5, 0.0);
         Point3D bottomP2 = new Point3D(0.5, 0.5, 0.0);
         Point3D bottomP3 = new Point3D(0.5, -0.5, 0.0);

         Point3D topP0 = new Point3D(-0.5, -0.5, 1.0);
         Point3D topP1 = new Point3D(-0.5, 0.5, 1.0);
         Point3D topP2 = new Point3D(0.5, 0.5, 1.0);
         Point3D topP3 = new Point3D(0.5, -0.5, 1.0);

         Vector3D bottomNormal = new Vector3D(0, 0, -1);
         Vector3D topNormal = new Vector3D(0, 0, 1);
         Vector3D xPlusSideNormal = new Vector3D(1, 0, 0);
         Vector3D xMinusSideNormal = new Vector3D(-1, 0, 0);
         Vector3D yPlusSideNormal = new Vector3D(0, 1, 0);
         Vector3D yMinusSideNormal = new Vector3D(0, -1, 0);

         Point3D bottomCenter = new Point3D(0, 0, 0);
         Point3D topCenter = new Point3D(0, 0, 1);
         Point3D xPlusSideCenter = new Point3D(0.5, 0, 0.5);
         Point3D xMinusSideCenter = new Point3D(-0.5, 0, 0.5);
         Point3D yPlusSideCenter = new Point3D(0, 0.5, 0.5);
         Point3D yMinusSideCenter = new Point3D(0, -0.5, 0.5);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Arrays.asList(bottomP0,
                       bottomP1,
                       bottomP2,
                       bottomP3,
                       topP0,
                       topP1,
                       topP2,
                       topP3,
                       bottomNormal,
                       topNormal,
                       xPlusSideNormal,
                       xMinusSideNormal,
                       yPlusSideNormal,
                       yMinusSideNormal,
                       bottomCenter,
                       topCenter,
                       xPlusSideCenter,
                       xMinusSideCenter,
                       yPlusSideCenter,
                       yMinusSideCenter)
               .forEach(o -> o.applyTransform(transform));

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         convexPolytope3D.addVertex(bottomP0);
         convexPolytope3D.addVertex(bottomP1);
         convexPolytope3D.addVertex(bottomP2);
         convexPolytope3D.addVertex(bottomP3);

         convexPolytope3D.addVertex(topP0);
         convexPolytope3D.addVertex(topP1);
         convexPolytope3D.addVertex(topP2);
         convexPolytope3D.addVertex(topP3);

         assertEquals(6, convexPolytope3D.getNumberOfFaces());
         assertEquals(8, convexPolytope3D.getNumberOfVertices());
         assertEquals(12, convexPolytope3D.getNumberOfEdges());

         List<Face3D> allFaces = convexPolytope3D.getFaces();
         allFaces.forEach(face -> assertEquals(4, face.getNumberOfEdges()));

         Face3D bottomFace = allFaces.stream().filter(face -> face.getNormal().epsilonEquals(bottomNormal, EPSILON)).findFirst().get();
         Face3D topFace = allFaces.stream().filter(face -> face.getNormal().epsilonEquals(topNormal, EPSILON)).findFirst().get();
         Face3D xPlusSideFace = allFaces.stream().filter(face -> face.getNormal().epsilonEquals(xPlusSideNormal, EPSILON)).findFirst().get();
         Face3D xMinusSideFace = allFaces.stream().filter(face -> face.getNormal().epsilonEquals(xMinusSideNormal, EPSILON)).findFirst().get();
         Face3D yPlusSideFace = allFaces.stream().filter(face -> face.getNormal().epsilonEquals(yPlusSideNormal, EPSILON)).findFirst().get();
         Face3D yMinusSideFace = allFaces.stream().filter(face -> face.getNormal().epsilonEquals(yMinusSideNormal, EPSILON)).findFirst().get();

         EuclidCoreTestTools.assertTuple3DEquals(bottomCenter, bottomFace.getCentroid(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(topCenter, topFace.getCentroid(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(xPlusSideCenter, xPlusSideFace.getCentroid(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(xMinusSideCenter, xMinusSideFace.getCentroid(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(yPlusSideCenter, yPlusSideFace.getCentroid(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(yMinusSideCenter, yMinusSideFace.getCentroid(), EPSILON);

         assertTrue(bottomFace.getVertices().containsAll(Arrays.asList(bottomP0, bottomP1, bottomP2, bottomP3)));
         assertTrue(topFace.getVertices().containsAll(Arrays.asList(topP0, topP1, topP2, topP3)));
         assertTrue(xPlusSideFace.getVertices().containsAll(Arrays.asList(topP2, topP3, bottomP2, bottomP3)));
         assertTrue(xMinusSideFace.getVertices().containsAll(Arrays.asList(topP0, topP1, bottomP0, bottomP1)));
         assertTrue(yPlusSideFace.getVertices().containsAll(Arrays.asList(topP1, topP2, bottomP1, bottomP2)));
         assertTrue(yMinusSideFace.getVertices().containsAll(Arrays.asList(topP0, topP3, bottomP0, bottomP3)));

         convexPolytope3D.getHalfEdges().forEach(edge -> assertNotNull(edge.getTwin()));
         convexPolytope3D.getHalfEdges().forEach(edge -> assertNotNull(edge.getNext()));
         convexPolytope3D.getHalfEdges().forEach(edge -> assertNotNull(edge.getPrevious()));

         convexPolytope3D.getVertices().forEach(vertex -> assertEquals(3, vertex.getNumberOfAssociatedEdges()));
      }
   }

   @Test
   void testConstructingIcosahedron() throws Exception
   {
      Random random = new Random(23423);

      TriangleMesh3D icosahedron = IcoSphereFactory.newIcoSphere(0);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         List<Point3D> shuffledVertices = new ArrayList<>(icosahedron.getVertices());
         Collections.shuffle(shuffledVertices, random);

         shuffledVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex));

         // https://en.wikipedia.org/wiki/Icosahedron
         assertEquals(12, convexPolytope3D.getNumberOfVertices());
         assertEquals(30, convexPolytope3D.getNumberOfEdges());
         assertEquals(20, convexPolytope3D.getNumberOfFaces());

         for (Vertex3DReadOnly vertex : convexPolytope3D.getVertices())
         {
            assertTrue(icosahedron.getVertices().stream().anyMatch(p -> p.epsilonEquals(vertex, EPSILON)));
         }

         for (Face3D face : convexPolytope3D.getFaces())
         {
            assertEquals(3, face.getNumberOfEdges());

            Vector3D normalDirectionGuess = new Vector3D();
            normalDirectionGuess.sub(face.getCentroid(), convexPolytope3D.getCentroid());
            assertTrue(normalDirectionGuess.dot(face.getNormal()) > 0.0);

            Vertex3D a = face.getVertex(0);
            Vertex3D b = face.getVertex(1);
            Vertex3D c = face.getVertex(2);
            assertTrue(icosahedron.getAllTriangles().stream().anyMatch(triangle -> triangle.geometricallyEquals(a, b, c, EPSILON)));
         }

         for (HalfEdge3D edge : convexPolytope3D.getHalfEdges())
         {
            assertNotNull(edge.getTwin());
            Vertex3D a0 = edge.getOrigin();
            Vertex3D b0 = edge.getDestination();
            Vertex3D a1 = edge.getTwin().getDestination();
            Vertex3D b1 = edge.getTwin().getOrigin();

            assertTrue(a0 == a1);
            assertTrue(b0 == b1);

            assertTrue(edge.getOrigin().getAssociatedEdges().contains(edge));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Now we transform the icosahedron
         icosahedron = IcoSphereFactory.newIcoSphere(0);
         icosahedron.applyTransform(EuclidCoreRandomTools.nextRigidBodyTransform(random));

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         List<Point3D> shuffledVertices = new ArrayList<>(icosahedron.getVertices());
         Collections.shuffle(shuffledVertices, random);

         shuffledVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex));

         // https://en.wikipedia.org/wiki/Icosahedron
         assertEquals(12, convexPolytope3D.getNumberOfVertices());
         assertEquals(30, convexPolytope3D.getNumberOfEdges());
         assertEquals(20, convexPolytope3D.getNumberOfFaces());

         for (Vertex3DReadOnly vertex : convexPolytope3D.getVertices())
         {
            assertTrue(icosahedron.getVertices().stream().anyMatch(p -> p.epsilonEquals(vertex, EPSILON)));
         }

         for (Face3D face : convexPolytope3D.getFaces())
         {
            assertEquals(3, face.getNumberOfEdges());

            Vector3D normalDirectionGuess = new Vector3D();
            normalDirectionGuess.sub(face.getCentroid(), convexPolytope3D.getCentroid());
            assertTrue(normalDirectionGuess.dot(face.getNormal()) > 0.0);

            Vertex3D a = face.getVertex(0);
            Vertex3D b = face.getVertex(1);
            Vertex3D c = face.getVertex(2);
            assertTrue(icosahedron.getAllTriangles().stream().anyMatch(triangle -> triangle.geometricallyEquals(a, b, c, EPSILON)));
         }

         for (HalfEdge3D edge : convexPolytope3D.getHalfEdges())
         {
            assertNotNull(edge.getTwin());
            Vertex3D a0 = edge.getOrigin();
            Vertex3D b0 = edge.getDestination();
            Vertex3D a1 = edge.getTwin().getDestination();
            Vertex3D b1 = edge.getTwin().getOrigin();

            assertTrue(a0 == a1);
            assertTrue(b0 == b1);

            assertTrue(edge.getOrigin().getAssociatedEdges().contains(edge));
         }
      }
   }

   @Test
   void testConstructingIcosphere() throws Exception
   {
      Random random = new Random(23423);

      for (int i = 0; i < ITERATIONS / 10; i++)
      { // Now we transform the ico-sphere
         TriangleMesh3D icoSphere = IcoSphereFactory.newIcoSphere(random.nextInt(2) + 1);
         icoSphere.applyTransform(EuclidCoreRandomTools.nextRigidBodyTransform(random));

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         List<Point3D> shuffledVertices = new ArrayList<>(icoSphere.getVertices());
         Collections.shuffle(shuffledVertices, random);

         shuffledVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex));

         // https://en.wikipedia.org/wiki/Icosahedron
         assertEquals(icoSphere.getNumberOfVertices(), convexPolytope3D.getNumberOfVertices());
         assertEquals(icoSphere.getNumberOfTriangles() * 3 / 2, convexPolytope3D.getNumberOfEdges());
         assertEquals(icoSphere.getNumberOfTriangles(), convexPolytope3D.getNumberOfFaces());

         for (Vertex3DReadOnly vertex : convexPolytope3D.getVertices())
         {
            assertTrue(icoSphere.getVertices().stream().anyMatch(p -> p.epsilonEquals(vertex, EPSILON)));
         }

         for (Face3D face : convexPolytope3D.getFaces())
         {
            assertEquals(3, face.getNumberOfEdges());

            Vector3D normalDirectionGuess = new Vector3D();
            normalDirectionGuess.sub(face.getCentroid(), convexPolytope3D.getCentroid());
            assertTrue(normalDirectionGuess.dot(face.getNormal()) > 0.0);

            Vertex3D a = face.getVertex(0);
            Vertex3D b = face.getVertex(1);
            Vertex3D c = face.getVertex(2);
            assertTrue(icoSphere.getAllTriangles().stream().anyMatch(triangle -> triangle.geometricallyEquals(a, b, c, EPSILON)));
         }

         for (HalfEdge3D edge : convexPolytope3D.getHalfEdges())
         {
            assertNotNull(edge.getTwin());
            Vertex3D a0 = edge.getOrigin();
            Vertex3D b0 = edge.getDestination();
            Vertex3D a1 = edge.getTwin().getDestination();
            Vertex3D b1 = edge.getTwin().getOrigin();

            assertTrue(a0 == a1);
            assertTrue(b0 == b1);

            assertTrue(edge.getOrigin().getAssociatedEdges().contains(edge));
         }
      }
   }

   @Test
   void testConstructingCone() throws Exception
   {
      Random random = new Random(435);

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing that face expansion to include more than 3 vertices on 1 face is working properly.
         Point3D top = new Point3D(0.0, 0.0, 1.0);

         List<Point3D> bottom = new ArrayList<>();

         int bottomSize = 10;
         double deltaTheta = 2.0 * Math.PI / bottomSize;

         for (double theta = 0.0; theta < 2.0 * Math.PI; theta += deltaTheta)
         {
            bottom.add(new Point3D(EuclidCoreTools.cos(theta), EuclidCoreTools.sin(theta), 0.0));
         }

         assertEquals(bottomSize, bottom.size());

         List<Point3D> allVertices = new ArrayList<>(bottom);
         allVertices.add(top);
         Collections.shuffle(allVertices, random);

         Vector3D bottomNormal = new Vector3D(0, 0, -1);
         Point3D bottomCentroid = new Point3D(0, 0, 0);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         allVertices.forEach(transform::transform);
         bottomNormal.applyTransform(transform);
         bottomCentroid.applyTransform(transform);

         // Now testing convex polytope class

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         allVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex));

         assertEquals(allVertices.size(), convexPolytope3D.getNumberOfVertices());

         Optional<Face3D> searchResult = convexPolytope3D.getFaces().stream().filter(face -> face.getNumberOfEdges() == bottomSize).findFirst();

         assertTrue(searchResult.isPresent());
         Face3D bottomFace = searchResult.get();
         EuclidCoreTestTools.assertTuple3DEquals(bottomNormal, bottomFace.getNormal(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(bottomCentroid, bottomFace.getCentroid(), EPSILON);

         for (Vertex3D vertex : bottomFace.getVertices())
            assertTrue(bottom.stream().anyMatch(point -> point.epsilonEquals(vertex, EPSILON)));

         List<Face3D> sideFaces = convexPolytope3D.getFaces().stream().filter(face -> face != bottomFace).collect(Collectors.toList());

         for (Face3D sideFace : sideFaces)
         {
            assertEquals(3, sideFace.getNumberOfEdges());
            assertTrue(sideFace.getVertices().stream().anyMatch(vertex -> vertex.epsilonEquals(top, EPSILON)));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // First generating a truncated cone. Then add the top vertex at the very end and check that all the intermediate vertices have been removed to form a full cone.
         Point3D top = new Point3D(0.0, 0.0, 1.0);

         List<Point3D> bottom = new ArrayList<>();
         List<Point3D> intermediate = new ArrayList<>();

         int bottomSize = 10;
         double deltaTheta = 2.0 * Math.PI / bottomSize;
         double bottomRadius = 1.0;
         double intermediateZ = 0.5;
         double intermediateRadius = bottomRadius * (top.getZ() - intermediateZ) / top.getZ(); // Thales' theorem

         for (double theta = 0.0; theta < 2.0 * Math.PI; theta += deltaTheta)
         {
            bottom.add(new Point3D(bottomRadius * EuclidCoreTools.cos(theta), bottomRadius * EuclidCoreTools.sin(theta), 0.0));
            intermediate.add(new Point3D(intermediateRadius * EuclidCoreTools.cos(theta), intermediateRadius * EuclidCoreTools.sin(theta), intermediateZ));
         }

         assertEquals(bottomSize, bottom.size());

         List<Point3D> truncatedConeVertices = new ArrayList<>();
         truncatedConeVertices.addAll(bottom);
         truncatedConeVertices.addAll(intermediate);
         Collections.shuffle(intermediate, random);

         Vector3D bottomNormal = new Vector3D(0, 0, -1);
         Point3D bottomCentroid = new Point3D(0, 0, 0);

         Vector3D intermediateNormal = new Vector3D(0, 0, 1);
         Point3D intermediateCentroid = new Point3D(0, 0, intermediateZ);

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         truncatedConeVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex));

         assertEquals(truncatedConeVertices.size(), convexPolytope3D.getNumberOfVertices());

         Optional<Face3D> searchResult = convexPolytope3D.getFaces().stream().filter(face -> face.getNumberOfEdges() == bottomSize)
                                                         .filter(face -> EuclidCoreTools.epsilonEquals(0.0, face.getVertex(0).getZ(), EPSILON)).findFirst();
         assertTrue(searchResult.isPresent());
         Face3D bottomFace = searchResult.get();
         EuclidCoreTestTools.assertTuple3DEquals(bottomNormal, bottomFace.getNormal(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(bottomCentroid, bottomFace.getCentroid(), EPSILON);

         for (Vertex3D vertex : bottomFace.getVertices())
            assertTrue(bottom.stream().anyMatch(point -> point.epsilonEquals(vertex, EPSILON)));

         searchResult = convexPolytope3D.getFaces().stream().filter(face -> face.getNumberOfEdges() == bottomSize)
                                        .filter(face -> EuclidCoreTools.epsilonEquals(intermediateZ, face.getVertex(0).getZ(), EPSILON)).findFirst();
         assertTrue(searchResult.isPresent());
         Face3D intermediateFace = searchResult.get();
         EuclidCoreTestTools.assertTuple3DEquals(intermediateNormal, intermediateFace.getNormal(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(intermediateCentroid, intermediateFace.getCentroid(), EPSILON);

         for (Vertex3D vertex : intermediateFace.getVertices())
            assertTrue(intermediate.stream().anyMatch(point -> point.epsilonEquals(vertex, EPSILON)));

         List<Face3D> sideFaces = convexPolytope3D.getFaces().stream().filter(face -> face != bottomFace).filter(face -> face != intermediateFace)
                                                  .collect(Collectors.toList());

         for (Face3D sideFace : sideFaces)
         {
            assertEquals(4, sideFace.getNumberOfEdges());
            assertEquals(2, sideFace.getVertices().stream().filter(v -> EuclidCoreTools.epsilonEquals(0.0, v.getZ(), EPSILON)).count());
            assertEquals(2, sideFace.getVertices().stream().filter(v -> EuclidCoreTools.epsilonEquals(intermediateZ, v.getZ(), EPSILON)).count());
         }

         // Now let's add the top and check that the intermediate face/vertices/edges are all gone.
         convexPolytope3D.addVertex(top);

         assertEquals(bottomSize + 1, convexPolytope3D.getNumberOfVertices());
      }
   }

   @Test
   void testConstructingCylinder() throws Exception
   {
      Random random = new Random(34);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double topZ = 1.0;

         List<Point3D> bottom = new ArrayList<>();

         int bottomSize = 10;
         double deltaTheta = 2.0 * Math.PI / bottomSize;

         for (double theta = 0.0; theta < 2.0 * Math.PI; theta += deltaTheta)
         {
            bottom.add(new Point3D(EuclidCoreTools.cos(theta), EuclidCoreTools.sin(theta), 0.0));
         }

         assertEquals(bottomSize, bottom.size());

         List<Point3D> top = bottom.stream().map(Point3D::new).peek(p -> p.setZ(topZ)).collect(Collectors.toList());

         List<Point3D> allVertices = new ArrayList<>();
         allVertices.addAll(top);
         allVertices.addAll(bottom);
         Collections.shuffle(allVertices, random);

         Vector3D bottomNormal = new Vector3D(0.0, 0.0, -1.0);
         Point3D bottomCentroid = new Point3D(0.0, 0.0, 0.0);

         Vector3D topNormal = new Vector3D(0.0, 0.0, 1.0);
         Point3D topCentroid = new Point3D(0.0, 0.0, topZ);

         double extensionZ = 0.3;
         Point3D aboveTop = new Point3D(0.0, 0.0, topZ + extensionZ);
         Point3D belowBottom = new Point3D(0.0, 0.0, -extensionZ);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         allVertices.forEach(transform::transform);
         Arrays.asList(bottomNormal, bottomCentroid, topNormal, topCentroid, aboveTop, belowBottom).forEach(o -> o.applyTransform(transform));

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         allVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex));

         assertEquals(allVertices.size(), convexPolytope3D.getNumberOfVertices());
         assertEquals(bottomSize + 2, convexPolytope3D.getNumberOfFaces());

         Optional<Face3D> searchResult = convexPolytope3D.getFaces().stream().filter(face -> face.getNumberOfEdges() == bottomSize)
                                                         .filter(face -> face.getCentroid().epsilonEquals(bottomCentroid, EPSILON)).findFirst();
         assertTrue(searchResult.isPresent());
         Face3D bottomFace = searchResult.get();
         EuclidCoreTestTools.assertTuple3DEquals(bottomNormal, bottomFace.getNormal(), EPSILON);

         for (Vertex3D vertex : bottomFace.getVertices())
            assertTrue(bottom.stream().anyMatch(point -> point.epsilonEquals(vertex, EPSILON)));

         searchResult = convexPolytope3D.getFaces().stream().filter(face -> face.getNumberOfEdges() == bottomSize)
                                        .filter(face -> face.getCentroid().epsilonEquals(topCentroid, EPSILON)).findFirst();
         assertTrue(searchResult.isPresent());
         Face3D topFace = searchResult.get();
         EuclidCoreTestTools.assertTuple3DEquals(topNormal, topFace.getNormal(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(topCentroid, topFace.getCentroid(), EPSILON);

         for (Vertex3D vertex : topFace.getVertices())
            assertTrue(top.stream().anyMatch(point -> point.epsilonEquals(vertex, EPSILON)));

         List<Face3D> sideFaces = convexPolytope3D.getFaces().stream().filter(face -> face != bottomFace).filter(face -> face != topFace)
                                                  .collect(Collectors.toList());

         for (Face3D sideFace : sideFaces)
         {
            assertEquals(4, sideFace.getNumberOfEdges());
            List<Vertex3D> vertices = sideFace.getVertices();
            assertEquals(2,
                         vertices.stream()
                                 .filter(v -> EuclidCoreTools.epsilonEquals(0.0, percentageAlongLineSegment3D(v, bottomCentroid, topCentroid), EPSILON))
                                 .count());
            assertEquals(2,
                         vertices.stream()
                                 .filter(v -> EuclidCoreTools.epsilonEquals(1.0, percentageAlongLineSegment3D(v, bottomCentroid, topCentroid), EPSILON))
                                 .count());
         }

         // Let's tweak the cylinder to be extended by a cone at the top:
         convexPolytope3D.addVertex(aboveTop);

         assertEquals(allVertices.size() + 1, convexPolytope3D.getNumberOfVertices());
         assertEquals(2 * bottomSize + 1, convexPolytope3D.getNumberOfFaces());
         assertTrue(convexPolytope3D.getFaces().contains(bottomFace));
         assertFalse(convexPolytope3D.getFaces().contains(topFace));
         assertTrue(convexPolytope3D.getFaces().containsAll(sideFaces));

         List<Face3D> topConeFaces = convexPolytope3D.getFaces().stream().filter(face -> face != bottomFace && !sideFaces.contains(face))
                                                     .collect(Collectors.toList());

         for (Face3D topConeFace : topConeFaces)
         {
            assertEquals(3, topConeFace.getNumberOfEdges());
            List<Vertex3D> vertices = topConeFace.getVertices();
            assertEquals(1, vertices.stream().filter(v -> aboveTop.epsilonEquals(v, EPSILON)).count());
            assertEquals(2,
                         vertices.stream().filter(v -> EuclidCoreTools.epsilonEquals(0.0, percentageAlongLineSegment3D(v, topCentroid, aboveTop), EPSILON))
                                 .count());
         }

         for (Face3D sideFace : sideFaces)
         {
            assertEquals(4, sideFace.getNumberOfEdges());
            List<Vertex3D> vertices = sideFace.getVertices();
            assertEquals(2,
                         vertices.stream()
                                 .filter(v -> EuclidCoreTools.epsilonEquals(0.0, percentageAlongLineSegment3D(v, bottomCentroid, topCentroid), EPSILON))
                                 .count());
            assertEquals(2,
                         vertices.stream()
                                 .filter(v -> EuclidCoreTools.epsilonEquals(1.0, percentageAlongLineSegment3D(v, bottomCentroid, topCentroid), EPSILON))
                                 .count());
         }

         // Let's tweak the cylinder to be extended by a cone at the bottom:
         convexPolytope3D.addVertex(belowBottom);

         assertEquals(allVertices.size() + 2, convexPolytope3D.getNumberOfVertices());
         assertEquals(3 * bottomSize, convexPolytope3D.getNumberOfFaces());
         assertFalse(convexPolytope3D.getFaces().contains(bottomFace));
         assertFalse(convexPolytope3D.getFaces().contains(topFace));
         assertTrue(convexPolytope3D.getFaces().containsAll(topConeFaces));
         assertTrue(convexPolytope3D.getFaces().containsAll(sideFaces));

         List<Face3D> bottomConeFaces = convexPolytope3D.getFaces().stream().filter(face -> !topConeFaces.contains(face) && !sideFaces.contains(face))
                                                        .collect(Collectors.toList());

         for (Face3D bottomConeFace : bottomConeFaces)
         {
            assertEquals(3, bottomConeFace.getNumberOfEdges());
            List<Vertex3D> vertices = bottomConeFace.getVertices();
            assertEquals(1, vertices.stream().filter(v -> belowBottom.epsilonEquals(v, EPSILON)).count());
            assertEquals(2,
                         vertices.stream()
                                 .filter(v -> EuclidCoreTools.epsilonEquals(0.0, percentageAlongLineSegment3D(v, bottomCentroid, belowBottom), EPSILON))
                                 .count());
         }

         convexPolytope3D.getHalfEdges().forEach(edge -> assertNotNull(edge.getTwin()));
      }
   }

   @Test
   void testGetClosestFace() throws Exception
   {
      Random random = new Random(34656);

      { // Testing with a tetrahedron
         Point3D top = new Point3D(0.0, 0.0, 1.0);
         Point3D bottomP0 = new Point3D(-0.5, -0.5, 0.0);
         Point3D bottomP1 = new Point3D(0.5, -0.5, 0.0);
         Point3D bottomP2 = new Point3D(0.0, 0.5, 0.0);

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         convexPolytope3D.addVertex(bottomP0);
         convexPolytope3D.addVertex(bottomP1);
         convexPolytope3D.addVertex(bottomP2);
         convexPolytope3D.addVertex(top);

         for (int i = 0; i < ITERATIONS; i++)
         {
            for (int faceIndex = 0; faceIndex < 4; faceIndex++)
            {
               Face3D face = convexPolytope3D.getFace(faceIndex);
               Point3D pointOnFace = EuclidGeometryRandomTools.nextPoint3DInTriangle(random, face.getVertex(0), face.getVertex(1), face.getVertex(2));
               assertTrue(face == convexPolytope3D.getClosestFace(pointOnFace), "Iteration " + i + ", face index: " + faceIndex);

               Point3D pointOutside = new Point3D();
               pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), face.getNormal(), pointOnFace);
               assertTrue(face == convexPolytope3D.getClosestFace(pointOutside), "Iteration " + i + ", face index: " + faceIndex);
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3D(random);

         { // Query inside: The closest face should be the one with its plane being the closest to the query.
            Point3D point = new Point3D();
            { // We generate the query by picking a face at random, then an edge on this face at random, and we get a random point from the polytope and face centroids and the edge end-points.
               Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
               HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
               point.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random,
                                                                            convexPolytope3D.getCentroid(),
                                                                            face.getCentroid(),
                                                                            edge.getOrigin(),
                                                                            edge.getDestination()));
            }

            Face3D expectedClosestFace = convexPolytope3D.getFaces().stream().sorted((f1,
                                                                                      f2) -> -Double.compare(f1.signedDistanceFromSupportPlane(point),
                                                                                                             f2.signedDistanceFromSupportPlane(point)))
                                                         .findFirst().get();
            Face3D actualClosestFace = convexPolytope3D.getClosestFace(point);
            assertTrue(expectedClosestFace == actualClosestFace, "Iteration " + i);
         }

         {
            Point3D point = new Point3D();
            { // This time, the query is generated to be within the tetrahedron formed by 4 vertices picked at random.
               Point3D a = new Point3D(convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices())));
               Point3D b = new Point3D(convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices())));
               Point3D c = new Point3D(convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices())));
               Point3D d = new Point3D(convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices())));
               Vector3D toInside = new Vector3D();
               for (Point3D vertex : Arrays.asList(a, b, c, d))
               { // We pull the vertices slightly towards the centroid to avoid edge-cases due to the construction epsilon.
                  toInside.sub(convexPolytope3D.getCentroid(), vertex);
                  toInside.scale(convexPolytope3D.getConstructionEpsilon() / toInside.length());
                  vertex.add(toInside);
               }
               point.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, a, b, c, d));
            }

            Face3D expectedClosestFace = convexPolytope3D.getFaces().stream().sorted((f1,
                                                                                      f2) -> -Double.compare(f1.signedDistanceFromSupportPlane(point),
                                                                                                             f2.signedDistanceFromSupportPlane(point)))
                                                         .findFirst().get();
            Face3D actualClosestFace = convexPolytope3D.getClosestFace(point);
            assertTrue(expectedClosestFace == actualClosestFace, "Iteration " + i);

            // We test with the Face3D.distance(Point3DReadOnly) method
            expectedClosestFace = convexPolytope3D.getFaces().stream().sorted((f1, f2) -> Double.compare(f1.distance(point), f2.distance(point))).findFirst()
                                                  .get();
            assertTrue(expectedClosestFace == actualClosestFace, "Iteration " + i);
         }

         { // Query is outside, hovering over a face.
            Point3D point = new Point3D();
            Face3D expectedClosestFace = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
            { // We generate the query to be first on a face picked at random, and then we offset it towards the outside.
               HalfEdge3D edge = expectedClosestFace.getEdge(random.nextInt(expectedClosestFace.getNumberOfEdges()));
               point.set(EuclidGeometryRandomTools.nextPoint3DInTriangle(random, expectedClosestFace.getCentroid(), edge.getOrigin(), edge.getDestination()));
               point.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), expectedClosestFace.getNormal(), point);
            }

            Face3D actualClosestFace = convexPolytope3D.getClosestFace(point);
            assertTrue(expectedClosestFace == actualClosestFace, "Iteration " + i);
         }

         { // Query is outside, closest to an edge.
            Point3D point = new Point3D();
            HalfEdge3D closestEdge = convexPolytope3D.getHalfEdge(random.nextInt(convexPolytope3D.getNumberOfHalfEdges()));
            {// We pick an edge at random, construct a vector that points outside, translate by the expectedDistance
               Vector3D towardOutside = new Vector3D();
               Vector3DBasics normalA = closestEdge.getFace().getNormal();
               Vector3DBasics normalB = closestEdge.getTwin().getFace().getNormal();
               towardOutside.interpolate(normalA, normalB, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
               towardOutside.normalize();
               point.interpolate(closestEdge.getOrigin(), closestEdge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
               point.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), towardOutside, point);
            }

            Face3D actualClosestFace = convexPolytope3D.getClosestFace(point);
            assertTrue(closestEdge.getFace() == actualClosestFace || closestEdge.getTwin().getFace() == actualClosestFace, "Iteration " + i);
         }

         { // Query is outside, closest to a vertex.
            Point3D point = new Point3D();
            Vertex3D closestVertex = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));
            {// We pick a vertex at random, construct a vector that points outside, translate by the expectedDistance
               Vector3D towardOutside = new Vector3D();
               for (int edgeIndex = 0; edgeIndex < closestVertex.getNumberOfAssociatedEdges(); edgeIndex++)
                  towardOutside.scaleAdd(random.nextDouble(), closestVertex.getAssociatedEdge(edgeIndex).getFace().getNormal(), towardOutside);
               towardOutside.normalize();
               point.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), towardOutside, closestVertex);
            }

            Face3D actualClosestFace = convexPolytope3D.getClosestFace(point);
            assertTrue(actualClosestFace.getVertices().contains(closestVertex), "Iteration " + i);
         }
      }
   }

   @Test
   void testSignedDistance() throws Exception
   {
      Random random = new Random(5434543);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3D(random);

         { // We start with a query that is inside. The returned distance should be the signed distance to the closest face's plane.
            Point3D point = new Point3D();
            { // We generate the query by picking a face at random, then an edge on this face at random, and we get a random point from the polytope and face centroids and the edge end-points.
               Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
               HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
               point.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random,
                                                                            convexPolytope3D.getCentroid(),
                                                                            face.getCentroid(),
                                                                            edge.getOrigin(),
                                                                            edge.getDestination()));
            }

            double expectedDistance = Double.NEGATIVE_INFINITY;

            for (Face3D face : convexPolytope3D.getFaces())
            {
               expectedDistance = Math.max(expectedDistance,
                                           EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(point, face.getCentroid(), face.getNormal()));
            }

            double actualDistance = convexPolytope3D.signedDistance(point);

            assertEquals(expectedDistance, actualDistance, EPSILON, "Iteration " + i);
         }

         { // Query is inside. The returned distance should be the signed distance to the closest face's plane.
            Point3D point = new Point3D();
            { // This time, the query is generated to be within the tetrahedron formed by 4 vertices picked at random.
               Point3DReadOnly a = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));
               Point3DReadOnly b = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));
               Point3DReadOnly c = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));
               Point3DReadOnly d = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));
               point.set(EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random, a, b, c, d));
            }

            double expectedDistance = Double.NEGATIVE_INFINITY;

            for (Face3D face : convexPolytope3D.getFaces())
            {
               expectedDistance = Math.max(expectedDistance,
                                           EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(point, face.getCentroid(), face.getNormal()));
            }

            double actualDistance = convexPolytope3D.signedDistance(point);

            assertEquals(expectedDistance, actualDistance, EPSILON, "Iteration " + i);
         }

         { // Query is outside, hovering over a face.
            double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
            Point3D point = new Point3D();
            { // We generate the query to be first on a face picked at random, and then we offset it towards the outside.
               Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
               HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
               point.set(EuclidGeometryRandomTools.nextPoint3DInTriangle(random, face.getCentroid(), edge.getOrigin(), edge.getDestination()));
               point.scaleAdd(expectedDistance, face.getNormal(), point);
            }

            double actualDistance = convexPolytope3D.signedDistance(point);
            assertEquals(expectedDistance, actualDistance, EPSILON, "Iteration " + i);
         }

         { // Query is outside, closest to an edge.
            double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
            Point3D point = new Point3D();
            {// We pick an edge at random, construct a vector that points outside, translate by the expectedDistance
               HalfEdge3D edge = convexPolytope3D.getHalfEdge(random.nextInt(convexPolytope3D.getNumberOfHalfEdges()));
               Vector3D towardOutside = new Vector3D();
               Vector3DBasics normalA = edge.getFace().getNormal();
               Vector3DBasics normalB = edge.getTwin().getFace().getNormal();
               towardOutside.interpolate(normalA, normalB, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
               towardOutside.normalize();
               point.interpolate(edge.getOrigin(), edge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
               point.scaleAdd(expectedDistance, towardOutside, point);
            }

            double actualDistance = convexPolytope3D.signedDistance(point);
            assertEquals(expectedDistance,
                         actualDistance,
                         EPSILON,
                         "Iteration " + i + ", naive method: " + convexPolytope3D.getClosestFace(point).distance(point));
         }

         { // Query outside, closest to a vertex.
            double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
            Point3D point = new Point3D();
            {// We pick a vertex at random, construct a vector that points outside, translate by the expectedDistance
               Vertex3D vertex = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));
               Vector3D towardOutside = new Vector3D();
               for (int edgeIndex = 0; edgeIndex < vertex.getNumberOfAssociatedEdges(); edgeIndex++)
                  towardOutside.scaleAdd(random.nextDouble(), vertex.getAssociatedEdge(edgeIndex).getFace().getNormal(), towardOutside);
               towardOutside.normalize();
               point.scaleAdd(expectedDistance, towardOutside, vertex);
            }

            double actualDistance = convexPolytope3D.signedDistance(point);
            assertEquals(expectedDistance, actualDistance, EPSILON, "Iteration " + i);
         }
      }
   }

   @Test
   void testApplyTransform() throws Exception
   {
      Random random = new Random(2342);

      for (int i = 0; i < ITERATIONS; i++)
      {
         TriangleMesh3D icosahedron = IcoSphereFactory.newIcoSphere(0);
         ConvexPolytope3D originalPolytope = new ConvexPolytope3D();
         icosahedron.getVertices().forEach(vertex -> originalPolytope.addVertex(vertex));
         ConvexPolytope3D actualPolytope = new ConvexPolytope3D();
         icosahedron.getVertices().forEach(vertex -> actualPolytope.addVertex(vertex));

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         actualPolytope.applyTransform(transform);

         icosahedron.applyTransform(transform);
         ConvexPolytope3D expectedPolytope = new ConvexPolytope3D();
         icosahedron.getVertices().forEach(vertex -> expectedPolytope.addVertex(vertex));

         EuclidShapeTestTools.assertConvexPolytope3DEquals(expectedPolytope, actualPolytope, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPolytope.getCentroid(), actualPolytope.getCentroid(), EPSILON);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expectedPolytope.getBoundingBox(), actualPolytope.getBoundingBox(), EPSILON);

         for (int faceIndex = 0; faceIndex < expectedPolytope.getNumberOfFaces(); faceIndex++)
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedPolytope.getFace(faceIndex).getCentroid(),
                                                    actualPolytope.getFace(faceIndex).getCentroid(),
                                                    EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedPolytope.getFace(faceIndex).getNormal(), actualPolytope.getFace(faceIndex).getNormal(), EPSILON);
            EuclidGeometryTestTools.assertBoundingBox3DEquals(expectedPolytope.getFace(faceIndex).getBoundingBox(),
                                                              actualPolytope.getFace(faceIndex).getBoundingBox(),
                                                              EPSILON);
            assertEquals(expectedPolytope.getFace(faceIndex).getArea(), actualPolytope.getFace(faceIndex).getArea(), EPSILON);
         }

         actualPolytope.applyInverseTransform(transform);

         EuclidShapeTestTools.assertConvexPolytope3DEquals(originalPolytope, actualPolytope, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(originalPolytope.getCentroid(), actualPolytope.getCentroid(), EPSILON);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(originalPolytope.getBoundingBox(), actualPolytope.getBoundingBox(), EPSILON);

         for (int faceIndex = 0; faceIndex < originalPolytope.getNumberOfFaces(); faceIndex++)
         {
            EuclidCoreTestTools.assertTuple3DEquals(originalPolytope.getFace(faceIndex).getCentroid(),
                                                    actualPolytope.getFace(faceIndex).getCentroid(),
                                                    EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(originalPolytope.getFace(faceIndex).getNormal(), actualPolytope.getFace(faceIndex).getNormal(), EPSILON);
            EuclidGeometryTestTools.assertBoundingBox3DEquals(originalPolytope.getFace(faceIndex).getBoundingBox(),
                                                              actualPolytope.getFace(faceIndex).getBoundingBox(),
                                                              EPSILON);
            assertEquals(originalPolytope.getFace(faceIndex).getArea(), actualPolytope.getFace(faceIndex).getArea(), EPSILON);
         }
      }
   }

   @Test
   void testSetConvexPolytope3D() throws Exception
   {
      Random random = new Random(234543);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D originalPolytope = EuclidShapeRandomTools.nextIcoSphereBasedConvexPolytope3D(random);
         ConvexPolytope3D copyPolytope = new ConvexPolytope3D();
         copyPolytope.set(originalPolytope);

         EuclidShapeTestTools.assertConvexPolytope3DEquals(originalPolytope, copyPolytope, EPSILON);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(originalPolytope.getBoundingBox(), copyPolytope.getBoundingBox(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(originalPolytope.getCentroid(), copyPolytope.getCentroid(), EPSILON);
         assertEquals(originalPolytope.getVolume(), copyPolytope.getVolume(), EPSILON);

         for (int faceIndex = 0; faceIndex < originalPolytope.getNumberOfFaces(); faceIndex++)
         {
            Face3D originalFace = originalPolytope.getFace(faceIndex);
            Face3D copyFace = copyPolytope.getFace(faceIndex);

            assertTrue(originalFace != copyFace);

            EuclidCoreTestTools.assertTuple3DEquals(originalFace.getCentroid(), copyFace.getCentroid(), EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(originalFace.getNormal(), copyFace.getNormal(), EPSILON);
            EuclidGeometryTestTools.assertBoundingBox3DEquals(originalFace.getBoundingBox(), copyFace.getBoundingBox(), EPSILON);
            assertEquals(originalFace.getArea(), copyFace.getArea(), EPSILON);
         }

         for (int edgeIndex = 0; edgeIndex < originalPolytope.getHalfEdges().size(); edgeIndex++)
         {
            HalfEdge3D originalEdge = originalPolytope.getHalfEdge(edgeIndex);
            HalfEdge3D copyEdge = copyPolytope.getHalfEdge(edgeIndex);

            HalfEdge3D originalNext = originalEdge.getNext();
            HalfEdge3D originalPrevious = originalEdge.getPrevious();
            HalfEdge3D originalTwin = originalEdge.getTwin();
            Face3D originalFace = originalEdge.getFace();
            HalfEdge3D copyNext = copyEdge.getNext();
            HalfEdge3D copyPrevious = copyEdge.getPrevious();
            HalfEdge3D copyTwin = copyEdge.getTwin();
            Face3D copyFace = copyEdge.getFace();

            assertTrue(originalEdge.getOrigin() != copyEdge.getOrigin());
            assertTrue(originalEdge.getDestination() != copyEdge.getDestination());
            assertTrue(originalEdge != copyEdge);
            assertTrue(originalNext != copyNext);
            assertTrue(originalPrevious != copyPrevious);
            assertTrue(originalTwin != copyTwin);
            assertTrue(originalFace != copyFace);

            EuclidGeometryTestTools.assertLineSegment3DEquals(originalEdge, copyEdge, EPSILON);
            EuclidGeometryTestTools.assertLineSegment3DEquals(originalNext, copyNext, EPSILON);
            EuclidGeometryTestTools.assertLineSegment3DEquals(originalPrevious, copyPrevious, EPSILON);
            EuclidGeometryTestTools.assertLineSegment3DEquals(originalTwin, copyTwin, EPSILON);
            EuclidShapeTestTools.assertFace3DEquals(originalFace, copyFace, EPSILON);
         }

         for (int vertexIndex = 0; vertexIndex < originalPolytope.getNumberOfVertices(); vertexIndex++)
         {
            Vertex3D originalVertex = originalPolytope.getVertex(vertexIndex);
            Vertex3D copyVertex = copyPolytope.getVertex(vertexIndex);

            assertTrue(originalVertex != copyVertex);
            EuclidCoreTestTools.assertTuple3DEquals(originalVertex, copyVertex, EPSILON);

            for (int edgeIndex = 0; edgeIndex < originalVertex.getNumberOfAssociatedEdges(); edgeIndex++)
            {
               HalfEdge3D originalEdge = originalVertex.getAssociatedEdge(edgeIndex);
               HalfEdge3D copyEdge = copyVertex.getAssociatedEdge(edgeIndex);
               assertTrue(originalEdge != copyEdge);
               EuclidGeometryTestTools.assertLineSegment3DEquals(originalEdge, copyEdge, EPSILON);
            }
         }
      }
   }

   @Test
   void testCopyConstructor() throws Exception
   {
      Random random = new Random(9574938);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D originalPolytope = EuclidShapeRandomTools.nextIcoSphereBasedConvexPolytope3D(random);
         ConvexPolytope3D copyPolytope = new ConvexPolytope3D(originalPolytope);

         EuclidShapeTestTools.assertConvexPolytope3DEquals(originalPolytope, copyPolytope, EPSILON);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(originalPolytope.getBoundingBox(), copyPolytope.getBoundingBox(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(originalPolytope.getCentroid(), copyPolytope.getCentroid(), EPSILON);
         assertEquals(originalPolytope.getVolume(), copyPolytope.getVolume(), EPSILON);

         for (int faceIndex = 0; faceIndex < originalPolytope.getNumberOfFaces(); faceIndex++)
         {
            Face3D originalFace = originalPolytope.getFace(faceIndex);
            Face3D copyFace = copyPolytope.getFace(faceIndex);

            assertTrue(originalFace != copyFace);

            EuclidCoreTestTools.assertTuple3DEquals(originalFace.getCentroid(), copyFace.getCentroid(), EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(originalFace.getNormal(), copyFace.getNormal(), EPSILON);
            EuclidGeometryTestTools.assertBoundingBox3DEquals(originalFace.getBoundingBox(), copyFace.getBoundingBox(), EPSILON);
            assertEquals(originalFace.getArea(), copyFace.getArea(), EPSILON);
         }

         for (int edgeIndex = 0; edgeIndex < originalPolytope.getHalfEdges().size(); edgeIndex++)
         {
            HalfEdge3D originalEdge = originalPolytope.getHalfEdge(edgeIndex);
            HalfEdge3D copyEdge = copyPolytope.getHalfEdge(edgeIndex);

            HalfEdge3D originalNext = originalEdge.getNext();
            HalfEdge3D originalPrevious = originalEdge.getPrevious();
            HalfEdge3D originalTwin = originalEdge.getTwin();
            Face3D originalFace = originalEdge.getFace();
            HalfEdge3D copyNext = copyEdge.getNext();
            HalfEdge3D copyPrevious = copyEdge.getPrevious();
            HalfEdge3D copyTwin = copyEdge.getTwin();
            Face3D copyFace = copyEdge.getFace();

            assertTrue(originalEdge.getOrigin() != copyEdge.getOrigin());
            assertTrue(originalEdge.getDestination() != copyEdge.getDestination());
            assertTrue(originalEdge != copyEdge);
            assertTrue(originalNext != copyNext);
            assertTrue(originalPrevious != copyPrevious);
            assertTrue(originalTwin != copyTwin);
            assertTrue(originalFace != copyFace);

            EuclidGeometryTestTools.assertLineSegment3DEquals(originalEdge, copyEdge, EPSILON);
            EuclidGeometryTestTools.assertLineSegment3DEquals(originalNext, copyNext, EPSILON);
            EuclidGeometryTestTools.assertLineSegment3DEquals(originalPrevious, copyPrevious, EPSILON);
            EuclidGeometryTestTools.assertLineSegment3DEquals(originalTwin, copyTwin, EPSILON);
            EuclidShapeTestTools.assertFace3DEquals(originalFace, copyFace, EPSILON);
         }

         for (int vertexIndex = 0; vertexIndex < originalPolytope.getNumberOfVertices(); vertexIndex++)
         {
            Vertex3D originalVertex = originalPolytope.getVertex(vertexIndex);
            Vertex3D copyVertex = copyPolytope.getVertex(vertexIndex);

            assertTrue(originalVertex != copyVertex);
            EuclidCoreTestTools.assertTuple3DEquals(originalVertex, copyVertex, EPSILON);

            for (int edgeIndex = 0; edgeIndex < originalVertex.getNumberOfAssociatedEdges(); edgeIndex++)
            {
               HalfEdge3D originalEdge = originalVertex.getAssociatedEdge(edgeIndex);
               HalfEdge3D copyEdge = copyVertex.getAssociatedEdge(edgeIndex);
               assertTrue(originalEdge != copyEdge);
               EuclidGeometryTestTools.assertLineSegment3DEquals(originalEdge, copyEdge, EPSILON);
            }
         }
      }
   }

   @Test
   void testBoundingBox() throws Exception
   {
      Random random = new Random(435354237);

      for (int i = 0; i < ITERATIONS; i++)
      { // Check that all the vertices are in the bounding box.
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextIcoSphereBasedConvexPolytope3D(random);

         convexPolytope3D.getVertices().forEach(vertex -> assertTrue(convexPolytope3D.getBoundingBox().isInsideInclusive(vertex)));

         // Now we assert that the bounding box is as small as possible
         // Considering the particular approach used to make the BBX in ConvexPolytope3D, we can simply rely on BoundingBox3D.updateToIncludePoint() and use a simple brute-force method.
         BoundingBox3D expectedBoundingBox = new BoundingBox3D();
         expectedBoundingBox.setToNaN();
         convexPolytope3D.getVertices().forEach(vertex -> expectedBoundingBox.updateToIncludePoint(vertex));
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expectedBoundingBox, convexPolytope3D.getBoundingBox(), EPSILON);
      }
   }

   @Test
   void testCentroidAndVolume() throws Exception
   {
      Random random = new Random(45);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial case: tetrahedron, the centroid is located at the average of the vertices.
         Point3D a = EuclidCoreRandomTools.nextPoint3D(random, 5.0); // Top vertex
         Point3D b = EuclidCoreRandomTools.nextPoint3D(random, 5.0); // Base vertex
         Point3D c = EuclidCoreRandomTools.nextPoint3D(random, 5.0); // Base vertex
         Point3D d = EuclidCoreRandomTools.nextPoint3D(random, 5.0); // Base vertex
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Arrays.asList(a, b, c, d).forEach(transform::transform);

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(a, b, c, d));
         Point3D expectedCentroid = EuclidGeometryTools.averagePoint3Ds(Arrays.asList(a, b, c, d));
         EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid, convexPolytope3D.getCentroid(), EPSILON);
         assertEquals(EuclidShapeTools.tetrahedronVolume(a, b, c, d), convexPolytope3D.getVolume(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial case: cylinder, the centroid is located at the average of the vertices.
         double length = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         List<Point3D> cylinderVertices = EuclidPolytopeFactories.newCylinderVertices(length, radius, 50);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         cylinderVertices.forEach(transform::transform);

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(cylinderVertices));
         Point3D expectedCentroid = new Point3D(transform.getTranslation());
         EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid, convexPolytope3D.getCentroid(), EPSILON);
         double expectedVolume = EuclidShapeTools.cylinderVolume(length, radius);
         assertEquals(expectedVolume, convexPolytope3D.getVolume(), 3.0e-3 * expectedVolume);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial case: icosahedron, the centroid is located at the average of the vertices.
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         List<Point3D> icosahedronVertices = EuclidPolytopeFactories.newIcosahedronVertices(radius);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         icosahedronVertices.forEach(transform::transform);

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(icosahedronVertices));

         Point3D expectedCentroid = new Point3D(transform.getTranslation());
         EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid, convexPolytope3D.getCentroid(), EPSILON);
         assertEquals(EuclidShapeTools.icosahedronVolume(EuclidShapeTools.icosahedronEdgeLength(radius)), convexPolytope3D.getVolume(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // For a cone the centroid at 1/4 of the height from the base.
         double height = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         int numberOfDivisions = 50;
         List<Point3D> coneVertices = EuclidPolytopeFactories.newConeVertices(height, radius, numberOfDivisions);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         coneVertices.forEach(transform::transform);
         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(coneVertices));
         // Making sure that the bottom face has "numberOfDivisions" vertices
         assertEquals(1, convexPolytope3D.getFaces().stream().filter(face -> face.getNumberOfEdges() == numberOfDivisions).count());

         Point3D expectedCentroid = new Point3D(0.0, 0.0, height / 4.0);
         expectedCentroid.applyTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid, convexPolytope3D.getCentroid(), EPSILON);
         double expectedVolume = EuclidShapeTools.coneVolume(height, radius);
         assertEquals(expectedVolume, convexPolytope3D.getVolume(), 3.0e-3 * expectedVolume);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // For a pyramid the centroid at 1/4 of the height from the base.
         double height = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double baseLength = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double baseWidth = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         List<Point3D> pyramidVertices = EuclidPolytopeFactories.newPyramidVertices(height, baseLength, baseWidth);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         pyramidVertices.forEach(transform::transform);
         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pyramidVertices));
         // Making sure that the bottom face has 4 vertices
         assertEquals(1, convexPolytope3D.getFaces().stream().filter(face -> face.getNumberOfEdges() == 4).count());

         Point3D expectedCentroid = new Point3D(0.0, 0.0, height / 4.0);
         expectedCentroid.applyTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid, convexPolytope3D.getCentroid(), EPSILON);
         assertEquals(EuclidShapeTools.pyramidVolume(height, baseLength, baseWidth), convexPolytope3D.getVolume(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // We make a composite shape: cylinder + a cone at one of its ends.
         double coneHeight = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double cylinderLength = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         int numberOfDivisions = 50;
         List<Point3D> shapeVertices = EuclidPolytopeFactories.newCylinderVertices(cylinderLength, radius, numberOfDivisions);
         shapeVertices.add(new Point3D(0.0, 0.0, 0.5 * cylinderLength + coneHeight));

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(shapeVertices));

         assertEquals(2 * numberOfDivisions + 1, convexPolytope3D.getNumberOfVertices());

         double coneVolume = EuclidShapeTools.coneVolume(coneHeight, radius);
         double cylinderVolume = EuclidShapeTools.cylinderVolume(cylinderLength, radius);
         Point3D coneCentroid = new Point3D(0.0, 0.0, 0.5 * cylinderLength + coneHeight / 4.0);
         Point3D cylinderCentroid = new Point3D();
         double shapeVolume = coneVolume + cylinderVolume;
         Point3D shapeCentroid = new Point3D();
         shapeCentroid.setAndScale(coneVolume / shapeVolume, coneCentroid);
         shapeCentroid.scaleAdd(cylinderVolume / shapeVolume, cylinderCentroid, shapeCentroid);

         EuclidCoreTestTools.assertTuple3DEquals(shapeCentroid, convexPolytope3D.getCentroid(), EPSILON);
         assertEquals(shapeVolume, convexPolytope3D.getVolume(), 3.0e-3 * shapeVolume);
      }
   }

   @Test
   void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(3409736);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            assertNull(convexPolytope3D.getSupportingVertex(EuclidCoreRandomTools.nextVector3D(random)));
            continue;
         }

         Vertex3DReadOnly expectedSupportVertex, actualSupportVertex;
         Vector3D supportDirection = new Vector3D();

         // Trivial case #1: supportingVector = +X
         supportDirection.set(Axis.X);
         expectedSupportVertex = convexPolytope3D.getVertices().stream().sorted((v1, v2) -> Double.compare(v2.getX(), v1.getX())).findFirst().get();
         actualSupportVertex = convexPolytope3D.getSupportingVertex(supportDirection);
         assertTrue(expectedSupportVertex == actualSupportVertex, "iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex);

         // Trivial case #2: supportingVector = -X
         supportDirection.setAndNegate(Axis.X);
         expectedSupportVertex = convexPolytope3D.getVertices().stream().sorted((v1, v2) -> Double.compare(v1.getX(), v2.getX())).findFirst().get();
         actualSupportVertex = convexPolytope3D.getSupportingVertex(supportDirection);
         assertTrue(expectedSupportVertex == actualSupportVertex, "iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex);

         // Trivial case #1: supportingVector = +Y
         supportDirection.set(Axis.Y);
         expectedSupportVertex = convexPolytope3D.getVertices().stream().sorted((v1, v2) -> Double.compare(v2.getY(), v1.getY())).findFirst().get();
         actualSupportVertex = convexPolytope3D.getSupportingVertex(supportDirection);
         assertTrue(expectedSupportVertex == actualSupportVertex, "iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex);

         // Trivial case #2: supportingVector = -Y
         supportDirection.setAndNegate(Axis.Y);
         expectedSupportVertex = convexPolytope3D.getVertices().stream().sorted((v1, v2) -> Double.compare(v1.getY(), v2.getY())).findFirst().get();
         actualSupportVertex = convexPolytope3D.getSupportingVertex(supportDirection);
         assertTrue(expectedSupportVertex == actualSupportVertex, "iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex);

         // Trivial case #1: supportingVector = +Z
         supportDirection.set(Axis.Z);
         expectedSupportVertex = convexPolytope3D.getVertices().stream().sorted((v1, v2) -> Double.compare(v2.getZ(), v1.getZ())).findFirst().get();
         actualSupportVertex = convexPolytope3D.getSupportingVertex(supportDirection);
         assertTrue(expectedSupportVertex == actualSupportVertex, "iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex);

         // Trivial case #2: supportingVector = -Z
         supportDirection.setAndNegate(Axis.Z);
         expectedSupportVertex = convexPolytope3D.getVertices().stream().sorted((v1, v2) -> Double.compare(v1.getZ(), v2.getZ())).findFirst().get();
         actualSupportVertex = convexPolytope3D.getSupportingVertex(supportDirection);
         assertTrue(expectedSupportVertex == actualSupportVertex, "iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex);

         // We apply a similar method compared to the one to be tested except that here we use brute force:
         supportDirection.set(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0)));
         expectedSupportVertex = convexPolytope3D.getVertices().stream().sorted((v1, v2) -> Double.compare(v2.dot(supportDirection), v1.dot(supportDirection)))
                                                 .findFirst().get();
         actualSupportVertex = convexPolytope3D.getSupportingVertex(supportDirection);
         assertTrue(expectedSupportVertex == actualSupportVertex, "iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex);

         // We create a point that is far away from the polytope in the supportDirection and find the closest vertex to that point.
         supportDirection.set(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0)));
         Point3D pointFarFarAway = new Point3D();
         pointFarFarAway.scaleAdd(10000.0, supportDirection, convexPolytope3D.getCentroid());
         expectedSupportVertex = convexPolytope3D.getVertices().stream()
                                                 .sorted((v1, v2) -> Double.compare(v1.distance(pointFarFarAway), v2.distance(pointFarFarAway))).findFirst()
                                                 .get();
         actualSupportVertex = convexPolytope3D.getSupportingVertex(supportDirection);
         assertTrue(expectedSupportVertex == actualSupportVertex, "iteration #" + i + " expected:\n" + expectedSupportVertex + "was:\n" + actualSupportVertex);
      }
   }

   @Test
   void testOrthogonalProjection() throws Exception
   {
      Random random = new Random(34535);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point directly above a face
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            assertNull(convexPolytope3D.orthogonalProjectionCopy(EuclidCoreRandomTools.nextPoint3D(random)));
            continue;
         }

         Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
         HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
         Point3D pointOutside = EuclidGeometryRandomTools.nextPoint3DInTriangle(random, face.getCentroid(), edge.getOrigin(), edge.getDestination());
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), face.getNormal(), pointOutside);

         Point3DBasics expectedProjection = face.orthogonalProjectionCopy(pointOutside);
         Point3DBasics actualProjection = convexPolytope3D.orthogonalProjectionCopy(pointOutside);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point directly below a face
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3D(random);

         if (convexPolytope3D.isEmpty())
         {
            assertNull(convexPolytope3D.orthogonalProjectionCopy(EuclidCoreRandomTools.nextPoint3D(random)));
            continue;
         }

         Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
         HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
         Point3D pointInside = EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random,
                                                                                  convexPolytope3D.getCentroid(),
                                                                                  face.getCentroid(),
                                                                                  edge.getOrigin(),
                                                                                  edge.getDestination());

         Point3DBasics actualProjection = convexPolytope3D.orthogonalProjectionCopy(pointInside);
         assertNull(actualProjection);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to an edge
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3D(random);

         if (convexPolytope3D.isEmpty())
         {
            assertNull(convexPolytope3D.orthogonalProjectionCopy(EuclidCoreRandomTools.nextPoint3D(random)));
            continue;
         }

         Face3D firstFace = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
         HalfEdge3D closestEdge = firstFace.getEdge(random.nextInt(firstFace.getNumberOfEdges()));
         Face3D secondFace = closestEdge.getTwin().getFace();

         Vector3D towardOutside = new Vector3D();
         towardOutside.interpolate(firstFace.getNormal(), secondFace.getNormal(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         towardOutside.normalize();

         Point3D pointOutside = new Point3D();
         pointOutside.interpolate(closestEdge.getOrigin(), closestEdge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), towardOutside, pointOutside);

         Point3DBasics expectedProjection = closestEdge.orthogonalProjectionCopy(pointOutside);
         Point3DBasics actualProjection = convexPolytope3D.orthogonalProjectionCopy(pointOutside);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to a vertex
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            assertNull(convexPolytope3D.orthogonalProjectionCopy(EuclidCoreRandomTools.nextPoint3D(random)));
            continue;
         }

         Vertex3D closestVertex = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));

         Vector3D towardOutside = new Vector3D();
         closestVertex.getAssociatedEdges().stream().forEach(edge -> towardOutside.scaleAdd(random.nextDouble(), edge.getFace().getNormal(), towardOutside));
         towardOutside.normalize();

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), towardOutside, closestVertex);

         Point3DBasics expectedProjection = closestVertex;
         Point3DBasics actualProjection = convexPolytope3D.orthogonalProjectionCopy(pointOutside);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
      }
   }

   @Test
   void testEvaluatePoint3DCollision() throws Exception
   {
      Random random = new Random(34535);
      Point3D expectedClosestPoint = new Point3D();
      Vector3D expectedNormal = new Vector3D();
      Point3D actualClosestPoint = new Point3D();
      Vector3D actualNormal = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      { // Point directly above a face
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(EuclidCoreRandomTools.nextPoint3D(random, 5.0), actualClosestPoint, actualNormal));
         }
         else if (convexPolytope3D.getNumberOfVertices() == 1)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal);
            expectedClosestPoint.set(convexPolytope3D.getVertex(0));
            expectedNormal.sub(point, expectedClosestPoint);
            expectedNormal.normalize();
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal), "Iteration: " + i);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
         else if (convexPolytope3D.getNumberOfVertices() == 2)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            expectedClosestPoint.set(EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(point,
                                                                                             convexPolytope3D.getVertex(0),
                                                                                             convexPolytope3D.getVertex(1)));
            expectedNormal.sub(point, expectedClosestPoint);
            expectedNormal.normalize();
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal), "Iteration: " + i);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
         else
         {
            Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
            HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
            Point3D pointOnFace = EuclidGeometryRandomTools.nextPoint3DInTriangle(random, face.getCentroid(), edge.getOrigin(), edge.getDestination());
            Point3D pointOutside = new Point3D();
            pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), face.getNormal(), pointOnFace);

            expectedClosestPoint.set(pointOnFace);
            expectedNormal.set(face.getNormal());
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal), "Iteration: " + i);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point directly below a face
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(EuclidCoreRandomTools.nextPoint3D(random, 5.0), actualClosestPoint, actualNormal));
         }
         else if (convexPolytope3D.getNumberOfVertices() == 1)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal);
            expectedClosestPoint.set(convexPolytope3D.getVertex(0));
            expectedNormal.sub(point, expectedClosestPoint);
            expectedNormal.normalize();
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal), "Iteration: " + i);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
         else if (convexPolytope3D.getNumberOfVertices() == 2)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            expectedClosestPoint.set(EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(point,
                                                                                             convexPolytope3D.getVertex(0),
                                                                                             convexPolytope3D.getVertex(1)));
            expectedNormal.sub(point, expectedClosestPoint);
            expectedNormal.normalize();
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal), "Iteration: " + i);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
         else
         {
            Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
            HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
            Point3D pointInside = EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random,
                                                                                     convexPolytope3D.getCentroid(),
                                                                                     face.getCentroid(),
                                                                                     edge.getOrigin(),
                                                                                     edge.getDestination());

            // It is tricky to generate a point that is closest to a given face.
            // So we use brute force to find the closest face and the normal should be the face's normal and the closest point be the orthogonal projection of pointInside onto the face's support plane.
            Face3D closestFace = convexPolytope3D.getFaces().stream().sorted((f1, f2) -> Double.compare(f1.distance(pointInside), f2.distance(pointInside)))
                                                 .findFirst().get();
            expectedClosestPoint.set(EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointInside, closestFace.getCentroid(), closestFace.getNormal()));
            expectedNormal.set(closestFace.getNormal());
            if (convexPolytope3D.getNumberOfFaces() == 1)
               assertFalse(convexPolytope3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal), "Iteration: " + i);
            else
               assertTrue(convexPolytope3D.evaluatePoint3DCollision(pointInside, actualClosestPoint, actualNormal), "Iteration: " + i);

            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to an edge
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(EuclidCoreRandomTools.nextPoint3D(random, 5.0), actualClosestPoint, actualNormal));
         }
         else if (convexPolytope3D.getNumberOfVertices() == 1)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal);
            expectedClosestPoint.set(convexPolytope3D.getVertex(0));
            expectedNormal.sub(point, expectedClosestPoint);
            expectedNormal.normalize();
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal), "Iteration: " + i);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
         else if (convexPolytope3D.getNumberOfVertices() == 2)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            expectedClosestPoint.set(EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(point,
                                                                                             convexPolytope3D.getVertex(0),
                                                                                             convexPolytope3D.getVertex(1)));
            expectedNormal.sub(point, expectedClosestPoint);
            expectedNormal.normalize();
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal), "Iteration: " + i);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
         else
         {
            Face3D firstFace = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
            HalfEdge3D closestEdge = firstFace.getEdge(random.nextInt(firstFace.getNumberOfEdges()));
            Vector3D towardOutside = new Vector3D();
            if (closestEdge.getTwin() != null)
            {
               Face3D secondFace = closestEdge.getTwin().getFace();
               towardOutside.interpolate(firstFace.getNormal(), secondFace.getNormal(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            }
            else
            {
               Vector3D faceNormal = new Vector3D(firstFace.getNormal());
               towardOutside.cross(faceNormal, closestEdge.getDirection(false));
               if (random.nextBoolean())
                  faceNormal.negate();
               towardOutside.interpolate(faceNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            }

            towardOutside.normalize();

            Point3D pointOnEdge = new Point3D();
            pointOnEdge.interpolate(closestEdge.getOrigin(), closestEdge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            Point3D pointOutside = new Point3D();
            pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), towardOutside, pointOnEdge);

            expectedClosestPoint.set(pointOnEdge);
            expectedNormal.set(towardOutside);
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal), "Iteration: " + i);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to a vertex
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(EuclidCoreRandomTools.nextPoint3D(random, 5.0), actualClosestPoint, actualNormal));
         }
         else if (convexPolytope3D.getNumberOfVertices() == 1)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal);
            expectedClosestPoint.set(convexPolytope3D.getVertex(0));
            expectedNormal.sub(point, expectedClosestPoint);
            expectedNormal.normalize();
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal), "Iteration: " + i);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
         else if (convexPolytope3D.getNumberOfVertices() == 2)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            expectedClosestPoint.set(EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(point,
                                                                                             convexPolytope3D.getVertex(0),
                                                                                             convexPolytope3D.getVertex(1)));
            expectedNormal.sub(point, expectedClosestPoint);
            expectedNormal.normalize();
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(point, actualClosestPoint, actualNormal), "Iteration: " + i);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
         else
         {
            Vertex3D closestVertex = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));

            Vector3D towardOutside = new Vector3D();
            closestVertex.getAssociatedEdges().stream().forEach(edge -> towardOutside.scaleAdd(random.nextDouble(), edge.getFace().getNormal(), towardOutside));
            towardOutside.normalize();

            Point3D pointOutside = new Point3D();
            pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), towardOutside, closestVertex);

            expectedClosestPoint.set(closestVertex);
            expectedNormal.set(towardOutside);
            assertFalse(convexPolytope3D.evaluatePoint3DCollision(pointOutside, actualClosestPoint, actualNormal), "Iteration: " + i);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedClosestPoint, actualClosestPoint, EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedNormal, actualNormal, EPSILON);
         }
      }
   }

   @Test
   void testFaceNormalAndVertexOrdering() throws Exception
   {
      Random random = new Random(34535);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<Point3D> vertices = EuclidGeometryRandomTools.nextPointCloud3D(random, 5.0, 5.0, 4);

         ConvexPolytope3D tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices));

         List<Vector3D> normals = tetrahedron.getFaces().stream().map(face -> new Vector3D(face.getNormal())).collect(Collectors.toList());

         for (int j = 0; j < tetrahedron.getNumberOfFaces(); j++)
         {
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i + ", face index: " + j, normals.get(j), tetrahedron.getFace(j).getNormal(), EPSILON);
         }
      }
   }

   @Test
   void testGeneralIntegrityOfRandomConvexPolytope3D() throws Exception
   {
      Random random = new Random(206422882);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3D(random);
         EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity(convexPolytope3D);

         convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);
         EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity(convexPolytope3D);
      }
   }

   @Test
   void testTroublesomeDatasets() throws Exception
   {
      List<ConvexPolytope3DTroublesomeDataset> datasets = new ArrayList<>();
      datasets.add(new DatasetGJKNullPointerExceptionBug1Original());
      datasets.add(new DatasetGJKNullPointerExceptionBug1Simplified());
      datasets.add(new DatasetGJKNullPointerExceptionBug2Original());
      datasets.add(new DatasetGJKNullPointerExceptionBug2OriginalV2());
      datasets.add(new DatasetGJKNullPointerExceptionBug2OriginalV3());
      datasets.add(new DatasetGJKNullPointerExceptionBug2Simplified());
      datasets.add(new DatasetGJKNullPointerExceptionBug3Original());
      datasets.add(new DatasetGJKNullPointerExceptionBug3OriginalV2());
      datasets.add(new DatasetGJKNullPointerExceptionBug3Simplified());
      datasets.add(new DatasetGJKNullPointerExceptionBug4Original());
      datasets.add(new DatasetGJKNullPointerExceptionBug4Simplified());
      datasets.add(new DatasetGJKNullPointerExceptionBug5());
      datasets.add(new DatasetGJKNullPointerExceptionBug6());
      datasets.add(new DatasetGJKNullPointerExceptionBug6V2());
      datasets.add(new DatasetGJKNullPointerExceptionBug6V3());
      datasets.add(new DatasetGJKNullPointerExceptionBug6V4());
      datasets.add(new DatasetGJKNullPointerExceptionBug7Original());
      datasets.add(new DatasetEPAFaceNormalIntegrityBug8Original());
      datasets.add(new DatasetEPAFaceNormalIntegrityBug8Simplified());
      datasets.add(new DatasetEPAFaceNormalIntegrityBug8SimplifiedV2());
      datasets.add(new DatasetEPAFaceNormalIntegrityBug9Original());
      datasets.add(new DatasetEPAFaceNormalIntegrityBug9Simplified());
      datasets.add(new DatasetGJKFaceNormalIntegrityBug10Original());
      datasets.add(new DatasetGJKFaceNormalIntegrityBug10Simplified());
      datasets.add(new DatasetGJKFaceNormalIntegrityBug11Original());
      datasets.add(new DatasetGJKFaceNormalIntegrityBug11Simplified());
      datasets.add(new DatasetGJKNullPointerExceptionBug12Original());
      datasets.add(new DatasetGJKNullPointerExceptionBug12Simplified());
      datasets.add(new DatasetGJKFaceNormalIntegrityBug13Original());
      datasets.add(new DatasetGJKFaceNormalIntegrityBug13Simplified());
      datasets.add(new DatasetGJKFaceNormalIntegrity_20190228_220911());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190302_160115());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190303_111711());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190303_120656());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190303_122006());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190303_142536());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190303_154201());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190303_165341());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190303_172836());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190303_180109());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190317_143836());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190317_161948());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190321_222438());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190323_122756());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190323_124929());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190323_150735());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190323_190624());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190323_193234());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190323_195449());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190323_213507());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190323_224417());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190324_182459());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190324_185429());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190325_205801());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190327_205357());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190327_211921());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190327_213133());
      datasets.add(new ConvexPolytope3DTroublesomeDataset_20190327_214757());

      for (int i = 0; i < datasets.size(); i++)
      {
         ConvexPolytope3DTroublesomeDataset dataset = datasets.get(i);
         String messagePrefix = "Dataset: " + dataset.getClass().getSimpleName();
         try
         {
            ConvexPolytope3D convexPolytope3D = dataset.getConvexPolytope3D();
            EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity(messagePrefix, convexPolytope3D);
            convexPolytope3D.addVertex(dataset.getTroublesomePoint());
            EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity(messagePrefix, convexPolytope3D);
         }
         catch (Exception | AssertionError e)
         {
            throw new AssertionFailedError(messagePrefix, e);
         }
      }
   }
}
