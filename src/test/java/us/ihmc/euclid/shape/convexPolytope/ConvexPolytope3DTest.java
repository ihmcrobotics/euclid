package us.ihmc.euclid.shape.convexPolytope;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.*;
import static us.ihmc.euclid.tools.EuclidCoreTools.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTestTools;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory.GeometryMesh3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.testSuite.EuclidTestSuite;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public class ConvexPolytope3DTest
{
   private static final int ITERATIONS = EuclidTestSuite.ITERATIONS;
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
         polytope.addVertex(firstVertex, 0.0);
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
         polytope.addVertex(firstVertex, 0.0);
         assertEquals(1, polytope.getNumberOfVertices());
         assertEquals(1, polytope.getNumberOfEdges());
         assertEquals(1, polytope.getNumberOfFaces());

         // Testing properties for single edge polytope.
         Point3D secondVertex = EuclidCoreRandomTools.nextPoint3D(random);
         polytope.addVertex(secondVertex, 0.0);
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
            Point3D pointInside = EuclidShapeRandomTools.nextWeightedAverage(random, pointsAdded);
            polytope.addVertex(pointInside, 1.0e-12);

            assertEquals(2, polytope.getNumberOfVertices());
            assertEquals(2, polytope.getNumberOfEdges());
            assertEquals(1, polytope.getNumberOfFaces());
         }

         // Testing properties for single triangle face polytope
         Point3D thirdVertex = EuclidCoreRandomTools.nextPoint3D(random);
         polytope.addVertex(thirdVertex, 0.0);
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
            Point3D pointInside = EuclidShapeRandomTools.nextWeightedAverage(random, pointsAdded);
            polytope.addVertex(pointInside, 1.0e-12);

            assertEquals(3, polytope.getNumberOfVertices());
            assertEquals(3, polytope.getNumberOfEdges());
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

            EuclidPolytopeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getTwin().getDestination(), EPSILON);
            EuclidPolytopeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getTwin().getOrigin(), EPSILON);

            EuclidPolytopeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getPrevious().getDestination(), EPSILON);
            EuclidPolytopeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getNext().getOrigin(), EPSILON);

            EuclidPolytopeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getPrevious().getTwin().getOrigin(), EPSILON);
            EuclidPolytopeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getNext().getTwin().getDestination(), EPSILON);

            EuclidPolytopeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getTwin().getNext().getOrigin(), EPSILON);
            EuclidPolytopeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getTwin().getPrevious().getDestination(), EPSILON);

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
            Point3D pointInside = EuclidShapeRandomTools.nextPoint3DInTetrahedron(random, firstVertex, secondVertex, thirdVertex, fourthVertex);

            assertTrue(polytope.isPointInside(pointInside, EPSILON));

            polytope.addVertex(pointInside, 0.0);

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
         convexPolytope3D.addVertex(bottomP0, 0.0);
         convexPolytope3D.addVertex(bottomP1, 0.0);
         convexPolytope3D.addVertex(bottomP2, 0.0);
         convexPolytope3D.addVertex(top, 0.0);

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
            Vector3D normal = face.getNormal();

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
         vertices.forEach(vertex -> convexPolytope3D.addVertex(vertex, 0.0));

         assertEquals(4, convexPolytope3D.getNumberOfVertices());
         assertEquals(6, convexPolytope3D.getNumberOfEdges());
         assertEquals(4, convexPolytope3D.getNumberOfFaces());

         HalfEdge3D edge = convexPolytope3D.getEdge(random.nextInt(convexPolytope3D.getNumberOfEdges()));
         Point3DBasics newVertex = edge.pointOnLineGivenPercentage(1.0 + random.nextDouble());
         Vertex3D expectedVertexRemoved = edge.getDestination();

         convexPolytope3D.addVertex(newVertex, 1.0e-10);

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
         Arrays.asList(bottomP0, bottomP1, bottomP2, bottomP3, topP0, topP1, topP2, topP3, bottomNormal, topNormal, xPlusSideNormal, xMinusSideNormal,
                       yPlusSideNormal, yMinusSideNormal, bottomCenter, topCenter, xPlusSideCenter, xMinusSideCenter, yPlusSideCenter, yMinusSideCenter)
               .forEach(o -> o.applyTransform(transform));

         double buildEpsilon = 1.0e-10;
         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         convexPolytope3D.addVertex(bottomP0, buildEpsilon);
         convexPolytope3D.addVertex(bottomP1, buildEpsilon);
         convexPolytope3D.addVertex(bottomP2, buildEpsilon);
         convexPolytope3D.addVertex(bottomP3, buildEpsilon);

         convexPolytope3D.addVertex(topP0, buildEpsilon);
         convexPolytope3D.addVertex(topP1, buildEpsilon);
         convexPolytope3D.addVertex(topP2, buildEpsilon);
         convexPolytope3D.addVertex(topP3, buildEpsilon);

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

         convexPolytope3D.getEdges().forEach(edge -> assertNotNull(edge.getTwin()));
         convexPolytope3D.getEdges().forEach(edge -> assertNotNull(edge.getNext()));
         convexPolytope3D.getEdges().forEach(edge -> assertNotNull(edge.getPrevious()));

         convexPolytope3D.getVertices().forEach(vertex -> assertEquals(3, vertex.getNumberOfAssociatedEdges()));
      }
   }

   @Test
   void testConstructingIcosahedron() throws Exception
   {
      Random random = new Random(23423);

      GeometryMesh3D icosahedron = IcoSphereFactory.newIcoSphere(0);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         List<Point3D> shuffledVertices = new ArrayList<>(icosahedron.getVertices());
         Collections.shuffle(shuffledVertices, random);

         shuffledVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex, 1.0e-10));

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
            assertTrue(icosahedron.getAllTriangles().stream().anyMatch(triangle -> triangle.geometryEquals(a, b, c, EPSILON)));
         }

         for (HalfEdge3D edge : convexPolytope3D.getEdges())
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

         shuffledVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex, 1.0e-10));

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
            assertTrue(icosahedron.getAllTriangles().stream().anyMatch(triangle -> triangle.geometryEquals(a, b, c, EPSILON)));
         }

         for (HalfEdge3D edge : convexPolytope3D.getEdges())
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
         GeometryMesh3D icoSphere = IcoSphereFactory.newIcoSphere(random.nextInt(2) + 1);
         icoSphere.applyTransform(EuclidCoreRandomTools.nextRigidBodyTransform(random));

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D();
         List<Point3D> shuffledVertices = new ArrayList<>(icoSphere.getVertices());
         Collections.shuffle(shuffledVertices, random);

         shuffledVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex, 1.0e-10));

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
            assertTrue(icoSphere.getAllTriangles().stream().anyMatch(triangle -> triangle.geometryEquals(a, b, c, EPSILON)));
         }

         for (HalfEdge3D edge : convexPolytope3D.getEdges())
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
            bottom.add(new Point3D(Math.cos(theta), Math.sin(theta), 0.0));
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
         allVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex, 1.0e-10));

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
            bottom.add(new Point3D(bottomRadius * Math.cos(theta), bottomRadius * Math.sin(theta), 0.0));
            intermediate.add(new Point3D(intermediateRadius * Math.cos(theta), intermediateRadius * Math.sin(theta), intermediateZ));
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
         truncatedConeVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex, 1.0e-10));

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
         convexPolytope3D.addVertex(top, 1.0e-10);

         /*
          * FIXME The top vertex is being rejected. the test passes when reducing (even just slightly)
          * intermediateRadius, making the sides visible from the top. When increasing intermediateRadius,
          * the resulting polytope is also acceptable, i.e. the intermediateVertices remain and the top is
          * added with an extra layer of side faces. However, when intermediateResult is left unchanged, the
          * sides are identified as not visible and the top is identified as being on the sides, when that
          * happens, the logic seems to be wrong and the top end up being rejected.
          */
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
            bottom.add(new Point3D(Math.cos(theta), Math.sin(theta), 0.0));
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
         allVertices.forEach(vertex -> convexPolytope3D.addVertex(vertex, 1.0e-10));

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
            assertEquals(2, vertices.stream().filter(v -> epsilonEquals(0.0, percentageAlongLineSegment3D(v, bottomCentroid, topCentroid), EPSILON)).count());
            assertEquals(2, vertices.stream().filter(v -> epsilonEquals(1.0, percentageAlongLineSegment3D(v, bottomCentroid, topCentroid), EPSILON)).count());
         }

         // Let's tweak the cylinder to be extended by a cone at the top:
         convexPolytope3D.addVertex(aboveTop, 1.0e-10);

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
            assertEquals(2, vertices.stream().filter(v -> epsilonEquals(0.0, percentageAlongLineSegment3D(v, topCentroid, aboveTop), EPSILON)).count());
         }

         for (Face3D sideFace : sideFaces)
         {
            assertEquals(4, sideFace.getNumberOfEdges());
            List<Vertex3D> vertices = sideFace.getVertices();
            assertEquals(2, vertices.stream().filter(v -> epsilonEquals(0.0, percentageAlongLineSegment3D(v, bottomCentroid, topCentroid), EPSILON)).count());
            assertEquals(2, vertices.stream().filter(v -> epsilonEquals(1.0, percentageAlongLineSegment3D(v, bottomCentroid, topCentroid), EPSILON)).count());
         }

         // Let's tweak the cylinder to be extended by a cone at the bottom:
         convexPolytope3D.addVertex(belowBottom, 1.0e-10);

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
            assertEquals(2, vertices.stream().filter(v -> epsilonEquals(0.0, percentageAlongLineSegment3D(v, bottomCentroid, belowBottom), EPSILON)).count());
         }

         convexPolytope3D.getEdges().forEach(edge -> assertNotNull(edge.getTwin()));
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
         convexPolytope3D.addVertex(bottomP0, 0.0);
         convexPolytope3D.addVertex(bottomP1, 0.0);
         convexPolytope3D.addVertex(bottomP2, 0.0);
         convexPolytope3D.addVertex(top, 0.0);

         for (int i = 0; i < ITERATIONS; i++)
         {
            for (int faceIndex = 0; faceIndex < 4; faceIndex++)
            {
               Face3D face = convexPolytope3D.getFace(faceIndex);
               Point3D pointOnFace = EuclidShapeRandomTools.nextPoint3DInTriangle(random, face.getVertex(0), face.getVertex(1), face.getVertex(2));
               assertTrue(face == convexPolytope3D.getClosestFace(pointOnFace));

               Point3D pointOutside = new Point3D();
               pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), face.getNormal(), pointOnFace);
               assertTrue(face == convexPolytope3D.getClosestFace(pointOutside));
            }

            // TODO need to add couple with point: closest to an edge, closest to a vertex
         }
      }
      // TODO need to add tests with more complicated polytope
   }

   @Test
   void testApplyTransform() throws Exception
   {
      Random random = new Random(2342);

      for (int i = 0; i < ITERATIONS; i++)
      { // Translation only
         GeometryMesh3D icosahedron = IcoSphereFactory.newIcoSphere(0);
         ConvexPolytope3D originalPolytope = new ConvexPolytope3D();
         icosahedron.getVertices().forEach(vertex -> originalPolytope.addVertex(vertex, 1.0e-10));
         ConvexPolytope3D actualPolytope = new ConvexPolytope3D();
         icosahedron.getVertices().forEach(vertex -> actualPolytope.addVertex(vertex, 1.0e-10));

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         actualPolytope.applyTransform(transform);

         icosahedron.applyTransform(transform);
         ConvexPolytope3D expectedPolytope = new ConvexPolytope3D();
         icosahedron.getVertices().forEach(vertex -> expectedPolytope.addVertex(vertex, 1.0e-10));

         EuclidPolytopeTestTools.assertConvexPolytope3DEquals(expectedPolytope, actualPolytope, EPSILON);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expectedPolytope.getBoundingBox(), actualPolytope.getBoundingBox(), EPSILON);

         for (int faceIndex = 0; faceIndex < expectedPolytope.getNumberOfFaces(); faceIndex++)
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedPolytope.getFace(faceIndex).getCentroid(), actualPolytope.getFace(faceIndex).getCentroid(),
                                                    EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedPolytope.getFace(faceIndex).getNormal(), actualPolytope.getFace(faceIndex).getNormal(), EPSILON);
            EuclidGeometryTestTools.assertBoundingBox3DEquals(expectedPolytope.getFace(faceIndex).getBoundingBox(),
                                                              actualPolytope.getFace(faceIndex).getBoundingBox(), EPSILON);
            assertEquals(expectedPolytope.getFace(faceIndex).getArea(), actualPolytope.getFace(faceIndex).getArea(), EPSILON);
         }

         actualPolytope.applyInverseTransform(transform);

         EuclidPolytopeTestTools.assertConvexPolytope3DEquals(originalPolytope, actualPolytope, EPSILON);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(originalPolytope.getBoundingBox(), actualPolytope.getBoundingBox(), EPSILON);

         for (int faceIndex = 0; faceIndex < originalPolytope.getNumberOfFaces(); faceIndex++)
         {
            EuclidCoreTestTools.assertTuple3DEquals(originalPolytope.getFace(faceIndex).getCentroid(), actualPolytope.getFace(faceIndex).getCentroid(),
                                                    EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(originalPolytope.getFace(faceIndex).getNormal(), actualPolytope.getFace(faceIndex).getNormal(), EPSILON);
            EuclidGeometryTestTools.assertBoundingBox3DEquals(originalPolytope.getFace(faceIndex).getBoundingBox(),
                                                              actualPolytope.getFace(faceIndex).getBoundingBox(), EPSILON);
            assertEquals(originalPolytope.getFace(faceIndex).getArea(), actualPolytope.getFace(faceIndex).getArea(), EPSILON);
         }
      }
   }
}
