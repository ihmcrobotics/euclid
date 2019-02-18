package us.ihmc.euclid.shape.convexPolytope;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory.GeometryMesh3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
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
            Point3D pointInside = EuclidShapeRandomTools.nextWeightedAverage(random, pointsAdded);
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
            Point3D pointInside = EuclidShapeRandomTools.nextWeightedAverage(random, pointsAdded);
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

            EuclidShapeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getTwin().getDestination(), EPSILON);
            EuclidShapeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getTwin().getOrigin(), EPSILON);

            EuclidShapeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getPrevious().getDestination(), EPSILON);
            EuclidShapeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getNext().getOrigin(), EPSILON);

            EuclidShapeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getPrevious().getTwin().getOrigin(), EPSILON);
            EuclidShapeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getNext().getTwin().getDestination(), EPSILON);

            EuclidShapeTestTools.assertVertex3DEquals(edge.getOrigin(), edge.getTwin().getNext().getOrigin(), EPSILON);
            EuclidShapeTestTools.assertVertex3DEquals(edge.getDestination(), edge.getTwin().getPrevious().getDestination(), EPSILON);

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
         Arrays.asList(bottomP0, bottomP1, bottomP2, bottomP3, topP0, topP1, topP2, topP3, bottomNormal, topNormal, xPlusSideNormal, xMinusSideNormal,
                       yPlusSideNormal, yMinusSideNormal, bottomCenter, topCenter, xPlusSideCenter, xMinusSideCenter, yPlusSideCenter, yMinusSideCenter)
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

      GeometryMesh3D icosahedron = IcoSphereFactory.newIcoSphere(0);

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
         GeometryMesh3D icoSphere = IcoSphereFactory.newIcoSphere(random.nextInt(2) + 1);
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
            assertEquals(2, vertices.stream().filter(v -> EuclidCoreTools.epsilonEquals(0.0, percentageAlongLineSegment3D(v, topCentroid, aboveTop), EPSILON))
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
      {
         GeometryMesh3D icosahedron = IcoSphereFactory.newIcoSphere(0);
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
            EuclidCoreTestTools.assertTuple3DEquals(expectedPolytope.getFace(faceIndex).getCentroid(), actualPolytope.getFace(faceIndex).getCentroid(),
                                                    EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(expectedPolytope.getFace(faceIndex).getNormal(), actualPolytope.getFace(faceIndex).getNormal(), EPSILON);
            EuclidGeometryTestTools.assertBoundingBox3DEquals(expectedPolytope.getFace(faceIndex).getBoundingBox(),
                                                              actualPolytope.getFace(faceIndex).getBoundingBox(), EPSILON);
            assertEquals(expectedPolytope.getFace(faceIndex).getArea(), actualPolytope.getFace(faceIndex).getArea(), EPSILON);
         }

         actualPolytope.applyInverseTransform(transform);

         EuclidShapeTestTools.assertConvexPolytope3DEquals(originalPolytope, actualPolytope, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(originalPolytope.getCentroid(), actualPolytope.getCentroid(), EPSILON);
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

            EuclidShapeTestTools.assertHalfEdge3DEquals(originalEdge, copyEdge, EPSILON);
            EuclidShapeTestTools.assertHalfEdge3DEquals(originalNext, copyNext, EPSILON);
            EuclidShapeTestTools.assertHalfEdge3DEquals(originalPrevious, copyPrevious, EPSILON);
            EuclidShapeTestTools.assertHalfEdge3DEquals(originalTwin, copyTwin, EPSILON);
            EuclidShapeTestTools.assertFace3DEquals(originalFace, copyFace, EPSILON);
         }

         for (int vertexIndex = 0; vertexIndex < originalPolytope.getNumberOfVertices(); vertexIndex++)
         {
            Vertex3D originalVertex = originalPolytope.getVertex(vertexIndex);
            Vertex3D copyVertex = copyPolytope.getVertex(vertexIndex);

            assertTrue(originalVertex != copyVertex);
            EuclidShapeTestTools.assertVertex3DEquals(originalVertex, copyVertex, EPSILON);

            for (int edgeIndex = 0; edgeIndex < originalVertex.getNumberOfAssociatedEdges(); edgeIndex++)
            {
               HalfEdge3D originalEdge = originalVertex.getAssociatedEdge(edgeIndex);
               HalfEdge3D copyEdge = copyVertex.getAssociatedEdge(edgeIndex);
               assertTrue(originalEdge != copyEdge);
               EuclidShapeTestTools.assertHalfEdge3DEquals(originalEdge, copyEdge, EPSILON);
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

            EuclidShapeTestTools.assertHalfEdge3DEquals(originalEdge, copyEdge, EPSILON);
            EuclidShapeTestTools.assertHalfEdge3DEquals(originalNext, copyNext, EPSILON);
            EuclidShapeTestTools.assertHalfEdge3DEquals(originalPrevious, copyPrevious, EPSILON);
            EuclidShapeTestTools.assertHalfEdge3DEquals(originalTwin, copyTwin, EPSILON);
            EuclidShapeTestTools.assertFace3DEquals(originalFace, copyFace, EPSILON);
         }

         for (int vertexIndex = 0; vertexIndex < originalPolytope.getNumberOfVertices(); vertexIndex++)
         {
            Vertex3D originalVertex = originalPolytope.getVertex(vertexIndex);
            Vertex3D copyVertex = copyPolytope.getVertex(vertexIndex);

            assertTrue(originalVertex != copyVertex);
            EuclidShapeTestTools.assertVertex3DEquals(originalVertex, copyVertex, EPSILON);

            for (int edgeIndex = 0; edgeIndex < originalVertex.getNumberOfAssociatedEdges(); edgeIndex++)
            {
               HalfEdge3D originalEdge = originalVertex.getAssociatedEdge(edgeIndex);
               HalfEdge3D copyEdge = copyVertex.getAssociatedEdge(edgeIndex);
               assertTrue(originalEdge != copyEdge);
               EuclidShapeTestTools.assertHalfEdge3DEquals(originalEdge, copyEdge, EPSILON);
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
         Point3D pointOutside = EuclidShapeRandomTools.nextPoint3DInTriangle(random, face.getCentroid(), edge.getOrigin(), edge.getDestination());
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
         Point3D pointInside = EuclidShapeRandomTools.nextPoint3DInTetrahedron(random, convexPolytope3D.getCentroid(), face.getCentroid(), edge.getOrigin(),
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
   void testGetSupportVectorDirectionTo() throws Exception
   {
      Random random = new Random(34535);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point directly above a face
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            assertNull(convexPolytope3D.getSupportVectorDirectionTo(EuclidCoreRandomTools.nextPoint3D(random, 5.0)));
         }
         else if (convexPolytope3D.getNumberOfVertices() == 1)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            Vector3DBasics expectedSupportVector = convexPolytope3D.getVertex(0).getSupportVectorDirectionTo(point);
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(point);
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
         }
         else if (convexPolytope3D.getNumberOfVertices() == 2)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            Vector3DBasics expectedSupportVector = convexPolytope3D.getHalfEdge(0).getSupportVectorDirectionTo(point);
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(point);
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
         }
         else
         {
            Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
            HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
            Point3D pointOnFace = EuclidShapeRandomTools.nextPoint3DInTriangle(random, face.getCentroid(), edge.getOrigin(), edge.getDestination());
            Point3D pointOutside = new Point3D();
            pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), face.getNormal(), pointOnFace);

            Vector3D expectedSupportVector = new Vector3D();
            expectedSupportVector.sub(pointOutside, pointOnFace);
            expectedSupportVector.normalize();
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(pointOutside);
            actualSupportVector.normalize();
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point directly below a face
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            assertNull(convexPolytope3D.getSupportVectorDirectionTo(EuclidCoreRandomTools.nextPoint3D(random, 5.0)));
         }
         else if (convexPolytope3D.getNumberOfVertices() == 1)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            Vector3DBasics expectedSupportVector = convexPolytope3D.getVertex(0).getSupportVectorDirectionTo(point);
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(point);
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
         }
         else if (convexPolytope3D.getNumberOfVertices() == 2)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            Vector3DBasics expectedSupportVector = convexPolytope3D.getHalfEdge(0).getSupportVectorDirectionTo(point);
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(point);
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
         }
         else
         {
            Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
            HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
            Point3D pointInside = EuclidShapeRandomTools.nextPoint3DInTetrahedron(random, convexPolytope3D.getCentroid(), face.getCentroid(), edge.getOrigin(),
                                                                                  edge.getDestination());

            // It is tricky to generate a point that is closest to a given face.
            // So we use brute force to find the closest face and the support vector should be the face's normal.

            Vector3D expectedSupportVector = new Vector3D(convexPolytope3D.getFaces().stream()
                                                                          .sorted((f1, f2) -> Double.compare(f1.distance(pointInside),
                                                                                                             f2.distance(pointInside)))
                                                                          .findFirst().get().getNormal());
            expectedSupportVector.negate();
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(pointInside);
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to an edge
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            assertNull(convexPolytope3D.getSupportVectorDirectionTo(EuclidCoreRandomTools.nextPoint3D(random, 5.0)));
         }
         else if (convexPolytope3D.getNumberOfVertices() == 1)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            Vector3DBasics expectedSupportVector = convexPolytope3D.getVertex(0).getSupportVectorDirectionTo(point);
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(point);
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
         }
         else if (convexPolytope3D.getNumberOfVertices() == 2)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            Vector3DBasics expectedSupportVector = convexPolytope3D.getHalfEdge(0).getSupportVectorDirectionTo(point);
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(point);
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
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

            Vector3DBasics expectedSupportVector = closestEdge.getSupportVectorDirectionTo(pointOutside);
            //         expectedSupportVector.normalize();
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(pointOutside);
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to a vertex
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            assertNull(convexPolytope3D.getSupportVectorDirectionTo(EuclidCoreRandomTools.nextPoint3D(random, 5.0)));
         }
         else if (convexPolytope3D.getNumberOfVertices() == 1)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            Vector3DBasics expectedSupportVector = convexPolytope3D.getVertex(0).getSupportVectorDirectionTo(point);
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(point);
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
         }
         else if (convexPolytope3D.getNumberOfVertices() == 2)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 5.0);
            Vector3DBasics expectedSupportVector = convexPolytope3D.getHalfEdge(0).getSupportVectorDirectionTo(point);
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(point);
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
         }
         else
         {
            Vertex3D closestVertex = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));

            Vector3D towardOutside = new Vector3D();
            closestVertex.getAssociatedEdges().stream().forEach(edge -> towardOutside.scaleAdd(random.nextDouble(), edge.getFace().getNormal(), towardOutside));
            towardOutside.normalize();

            Point3D pointOutside = new Point3D();
            pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), towardOutside, closestVertex);

            Vector3D expectedSupportVector = new Vector3D();
            expectedSupportVector.sub(pointOutside, closestVertex);
            //         expectedSupportVector.normalize();
            Vector3DBasics actualSupportVector = convexPolytope3D.getSupportVectorDirectionTo(pointOutside);
            EuclidCoreTestTools.assertTuple3DEquals(expectedSupportVector, actualSupportVector, EPSILON);
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

         tetrahedron.getFaces().forEach(Face3D::updateNormal);

         for (int j = 0; j < tetrahedron.getNumberOfFaces(); j++)
         {
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i + ", face index: " + j, normals.get(j), tetrahedron.getFace(j).getNormal(), EPSILON);
         }
      }
   }

   @Test
   void testGJKNullPointerExceptionBug1() throws Exception
   {
      String detailedFormat = EuclidCoreIOTools.getStringFormat(9, 6);
      List<Point3D> vertices = new ArrayList<>();
      // Commented out the vertices from the original dataset that do not seem to cause the issue.
      // vertices.add(new Point3D(0.1727226445760459, 0.3408246844828842, 0.0707692371805856));
      // vertices.add(new Point3D(-0.0010094559634557, 0.1846852819495834, -0.0004136019841320));
      // vertices.add(new Point3D(0.0131019603409140, 0.0679884843470001, 0.0053682349594210));
      vertices.add(new Point3D(0.0268762363673278, 0.0420820755879367, 0.0110119362210409));
      vertices.add(new Point3D(0.0382545893376212, 0.0262443294907677, 0.0151346721330367));
      vertices.add(new Point3D(0.0442214133931064, 0.0188148816283424, 0.0181187342317446));
      // vertices.add(new Point3D(0.0878487223095894, -0.0176017339231975, 0.0359940474533526));
      vertices.add(new Point3D(0.0317197072764835, 0.0339730913830635, 0.0146770460780876));
      Point3D troublingVertex = new Point3D(0.0356799421574458, 0.0305305995258588, 0.0121379970533146);

      double constructionEpsilon = 1.0e-3;
      ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices), constructionEpsilon);
      assertEquals(vertices.size(), convexPolytope3D.getNumberOfVertices());

      System.out.println("Polytope edge lengths:");
      for (HalfEdge3D edge : convexPolytope3D.getHalfEdges())
         System.out.println(edge.getDirection(false).length());
      System.out.println("Distance of troublingVertex from polytope vertices:");
      for (Point3D vertex : vertices)
         System.out.println(vertex.distance(troublingVertex));
      System.out.println("Distance of troublingVertex from polytope faces:");
      for (Face3D face : convexPolytope3D.getFaces())
         System.out.println(face.distance(troublingVertex));
      System.out.println("Signed-distance of troublingVertex from polytope faces' support plane:");
      for (Face3D face : convexPolytope3D.getFaces())
         System.out.println(face.signedDistanceToPlane(troublingVertex));
      System.out.println("Distance of troublingVertex from polytope edges' support line:");
      for (HalfEdge3D edge : convexPolytope3D.getHalfEdges())
         System.out.println(edge.distanceFromSupportLine(troublingVertex));
      System.out.println("Faces centroid and normal:");
      for (Face3D face : convexPolytope3D.getFaces())
         System.out.println(face.getCentroid().toString() + ", " + face.getNormal().toString());

      // Checking each face normal sanity
      for (Face3D face : convexPolytope3D.getFaces())
      {
         Vector3D awayFromPolytope = new Vector3D();
         awayFromPolytope.sub(face.getCentroid(), convexPolytope3D.getCentroid());
         assertEquals(1.0, face.getNormal().length(), EPSILON);
         assertTrue(face.getNormal().dot(awayFromPolytope) > 0.0);
      }

      List<Face3D> visibleFaces = new ArrayList<>();
      List<Face3D> inPlaneFaces = new ArrayList<>();
      List<HalfEdge3DReadOnly> silhouette = new ArrayList<>(EuclidPolytopeTools.computeSilhouette(convexPolytope3D.getFaces(), troublingVertex,
                                                                                                  constructionEpsilon, visibleFaces, inPlaneFaces));

      assertEquals(3, silhouette.size());
      assertTrue(silhouette.get(0).getDestination() == silhouette.get(1).getOrigin());
      assertTrue(silhouette.get(1).getDestination() == silhouette.get(2).getOrigin());
      assertTrue(silhouette.get(2).getDestination() == silhouette.get(0).getOrigin());

      //      assertEquals(2, inPlaneFaces.size());
      //
      //      System.out.println("Distance centroid <-> plane for comparing the 2 inPlaneFaces:");
      //      System.out.println(inPlaneFaces.get(0).distanceToPlane(inPlaneFaces.get(1).getCentroid()));
      //      System.out.println(inPlaneFaces.get(1).distanceToPlane(inPlaneFaces.get(0).getCentroid()));
      //      System.out.println("Angle between the 2 inPlaneFaces:");
      //      System.out.println(inPlaneFaces.get(0).getNormal().angle(inPlaneFaces.get(1).getNormal()));
      //      HalfEdge3D commonEdgeWith = inPlaneFaces.get(0).getCommonEdgeWith(inPlaneFaces.get(1));
      //      System.out.println("inPlaneFaces common edge:\n"+ EuclidGeometryIOTools.getLineSegment3DString(detailedFormat, commonEdgeWith));
      //      System.out.println("troublingVertex distance to common-edge: " + commonEdgeWith.distanceFromSupportLine(troublingVertex));

      System.out.println("Polytope edges:");
      for (HalfEdge3D edge : convexPolytope3D.getHalfEdges())
      {
         if (edge.hashCode() < edge.getTwin().hashCode())
            System.out.println(EuclidGeometryIOTools.getLineSegment3DString(detailedFormat, edge));
      }

      for (Face3D inPlaneFace : inPlaneFaces)
         assertFalse(visibleFaces.contains(inPlaneFace));

      convexPolytope3D.addVertex(troublingVertex);
   }

   @Test
   void testGJKNullPointerExceptionBug2() throws Exception
   { // This dataset caused a newly created face to reject the new vertex
      List<Point3D> vertices = new ArrayList<>();
      // Commented out the vertices from the original dataset that do not seem to cause the issue.
      vertices.add(new Point3D(0.1075136584776464, -0.4824945590361311, 0.0090899461159321));
      vertices.add(new Point3D(0.5607657881997890, 0.4040329322380422, 0.0474110068485305));
      vertices.add(new Point3D(0.0289073412127943, 0.0078295230557014, -0.0147523787325835));
      vertices.add(new Point3D(0.0043375074277710, -0.1974921037188990, 0.0003667227899621));
      vertices.add(new Point3D(0.0139072921615079, -0.0447432688724709, 0.0011758183858351));
      vertices.add(new Point3D(0.0186428968993924, -0.0235249738131959, 0.0012995156961376));
      vertices.add(new Point3D(0.0212072625303128, -0.0130016538284213, 0.0026586776633881));
      vertices.add(new Point3D(0.0341784903074217, 0.0292151862876490, 0.0028896852699534));
      // vertices.add(new Point3D(0.0643309020398054, 0.0996995467973626, 0.0054389780927482));
      vertices.add(new Point3D(0.0269036070083187, 0.0083216605291816, 0.0078200543380451));
      Point3D troublingVertex = new Point3D(0.0325313915792233, 0.0068659303400010, -0.0421477673824605);

      double constructionEpsilon = 1.0e-3;
      ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices), constructionEpsilon);
      assertEquals(vertices.size(), convexPolytope3D.getNumberOfVertices());

      convexPolytope3D.addVertex(troublingVertex);
   }

   @Test
   void testGJKNullPointerExceptionBug3() throws Exception
   { // This dataset caused the search for silhouetteStartEdge to fail 
      List<Point3D> vertices = new ArrayList<>();
      // Commented out the vertices from the original dataset that do not seem to cause the issue.
      vertices.add(new Point3D(-0.0037210269862168, 0.0766427555019901, 0.1361514789125481));
      vertices.add(new Point3D(-0.0098691038563419, -0.0842712352669562, 0.3611081270196839));
      vertices.add(new Point3D(-0.0013371474796426, 0.1104004526675205, 0.1045058111402412));
      vertices.add(new Point3D(-0.0036362971658496, 0.0933043645625930, 0.1200567135497745));
      vertices.add(new Point3D(-0.0024413371416485, 0.1279448469369309, 0.0893279365053837));
      vertices.add(new Point3D(-0.0064317125888430, 0.1020382766797260, 0.1119023450212495));
      vertices.add(new Point3D(-0.0012837946563208, 0.1829972596968365, 0.0469737364779878));
      vertices.add(new Point3D(0.0006330947545868, 0.3027754040393130, -0.0231647841974971));
      vertices.add(new Point3D(0.0022656588195281, 0.8450024017573801, -0.0828999091198799));
      vertices.add(new Point3D(-0.0197511024523884, 1.4863939764561370, 0.7226880694514805));
      Point3D troublingVertex = new Point3D(0.0153396366110251, 0.1027009848979795, 0.1121607898997810);

      double constructionEpsilon = 1.0e-3;
      ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices), constructionEpsilon);
      assertEquals(vertices.size(), convexPolytope3D.getNumberOfVertices());

      convexPolytope3D.addVertex(troublingVertex);
   }
}
