package us.ihmc.euclid.shape.convexPolytope.tools;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Triangle3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory.GeometryMesh3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidPolytopeToolsTest
{
   private final static int ITERATIONS = EuclidTestConstants.ITERATIONS;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testSignedDistanceFromPoint3DToLine3D() throws Exception
   {
      Random random = new Random(243234);

      for (int i = 0; i < ITERATIONS; i++)
      { // Ensure consistency with the 2D method.
         Line3D line3D = EuclidGeometryRandomTools.nextLine3D(random);
         Line2D line2D = new Line2D(new Point2D(line3D.getPoint()), new Vector2D(line3D.getDirection()));

         Point3D query3D = EuclidCoreRandomTools.nextPoint3D(random);
         Point2D query2D = new Point2D(query3D);

         Vector3DReadOnly planeNormal = Axis.Z;

         double expected = EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(query2D, line2D.getPoint(), line2D.getDirection());
         double actual = EuclidPolytopeTools.signedDistanceFromPoint3DToLine3D(query3D, line3D.getPoint(), line3D.getDirection(), planeNormal);
         assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testIsPoint3DOnSideOfLine3D() throws Exception
   {
      Random random = new Random(243234);

      for (int i = 0; i < ITERATIONS; i++)
      { // Ensure consistency with the 2D method.
         Line3D line3D = EuclidGeometryRandomTools.nextLine3D(random);
         Line2D line2D = new Line2D(new Point2D(line3D.getPoint()), new Vector2D(line3D.getDirection()));

         Point3D query3D = EuclidCoreRandomTools.nextPoint3D(random);
         Point2D query2D = new Point2D(query3D);

         Vector3DReadOnly planeNormal = Axis.Z;

         boolean testForLeftSide = random.nextBoolean();
         boolean expected = EuclidGeometryTools.isPoint2DOnSideOfLine2D(query2D, line2D.getPoint(), line2D.getDirection(), testForLeftSide);
         boolean actual = EuclidPolytopeTools.isPoint3DOnSideOfLine3D(query3D, line3D.getPoint(), line3D.getDirection(), planeNormal, testForLeftSide);
         assertEquals(expected, actual);
      }
   }

   @Test
   void testGetSilhouette() throws Exception
   {
      Random random = new Random(454353);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextIcoSphereBasedConvexPolytope3D(random);
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0e-3);
         // First we build an observer that is outside the polytope
         // We pick a face at random and create a point that is above the face's support plane.
         Face3D aFace = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
         Point3D observer = new Point3D(aFace.getCentroid());
         Vector3D tangential = EuclidCoreRandomTools.nextOrthogonalVector3D(random, aFace.getNormal(), true);
         observer.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), tangential, observer);
         observer.scaleAdd(EuclidCoreRandomTools.nextDouble(random, epsilon, 10.0), aFace.getNormal(), observer);

         // Evaluate the silhouette using brute force
         List<HalfEdge3D> expectedSilhouette = new ArrayList<>();

         for (HalfEdge3D edge : convexPolytope3D.getHalfEdges())
         { // This edge is part of the silhouette if its face is not visible and the face of its twin is visible
            boolean isEdgeFaceVisible = EuclidPolytopeTools.canObserverSeeFace(observer, edge.getFace(), epsilon);
            boolean isTwinFaceVisible = EuclidPolytopeTools.canObserverSeeFace(observer, edge.getTwin().getFace(), epsilon);

            if (!isEdgeFaceVisible && isTwinFaceVisible)
               expectedSilhouette.add(edge);
         }

         Collection<HalfEdge3D> actualSilhouette = EuclidPolytopeTools.computeSilhouette(convexPolytope3D.getFaces(), observer, epsilon);

         if (expectedSilhouette.isEmpty())
         {
            assertNull(actualSilhouette);
         }
         else
         {
            assertEquals(expectedSilhouette.size(), actualSilhouette.size());

            for (HalfEdge3D expectedSilhouetteEdge : expectedSilhouette)
            {
               assertTrue(actualSilhouette.contains(expectedSilhouetteEdge));
            }
         }
      }
   }

   @Test
   void testGetVisibleFaces() throws Exception
   {
      Random random = new Random(4456453);

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
            { // Expecting only 1 visible face
               Face3D face = convexPolytope3D.getFace(faceIndex);
               Point3D pointOnFace = EuclidShapeRandomTools.nextPoint3DInTriangle(random, face.getVertex(0), face.getVertex(1), face.getVertex(2));
               Point3D pointOutside = new Point3D();
               pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), face.getNormal(), pointOnFace);

               List<Face3D> actualVisibleFaces = EuclidPolytopeTools.extractVisibleFaces(convexPolytope3D.getFaces(), pointOutside, 0.0);

               assertEquals(1, actualVisibleFaces.size());
               assertTrue(face == actualVisibleFaces.get(0));
            }

            for (int faceIndex = 0; faceIndex < 4; faceIndex++)
            { // Expecting only 2 visible faces
               Face3D firstFace = convexPolytope3D.getFace(faceIndex);
               HalfEdge3D edge = firstFace.getEdge(random.nextInt(3));
               Face3D secondFace = edge.getTwin().getFace();

               Vector3D edgeNormal = new Vector3D();
               edgeNormal.interpolate(firstFace.getNormal(), secondFace.getNormal(), 0.5);
               edgeNormal.normalize();

               Point3D pointOnEdge = new Point3D();
               pointOnEdge.interpolate(edge.getOrigin(), edge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

               { // Case #1: the firstFace is the most visible
                  Vector3D directionLimit = new Vector3D(); // Represents the limit before secondFace becomes invisible.
                  directionLimit.cross(edge.getDirection(false), secondFace.getNormal());
                  directionLimit.normalize();
                  assertTrue(directionLimit.dot(firstFace.getNormal()) > 0.0); // This is only to ensure that we've constructed the limit such that it is on the firstFace side.

                  Vector3D extractionDirection = new Vector3D();
                  extractionDirection.interpolate(directionLimit, edgeNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
                  extractionDirection.normalize();

                  Point3D pointOutside = new Point3D();
                  pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), extractionDirection, pointOnEdge);

                  List<Face3D> actualVisibleFaces = EuclidPolytopeTools.extractVisibleFaces(convexPolytope3D.getFaces(), pointOutside, 0.0);

                  assertEquals(2, actualVisibleFaces.size());
                  assertTrue(actualVisibleFaces.contains(firstFace));
                  assertTrue(actualVisibleFaces.contains(secondFace));
                  assertTrue(secondFace == actualVisibleFaces.get(0));
               }

               { // Case #2: the secondFace is the most visible (redundant test)
                  Vector3D directionLimit = new Vector3D(); // Represents the limit before firstFace becomes invisible.
                  directionLimit.cross(firstFace.getNormal(), edge.getDirection(false));
                  directionLimit.normalize();
                  assertTrue(directionLimit.dot(secondFace.getNormal()) > 0.0); // This is only to ensure that we've constructed the limit such that it is on the secondFace side.

                  Vector3D extractionDirection = new Vector3D();
                  extractionDirection.interpolate(directionLimit, edgeNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
                  extractionDirection.normalize();

                  Point3D pointOutside = new Point3D();
                  pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), extractionDirection, pointOnEdge);

                  List<Face3D> actualVisibleFaces = EuclidPolytopeTools.extractVisibleFaces(convexPolytope3D.getFaces(), pointOutside, 0.0);

                  assertEquals(2, actualVisibleFaces.size());
                  assertTrue(actualVisibleFaces.contains(firstFace));
                  assertTrue(actualVisibleFaces.contains(secondFace));
                  assertTrue(firstFace == actualVisibleFaces.get(0));
               }
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextIcoSphereBasedConvexPolytope3D(random);
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0e-3);
         // First we build an observer that is outside the polytope
         // We pick a face at random and create a point that is above the face's support plane.
         Face3D aFace = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
         Point3D observer = new Point3D(aFace.getCentroid());
         Vector3D tangential = EuclidCoreRandomTools.nextOrthogonalVector3D(random, aFace.getNormal(), true);
         observer.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), tangential, observer);
         observer.scaleAdd(EuclidCoreRandomTools.nextDouble(random, epsilon, 10.0), aFace.getNormal(), observer);

         List<Face3D> visibleFaces = EuclidPolytopeTools.extractVisibleFaces(convexPolytope3D.getFaces(), observer, epsilon);
         List<Face3D> hiddenFaces = convexPolytope3D.getFaces().stream().filter(face -> !visibleFaces.contains(face)).collect(Collectors.toList());

         // Check consistency with EuclidPolytopeTools.canObserverSeeFace(...)
         assertTrue(visibleFaces.stream().allMatch(face -> EuclidPolytopeTools.canObserverSeeFace(observer, face, epsilon)));
         assertTrue(hiddenFaces.stream().noneMatch(face -> EuclidPolytopeTools.canObserverSeeFace(observer, face, epsilon)));

         // Check that the set of visible faces form a continuous set without isolated faces.
         if (visibleFaces.size() == 1)
         {
            Face3D visibleFace = visibleFaces.get(0);
            for (int neighborIndex = 0; neighborIndex < visibleFace.getNumberOfEdges(); neighborIndex++)
            {
               assertFalse(EuclidPolytopeTools.canObserverSeeFace(observer, visibleFace.getNeighbor(neighborIndex), epsilon));
            }
         }
         else
         {
            for (Face3D visibleFace : visibleFaces)
            {
               int numberOfVisibleNeighbors = 0;

               for (int neighborIndex = 0; neighborIndex < visibleFace.getNumberOfEdges(); neighborIndex++)
               {
                  Face3D neighbor = visibleFace.getNeighbor(neighborIndex);

                  if (EuclidPolytopeTools.canObserverSeeFace(observer, neighbor, epsilon))
                     numberOfVisibleNeighbors++;
               }

               assertTrue(numberOfVisibleNeighbors > 0);
            }
         }
      }
   }

   @Test
   void testTetrahedronVolume() throws Exception
   {
      Random random = new Random(9654);

      for (int i = 0; i < ITERATIONS; i++)
      { // We basically test the calculation by comparing multiple methods
         Point3D a = EuclidCoreRandomTools.nextPoint3D(random, 5.0); // Top vertex
         Point3D b = EuclidCoreRandomTools.nextPoint3D(random, 5.0); // Base vertex
         Point3D c = EuclidCoreRandomTools.nextPoint3D(random, 5.0); // Base vertex
         Point3D d = EuclidCoreRandomTools.nextPoint3D(random, 5.0); // Base vertex

         double ab2 = a.distanceSquared(b);
         double ac2 = a.distanceSquared(c);
         double ad2 = a.distanceSquared(d);
         double bc2 = b.distanceSquared(c);
         double cd2 = c.distanceSquared(d);
         double db2 = d.distanceSquared(b);

         // http://mathcentral.uregina.ca/QQ/database/QQ.09.06/haivan1.html
         // http://mathworld.wolfram.com/Tetrahedron.html
         // @formatter:off
         DenseMatrix64F matrix = new DenseMatrix64F(new double[][] {
            {0, 1, 1, 1, 1},        // | 0   1     1     1     1   |
            {1, 0, ab2, ac2, ad2},  // | 1   0   d12^2 d13^2 d14^2 |
            {1, ab2, 0, bc2, db2},  // | 1 d21^2   0   d23^2 d24^2 |
            {1, ac2, bc2, 0, cd2},  // | 1 d31^2 d32^2   0   d34^2 |
            {1, ad2, db2, cd2, 0}   // | 1 d41^2 d42^2 d43^2   0   |
            });
         // @formatter:on

         double volume1 = Math.sqrt(CommonOps.det(matrix) / 288.0);
         Plane3D basePlane = new Plane3D(b, c, d);
         double height = basePlane.distance(a);
         double baseArea = EuclidGeometryTools.triangleArea(b, c, d);
         double volume2 = baseArea * height / 3.0;
         assertEquals(volume1, volume2, 1.0e-7);

         Vector3D v1 = new Vector3D();
         Vector3D v2 = new Vector3D();
         Vector3D v3 = new Vector3D();

         v1.sub(b, a);
         v2.sub(c, a);
         v3.sub(d, a);

         Vector3D temp = new Vector3D();
         temp.cross(v2, v3);
         double volume3 = Math.abs(temp.dot(v1) / 6.0);
         assertEquals(volume2, volume3, EPSILON);

         assertEquals(volume3, EuclidShapeTools.tetrahedronVolume(a, b, c, d), EPSILON);
      }
   }

   @Test
   void testIcosahedronEdgeRadiusCalculation() throws Exception
   {
      Random random = new Random(3645646);

      for (int i = 0; i < ITERATIONS; i++)
      {
         // This generates a icosahedron that has a circumscribed unit-sphere
         GeometryMesh3D icosahedron = IcoSphereFactory.newIcoSphere(0);
         // We can easily define the radius simply by scaling the vertices.
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.1, 5.0);
         icosahedron.getVertices().forEach(vertex -> vertex.scale(radius));

         for (Triangle3D triangle : icosahedron.getAllTriangles())
         {
            double actualEdgeLength = EuclidShapeTools.icosahedronEdgeLength(radius);
            assertEquals(triangle.getAB(), actualEdgeLength, EPSILON);
            assertEquals(triangle.getBC(), actualEdgeLength, EPSILON);
            assertEquals(triangle.getCA(), actualEdgeLength, EPSILON);

            assertEquals(radius, EuclidShapeTools.icosahedronRadius(triangle.getAB()), EPSILON);
            assertEquals(radius, EuclidShapeTools.icosahedronRadius(triangle.getBC()), EPSILON);
            assertEquals(radius, EuclidShapeTools.icosahedronRadius(triangle.getCA()), EPSILON);
         }
      }
   }
}
