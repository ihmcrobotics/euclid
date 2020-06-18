package us.ihmc.euclid.shape.convexPolytope.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
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
import us.ihmc.euclid.shape.convexPolytope.tools.IcoSphereFactory.TriangleMesh3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidPolytopeToolsTest
{
   private final static int ITERATIONS = EuclidTestConstants.ITERATIONS;
   private static final double EPSILON = 1.0e-12;

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

         Vector3DReadOnly planeNormal = Axis3D.Z;

         boolean testForLeftSide = random.nextBoolean();
         boolean expected = EuclidGeometryTools.isPoint2DOnSideOfLine2D(query2D, line2D.getPoint(), line2D.getDirection(), testForLeftSide);
         boolean actual = EuclidPolytopeTools.isPoint3DOnSideOfLine3D(query3D, line3D.getPoint(), line3D.getDirection(), planeNormal, testForLeftSide);
         assertEquals(expected, actual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Plane3D plane = EuclidGeometryRandomTools.nextPlane3D(random);
         Vector3D firstTangent = EuclidCoreRandomTools.nextOrthogonalVector3D(random, plane.getNormal(), true);
         Vector3D secondTangent = new Vector3D();
         secondTangent.cross(firstTangent, plane.getNormal());

         // We start off with the line and query lying on the plane
         Point3D firstPointOnLine = new Point3D();
         firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random), firstTangent, plane.getPoint());
         firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random), secondTangent, firstPointOnLine);
         Point3D secondPointOnLine = new Point3D();
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random), firstTangent, plane.getPoint());
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random), secondTangent, secondPointOnLine);

         Line3D line = new Line3D(firstPointOnLine, secondPointOnLine);

         Vector3D orthogonalToLinePointingLeft = new Vector3D();
         orthogonalToLinePointingLeft.cross(plane.getNormal(), line.getDirection());

         Point3D pointOnLeftSide = new Point3D();
         pointOnLeftSide.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 5.0), line.getDirection(), line.getPoint());
         pointOnLeftSide.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), orthogonalToLinePointingLeft, pointOnLeftSide);

         Point3D pointOnRightSide = new Point3D();
         pointOnRightSide.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 5.0), line.getDirection(), line.getPoint());
         pointOnRightSide.scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), orthogonalToLinePointingLeft, pointOnRightSide);

         assertTrue(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnLeftSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertTrue(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnRightSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertTrue(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnLeftSide, line.getPoint(), line.getDirection(), plane.getNormal()));
         assertTrue(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnRightSide, line.getPoint(), line.getDirection(), plane.getNormal()));

         assertFalse(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnLeftSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertFalse(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnRightSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertFalse(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnLeftSide, line.getPoint(), line.getDirection(), plane.getNormal()));
         assertFalse(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnRightSide, line.getPoint(), line.getDirection(), plane.getNormal()));

         // Now we move the query along the plane's normal, the result should be the same
         pointOnLeftSide.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), plane.getNormal(), pointOnLeftSide);
         pointOnRightSide.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), plane.getNormal(), pointOnRightSide);

         assertTrue(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnLeftSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertTrue(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnRightSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertTrue(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnLeftSide, line.getPoint(), line.getDirection(), plane.getNormal()));
         assertTrue(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnRightSide, line.getPoint(), line.getDirection(), plane.getNormal()));

         assertFalse(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnLeftSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertFalse(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnRightSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertFalse(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnLeftSide, line.getPoint(), line.getDirection(), plane.getNormal()));
         assertFalse(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnRightSide, line.getPoint(), line.getDirection(), plane.getNormal()));

         // Finally we move the 2 points defining along the plane's normal, the result should be the same
         firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), plane.getNormal(), firstPointOnLine);
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), plane.getNormal(), secondPointOnLine);
         line = new Line3D(firstPointOnLine, secondPointOnLine);

         assertTrue(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnLeftSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertTrue(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnRightSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertTrue(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnLeftSide, line.getPoint(), line.getDirection(), plane.getNormal()));
         assertTrue(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnRightSide, line.getPoint(), line.getDirection(), plane.getNormal()));

         assertFalse(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnLeftSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertFalse(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnRightSide, firstPointOnLine, secondPointOnLine, plane.getNormal()));
         assertFalse(EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(pointOnLeftSide, line.getPoint(), line.getDirection(), plane.getNormal()));
         assertFalse(EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(pointOnRightSide, line.getPoint(), line.getDirection(), plane.getNormal()));
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
            boolean isEdgeFaceVisible = edge.getFace().canObserverSeeFace(observer, epsilon);
            boolean isTwinFaceVisible = edge.getTwin().getFace().canObserverSeeFace(observer, epsilon);

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
         DMatrixRMaj matrix = new DMatrixRMaj(new double[][] {
            {0, 1, 1, 1, 1},        // | 0   1     1     1     1   |
            {1, 0, ab2, ac2, ad2},  // | 1   0   d12^2 d13^2 d14^2 |
            {1, ab2, 0, bc2, db2},  // | 1 d21^2   0   d23^2 d24^2 |
            {1, ac2, bc2, 0, cd2},  // | 1 d31^2 d32^2   0   d34^2 |
            {1, ad2, db2, cd2, 0}   // | 1 d41^2 d42^2 d43^2   0   |
            });
         // @formatter:on

         double volume1 = EuclidCoreTools.squareRoot(CommonOps_DDRM.det(matrix) / 288.0);
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
         TriangleMesh3D icosahedron = IcoSphereFactory.newIcoSphere(0);
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

   @Test
   public void testIsConvexPolygonConcyclic()
   {
      Random random = new Random(4365);

      for (int i = 0; i < ITERATIONS; i++)
      { // Generating points to be on circle, the resulting polygon is concyclic.
         int numberOfVertices = random.nextInt(20);
         List<Point3D> concyclicPoints = EuclidShapeRandomTools.nextCircleBasedConvexPolygon3D(random,
                                                                                               10.0,
                                                                                               1.0,
                                                                                               numberOfVertices,
                                                                                               EuclidCoreRandomTools.nextVector3D(random));
         if (random.nextBoolean())
            Collections.reverse(concyclicPoints);

         if (concyclicPoints.isEmpty())
            assertFalse(EuclidPolytopeTools.isConvexPolygon3DConcyclic(concyclicPoints, numberOfVertices, EPSILON), "Iteration " + i);
         else
            assertTrue(EuclidPolytopeTools.isConvexPolygon3DConcyclic(concyclicPoints, numberOfVertices, EPSILON), "Iteration " + i);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing for a polygon that is not concyclic. To do so, we take a concyclic polygon and shift one of its vertices away for the circumcenter so it is not concyclic.
         int numberOfVertices = random.nextInt(16) + 4; // Need at least 4 vertices as anything with less vertices is considered concyclic automatically.
         Point3D circumcenter = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         List<Point3D> notConcyclicPoints = EuclidShapeRandomTools.nextCircleBasedConvexPolygon3D(random,
                                                                                                  circumcenter,
                                                                                                  1.0,
                                                                                                  numberOfVertices,
                                                                                                  EuclidCoreRandomTools.nextVector3D(random));
         if (random.nextBoolean())
            Collections.reverse(notConcyclicPoints);

         Point3D vertexUnmodified = new Point3D(notConcyclicPoints.get(random.nextInt(numberOfVertices)));
         Point3D vertex = notConcyclicPoints.get(random.nextInt(numberOfVertices));

         UnitVector3D directionAway = new UnitVector3D();
         directionAway.sub(vertex, circumcenter);
         double distanceOutside = EuclidCoreRandomTools.nextDouble(random, 2.0 * EPSILON, 1.0);
         vertex.scaleAdd(distanceOutside, directionAway, vertex);
         assertFalse(EuclidPolytopeTools.isConvexPolygon3DConcyclic(notConcyclicPoints, numberOfVertices, EPSILON), "Iteration " + i);

         vertex.set(vertexUnmodified);
         double radius = vertex.distance(circumcenter);
         double distanceInside = EuclidCoreRandomTools.nextDouble(random, 2.0 * EPSILON, radius);
         vertex.scaleAdd(-distanceInside, directionAway, vertex);
         assertFalse(EuclidPolytopeTools.isConvexPolygon3DConcyclic(notConcyclicPoints, numberOfVertices, EPSILON), "Iteration " + i);
      }
   }
}
