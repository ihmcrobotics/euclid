package us.ihmc.euclid.shape.collision;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.function.BiFunction;
import java.util.function.Supplier;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

class GilbertJohnsonKeerthiCollisionDetectorTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-10;

   @Test
   void testSimpleCollisionWithNonCollidingCubeAndTetrahedron()
   {
      Random random = new Random(34534);

      ConvexPolytope3D cube = EuclidPolytopeFactories.newCube(1.0);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D tetrahedronClosest = new Point3D(0.5, 0.0, 0.0);
         Point3D tetrahedronFarthest0 = new Point3D(1.0, 1.0, 0.0);
         Point3D tetrahedronFarthest1 = new Point3D(1.0, 0.0, 1.0);
         Point3D tetrahedronFarthest2 = new Point3D(1.0, -1.0, 0.0);

         Vector3D translation = new Vector3D();
         translation.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)); // Positive => no collision
         translation.setY(EuclidCoreRandomTools.nextDouble(random, 0.5));
         translation.setZ(EuclidCoreRandomTools.nextDouble(random, 0.5));

         Arrays.asList(tetrahedronClosest, tetrahedronFarthest0, tetrahedronFarthest1, tetrahedronFarthest2).forEach(p -> p.add(translation));

         ConvexPolytope3D tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(tetrahedronClosest, tetrahedronFarthest0, tetrahedronFarthest1,
                                                                                                 tetrahedronFarthest2));

         double distance = Double.POSITIVE_INFINITY;

         for (Vertex3D tetraVertex : tetrahedron.getVertices())
         {
            assertFalse(cube.isPointInside(tetraVertex));

            for (Face3D cubeFace : cube.getFaces())
            {
               distance = Math.min(distance, cubeFace.distance(tetraVertex));
            }
         }

         for (Vertex3D cubeVertex : cube.getVertices())
         {
            assertFalse(tetrahedron.isPointInside(cubeVertex));

            for (Face3D tetraFace : tetrahedron.getFaces())
            {
               distance = Math.min(distance, tetraFace.distance(cubeVertex));
            }
         }

         assertEquals(translation.getX(), distance, EPSILON);

         performAssertionsTwoCombinations(cube, tetrahedron, false, new Point3D(0.5, tetrahedronClosest.getY(), tetrahedronClosest.getZ()), tetrahedronClosest);
      }
   }

   @Test
   void testSimpleCollisionWithCollidingCubeAndTetrahedron()
   {
      Random random = new Random(34534);

      ConvexPolytope3D cube = EuclidPolytopeFactories.newCube(1.0);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D tetrahedronClosest = new Point3D(0.5, 0.0, 0.0);
         Point3D tetrahedronFarthest0 = new Point3D(1.0, 1.0, 0.0);
         Point3D tetrahedronFarthest1 = new Point3D(1.0, 0.0, 1.0);
         Point3D tetrahedronFarthest2 = new Point3D(1.0, -1.0, 0.0);

         Vector3D translation = new Vector3D();
         translation.setX(EuclidCoreRandomTools.nextDouble(random, -0.5, 0.0)); // Negative => collision
         translation.setY(EuclidCoreRandomTools.nextDouble(random, 0.5));
         translation.setZ(EuclidCoreRandomTools.nextDouble(random, 0.5));

         Arrays.asList(tetrahedronClosest, tetrahedronFarthest0, tetrahedronFarthest1, tetrahedronFarthest2).forEach(p -> p.add(translation));

         ConvexPolytope3D tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(tetrahedronClosest, tetrahedronFarthest0, tetrahedronFarthest1,
                                                                                                 tetrahedronFarthest2));

         GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
         assertTrue(collisionDetector.doCollisionTest(cube, tetrahedron));
         assertTrue(collisionDetector.doCollisionTest(tetrahedron, cube));

         performAssertionsTwoCombinations(cube, tetrahedron, true, null, null);
      }
   }

   @Test
   void testNonCollidingConvexPolytope3DWithTetrahedron() throws Exception
   {
      Random random = new Random(45345);

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the tetrahedron to have its top vertex closest to a face. 
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            performAssertionsTwoCombinations(convexPolytope3D, EuclidShapeRandomTools.nextConvexPolytope3D(random), false, null, null);
         }
         else
         {
            Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
            HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
            Point3D pointOnConvexPolytope3D = EuclidShapeRandomTools.nextPoint3DInTriangle(random, face.getCentroid(), edge.getOrigin(), edge.getDestination());
            Point3D pointOnTetrahedron = new Point3D(pointOnConvexPolytope3D);
            pointOnTetrahedron.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), face.getNormal(), pointOnTetrahedron);

            ConvexPolytope3D tetrahedron = newTetrahedron(random, pointOnTetrahedron, face.getNormal(), 1.0);
            performAssertionsTwoCombinations(convexPolytope3D, tetrahedron, false, pointOnConvexPolytope3D, pointOnTetrahedron);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the tetrahedron to have its top vertex closest to an edge. 
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            performAssertionsTwoCombinations(convexPolytope3D, EuclidShapeRandomTools.nextConvexPolytope3D(random), false, null, null);
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

            Point3D pointOnConvexPolytope3D = new Point3D();
            pointOnConvexPolytope3D.interpolate(closestEdge.getOrigin(), closestEdge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            Point3D pointOnTetrahedron = new Point3D();
            pointOnTetrahedron.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), towardOutside, pointOnConvexPolytope3D);

            ConvexPolytope3D tetrahedron = newTetrahedron(random, pointOnTetrahedron, towardOutside, 1.0);

            performAssertionsTwoCombinations(convexPolytope3D, tetrahedron, false, pointOnConvexPolytope3D, pointOnTetrahedron);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to a vertex
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            performAssertionsTwoCombinations(convexPolytope3D, EuclidShapeRandomTools.nextConvexPolytope3D(random), false, null, null);
         }
         else
         {
            Vertex3D pointOnConvexPolytope3D = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));

            Vector3D towardOutside = new Vector3D();
            pointOnConvexPolytope3D.getAssociatedEdges().stream()
                                   .forEach(edge -> towardOutside.scaleAdd(random.nextDouble(), edge.getFace().getNormal(), towardOutside));
            towardOutside.normalize();

            Point3D pointOnTetrahedron = new Point3D();
            pointOnTetrahedron.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), towardOutside, pointOnConvexPolytope3D);

            ConvexPolytope3D tetrahedron = newTetrahedron(random, pointOnTetrahedron, towardOutside, 1.0);

            performAssertionsTwoCombinations(convexPolytope3D, tetrahedron, false, pointOnConvexPolytope3D, pointOnTetrahedron);
         }
      }
   }

   @Test
   void testVertexCollidingConvexPolytope3DWithTetrahedron() throws Exception
   {
      Random random = new Random(45345);

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the tetrahedron from its top vertex lying inside the polytope 
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);
         ConvexPolytope3D tetrahedron;

         if (convexPolytope3D.isEmpty())
         {
            performAssertionsTwoCombinations(convexPolytope3D, EuclidShapeRandomTools.nextConvexPolytope3D(random), false, null, null);
         }
         else
         {
            if (convexPolytope3D.getNumberOfVertices() == 1)
            {
               tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, convexPolytope3D.getVertex(0));
               assertTrue(tetrahedron.isPointInside(convexPolytope3D.getVertex(0)));
            }
            else if (convexPolytope3D.getNumberOfVertices() == 2)
            {
               HalfEdge3D edge = convexPolytope3D.getHalfEdge(0);
               Vector3DBasics edgeDirection = edge.getDirection(true);
               Vector3D firstOrthogonalToEdge = EuclidCoreRandomTools.nextOrthogonalVector3D(random, edgeDirection, true);
               Vector3D secondOrthogonalToEdge = new Vector3D();
               secondOrthogonalToEdge.cross(firstOrthogonalToEdge, edgeDirection);

               LineSegment3D firstTetraSegment = new LineSegment3D();
               firstTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
               firstTetraSegment.getSecondEndpoint().scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
               firstTetraSegment.translate(firstOrthogonalToEdge);
               LineSegment3D secondTetraSegment = new LineSegment3D();
               secondTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
               secondTetraSegment.getSecondEndpoint().scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
               firstOrthogonalToEdge.negate();
               secondTetraSegment.translate(firstOrthogonalToEdge);
               tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(firstTetraSegment.getFirstEndpoint(),
                                                                                      firstTetraSegment.getSecondEndpoint(),
                                                                                      secondTetraSegment.getFirstEndpoint(),
                                                                                      secondTetraSegment.getSecondEndpoint()));
            }
            else if (convexPolytope3D.getNumberOfFaces() == 1)
            {
               Face3D face = convexPolytope3D.getFace(0);
               Point3D pointOnFace = EuclidShapeRandomTools.nextPoint3DOnFace3D(random, face);
               tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, pointOnFace);
            }
            else
            {
               Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
               HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
               Point3D pointInside = EuclidShapeRandomTools.nextPoint3DInTetrahedron(random, convexPolytope3D.getCentroid(), face.getCentroid(),
                                                                                     edge.getOrigin(), edge.getDestination());

               tetrahedron = newTetrahedron(random, pointInside, face.getNormal(), 1.0);
            }

            performAssertionsTwoCombinations(convexPolytope3D, tetrahedron, true, null, null);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the tetrahedron from its top vertex: 1- make it lie on an edge, 2- go inside slightly
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);
         ConvexPolytope3D tetrahedron;

         if (convexPolytope3D.isEmpty())
         {
            performAssertionsTwoCombinations(convexPolytope3D, EuclidShapeRandomTools.nextConvexPolytope3D(random), false, null, null);
         }
         else
         {
            if (convexPolytope3D.getNumberOfVertices() == 1)
            {
               tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, convexPolytope3D.getVertex(0));
               assertTrue(tetrahedron.isPointInside(convexPolytope3D.getVertex(0)));
            }
            else if (convexPolytope3D.getNumberOfVertices() == 2)
            {
               HalfEdge3D edge = convexPolytope3D.getHalfEdge(0);
               Vector3DBasics edgeDirection = edge.getDirection(true);
               Vector3D firstOrthogonalToEdge = EuclidCoreRandomTools.nextOrthogonalVector3D(random, edgeDirection, true);
               Vector3D secondOrthogonalToEdge = new Vector3D();
               secondOrthogonalToEdge.cross(firstOrthogonalToEdge, edgeDirection);

               LineSegment3D firstTetraSegment = new LineSegment3D();
               firstTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
               firstTetraSegment.getSecondEndpoint().scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
               firstTetraSegment.translate(firstOrthogonalToEdge);
               LineSegment3D secondTetraSegment = new LineSegment3D();
               secondTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
               secondTetraSegment.getSecondEndpoint().scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
               firstOrthogonalToEdge.negate();
               secondTetraSegment.translate(firstOrthogonalToEdge);
               tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(firstTetraSegment.getFirstEndpoint(),
                                                                                      firstTetraSegment.getSecondEndpoint(),
                                                                                      secondTetraSegment.getFirstEndpoint(),
                                                                                      secondTetraSegment.getSecondEndpoint()));
            }
            else if (convexPolytope3D.getNumberOfFaces() == 1)
            {
               Face3D face = convexPolytope3D.getFace(0);
               Point3D pointOnFace = EuclidShapeRandomTools.nextPoint3DOnFace3D(random, face);
               tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, pointOnFace);
            }
            else
            {
               Face3D firstFace = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
               HalfEdge3D closestEdge = firstFace.getEdge(random.nextInt(firstFace.getNumberOfEdges()));

               Point3D pointOnEdge = new Point3D();
               pointOnEdge.interpolate(closestEdge.getOrigin(), closestEdge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

               Vector3D towardInside = new Vector3D();
               towardInside.sub(convexPolytope3D.getCentroid(), pointOnEdge);
               towardInside.normalize();

               Point3D pointInside = new Point3D();
               double distanceInside = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0e-3);
               pointInside.scaleAdd(distanceInside, towardInside, pointOnEdge);

               tetrahedron = newTetrahedron(random, pointInside, towardInside, 1.0);
            }

            performAssertionsTwoCombinations(convexPolytope3D, tetrahedron, true, null, null);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the tetrahedron from its top vertex: 1- make it to be at a vertex, 2- go inside slightly
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            performAssertionsTwoCombinations(convexPolytope3D, EuclidShapeRandomTools.nextConvexPolytope3D(random), false, null, null);
         }
         else
         {
            ConvexPolytope3D tetrahedron;

            if (convexPolytope3D.getNumberOfVertices() == 1)
            {
               tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, convexPolytope3D.getVertex(0));
               assertTrue(tetrahedron.isPointInside(convexPolytope3D.getVertex(0)));
            }
            else if (convexPolytope3D.getNumberOfVertices() == 2)
            {
               HalfEdge3D edge = convexPolytope3D.getHalfEdge(0);
               Vector3DBasics edgeDirection = edge.getDirection(true);
               Vector3D firstOrthogonalToEdge = EuclidCoreRandomTools.nextOrthogonalVector3D(random, edgeDirection, true);
               Vector3D secondOrthogonalToEdge = new Vector3D();
               secondOrthogonalToEdge.cross(firstOrthogonalToEdge, edgeDirection);

               LineSegment3D firstTetraSegment = new LineSegment3D();
               firstTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
               firstTetraSegment.getSecondEndpoint().scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
               firstTetraSegment.translate(firstOrthogonalToEdge);
               LineSegment3D secondTetraSegment = new LineSegment3D();
               secondTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
               secondTetraSegment.getSecondEndpoint().scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
               firstOrthogonalToEdge.negate();
               secondTetraSegment.translate(firstOrthogonalToEdge);
               tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(firstTetraSegment.getFirstEndpoint(),
                                                                                      firstTetraSegment.getSecondEndpoint(),
                                                                                      secondTetraSegment.getFirstEndpoint(),
                                                                                      secondTetraSegment.getSecondEndpoint()));
            }
            else if (convexPolytope3D.getNumberOfFaces() == 1)
            {
               Face3D face = convexPolytope3D.getFace(0);
               Point3D pointOnFace = EuclidShapeRandomTools.nextPoint3DOnFace3D(random, face);
               tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, pointOnFace);
            }
            else
            {
               Vertex3D closestVertex = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));

               Vector3D towardInside = new Vector3D();
               towardInside.sub(convexPolytope3D.getCentroid(), closestVertex);

               Point3D pointInside = new Point3D();
               double distanceInside = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0e-3);
               pointInside.scaleAdd(distanceInside, towardInside, closestVertex);

               tetrahedron = newTetrahedron(random, pointInside, towardInside, 1.0);
            }

            performAssertionsTwoCombinations(convexPolytope3D, tetrahedron, true, null, null);
         }
      }
   }

   @Test
   void testEdgeCollidingWithIcosahedronAndTetrahedron() throws Exception
   {
      Random random = new Random(54675476);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D icosahedron = EuclidShapeRandomTools.nextIcosahedronBasedConvexPolytope3D(random);

         HalfEdge3D edge = icosahedron.getHalfEdge(random.nextInt(icosahedron.getNumberOfEdges()));

         Point3D pointOnEdge = new Point3D();
         pointOnEdge.interpolate(edge.getOrigin(), edge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Vector3D towardInside = new Vector3D();
         towardInside.sub(icosahedron.getCentroid(), pointOnEdge);
         towardInside.normalize();
         Vector3D towardOutside = new Vector3D();
         towardOutside.setAndNegate(towardInside);

         Vector3D orthogonalToEdge = new Vector3D();
         orthogonalToEdge.cross(edge.getDirection(true), towardOutside);
         orthogonalToEdge.normalize();

         LineSegment3D firstTetrahedronEdge = new LineSegment3D();
         firstTetrahedronEdge.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), orthogonalToEdge, pointOnEdge);
         firstTetrahedronEdge.getSecondEndpoint().scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), orthogonalToEdge, pointOnEdge);
         towardInside.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0e-3));
         firstTetrahedronEdge.translate(towardInside);

         LineSegment3D secondTetrahedronEdge = new LineSegment3D(edge);
         secondTetrahedronEdge.translate(towardOutside);

         ConvexPolytope3D tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(firstTetrahedronEdge.getFirstEndpoint(),
                                                                                                 firstTetrahedronEdge.getSecondEndpoint(),
                                                                                                 secondTetrahedronEdge.getFirstEndpoint(),
                                                                                                 secondTetrahedronEdge.getSecondEndpoint()));

         performAssertionsTwoCombinations(icosahedron, tetrahedron, true, null, null);
      }
   }

   @Test
   void testCollidingLineSegmentAndTriangle() throws Exception
   {
      Random random = new Random(34534);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D triangleA = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D triangleB = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D triangleC = EuclidCoreRandomTools.nextPoint3D(random);
         ConvexPolytope3D triangle = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(triangleA, triangleB, triangleC));

         Point3D pointOnTriangle = EuclidShapeRandomTools.nextPoint3DInTriangle(random, triangleA, triangleB, triangleC);

         Vector3D orthogonalToNormal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, triangle.getFace(0).getNormal(), true);

         Vector3D direction = new Vector3D();
         direction.interpolate(triangle.getFace(0).getNormal(), orthogonalToNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         direction.normalize();

         Point3D segmentStart = new Point3D();
         segmentStart.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction, pointOnTriangle);
         Point3D segmentEnd = new Point3D();
         segmentEnd.scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction, pointOnTriangle);

         ConvexPolytope3D lineSegment = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(segmentStart, segmentEnd));

         performAssertionsTwoCombinations(triangle, lineSegment, true, null, null);
      }
   }

   @Test
   void testCollidingTriangles() throws Exception
   {
      Random random = new Random(34534);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D firstTriangleA = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D firstTriangleB = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D firstTriangleC = EuclidCoreRandomTools.nextPoint3D(random);
         ConvexPolytope3D firstTriangle = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(firstTriangleA, firstTriangleB, firstTriangleC));

         Point3D pointOnTriangle = EuclidShapeRandomTools.nextPoint3DInTriangle(random, firstTriangleA, firstTriangleB, firstTriangleC);

         Vector3D orthogonalToNormal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, firstTriangle.getFace(0).getNormal(), true);

         Vector3D direction = new Vector3D();
         direction.interpolate(firstTriangle.getFace(0).getNormal(), orthogonalToNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         direction.normalize();

         Point3D secondTriangleA = new Point3D();
         Point3D secondTriangleBase = new Point3D();
         secondTriangleA.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction, pointOnTriangle);
         secondTriangleBase.scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), direction, pointOnTriangle);

         Vector3D secondTriangleBaseDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, direction, true);
         Point3D secondTriangleB = new Point3D();
         Point3D secondTriangleC = new Point3D();
         secondTriangleB.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), secondTriangleBaseDirection, secondTriangleBase);
         secondTriangleC.scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), secondTriangleBaseDirection, secondTriangleBase);

         ConvexPolytope3D secondTriangle = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(secondTriangleA, secondTriangleB, secondTriangleC));

         performAssertionsTwoCombinations(firstTriangle, secondTriangle, true, null, null);
      }
   }

   public static ConvexPolytope3D newTetrahedron(Random random, Point3DReadOnly topVertex, Vector3DReadOnly topToBaseCentroid, double baseSize)
   {
      Point3D baseCentroid = new Point3D();
      baseCentroid.add(topVertex, topToBaseCentroid);

      Point3D base0 = new Point3D(baseSize * Math.cos(0.0 * Math.PI / 3.0), baseSize * Math.sin(0.0 * Math.PI / 3.0), 0);
      Point3D base1 = new Point3D(baseSize * Math.cos(1.0 * Math.PI / 3.0), baseSize * Math.sin(1.0 * Math.PI / 3.0), 0);
      Point3D base2 = new Point3D(baseSize * Math.cos(2.0 * Math.PI / 3.0), baseSize * Math.sin(2.0 * Math.PI / 3.0), 0);

      AxisAngle rotation = EuclidGeometryTools.axisAngleFromZUpToVector3D(topToBaseCentroid);
      rotation.transform(base0);
      rotation.transform(base1);
      rotation.transform(base2);

      base0.add(baseCentroid);
      base1.add(baseCentroid);
      base2.add(baseCentroid);

      List<Point3DReadOnly> vertices = new ArrayList<>(Arrays.asList(topVertex, base0, base1, base2));
      Collections.shuffle(vertices, random);

      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices));
   }

   public static void performAssertionsTwoCombinations(ConvexPolytope3DReadOnly polytopeA, ConvexPolytope3DReadOnly polytopeB,
                                                       boolean expectedCollisionTestResult, Point3DReadOnly expectedPointOnA, Point3DReadOnly expectedPointOnB)
   {
      performAssertions(polytopeA, polytopeB, expectedCollisionTestResult, expectedPointOnA, expectedPointOnB);
      performAssertions(polytopeB, polytopeA, expectedCollisionTestResult, expectedPointOnB, expectedPointOnA);
   }

   public static void performAssertions(ConvexPolytope3DReadOnly polytopeA, ConvexPolytope3DReadOnly polytopeB, boolean expectedCollisionTestResult,
                                        Point3DReadOnly expectedPointOnA, Point3DReadOnly expectedPointOnB)
   {
      GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
      boolean actualCollisionTestResult = collisionDetector.doCollisionTest(polytopeA, polytopeB);
      assertEquals(expectedCollisionTestResult, actualCollisionTestResult);

      boolean isOnePolytopeEmpty = polytopeA.isEmpty() || polytopeB.isEmpty();

      if (isOnePolytopeEmpty)
         assertNull(collisionDetector.getSimplex());
      else
         assertNotNull(collisionDetector.getSimplex());

      if (isOnePolytopeEmpty || actualCollisionTestResult)
      {
         assertNull(collisionDetector.getClosestPointOnA());
         assertNull(collisionDetector.getClosestPointOnB());
         assertNull(collisionDetector.getSeparationVector());
      }
      else
      {
         Point3D actualPointOnA = new Point3D(collisionDetector.getClosestPointOnA());
         Point3D actualPointOnB = new Point3D(collisionDetector.getClosestPointOnB());

         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnA, actualPointOnA, 20.0 * EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnB, actualPointOnB, 20.0 * EPSILON);

         Vector3D separationVector = new Vector3D(collisionDetector.getSeparationVector());
         assertEquals(actualPointOnA.distance(actualPointOnB), separationVector.length(), EPSILON);
      }
   }

   @Test
   void testSphere3DToSphere3D() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      /*
       * Interestingly, this test also exercises a LOT the use of the ConvexPolytope3D.constructionEpsilon
       * as GJK will reduce the size of its internal polytope's faces and edges until new vertices are not
       * supposed to be added.
       */
      Random random = new Random(13741);

      double distanceMaxEpsilon = 2.0e-6;
      double positionMaxEpsilon = 1.0e-3;

      double distanceMeanEpsilon = 5.0e-7;
      double positionMeanEpsilon = 5.0e-5;

      AnalyticalShapeCollisionDetection<Sphere3D, Sphere3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         Sphere3D sphereA = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D sphereB = EuclidShapeRandomTools.nextSphere3D(random);
         sphereB.getPosition().add(sphereA.getPosition(),
                                   EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 1.0, 5.0)
                                         * (sphereA.getRadius() + sphereB.getRadius())));
         return new Pair<>(sphereA, sphereB);
      }, EuclidShapeCollisionTools::doSphere3DSphere3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   @Test
   void testPointShape3DBox3DCollisionTest() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-12;
      double positionMaxEpsilon = 1.0e-12;

      double distanceMeanEpsilon = 5.0e-15;
      double positionMeanEpsilon = 5.0e-15;

      AnalyticalShapeCollisionDetection<PointShape3D, Box3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         PointShape3D shapeA = EuclidShapeRandomTools.nextPointShape3D(random);
         Box3D shapeB = EuclidShapeRandomTools.nextBox3D(random);
         Point3D closestPointOnSurface = new Point3D();
         Vector3D normal = new Vector3D();
         boolean colliding = shapeB.doPoint3DCollisionTest(shapeA, closestPointOnSurface, normal);
         if (colliding)
            shapeA.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 4.0), normal, closestPointOnSurface);
         return new Pair<>(shapeA, shapeB);
      }, EuclidShapeCollisionTools::doPointShape3DBox3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   @Test
   void testSphere3DBox3DCollisionTest() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 2.0e-6;
      double positionMaxEpsilon = 2.0e-3;

      double distanceMeanEpsilon = 1.0e-7;
      double positionMeanEpsilon = 5.0e-5;

      AnalyticalShapeCollisionDetection<Sphere3D, Box3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         Sphere3D shapeA = EuclidShapeRandomTools.nextSphere3D(random);
         Box3D shapeB = EuclidShapeRandomTools.nextBox3D(random);
         Point3D closestPointOnSurface = new Point3D();
         Vector3D normal = new Vector3D();
         shapeB.doPoint3DCollisionTest(shapeA.getPosition(), closestPointOnSurface, normal);
         shapeA.getPosition().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 4.0) + shapeA.getRadius(), normal, closestPointOnSurface);
         return new Pair<>(shapeA, shapeB);
      }, EuclidShapeCollisionTools::doSphere3DBox3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   @Test
   void testPointShape3DCapsule3DCollisionTest() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 2.0e-6;
      double positionMaxEpsilon = 7.0e-4;

      double distanceMeanEpsilon = 1.0e-7;
      double positionMeanEpsilon = 5.0e-5;

      AnalyticalShapeCollisionDetection<PointShape3D, Capsule3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         PointShape3D shapeA = EuclidShapeRandomTools.nextPointShape3D(random);
         Capsule3D shapeB = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D closestPointOnSurface = new Point3D();
         Vector3D normal = new Vector3D();
         shapeB.doPoint3DCollisionTest(shapeA, closestPointOnSurface, normal);
         shapeA.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 4.0), normal, closestPointOnSurface);
         return new Pair<>(shapeA, shapeB);
      }, EuclidShapeCollisionTools::doPointShape3DCapsule3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   @Test
   void testCapsule3DCapsule3DCollisionTest() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 2.0e-6;
      double positionMaxEpsilon = 8.0e-4;

      double distanceMeanEpsilon = 5.0e-7;
      double positionMeanEpsilon = 5.0e-5;

      AnalyticalShapeCollisionDetection<Capsule3D, Capsule3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         Capsule3D shapeA = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D shapeB = EuclidShapeRandomTools.nextCapsule3D(random);
         return new Pair<>(shapeA, shapeB);
      }, EuclidShapeCollisionTools::doCapsule3DCapsule3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   @Test
   void testSphere3DCapsule3DCollisionTest() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 2.0e-6;
      double positionMaxEpsilon = 7.0e-4;

      double distanceMeanEpsilon = 2.0e-7;
      double positionMeanEpsilon = 5.0e-5;

      AnalyticalShapeCollisionDetection<Sphere3D, Capsule3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         Sphere3D shapeA = EuclidShapeRandomTools.nextSphere3D(random);
         Capsule3D shapeB = EuclidShapeRandomTools.nextCapsule3D(random);
         Point3D closestPointOnSurface = new Point3D();
         Vector3D normal = new Vector3D();
         shapeB.doPoint3DCollisionTest(shapeA.getPosition(), closestPointOnSurface, normal);
         shapeA.getPosition().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 4.0) + shapeA.getRadius(), normal, closestPointOnSurface);
         return new Pair<>(shapeA, shapeB);
      }, EuclidShapeCollisionTools::doSphere3DCapsule3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   @Test
   void testPointShape3DCylinder3DCollisionTest() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 3.0e-6;
      double positionMaxEpsilon = 6.0e-4;

      double distanceMeanEpsilon = 4.0e-8;
      double positionMeanEpsilon = 2.0e-5;

      AnalyticalShapeCollisionDetection<PointShape3D, Cylinder3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         PointShape3D shapeA = EuclidShapeRandomTools.nextPointShape3D(random);
         Cylinder3D shapeB = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D closestPointOnSurface = new Point3D();
         Vector3D normal = new Vector3D();
         shapeB.doPoint3DCollisionTest(shapeA, closestPointOnSurface, normal);
         shapeA.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 4.0), normal, closestPointOnSurface);
         return new Pair<>(shapeA, shapeB);
      }, EuclidShapeCollisionTools::doPointShape3DCylinder3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   @Test
   void testSphere3DCylinder3DCollisionTest() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 4.0e-6;
      double positionMaxEpsilon = 1.0e-3;

      double distanceMeanEpsilon = 1.0e-7;
      double positionMeanEpsilon = 7.0e-5;

      AnalyticalShapeCollisionDetection<Sphere3D, Cylinder3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         Sphere3D shapeA = EuclidShapeRandomTools.nextSphere3D(random);
         Cylinder3D shapeB = EuclidShapeRandomTools.nextCylinder3D(random);
         Point3D closestPointOnSurface = new Point3D();
         Vector3D normal = new Vector3D();
         shapeB.doPoint3DCollisionTest(shapeA.getPosition(), closestPointOnSurface, normal);
         shapeA.getPosition().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 4.0) + shapeA.getRadius(), normal, closestPointOnSurface);
         return new Pair<>(shapeA, shapeB);
      }, EuclidShapeCollisionTools::doSphere3DCylinder3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   @Test
   void testPointShape3DEllipsoid3DCollisionTest() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 2.0e-6;
      double positionMaxEpsilon = 8.0e-4;

      double distanceMeanEpsilon = 2.0e-7;
      double positionMeanEpsilon = 3.0e-5;

      AnalyticalShapeCollisionDetection<PointShape3D, Ellipsoid3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         PointShape3D shapeA = EuclidShapeRandomTools.nextPointShape3D(random);
         Ellipsoid3D shapeB = EuclidShapeRandomTools.nextEllipsoid3D(random);
         Point3D closestPointOnSurface = new Point3D();
         Vector3D normal = new Vector3D();
         boolean colliding = shapeB.doPoint3DCollisionTest(shapeA, closestPointOnSurface, normal);
         if (colliding)
            shapeA.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 4.0), normal, closestPointOnSurface);
         return new Pair<>(shapeA, shapeB);
      }, EuclidShapeCollisionTools::doPointShape3DEllipsoid3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   @Test
   void testSphere3DEllipsoid3DCollisionTest() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 2.0e-6;
      double positionMaxEpsilon = 1.0e-3;

      double distanceMeanEpsilon = 1.0e-7;
      double positionMeanEpsilon = 5.0e-5;

      AnalyticalShapeCollisionDetection<Sphere3D, Ellipsoid3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         Sphere3D shapeA = EuclidShapeRandomTools.nextSphere3D(random);
         Ellipsoid3D shapeB = EuclidShapeRandomTools.nextEllipsoid3D(random);
         Point3D closestPointOnSurface = new Point3D();
         Vector3D normal = new Vector3D();
         shapeB.doPoint3DCollisionTest(shapeA.getPosition(), closestPointOnSurface, normal);
         shapeA.getPosition().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 4.0) + shapeA.getRadius(), normal, closestPointOnSurface);
         return new Pair<>(shapeA, shapeB);
      }, EuclidShapeCollisionTools::doSphere3DEllipsoid3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   @Test
   void testPointShape3DRamp3DCollisionTest() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-12;
      double positionMaxEpsilon = 1.0e-12;

      double distanceMeanEpsilon = 5.0e-15;
      double positionMeanEpsilon = 5.0e-15;

      AnalyticalShapeCollisionDetection<PointShape3D, Ramp3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         PointShape3D shapeA = EuclidShapeRandomTools.nextPointShape3D(random);
         Ramp3D shapeB = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D closestPointOnSurface = new Point3D();
         Vector3D normal = new Vector3D();
         boolean colliding = shapeB.doPoint3DCollisionTest(shapeA, closestPointOnSurface, normal);
         if (colliding)
            shapeA.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 4.0), normal, closestPointOnSurface);
         return new Pair<>(shapeA, shapeB);
      }, EuclidShapeCollisionTools::doPointShape3DRamp3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   @Test
   void testSphere3DRamp3DCollisionTest() throws Exception
   { // This test confirms that GJK can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 2.0e-6;
      double positionMaxEpsilon = 2.0e-3;

      double distanceMeanEpsilon = 8.0e-8;
      double positionMeanEpsilon = 5.0e-5;

      AnalyticalShapeCollisionDetection<Sphere3D, Ramp3D> function = new AnalyticalShapeCollisionDetection<>(() -> {
         Sphere3D shapeA = EuclidShapeRandomTools.nextSphere3D(random);
         Ramp3D shapeB = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D closestPointOnSurface = new Point3D();
         Vector3D normal = new Vector3D();
         shapeB.doPoint3DCollisionTest(shapeA.getPosition(), closestPointOnSurface, normal);
         shapeA.getPosition().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 4.0) + shapeA.getRadius(), normal, closestPointOnSurface);
         return new Pair<>(shapeA, shapeB);
      }, EuclidShapeCollisionTools::doSphere3DRamp3DCollisionTest);

      assertAgainstAnalyticalFunction(function, distanceMaxEpsilon, positionMaxEpsilon, distanceMeanEpsilon, positionMeanEpsilon);
   }

   private static <A extends Shape3DReadOnly, B extends Shape3DReadOnly> void assertAgainstAnalyticalFunction(AnalyticalShapeCollisionDetection<A, B> function,
                                                                                                              double distanceMaxEpsilon,
                                                                                                              double positionMaxEpsilon,
                                                                                                              double distanceMeanEpsilon,
                                                                                                              double positionMeanEpsilon)
   {
      boolean verbose = true;

      double meanDistanceError = 0.0;
      double meanPositionError = 0.0;

      int numberOfNonCollidingSamples = 0;

      for (int i = 0; i < ITERATIONS; i++)
      {
         Pair<A, B> shapes = function.shapeSupplier.get();
         A shapeA = shapes.a;
         B shapeB = shapes.b;

         Shape3DCollisionTestResult expectedResult = function.collisionFunction.apply(shapeA, shapeB);
         Shape3DCollisionTestResult gjkResult = new Shape3DCollisionTestResult();

         GilbertJohnsonKeerthiCollisionDetector gjkDetector = new GilbertJohnsonKeerthiCollisionDetector();
         gjkDetector.setSimplexConstructionEpsilon(1.0e-6);
         gjkDetector.doShapeCollisionTest(shapeA, shapeB, gjkResult);

         if (verbose && (i % 5000) == 0)
         {
            System.out.println("Iteration #" + i + " Analytical: " + expectedResult.getDistance() + ", GJK: " + gjkResult.getDistance() + ", diff: "
                  + Math.abs(expectedResult.getDistance() - gjkResult.getDistance()));
         }

         // Asserts the internal sanity of the collision result
         assertEquals(gjkDetector.getSimplex().getPolytope().signedDistance(new Point3D()) <= 0.0, gjkResult.areShapesColliding());

         assertEquals(expectedResult.areShapesColliding(), gjkResult.areShapesColliding());

         if (gjkResult.areShapesColliding())
         {
            assertTrue(gjkResult.containsNaN());
            assertTrue(gjkResult.getPointOnA().containsNaN());
            assertTrue(gjkResult.getPointOnB().containsNaN());
            assertTrue(Double.isNaN(gjkResult.getDistance()));
         }
         else
         {
            assertEquals(expectedResult.getDistance(), gjkResult.getDistance(), distanceMaxEpsilon,
                         "difference: " + Math.abs(expectedResult.getDistance() - gjkResult.getDistance()));
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedResult.getPointOnA(), gjkResult.getPointOnA(), positionMaxEpsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedResult.getPointOnB(), gjkResult.getPointOnB(), positionMaxEpsilon);

            numberOfNonCollidingSamples++;
            meanDistanceError += Math.abs(expectedResult.getDistance() - gjkResult.getDistance());
            meanPositionError += expectedResult.getPointOnA().distance(gjkResult.getPointOnA());
            meanPositionError += expectedResult.getPointOnB().distance(gjkResult.getPointOnB());

         }

         // GJK does not estimate either the depth (collision case not covered) nor the normal on each shape.
         assertTrue(gjkResult.getNormalOnA().containsNaN());
         assertTrue(gjkResult.getNormalOnB().containsNaN());
      }

      meanDistanceError /= numberOfNonCollidingSamples;
      meanPositionError /= 2.0 * numberOfNonCollidingSamples;

      if (verbose)
      {
         System.out.println("Average error for the distance: " + meanDistanceError + ", position: " + meanPositionError);
         System.out.println("Number of iterations: " + ITERATIONS + ", number of non-colliding samples: " + numberOfNonCollidingSamples);
      }

      assertTrue(meanDistanceError < distanceMeanEpsilon, "mean distance error: " + meanDistanceError + " expected less than: " + distanceMeanEpsilon);
      assertTrue(meanPositionError < positionMeanEpsilon, "mean position error: " + meanPositionError + " expected less than: " + positionMeanEpsilon);
   }

   static class AnalyticalShapeCollisionDetection<A extends Shape3DReadOnly, B extends Shape3DReadOnly>
   {
      final Supplier<Pair<A, B>> shapeSupplier;
      final BiFunction<A, B, Shape3DCollisionTestResult> collisionFunction;

      public AnalyticalShapeCollisionDetection(Supplier<Pair<A, B>> shapeSupplier, TriConsumer<A, B, Shape3DCollisionTestResult> collisionFunction)
      {
         this(shapeSupplier, toBiFunction(collisionFunction));
      }

      public AnalyticalShapeCollisionDetection(Supplier<Pair<A, B>> shapeSupplier, BiFunction<A, B, Shape3DCollisionTestResult> collisionFunction)
      {
         this.shapeSupplier = shapeSupplier;
         this.collisionFunction = collisionFunction;
      }

      private static <A extends Shape3DReadOnly, B extends Shape3DReadOnly> BiFunction<A, B, Shape3DCollisionTestResult> toBiFunction(TriConsumer<A, B, Shape3DCollisionTestResult> triConsumer)
      {
         return (t, u) -> {
            Shape3DCollisionTestResult result = new Shape3DCollisionTestResult();
            triConsumer.accept(t, u, result);
            return result;
         };
      }

   }

   static class Pair<A, B>
   {
      final A a;
      final B b;

      public Pair(A a, B b)
      {
         this.a = a;
         this.b = b;
      }
   }

   static interface TriConsumer<T, U, V>
   {
      void accept(T t, U u, V v);
   }
}
