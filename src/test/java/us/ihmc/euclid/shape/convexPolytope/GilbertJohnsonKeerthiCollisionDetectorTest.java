package us.ihmc.euclid.shape.convexPolytope;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

class GilbertJohnsonKeerthiCollisionDetectorTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

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

         GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
         assertFalse(collisionDetector.doCollisionTest(cube, tetrahedron));
         assertFalse(collisionDetector.doCollisionTest(tetrahedron, cube));
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
      }
   }

   @Test
   void testNonCollidingConvexPolytope3DWithTetrahedron() throws Exception
   {
      Random random = new Random(45345);

      double iterationsMean = 0.0;
      double iterationsMax = 0.0;

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the tetrahedron to have its top vertex closest to a face. 
         ConvexPolytope3D convexPolytope3D = EuclidPolytopeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertFalse(collisionDetector.doCollisionTest(convexPolytope3D, EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random)));
            assertFalse(collisionDetector.doCollisionTest(EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random), convexPolytope3D));
         }
         else
         {
            Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
            HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
            Point3D pointOutside = EuclidShapeRandomTools.nextPoint3DInTriangle(random, face.getCentroid(), edge.getOrigin(), edge.getDestination());
            pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), face.getNormal(), pointOutside);

            ConvexPolytope3D tetrahedron = newTetrahedron(random, pointOutside, face.getNormal(), 1.0);

            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertFalse(collisionDetector.doCollisionTest(convexPolytope3D, tetrahedron));
            assertFalse(collisionDetector.doCollisionTest(tetrahedron, convexPolytope3D));
            iterationsMean += (double) collisionDetector.getIterations() / (double) ITERATIONS;
            iterationsMax = Math.max(iterationsMax, collisionDetector.getIterations());
         }
      }

      iterationsMean = 0.0;
      iterationsMax = 0.0;
      System.out.println("Iterations on average: " + iterationsMean + ", max number of iterations: " + iterationsMax);

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the tetrahedron to have its top vertex closest to an edge. 
         ConvexPolytope3D convexPolytope3D = EuclidPolytopeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertFalse(collisionDetector.doCollisionTest(convexPolytope3D, EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random)));
            assertFalse(collisionDetector.doCollisionTest(EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random), convexPolytope3D));
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

            ConvexPolytope3D tetrahedron = newTetrahedron(random, pointOutside, towardOutside, 1.0);

            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertFalse(collisionDetector.doCollisionTest(convexPolytope3D, tetrahedron));
            assertFalse(collisionDetector.doCollisionTest(tetrahedron, convexPolytope3D));
            iterationsMean += (double) collisionDetector.getIterations() / (double) ITERATIONS;
            iterationsMax = Math.max(iterationsMax, collisionDetector.getIterations());
         }
      }

      iterationsMean = 0.0;
      iterationsMax = 0.0;
      System.out.println("Iterations on average: " + iterationsMean + ", max number of iterations: " + iterationsMax);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to a vertex
         ConvexPolytope3D convexPolytope3D = EuclidPolytopeRandomTools.nextConvexPolytope3DWithEdgeCases(random);

         if (convexPolytope3D.isEmpty())
         {
            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertFalse(collisionDetector.doCollisionTest(convexPolytope3D, EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random)));
            assertFalse(collisionDetector.doCollisionTest(EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random), convexPolytope3D));
         }
         else
         {
            Vertex3D closestVertex = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));

            Vector3D towardOutside = new Vector3D();
            closestVertex.getAssociatedEdges().stream().forEach(edge -> towardOutside.scaleAdd(random.nextDouble(), edge.getFace().getNormal(), towardOutside));
            towardOutside.normalize();

            Point3D pointOutside = new Point3D();
            pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), towardOutside, closestVertex);

            ConvexPolytope3D tetrahedron = newTetrahedron(random, pointOutside, towardOutside, 1.0);

            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertFalse(collisionDetector.doCollisionTest(convexPolytope3D, tetrahedron));
            assertFalse(collisionDetector.doCollisionTest(tetrahedron, convexPolytope3D));
            iterationsMean += (double) collisionDetector.getIterations() / (double) ITERATIONS;
            iterationsMax = Math.max(iterationsMax, collisionDetector.getIterations());
         }
      }

      System.out.println("Iterations on average: " + iterationsMean + ", max number of iterations: " + iterationsMax);
   }

   @Test
   void testVertexCollidingConvexPolytope3DWithTetrahedron() throws Exception
   {
      Random random = new Random(45345);

      double iterationsMean = 0.0;
      double iterationsMax = 0.0;

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the tetrahedron from its top vertex lying inside the polytope 
         ConvexPolytope3D convexPolytope3D = EuclidPolytopeRandomTools.nextConvexPolytope3DWithEdgeCases(random);
         ConvexPolytope3D tetrahedron;

         if (convexPolytope3D.isEmpty())
         {
            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertFalse(collisionDetector.doCollisionTest(convexPolytope3D, EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random)));
            assertFalse(collisionDetector.doCollisionTest(EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random), convexPolytope3D));
         }
         else
         {
            if (convexPolytope3D.getNumberOfVertices() == 1)
            {
               tetrahedron = EuclidPolytopeRandomTools.nextTetrahedronContainingPoint3D(random, convexPolytope3D.getVertex(0));
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
               Point3D pointOnFace = EuclidPolytopeRandomTools.nextPoint3DOnFace3D(random, face);
               tetrahedron = EuclidPolytopeRandomTools.nextTetrahedronContainingPoint3D(random, pointOnFace);
            }
            else
            {
               Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
               HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
               Point3D pointInside = EuclidShapeRandomTools.nextPoint3DInTetrahedron(random, convexPolytope3D.getCentroid(), face.getCentroid(),
                                                                                     edge.getOrigin(), edge.getDestination());

               tetrahedron = newTetrahedron(random, pointInside, face.getNormal(), 1.0);
            }

            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertTrue(collisionDetector.doCollisionTest(convexPolytope3D, tetrahedron));
            assertTrue(collisionDetector.doCollisionTest(tetrahedron, convexPolytope3D));
            iterationsMean += (double) collisionDetector.getIterations() / (double) ITERATIONS;
            iterationsMax = Math.max(iterationsMax, collisionDetector.getIterations());
         }
      }

      iterationsMean = 0.0;
      iterationsMax = 0.0;
      System.out.println("Iterations on average: " + iterationsMean + ", max number of iterations: " + iterationsMax);

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the tetrahedron from its top vertex: 1- make it lie on an edge, 2- go inside slightly
         ConvexPolytope3D convexPolytope3D = EuclidPolytopeRandomTools.nextConvexPolytope3DWithEdgeCases(random);
         ConvexPolytope3D tetrahedron;

         if (convexPolytope3D.isEmpty())
         {
            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertFalse(collisionDetector.doCollisionTest(convexPolytope3D, EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random)));
            assertFalse(collisionDetector.doCollisionTest(EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random), convexPolytope3D));
         }
         else
         {
            if (convexPolytope3D.getNumberOfVertices() == 1)
            {
               tetrahedron = EuclidPolytopeRandomTools.nextTetrahedronContainingPoint3D(random, convexPolytope3D.getVertex(0));
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
               Point3D pointOnFace = EuclidPolytopeRandomTools.nextPoint3DOnFace3D(random, face);
               tetrahedron = EuclidPolytopeRandomTools.nextTetrahedronContainingPoint3D(random, pointOnFace);
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

            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertTrue(collisionDetector.doCollisionTest(convexPolytope3D, tetrahedron));
            assertTrue(collisionDetector.doCollisionTest(tetrahedron, convexPolytope3D));
            iterationsMean += (double) collisionDetector.getIterations() / (double) ITERATIONS;
            iterationsMax = Math.max(iterationsMax, collisionDetector.getIterations());
         }
      }

      iterationsMean = 0.0;
      iterationsMax = 0.0;
      System.out.println("Iterations on average: " + iterationsMean + ", max number of iterations: " + iterationsMax);

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the tetrahedron from its top vertex: 1- make it to be at a vertex, 2- go inside slightly
         ConvexPolytope3D convexPolytope3D = EuclidPolytopeRandomTools.nextConvexPolytope3DWithEdgeCases(random);
         ConvexPolytope3D tetrahedron;

         if (convexPolytope3D.isEmpty())
         {
            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertFalse(collisionDetector.doCollisionTest(convexPolytope3D, EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random)));
            assertFalse(collisionDetector.doCollisionTest(EuclidPolytopeRandomTools.nextConeConvexPolytope3D(random), convexPolytope3D));
         }
         else
         {
            if (convexPolytope3D.getNumberOfVertices() == 1)
            {
               tetrahedron = EuclidPolytopeRandomTools.nextTetrahedronContainingPoint3D(random, convexPolytope3D.getVertex(0));
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
               Point3D pointOnFace = EuclidPolytopeRandomTools.nextPoint3DOnFace3D(random, face);
               tetrahedron = EuclidPolytopeRandomTools.nextTetrahedronContainingPoint3D(random, pointOnFace);
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

            GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
            assertTrue(collisionDetector.doCollisionTest(convexPolytope3D, tetrahedron));
            assertTrue(collisionDetector.doCollisionTest(tetrahedron, convexPolytope3D));
            iterationsMean += (double) collisionDetector.getIterations() / (double) ITERATIONS;
            iterationsMax = Math.max(iterationsMax, collisionDetector.getIterations());
         }
      }

      System.out.println("Iterations on average: " + iterationsMean + ", max number of iterations: " + iterationsMax);
   }

   @Test
   void testEdgeCollidingWithIcosahedronAndTetrahedron() throws Exception
   {
      Random random = new Random(54675476);

      double iterationsMean = 0.0;
      double iterationsMax = 0.0;

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D icosahedron = EuclidPolytopeRandomTools.nextIcosahedronBasedConvexPolytope3D(random);

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

         GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
         assertTrue(collisionDetector.doCollisionTest(icosahedron, tetrahedron));
         assertTrue(collisionDetector.doCollisionTest(tetrahedron, icosahedron));
         iterationsMean += (double) collisionDetector.getIterations() / (double) ITERATIONS;
         iterationsMax = Math.max(iterationsMax, collisionDetector.getIterations());
      }

      System.out.println("Iterations on average: " + iterationsMean + ", max number of iterations: " + iterationsMax);
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

         GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
         assertTrue(collisionDetector.doCollisionTest(triangle, lineSegment));
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

         GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
         assertTrue(collisionDetector.doCollisionTest(firstTriangle, secondTriangle));
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
}
