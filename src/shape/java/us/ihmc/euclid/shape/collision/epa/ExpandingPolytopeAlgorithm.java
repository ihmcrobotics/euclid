package us.ihmc.euclid.shape.collision.epa;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GJKVertex3D;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class ExpandingPolytopeAlgorithm
{
   private static final boolean VERBOSE = false;

   private double epsilon = 1.0e-12;
   private int numberOfIterations = 0;
   private int maxIterations = 10000;
   private final GilbertJohnsonKeerthiCollisionDetector gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private EPAFace3D lastResult = null;

   public ExpandingPolytopeAlgorithm()
   {
   }

   public EuclidShape3DCollisionResult evaluateCollision(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      evaluateCollision(shapeA, shapeB, result);
      return result;
   }

   public boolean evaluateCollision(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, EuclidShape3DCollisionResult resultToPack)
   {
      boolean areShapesColliding = gjkCollisionDetector.evaluateCollision(shapeA, shapeB, resultToPack);
      if (areShapesColliding && gjkCollisionDetector.getSimplex() != null)
         evaluateCollision(shapeA, shapeB, gjkCollisionDetector.getSimplex().getVertices(), resultToPack);
      return areShapesColliding;
   }

   public boolean evaluateCollision(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, GJKVertex3D[] simplex, EuclidShape3DCollisionResult resultToPack)
   {
      boolean areShapesColliding = evaluateCollision((SupportingVertexHolder) shapeA, (SupportingVertexHolder) shapeB, simplex, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
      return areShapesColliding;
   }

   public EuclidShape3DCollisionResult evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB)
   {
      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      evaluateCollision(shapeA, shapeB, result);
      return result;
   }

   public boolean evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, EuclidShape3DCollisionResultBasics resultToPack)
   {
      boolean areShapesColliding = gjkCollisionDetector.evaluateCollision(shapeA, shapeB, resultToPack);
      if (areShapesColliding && gjkCollisionDetector.getSimplex() != null)
         evaluateCollision(shapeA, shapeB, gjkCollisionDetector.getSimplex().getVertices(), resultToPack);
      return areShapesColliding;
   }

   public boolean evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, GJKVertex3D[] simplex,
                                    EuclidShape3DCollisionResultBasics resultToPack)
   {
      PriorityQueue<EPAFace3D> queue = new PriorityQueue<>();
      double mu = Double.POSITIVE_INFINITY;

      List<EPAFace3D> initialPolytope = EPATools.newEPAPolytopeFromGJKSimplex(shapeA, shapeB, simplex, epsilon);

      if (initialPolytope == null)
      {
         lastResult = null;
         if (VERBOSE)
            System.out.println("Initial polytope has triangle face that is affinely dependent.");
      }
      else
      {
         initialPolytope.stream().forEach(queue::add);
         Vector3D supportDirection = new Vector3D();
         numberOfIterations = 0;

         while (numberOfIterations < maxIterations)
         {
            if (queue.isEmpty())
            {
               if (VERBOSE)
                  System.out.println("Queue is empty, terminating.");
               break;
            }

            EPAFace3D entry = queue.poll();
            if (entry.isObsolete())
               continue;
            double currentNormSquared = entry.getNormSquared();

            numberOfIterations++;

            /*
             * In the original algorithm, the comparison does not add epsilon to mu, but this appears to be
             * needed to handle some edge-cases.
             */
            if (currentNormSquared > mu + epsilon)
            {
               if (VERBOSE)
                  System.out.println("Best norm exceeds upper bound, terminating.");
               break;
            }

            lastResult = entry;

            if (currentNormSquared > epsilon)
               supportDirection.set(entry.getClosestPoint());
            else
               supportDirection.set(entry.getNormal());

            Point3DReadOnly vertexA = shapeA.getSupportingVertex(supportDirection);
            supportDirection.negate();
            Point3DReadOnly vertexB = shapeB.getSupportingVertex(supportDirection);

            EPAVertex3D newVertex = new EPAVertex3D(vertexA, vertexB);

            if (entry.contains(newVertex))
            {
               if (VERBOSE)
                  System.out.println("New vertex equals to a vertex of entry, terminating.");
               break;
            }

            mu = Math.min(mu, EuclidCoreTools.square(TupleTools.dot(newVertex, supportDirection)) / currentNormSquared);

            if (mu <= EuclidCoreTools.square(1.0 + epsilon) * currentNormSquared)
            {
               if (VERBOSE)
                  System.out.println("Reached max accuracy, terminating.");
               break;
            }

            if (!entry.canObserverSeeFace(newVertex))
            {
               if (VERBOSE)
                  System.out.println("Malformed silhouette, terminating.");
               break;
            }

            entry.markObsolete();
            List<EPAEdge3D> silhouette = new ArrayList<>();
            EPATools.silhouette(entry.getEdge0().getTwin(), newVertex, silhouette);
            EPATools.silhouette(entry.getEdge1().getTwin(), newVertex, silhouette);
            EPATools.silhouette(entry.getEdge2().getTwin(), newVertex, silhouette);

            boolean areNewTrianglesFine = true;

            for (EPAEdge3D sentryEdge : silhouette)
            {
               EPAFace3D newEntry = EPAFace3D.fromVertexAndTwinEdge(newVertex, sentryEdge, epsilon);

               if (newEntry.isTriangleAffinelyDependent())
               {
                  if (VERBOSE)
                     System.out.println("New triangle affinely dependent, terminating.");
                  areNewTrianglesFine = false;
                  break;
               }

               /*
                * Same as above, the original check with mu does not include epsilon, but it seems to be needed to
                * handle some edge-cases.
                */
               if (newEntry.isClosestPointInternal() && currentNormSquared <= newEntry.getNormSquared() && newEntry.getNormSquared() <= mu + epsilon)
               {
                  queue.add(newEntry);
               }
            }

            if (!areNewTrianglesFine)
               break;

            boolean terminate = false;

            for (int edgeIndex = 0; edgeIndex < newVertex.getNumberOfAssociatedEdges(); edgeIndex++)
            {
               EPAEdge3D edge = newVertex.getAssociatedEdge(edgeIndex);
               EPAEdge3D twin = edge.getDestination().getEdgeTo(newVertex);

               if (twin == null)
               {
                  if (VERBOSE)
                     System.out.println("Could not find twin of an edge, silhouette probably malformed, terminating.");
                  terminate = true;
                  break;
               }

               edge.setTwin(twin);
            }

            if (terminate)
               break;

            entry.destroy();
         }
      }

      if (initialPolytope == null)
      {
         resultToPack.setToNaN();
      }
      else
      {
         resultToPack.setShapesAreColliding(true);
         resultToPack.setSignedDistance(-lastResult.getNorm());
         lastResult.computePointOnA(resultToPack.getPointOnA());
         lastResult.computePointOnB(resultToPack.getPointOnB());
         resultToPack.getNormalOnA().setToNaN();
         resultToPack.getNormalOnB().setToNaN();
      }

      if (VERBOSE)
         System.out.println("Number of iterations: " + numberOfIterations);

      return true;
   }

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   public void setTerminalConditionEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }

   public double getTerminalConditionEpsilon()
   {
      return epsilon;
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }

   public EPAFace3D getClosestFace()
   {
      return lastResult;
   }
}
