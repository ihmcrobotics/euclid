package us.ihmc.euclid.shape.collision.gjk;

import static us.ihmc.euclid.shape.collision.gjk.GJKTools.*;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

// From "Improving the GJK algorithm for faster and more reliable distance queries between convex
// objects."
// TODO Make the initial supportDirection be settable to reduce number of iterations when possible.
public class GilbertJohnsonKeerthiCollisionDetector
{
   private static final boolean VERBOSE = false;
   private double epsilon = 1.0e-16;
   private double epsilonTriangleNormalSwitch = 1.0e-6;
   private int maxIterations = 500;
   private GJKSimplex3D simplex = null;
   private int numberOfIterations = 0;
   private final Vector3D supportDirection = new Vector3D();
   private final Vector3D initialSupportDirection = new Vector3D(Axis.Y);

   public GilbertJohnsonKeerthiCollisionDetector()
   {
   }

   public EuclidShape3DCollisionResult evaluateCollision(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      evaluateCollision(shapeA, shapeB, result);
      return result;
   }

   public void evaluateCollision(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, EuclidShape3DCollisionResult resultToPack)
   {
      evaluateCollision((SupportingVertexHolder) shapeA, (SupportingVertexHolder) shapeB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
   }

   public EuclidShape3DCollisionResult evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB)
   {
      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      evaluateCollision(shapeA, shapeB, result);
      return result;
   }

   public boolean evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, EuclidShape3DCollisionResultBasics resultToPack)
   {
      GJKSimplex3D previousOutput = new GJKSimplex3D();

      supportDirection.set(initialSupportDirection);
      Point3DReadOnly vertexA = shapeA.getSupportingVertex(supportDirection);
      supportDirection.negate();
      Point3DReadOnly vertexB = shapeB.getSupportingVertex(supportDirection);

      boolean areColliding = false;

      if (vertexA == null || vertexB == null)
      {
         simplex = null;
         areColliding = false;
      }
      else
      {
         double closestPointNormSquared = 1.0;

         for (int i = 0; i < maxIterations; i++)
         {
            numberOfIterations = i;
            GJKVertex3D newVertex = new GJKVertex3D(vertexA, vertexB);

            if (previousOutput.contains(newVertex))
            {
               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("New vertex belongs to simplex, terminating.");
               break;
            }

            if (Math.abs(closestPointNormSquared - newVertex.dot(supportDirection)) <= epsilon * closestPointNormSquared)
            {
               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("Progression under tolerance, terminating.");
               break;
            }

            GJKSimplex3D output = simplexClosestToOrigin(previousOutput.getVertices(), newVertex);

            if (output == null)
            { // End of process
               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("Closest simplex is null, terminating.");
               break;
            }

            if (output.getDistanceSquaredToOrigin() >= previousOutput.getDistanceSquaredToOrigin())
            {
               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("No progression, terminating.");
               break;
            }

            closestPointNormSquared = output.getDistanceSquaredToOrigin();

            if (output.getNumberOfVertices() == 4 || closestPointNormSquared <= epsilon * output.getMaxDistanceSquaredToOrigin())
            { // End of process
               simplex = output;
               areColliding = true;
               if (VERBOSE)
                  System.out.println("Collision detected, terminating.");
               break;
            }

            if (closestPointNormSquared < epsilonTriangleNormalSwitch && output.getNumberOfVertices() == 3)
               supportDirection.set(output.getTriangleNormal());
            else
               supportDirection.setAndNegate(output.getClosestPointToOrigin());
            vertexA = shapeA.getSupportingVertex(supportDirection);
            supportDirection.negate();
            vertexB = shapeB.getSupportingVertex(supportDirection);

            previousOutput = output;
         }
      }

      if (areColliding)
      {
         resultToPack.setToNaN();
         resultToPack.setShapesAreColliding(areColliding);
      }
      else if (simplex == null)
      {
         resultToPack.setToNaN();
      }
      else
      {
         resultToPack.setSignedDistance(simplex.getDistanceToOrigin());
         resultToPack.setShapeA(null);
         resultToPack.setShapeB(null);
         simplex.computePointOnA(resultToPack.getPointOnA());
         simplex.computePointOnB(resultToPack.getPointOnB());
         resultToPack.getNormalOnA().setToNaN();
         resultToPack.getNormalOnB().setToNaN();
      }

      if (VERBOSE)
         System.out.println("Number of iterations: " + numberOfIterations);

      return areColliding;
   }

   public void setInitialSupportDirection(Vector3DReadOnly initialSupportDirection)
   {
      this.initialSupportDirection.set(initialSupportDirection);
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

   public GJKSimplex3D getSimplex()
   {
      return simplex;
   }

   public GJKVertex3D[] getSimplexVertices()
   {
      if (simplex == null)
         return null;
      else
         return simplex.getVertices();
   }
}
