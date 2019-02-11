package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class ExpandingPolytopeAlgorithm
{
   private static final double defaultCollisionEpsilon = 1.0e-10;

   private static final Point3DReadOnly origin = new Point3D();

   private int iterations;
   private final int maxIterations = 1000;
   private double epsilon;

   private SimplexPolytope3D simplex;
   private final Vector3D supportDirection = new Vector3D();
   private final Vector3D previousSupportDirection = new Vector3D();
   private final Vector3DReadOnly supportDirectionNegative = EuclidCoreFactories.newNegativeLinkedVector3D(supportDirection);
   private final GilbertJohnsonKeerthiCollisionDetector gjkCollisionDetector;

   private final Vector3D collisionVector = new Vector3D();
   private final Point3D pointOnA = new Point3D();
   private final Point3D pointOnB = new Point3D();
   private boolean isCollisionVectorUpToDate = false;
   private boolean areCollisionPointsUpToDate = false;

   public ExpandingPolytopeAlgorithm()
   {
      this(defaultCollisionEpsilon);
   }

   public ExpandingPolytopeAlgorithm(double epsilon)
   {
      gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector(epsilon);
      setEpsilon(epsilon);
   }

   public void setEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }

   public double getEpsilon()
   {
      return epsilon;
   }

   public void doCollisionTest(ConvexPolytope3DReadOnly convexPolytopeA, ConvexPolytope3DReadOnly convexPolytopeB)
   {
      isCollisionVectorUpToDate = false;
      areCollisionPointsUpToDate = false;

      if (convexPolytopeA.isEmpty() || convexPolytopeB.isEmpty())
      {
         this.simplex = null;
         return;
      }

      gjkCollisionDetector.setEpsilon(epsilon);
      boolean areShapesColliding = gjkCollisionDetector.doCollisionTest(convexPolytopeA, convexPolytopeB);
      simplex = gjkCollisionDetector.getSimplex();
      supportDirection.set(gjkCollisionDetector.getSupportDirection());

      if (!areShapesColliding)
         return;

      previousSupportDirection.setToNaN();

      for (iterations = 0; iterations < maxIterations; iterations++)
      {
         simplex.getSupportVectorDirectionTo(origin, supportDirection);
         // We need to negate the support direction to point toward the outside of the simplex and thus force the expansion.
         supportDirection.negate();

         if (supportDirection.epsilonEquals(previousSupportDirection, epsilon))
            break;

         Vertex3DReadOnly supportingVertexA = convexPolytopeA.getSupportingVertex(supportDirection);
         Vertex3DReadOnly supportingVertexB = convexPolytopeB.getSupportingVertex(supportDirectionNegative);
         simplex.addVertex(supportingVertexA, supportingVertexB);

         previousSupportDirection.set(supportDirection);
      }
   }

   public Vector3DReadOnly getCollisionVector()
   {
      if (simplex == null)
         return null;

      if (!isCollisionVectorUpToDate)
      {
         collisionVector.set(supportDirection);
         collisionVector.normalize();
         collisionVector.scale(simplex.getSmallestSimplexMemberReference(origin).distance(origin));
         isCollisionVectorUpToDate = true;
      }

      return collisionVector;
   }

   public Point3DReadOnly getCollisionPointOnA()
   {
      if (simplex == null)
         return null;

      updatePoints();

      return pointOnA;
   }

   public Point3DReadOnly getCollisionPointOnB()
   {
      if (simplex == null)
         return null;

      updatePoints();

      return pointOnB;
   }

   private void updatePoints()
   {
      if (areCollisionPointsUpToDate)
         return;

      simplex.getCollidingPointsOnSimplex(origin, pointOnA, pointOnB);
      areCollisionPointsUpToDate = true;
   }

   public int getIterations()
   {
      return iterations;
   }
}
