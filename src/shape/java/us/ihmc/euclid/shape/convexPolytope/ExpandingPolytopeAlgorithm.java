package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class ExpandingPolytopeAlgorithm
{
   private static final double defaultCollisionEpsilon = 1.0e-10;

   private static final Point3DReadOnly origin = new Point3D();

   private int iterations;
   private final int maxIterations = 1000;
   private double epsilon;
   private SimplexPolytope3D simplex;
   private Vector3D supportDirection = new Vector3D();
   private Vector3D previousSupportDirection = new Vector3D();
   private GilbertJohnsonKeerthiCollisionDetector gjkCollisionDetector;

   private Vector3DReadOnly supportDirectionNegative = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return -supportDirection.getX();
      }

      @Override
      public double getY()
      {
         return -supportDirection.getY();
      }

      @Override
      public double getZ()
      {
         return -supportDirection.getZ();
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getTuple3DString(this);
      }
   };

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

   public void runEPAExpansion(ConvexPolytope3DReadOnly convexPolytopeA, ConvexPolytope3DReadOnly convexPolytopeB)
   {
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

         supportDirection.normalize();

         if (supportDirection.epsilonEquals(previousSupportDirection, epsilon))
            break;

         Vertex3DReadOnly supportingVertexA = convexPolytopeA.getSupportingVertex(supportDirection);
         Vertex3DReadOnly supportingVertexB = convexPolytopeB.getSupportingVertex(supportDirectionNegative);
         simplex.addVertex(supportingVertexA, supportingVertexB);

         previousSupportDirection.set(supportDirection);
      }
      System.out.println(iterations);
   }

   public boolean getCollisionVector(Vector3DBasics collisionVectorToPack)
   {
      if (simplex == null)
         return false;

      collisionVectorToPack.set(supportDirection);
      collisionVectorToPack.normalize();
      collisionVectorToPack.scale(simplex.getSmallestSimplexMemberReference(origin).distance(origin));
      return true;
   }

   public boolean getCollisionPoints(Point3DBasics pointOnAToPack, Point3DBasics pointOnBToPack)
   {
      if (simplex == null)
         return false;

      simplex.getCollidingPointsOnSimplex(origin, pointOnAToPack, pointOnBToPack);
      return true;
   }

   public int getIterations()
   {
      return iterations;
   }
}
