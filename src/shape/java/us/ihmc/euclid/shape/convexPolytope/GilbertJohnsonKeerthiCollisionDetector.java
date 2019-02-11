package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class GilbertJohnsonKeerthiCollisionDetector
{
   private static final double defaultCollisionEpsilon = 1.0e-10;

   private static final Point3DReadOnly origin = new Point3D();

   private int iterations;
   private final int maxIterations = 1000;
   private double epsilon;
   private boolean latestCollisionTestResult;

   private SimplexPolytope3D simplex;
   private final Vector3D supportDirection = new Vector3D();
   private final Vector3D previousSupportDirection = new Vector3D();
   private final Vector3DReadOnly supportDirectionNegative = EuclidCoreFactories.newNegativeLinkedVector3D(supportDirection);

   private final Vector3D separationVector = new Vector3D();
   private final Point3D closestPointOnA = new Point3D();
   private final Point3D closestPointOnB = new Point3D();
   private boolean isSeparationVectorUpToDate = false;
   private boolean areClosestPointsUpToDate = false;

   public GilbertJohnsonKeerthiCollisionDetector()
   {
      this(defaultCollisionEpsilon);
   }

   public GilbertJohnsonKeerthiCollisionDetector(double epsilon)
   {
      setEpsilon(epsilon);
   }

   public SimplexPolytope3D getSimplex()
   {
      return simplex;
   }

   public void setEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }

   public double getEpsilon()
   {
      return epsilon;
   }

   public boolean doCollisionTest(ConvexPolytope3DReadOnly convexPolytopeA, ConvexPolytope3DReadOnly convexPolytopeB)
   {
      latestCollisionTestResult = false;
      isSeparationVectorUpToDate = false;
      areClosestPointsUpToDate = false;

      if (convexPolytopeA.isEmpty() || convexPolytopeB.isEmpty())
      {
         simplex = null;
         return false;
      }

      simplex = new SimplexPolytope3D();
      supportDirection.set(Axis.Y);

      for (iterations = 0; iterations < maxIterations; iterations++)
      {
         Vertex3DReadOnly supportingVertexA = convexPolytopeA.getSupportingVertex(supportDirection);
         Vertex3DReadOnly supportingVertexB = convexPolytopeB.getSupportingVertex(supportDirectionNegative);
         simplex.addVertex(supportingVertexA, supportingVertexB);

         // TODO Inefficient approach here, the simplex is growing with the number of iterations whereas the most complex shape should remain a tetrahedron.
         if (simplex.isPointInside(origin, epsilon))
         {
            latestCollisionTestResult = true;
            return true;
         }

         simplex.getSupportVectorDirectionTo(origin, supportDirection);

         if (previousSupportDirection.epsilonEquals(supportDirection, epsilon))
            return false;

         previousSupportDirection.set(supportDirection);
      }
      return false;
   }

   public int getIterations()
   {
      return iterations;
   }

   public Vector3DReadOnly getSupportDirection()
   {
      return supportDirection;
   }

   public Vector3DReadOnly getSeparationVector()
   {
      if (simplex == null || latestCollisionTestResult)
         return null;

      if (!isSeparationVectorUpToDate)
      {
         separationVector.set(supportDirection);
         separationVector.normalize();
         separationVector.scale(simplex.getSmallestSimplexMemberReference(origin).distance(origin));
         isSeparationVectorUpToDate = true;
      }

      return separationVector;
   }

   public Point3DReadOnly getClosestPointOnA()
   {
      if (simplex == null || latestCollisionTestResult)
         return null;

      updateClosestPoints();

      return closestPointOnA;
   }

   public Point3DReadOnly getClosestPointOnB()
   {
      if (simplex == null || latestCollisionTestResult)
         return null;

      updateClosestPoints();

      return closestPointOnB;
   }

   private void updateClosestPoints()
   {
      if (areClosestPointsUpToDate)
         return;

      simplex.getCollidingPointsOnSimplex(origin, closestPointOnA, closestPointOnB);
      areClosestPointsUpToDate = true;
   }
}
