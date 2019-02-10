package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
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

   private SimplexPolytope3D simplex;
   private Vector3D supportDirection = new Vector3D();
   private Vector3D previousSupportDirection = new Vector3D();

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
            return true;

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
}
