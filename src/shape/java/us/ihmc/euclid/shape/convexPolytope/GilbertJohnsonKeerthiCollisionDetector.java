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
   private static final double defaultCollisionEpsilon = 1.0e-12;

   private static final Point3DReadOnly origin = new Point3D();

   private final int maxIterations = 1000;
   private double epsilon = defaultCollisionEpsilon;

   private SimplexPolytope3D simplex;
   private Vector3D supportVectorDirection = new Vector3D();

   private Vector3DReadOnly supportVectorDirectionNegative = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return -supportVectorDirection.getX();
      }

      @Override
      public double getY()
      {
         return -supportVectorDirection.getY();
      }

      @Override
      public double getZ()
      {
         return -supportVectorDirection.getZ();
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

   private int iterations;

   public boolean doCollisionTest(ConvexPolytope3DReadOnly convexPolytopeA, ConvexPolytope3DReadOnly convexPolytopeB)
   {
      if (convexPolytopeA.isEmpty() || convexPolytopeB.isEmpty())
         return false;

      simplex = new SimplexPolytope3D();
      supportVectorDirection.set(Axis.Y);

      for (iterations = 0; iterations < maxIterations; iterations++)
      {
         Vertex3DReadOnly supportingPolytopeVertexA = convexPolytopeA.getSupportingVertex(supportVectorDirection);
         Vertex3DReadOnly supportingPolytopeVertexB = convexPolytopeB.getSupportingVertex(supportVectorDirectionNegative);
         SimplexVertex3D newVertex = simplex.addVertex(supportingPolytopeVertexA, supportingPolytopeVertexB);

         if (newVertex.dot(supportVectorDirection) < 0.0)
            return false;

         // TODO Inefficient approach here, the simplex is growing with the number of iterations whereas the most complex shape should remain a tetrahedron.
         if (simplex.isPointInside(origin, epsilon))
            return true;

         simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);
      }
      return false;
   }
}
