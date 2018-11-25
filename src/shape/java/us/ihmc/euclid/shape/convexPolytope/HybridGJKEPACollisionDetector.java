package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytopeReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.PolytopeListener;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Combines the GJK and EPA collision detection into a single class for improving the computational
 * speed Unlike traditional GJK / EPA this leverages the ability of the polytope classes to
 * automatically update their internal structures when adding new vertices TODO allow for the
 * simplices to be updated instead of recomputed each iteration. This needs the simplex to be able
 * to remove vertices to maintain convexity
 * 
 * @author Apoorv S
 *
 */
public class HybridGJKEPACollisionDetector
{
   private static final double defaultCollisionEpsilon = 1.0e-12;

   private static final Point3D origin = new Point3D();

   private double epsilon;
   private ExtendedSimplexPolytope simplex;
   private ConvexPolytopeReadOnly polytopeA;
   private ConvexPolytopeReadOnly polytopeB;
   private Vector3D supportVectorDirectionNegative = new Vector3D();
   private Vector3D supportVectorDirection = new Vector3D()
   {
      @Override
      public final void setX(double x)
      {
         super.setX(x);
         supportVectorDirectionNegative.setX(-x);
      };

      @Override
      public final void setY(double y)
      {
         super.setY(y);
         supportVectorDirectionNegative.setY(-y);
      };

      @Override
      public final void setZ(double z)
      {
         super.setZ(z);
         supportVectorDirectionNegative.setZ(-z);
      };
   };

   private Vector3D previousSupportVectorDirection = new Vector3D();
   private final int iterations = 1000;
   private final PolytopeListener listener;

   public void setSupportVectorDirection(Vector3DReadOnly vectorToSet)
   {
      supportVectorDirection.set(vectorToSet);
   }

   public void getSupportVectorDirection(Vector3D vectorToPack)
   {
      vectorToPack.set(supportVectorDirection);
   }

   public void getSupportVectorDirectionNegative(Vector3D vectorToPack)
   {
      vectorToPack.set(supportVectorDirectionNegative);
   }

   public ExtendedConvexPolytope getSimplex()
   {
      return simplex.getPolytope();
   }

   public void setSimplex(ExtendedSimplexPolytope simplex)
   {
      this.simplex = simplex;
      if (listener != null)
         listener.attachPolytope(getSimplex());
   }

   public void setPolytopeA(ConvexPolytopeReadOnly polytopeA)
   {
      this.polytopeA = polytopeA;
   }

   public void setPolytopeB(ConvexPolytopeReadOnly polytopeB)
   {
      this.polytopeB = polytopeB;
   }

   public void setEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }

   public double getEpsilon()
   {
      return epsilon;
   }

   public HybridGJKEPACollisionDetector(PolytopeListener listener)
   {
      this(null, defaultCollisionEpsilon, listener);
   }

   public HybridGJKEPACollisionDetector()
   {
      this(null, defaultCollisionEpsilon);
   }

   public HybridGJKEPACollisionDetector(double epsilon)
   {
      this(null, epsilon);
   }

   public HybridGJKEPACollisionDetector(ExtendedSimplexPolytope simplex)
   {
      this(simplex, defaultCollisionEpsilon);
   }

   public HybridGJKEPACollisionDetector(ExtendedSimplexPolytope simplex, double epsilon)
   {
      this(simplex, epsilon, null);
   }

   public HybridGJKEPACollisionDetector(ExtendedSimplexPolytope simplex, double epsilon, PolytopeListener listener)
   {
      setSimplex(simplex);
      setEpsilon(epsilon);
      this.listener = listener;
   }

   public boolean checkCollision()
   {
      if (polytopeA.isEmpty() || polytopeB.isEmpty())
      {
         return false;
      }

      if (simplex.isEmpty())
         supportVectorDirection.set(Axis.Y);
      else
         simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);
      previousSupportVectorDirection.set(supportVectorDirection);

      for (int i = 0; i < iterations; i++)
      {
         updateListeners();

         Vertex3DReadOnly supportingPolytopeVertexA = polytopeA.getSupportingPolytopeVertex(supportVectorDirection);
         Vertex3DReadOnly supportingPolytopeVertexB = polytopeB.getSupportingPolytopeVertex(supportVectorDirectionNegative);
         simplex.addVertex(supportingPolytopeVertexA, supportingPolytopeVertexB);

         if (simplex.isInteriorPoint(origin, epsilon))
            return true;
         else
            simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);

         if (previousSupportVectorDirection.epsilonEquals(supportVectorDirection, epsilon))
            return false;
         else
            previousSupportVectorDirection.set(supportVectorDirection);
      }
      return false;
   }

   private void updateListeners()
   {
      if (listener != null)
         listener.updateAll();
   }

   public void runEPAExpansion()
   {
      simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);
      previousSupportVectorDirection.set(supportVectorDirection);

      while (true)
      {
         Vertex3DReadOnly supportingPolytopeVertexA = polytopeA.getSupportingPolytopeVertex(supportVectorDirection);
         Vertex3DReadOnly supportingPolytopeVertexB = polytopeB.getSupportingPolytopeVertex(supportVectorDirectionNegative);
         simplex.addVertex(supportingPolytopeVertexA, supportingPolytopeVertexB);

         simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);

         if (supportVectorDirection.epsilonEquals(previousSupportVectorDirection, epsilon))
            break;
         else
            previousSupportVectorDirection.set(supportVectorDirection);
      }
   }

   public void getCollisionVector(Vector3D collisionVectorToPack)
   {
      collisionVectorToPack.set(supportVectorDirection);
      collisionVectorToPack.normalize();
      collisionVectorToPack.scale(simplex.getSmallestSimplexMemberReference(origin).distance(origin));
   }

   public void getCollisionPoints(Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
      simplex.getCollidingPointsOnSimplex(origin, pointOnAToPack, pointOnBToPack);
   }
}
