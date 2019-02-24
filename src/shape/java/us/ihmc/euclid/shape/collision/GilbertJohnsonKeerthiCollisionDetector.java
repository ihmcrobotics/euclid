package us.ihmc.euclid.shape.collision;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
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
   private double terminalConditionEpsilon;
   private double simplexConstructionEpsilon = EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON;
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

   public GilbertJohnsonKeerthiCollisionDetector(double terminalConditionEpsilon)
   {
      setTerminalConditionEpsilon(terminalConditionEpsilon);
   }

   public SimplexPolytope3D getSimplex()
   {
      return simplex;
   }

   public void setTerminalConditionEpsilon(double epsilon)
   {
      this.terminalConditionEpsilon = epsilon;
   }

   public void setSimplexConstructionEpsilon(double simplexConstructionEpsilon)
   {
      this.simplexConstructionEpsilon = simplexConstructionEpsilon;
   }

   public double getTerminalConditionEpsilon()
   {
      return terminalConditionEpsilon;
   }

   public double getSimplexConstructionEpsilon()
   {
      return simplexConstructionEpsilon;
   }

   public boolean doCollisionTest(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB)
   {
      latestCollisionTestResult = false;
      isSeparationVectorUpToDate = false;
      areClosestPointsUpToDate = false;

      supportDirection.set(Axis.Y);
      Point3DReadOnly supportingVertexA = shapeA.getSupportingVertex(supportDirection);
      Point3DReadOnly supportingVertexB = shapeB.getSupportingVertex(supportDirectionNegative);

      if (supportingVertexA == null || supportingVertexB == null)
      {
         simplex = null;
         return false;
      }

      simplex = new SimplexPolytope3D(simplexConstructionEpsilon);

      for (iterations = 0; iterations < maxIterations; iterations++)
      {
         // FIXME cleanup the following once testing is done.
         ConvexPolytope3D backup = new ConvexPolytope3D(simplex.getPolytope());
         simplex.addVertex(supportingVertexA, supportingVertexB);
         try
         {
            EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity(simplex.getPolytope());
         }
         catch (Error e)
         {
            System.out.println(EuclidShapeIOTools.getConvexPolytope3DStringForUnitTesting(backup, new SimplexVertex3D(supportingVertexA, supportingVertexB)));
            throw e;
         }

         // TODO Inefficient approach here, the simplex is growing with the number of iterations whereas the most complex shape should remain a tetrahedron.
         if (simplex.isPointInside(origin, terminalConditionEpsilon))
         {
            latestCollisionTestResult = true;
            return true;
         }

         simplex.getSupportVectorDirectionTo(origin, supportDirection);

         if (previousSupportDirection.epsilonEquals(supportDirection, terminalConditionEpsilon))
            return false;

         previousSupportDirection.set(supportDirection);

         supportingVertexA = shapeA.getSupportingVertex(supportDirection);
         supportingVertexB = shapeB.getSupportingVertex(supportDirectionNegative);
      }
      return false;
   }

   public Shape3DCollisionTestResult doShapeCollisionTest(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      Shape3DCollisionTestResult result = new Shape3DCollisionTestResult();
      doShapeCollisionTest(shapeA, shapeB, result);
      return result;
   }

   public void doShapeCollisionTest(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, Shape3DCollisionTestResult result)
   {
      boolean areShapesColliding = doCollisionTest((SupportingVertexHolder) shapeA, (SupportingVertexHolder) shapeB);
      result.setToNaN();
      result.setShapesAreColliding(areShapesColliding);
      result.setShapeA(shapeA);
      result.setShapeB(shapeB);

      if (areShapesColliding)
         return;

      Vector3DReadOnly separationVector = getSeparationVector();
      result.setDistance(separationVector.length());

      updateClosestPoints();
      result.getPointOnA().set(closestPointOnA);
      result.getPointOnB().set(closestPointOnB);
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
         separationVector.scale(getDistance());
         isSeparationVectorUpToDate = true;
      }

      return separationVector;
   }

   public double getDistance()
   {
      return simplex.getSmallestSimplexMemberReference(origin).distance(origin);
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
