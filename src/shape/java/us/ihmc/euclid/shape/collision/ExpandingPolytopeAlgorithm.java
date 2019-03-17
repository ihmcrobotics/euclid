package us.ihmc.euclid.shape.collision;

import us.ihmc.euclid.shape.convexPolytope.interfaces.Simplex3D;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
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
   private boolean latestCollisionTestResult;

   private MinkowskiDifferencePolytope3D simplex;
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

   public ExpandingPolytopeAlgorithm(double terminalConditionEpsilon)
   {
      gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector(terminalConditionEpsilon);
   }

   public MinkowskiDifferencePolytope3D getSimplex()
   {
      return simplex;
   }

   public void setTerminalConditionEpsilon(double epsilon)
   {
      gjkCollisionDetector.setTerminalConditionEpsilon(epsilon);
   }

   public void setSimplexConstructionEpsilon(double simplexConstructionEpsilon)
   {
      gjkCollisionDetector.setSimplexConstructionEpsilon(simplexConstructionEpsilon);
   }

   public double getTerminalConditionEpsilon()
   {
      return gjkCollisionDetector.getTerminalConditionEpsilon();
   }

   public double getSimplexConstructionEpsilon()
   {
      return gjkCollisionDetector.getSimplexConstructionEpsilon();
   }

   public void doCollisionTest(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB)
   {
      isCollisionVectorUpToDate = false;
      areCollisionPointsUpToDate = false;

      latestCollisionTestResult = gjkCollisionDetector.doCollisionTest(shapeA, shapeB);
      simplex = gjkCollisionDetector.getSimplex();
      supportDirection.set(gjkCollisionDetector.getSupportDirection());
      // FIXME cleanup the following once testing is done.
      EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity(simplex.getPolytope()); 

      if (!latestCollisionTestResult)
         return;

      previousSupportDirection.setToNaN();

      for (iterations = 0; iterations < maxIterations; iterations++)
      {
         Simplex3D smallestSimplex = simplex.getSmallestSimplexMemberReference(origin);

         if (smallestSimplex.distance(origin) > 1.0e-12)
            smallestSimplex.getSupportVectorDirectionTo(origin, supportDirection);
         else
            smallestSimplex.getSupportVectorDirectionTo(simplex.getPolytope().getCentroid(), supportDirection);

         // We need to negate the support direction to point toward the outside of the simplex and thus force the expansion.
         supportDirection.negate();

         if (supportDirection.epsilonEquals(previousSupportDirection, gjkCollisionDetector.getTerminalConditionEpsilon()))
            break;

         Point3DReadOnly supportingVertexA = shapeA.getSupportingVertex(supportDirection);
         Point3DReadOnly supportingVertexB = shapeB.getSupportingVertex(supportDirectionNegative);
         // FIXME cleanup the following once testing is done.
         
         try
         {
            simplex.addVertex(supportingVertexA, supportingVertexB);
            EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity(simplex.getPolytope());
            Point3D difference = new Point3D();
            difference.sub(supportingVertexA, supportingVertexB);
            gjkCollisionDetector.vertices.add(difference);
         }
         catch (Exception | Error e)
         {
            Point3D difference = new Point3D();
            difference.sub(supportingVertexA, supportingVertexB);
            System.out.println(ConvexPolytope3DTroublesomeDataset.generateDatasetAsString(gjkCollisionDetector.vertices, difference, gjkCollisionDetector.getSimplexConstructionEpsilon()));
            throw e;
         }

         previousSupportDirection.set(supportDirection);
      }
   }

   public Shape3DCollisionTestResult doShapeCollisionTest(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      Shape3DCollisionTestResult result = new Shape3DCollisionTestResult();
      doShapeCollisionTest(shapeA, shapeB, result);
      return result;
   }

   public void doShapeCollisionTest(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, Shape3DCollisionTestResult result)
   {
      doCollisionTest((SupportingVertexHolder) shapeA, (SupportingVertexHolder) shapeB);
      result.setToNaN();
      result.setShapesAreColliding(latestCollisionTestResult);
      result.setShapeA(shapeA);
      result.setShapeB(shapeB);
      result.setDistance(simplex.getPolytope().signedDistance(origin));

      updatePoints();
      result.getPointOnA().set(pointOnA);
      result.getPointOnB().set(pointOnB);
   }

   public Vector3DReadOnly getCollisionVector()
   {
      if (simplex == null)
         return null;

      if (!isCollisionVectorUpToDate)
      {
         collisionVector.set(supportDirection);
         collisionVector.normalize();
         collisionVector.scale(getDistance());
         isCollisionVectorUpToDate = true;
      }

      return collisionVector;
   }

   public double getDistance()
   {
      return simplex.getSmallestSimplexMemberReference(origin).distance(origin);
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
