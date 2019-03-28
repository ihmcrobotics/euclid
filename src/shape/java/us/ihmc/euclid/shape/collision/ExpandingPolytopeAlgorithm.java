package us.ihmc.euclid.shape.collision;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytopeFeature3D;
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
   public static boolean PERFORM_EXTRA_ASSERTIONS = false;
   private static final double SMALLEST_DISTANCE_FOR_SUPPORT_DIRECTION = 1.0e-12;

   private static final Point3DReadOnly origin = new Point3D();

   private int iterations;
   private final int maxIterations = 1000;
   private boolean latestCollisionTestResult;

   private MinkowskiDifferencePolytope3D minkowskiDifference;
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
      this(GilbertJohnsonKeerthiCollisionDetector.DEFAULT_COLLISION_EPSILON);
   }

   public ExpandingPolytopeAlgorithm(double terminalConditionEpsilon)
   {
      gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector(terminalConditionEpsilon);
   }

   public MinkowskiDifferencePolytope3D getMinkowskiDifference()
   {
      return minkowskiDifference;
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

      if (!latestCollisionTestResult || gjkCollisionDetector.getSimplex() == null)
      {
         minkowskiDifference = null;
         return;
      }

      minkowskiDifference = new MinkowskiDifferencePolytope3D(gjkCollisionDetector.getSimplex().getDifferenceVertices(),
                                                              gjkCollisionDetector.getSimplexConstructionEpsilon());
      supportDirection.set(gjkCollisionDetector.getSupportDirection());

      if (PERFORM_EXTRA_ASSERTIONS)
         EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity(minkowskiDifference.getPolytope());

      previousSupportDirection.setToNaN();

      for (iterations = 0; iterations < maxIterations; iterations++)
      {
         ConvexPolytopeFeature3D smallestSimplex = minkowskiDifference.getSmallestFeature(origin);

         if (smallestSimplex.distance(origin) > SMALLEST_DISTANCE_FOR_SUPPORT_DIRECTION)
            smallestSimplex.getSupportVectorDirectionTo(origin, supportDirection);
         else
            smallestSimplex.getSupportVectorDirectionTo(minkowskiDifference.getPolytope().getCentroid(), supportDirection);

         // We need to negate the support direction to point toward the outside of the simplex and thus force the expansion.
         supportDirection.negate();

         if (supportDirection.epsilonEquals(previousSupportDirection, gjkCollisionDetector.getTerminalConditionEpsilon()))
            break;

         Point3DReadOnly supportingVertexA = shapeA.getSupportingVertex(supportDirection);
         Point3DReadOnly supportingVertexB = shapeB.getSupportingVertex(supportDirectionNegative);

         if (PERFORM_EXTRA_ASSERTIONS)
         {
            DifferenceVertex3D newVertex = new DifferenceVertex3D(supportingVertexA, supportingVertexB);
            ConvexPolytope3D minkowksiDifferenceCopy = new ConvexPolytope3D(minkowskiDifference.getPolytope());

            try
            {
               minkowskiDifference.addVertex(newVertex);
               EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity(minkowskiDifference.getPolytope());
            }
            catch (Throwable e)
            {
               System.out.println(ConvexPolytope3DTroublesomeDataset.generateDatasetAsString(minkowksiDifferenceCopy, newVertex));
               throw e;
            }
         }
         else
         {
            minkowskiDifference.addVertex(supportingVertexA, supportingVertexB);
         }

         previousSupportDirection.set(supportDirection);
      }
   }

   public Shape3DCollisionTestResult doShapeCollisionTest(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      doCollisionTest((SupportingVertexHolder) shapeA, (SupportingVertexHolder) shapeB);

      if (gjkCollisionDetector.getSimplex() == null)
         return null;

      Shape3DCollisionTestResult result = new Shape3DCollisionTestResult();
      packResult(shapeA, shapeB, result);
      return result;
   }

   public void doShapeCollisionTest(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, Shape3DCollisionTestResult result)
   {
      doCollisionTest((SupportingVertexHolder) shapeA, (SupportingVertexHolder) shapeB);
      result.setToNaN();

      if (gjkCollisionDetector.getSimplex() == null)
         return;

      packResult(shapeA, shapeB, result);
   }

   private void packResult(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, Shape3DCollisionTestResult result)
   {
      if (latestCollisionTestResult)
      {
         result.setToNaN();
         result.setShapesAreColliding(latestCollisionTestResult);
         result.setShapeA(shapeA);
         result.setShapeB(shapeB);
         result.setDistance(getSignedDistance());

         updatePoints();
         result.getPointOnA().set(pointOnA);
         result.getPointOnB().set(pointOnB);
      }
      else
      {
         gjkCollisionDetector.packResult(shapeA, shapeB, result, false);
      }
   }

   public Vector3DReadOnly getCollisionVector()
   {
      if (!latestCollisionTestResult)
         return gjkCollisionDetector.getSeparationVector();

      if (!isCollisionVectorUpToDate)
      {
         collisionVector.set(supportDirection);
         collisionVector.normalize();
         // Instead of Math.abs(double) we already know that getSignedDistance() is negative.
         collisionVector.scale(-getSignedDistance());
         isCollisionVectorUpToDate = true;
      }

      return collisionVector;
   }

   public double getSignedDistance()
   {
      if (!latestCollisionTestResult)
         return gjkCollisionDetector.getDistance();
      else
         return minkowskiDifference.getPolytope().signedDistance(origin);
   }

   public Point3DReadOnly getCollisionPointOnA()
   {
      if (!latestCollisionTestResult)
         return gjkCollisionDetector.getClosestPointOnA();

      updatePoints();
      return pointOnA;
   }

   public Point3DReadOnly getCollisionPointOnB()
   {
      if (!latestCollisionTestResult)
         return gjkCollisionDetector.getClosestPointOnB();

      updatePoints();
      return pointOnB;
   }

   private void updatePoints()
   {
      if (!latestCollisionTestResult || areCollisionPointsUpToDate)
         return;

      minkowskiDifference.getCollidingPointsOnSimplex(origin, pointOnA, pointOnB);
      areCollisionPointsUpToDate = true;
   }

   public int getIterations()
   {
      return iterations;
   }
}
