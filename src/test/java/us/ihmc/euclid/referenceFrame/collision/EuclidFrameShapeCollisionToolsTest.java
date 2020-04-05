package us.ihmc.euclid.referenceFrame.collision;

import java.util.Random;
import java.util.function.BiFunction;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameCylinder3D;
import us.ihmc.euclid.referenceFrame.FrameEllipsoid3D;
import us.ihmc.euclid.referenceFrame.FramePointShape3D;
import us.ihmc.euclid.referenceFrame.FrameRamp3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.interfaces.EuclidFrameShape3DCollisionResultBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.EuclidShapeCollisionTools;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;

public class EuclidFrameShapeCollisionToolsTest
{
   private static final int ITERATIONS = 10000;
   private static final double EPSILON = 1.0e-12;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   static interface Shape3DCollisionEvaluator<A extends Shape3DReadOnly, B extends Shape3DReadOnly>
   {
      void evaluateCollision(A shapeA, B shapeB, EuclidShape3DCollisionResultBasics resultToPack);
   }

   static interface FrameShape3DCollisionEvaluator<A extends FrameShape3DReadOnly, B extends FrameShape3DReadOnly>
   {
      void evaluateCollision(A shapeA, B shapeB, EuclidFrameShape3DCollisionResultBasics resultToPack);
   }

   private <A extends FrameShape3DReadOnly, B extends FrameShape3DReadOnly> void performAssertionsForCollisionEvaluator(Random random,
                                                                                                                        FrameShape3DCollisionEvaluator<A, B> evaluatorToTest,
                                                                                                                        Shape3DCollisionEvaluator<A, B> referenceForTesting,
                                                                                                                        BiFunction<Random, ReferenceFrame, A> shapeARandomGenerator,
                                                                                                                        BiFunction<Random, ReferenceFrame, B> shapeBRandomGenerator,
                                                                                                                        boolean testNormal, double epsilon)
   {
      for (int i = 0; i < ITERATIONS; i++)
      { // Test in world frame
         EuclidFrameShape3DCollisionResult actual = new EuclidFrameShape3DCollisionResult();
         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         A shapeA = shapeARandomGenerator.apply(random, worldFrame);
         B shapeB = shapeBRandomGenerator.apply(random, worldFrame);
         evaluatorToTest.evaluateCollision(shapeA, shapeB, actual);
         referenceForTesting.evaluateCollision(shapeA, shapeB, expected);
         assertCollisionResultsEqual(expected, actual, testNormal, epsilon);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test in a random frame (frameA == frameB)
         EuclidFrameShape3DCollisionResult actual = new EuclidFrameShape3DCollisionResult();
         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("aFrame", random, worldFrame);
         ReferenceFrame frameB = frameA;
         A shapeA = shapeARandomGenerator.apply(random, frameA);
         B shapeB = shapeBRandomGenerator.apply(random, frameB);
         evaluatorToTest.evaluateCollision(shapeA, shapeB, actual);
         referenceForTesting.evaluateCollision(changeFrame(shapeA, worldFrame), changeFrame(shapeB, worldFrame), expected);
         expected.setShapeA(shapeA);
         expected.setShapeB(shapeB);
         actual.getPointOnA().changeFrame(worldFrame);
         actual.getPointOnB().changeFrame(worldFrame);
         actual.getNormalOnA().changeFrame(worldFrame);
         actual.getNormalOnB().changeFrame(worldFrame);
         assertCollisionResultsEqual(expected, actual, testNormal, epsilon);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test in random frames (frameA != frameB)
         EuclidFrameShape3DCollisionResult actual = new EuclidFrameShape3DCollisionResult();
         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);
         A shapeA = shapeARandomGenerator.apply(random, frameA);
         B shapeB = shapeBRandomGenerator.apply(random, frameB);
         evaluatorToTest.evaluateCollision(shapeA, shapeB, actual);
         referenceForTesting.evaluateCollision(changeFrame(shapeA, worldFrame), changeFrame(shapeB, worldFrame), expected);
         expected.setShapeA(shapeA);
         expected.setShapeB(shapeB);
         actual.getPointOnA().changeFrame(worldFrame);
         actual.getPointOnB().changeFrame(worldFrame);
         actual.getNormalOnA().changeFrame(worldFrame);
         actual.getNormalOnB().changeFrame(worldFrame);
         assertCollisionResultsEqual(expected, actual, testNormal, epsilon);
      }
   }

   private static void assertCollisionResultsEqual(EuclidShape3DCollisionResult expected, EuclidFrameShape3DCollisionResult actual, boolean testNormal,
                                                   double epsilon)
   {
      if (!testNormal)
      {
         expected.getNormalOnA().setToNaN();
         expected.getNormalOnB().setToNaN();
      }
      else
      {
         expected.getNormalOnA().normalize();
         expected.getNormalOnB().normalize();
         actual.getNormalOnA().normalize();
         actual.getNormalOnB().normalize();
      }

      EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals(expected, actual, epsilon);
   }

   @Test
   public void testEvaluatePointShape3DBox3DCollision()
   {
      Random random = new Random(5165);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DBox3DCollision,
                                             EuclidShapeCollisionTools::evaluatePointShape3DBox3DCollision,
                                             EuclidFrameShapeRandomTools::nextFramePointShape3D,
                                             EuclidFrameShapeRandomTools::nextFrameBox3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateSphere3DBox3DCollision()
   {
      Random random = new Random(5165);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DBox3DCollision,
                                             EuclidShapeCollisionTools::evaluateSphere3DBox3DCollision,
                                             EuclidFrameShapeRandomTools::nextFrameSphere3D,
                                             EuclidFrameShapeRandomTools::nextFrameBox3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DCapsule3DCollision()
   {
      Random random = new Random(54687);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DCapsule3DCollision,
                                             EuclidShapeCollisionTools::evaluatePointShape3DCapsule3DCollision,
                                             EuclidFrameShapeRandomTools::nextFramePointShape3D,
                                             EuclidFrameShapeRandomTools::nextFrameCapsule3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateCapsule3DCapsule3DCollision()
   {
      Random random = new Random(345);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateCapsule3DCapsule3DCollision,
                                             EuclidShapeCollisionTools::evaluateCapsule3DCapsule3DCollision,
                                             EuclidFrameShapeRandomTools::nextFrameCapsule3D,
                                             EuclidFrameShapeRandomTools::nextFrameCapsule3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateSphere3DCapsule3DCollision()
   {
      Random random = new Random(54687);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DCapsule3DCollision,
                                             EuclidShapeCollisionTools::evaluateSphere3DCapsule3DCollision,
                                             EuclidFrameShapeRandomTools::nextFrameSphere3D,
                                             EuclidFrameShapeRandomTools::nextFrameCapsule3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DCylinder3DCollision()
   {
      Random random = new Random(86);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DCylinder3DCollision,
                                             EuclidShapeCollisionTools::evaluatePointShape3DCylinder3DCollision,
                                             EuclidFrameShapeRandomTools::nextFramePointShape3D,
                                             EuclidFrameShapeRandomTools::nextFrameCylinder3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateSphere3DCylinder3DCollision()
   {
      Random random = new Random(86);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DCylinder3DCollision,
                                             EuclidShapeCollisionTools::evaluateSphere3DCylinder3DCollision,
                                             EuclidFrameShapeRandomTools::nextFrameSphere3D,
                                             EuclidFrameShapeRandomTools::nextFrameCylinder3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DEllipsoid3DCollision()
   {
      Random random = new Random(285);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DEllipsoid3DCollision,
                                             EuclidShapeCollisionTools::evaluatePointShape3DEllipsoid3DCollision,
                                             EuclidFrameShapeRandomTools::nextFramePointShape3D,
                                             EuclidFrameShapeRandomTools::nextFrameEllipsoid3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateSphere3DEllipsoid3DCollision()
   {
      Random random = new Random(285);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DEllipsoid3DCollision,
                                             EuclidShapeCollisionTools::evaluateSphere3DEllipsoid3DCollision,
                                             EuclidFrameShapeRandomTools::nextFrameSphere3D,
                                             EuclidFrameShapeRandomTools::nextFrameEllipsoid3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DPointShape3DCollision()
   {
      Random random = new Random(6);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DPointShape3DCollision,
                                             EuclidShapeCollisionTools::evaluatePointShape3DPointShape3DCollision,
                                             EuclidFrameShapeRandomTools::nextFramePointShape3D,
                                             EuclidFrameShapeRandomTools::nextFramePointShape3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DRamp3DCollision()
   {
      Random random = new Random(98587621);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DRamp3DCollision,
                                             EuclidShapeCollisionTools::evaluatePointShape3DRamp3DCollision,
                                             EuclidFrameShapeRandomTools::nextFramePointShape3D,
                                             EuclidFrameShapeRandomTools::nextFrameRamp3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateSphere3DRamp3DCollision()
   {
      Random random = new Random(98587621);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DRamp3DCollision,
                                             EuclidShapeCollisionTools::evaluateSphere3DRamp3DCollision,
                                             EuclidFrameShapeRandomTools::nextFrameSphere3D,
                                             EuclidFrameShapeRandomTools::nextFrameRamp3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluatePointShape3DSphere3DCollision()
   {
      Random random = new Random(74275);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluatePointShape3DSphere3DCollision,
                                             EuclidShapeCollisionTools::evaluatePointShape3DSphere3DCollision,
                                             EuclidFrameShapeRandomTools::nextFramePointShape3D,
                                             EuclidFrameShapeRandomTools::nextFrameSphere3D,
                                             true,
                                             EPSILON);
   }

   @Test
   public void testEvaluateSphere3DSphere3DCollision()
   {
      Random random = new Random(74275);
      performAssertionsForCollisionEvaluator(random,
                                             EuclidFrameShapeCollisionTools::evaluateSphere3DSphere3DCollision,
                                             EuclidShapeCollisionTools::evaluateSphere3DSphere3DCollision,
                                             EuclidFrameShapeRandomTools::nextFrameSphere3D,
                                             EuclidFrameShapeRandomTools::nextFrameSphere3D,
                                             true,
                                             EPSILON);
   }

   private static <S extends FrameShape3DReadOnly> S changeFrame(S shape3D, ReferenceFrame desiredFrame)
   {
      S clone = clone(shape3D);
      ((FrameChangeable) clone).changeFrame(desiredFrame);
      return clone;
   }

   @SuppressWarnings("unchecked")
   private static <S extends FrameShape3DReadOnly> S clone(S shape3D)
   {
      if (shape3D instanceof FrameBox3DReadOnly)
         return (S) new FrameBox3D((FrameBox3DReadOnly) shape3D);
      else if (shape3D instanceof FrameCapsule3DReadOnly)
         return (S) new FrameCapsule3D((FrameCapsule3DReadOnly) shape3D);
      else if (shape3D instanceof FrameCylinder3DReadOnly)
         return (S) new FrameCylinder3D((FrameCylinder3DReadOnly) shape3D);
      else if (shape3D instanceof FrameEllipsoid3DReadOnly)
         return (S) new FrameEllipsoid3D((FrameEllipsoid3DReadOnly) shape3D);
      else if (shape3D instanceof FramePointShape3DReadOnly)
         return (S) new FramePointShape3D((FramePointShape3DReadOnly) shape3D);
      else if (shape3D instanceof FrameRamp3DReadOnly)
         return (S) new FrameRamp3D((FrameRamp3DReadOnly) shape3D);
      else if (shape3D instanceof FrameSphere3DReadOnly)
         return (S) new FrameSphere3D((FrameSphere3DReadOnly) shape3D);
      else if (shape3D instanceof FrameConvexPolytope3DReadOnly)
         return (S) new FrameConvexPolytope3D((FrameConvexPolytope3DReadOnly) shape3D);
      else
         throw new IllegalStateException("Unhandled type of frame shape: " + shape3D);
   }
}
