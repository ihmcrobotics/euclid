package us.ihmc.euclid.referenceFrame.collision.gjk;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;

public class FrameGilbertJohnsonKeerthiCollisionDetectorTest
{
   private static final int ITERATIONS = 5000;
   private static final double DISTANCE_EPSILON = 1.0e-5;
   private static final double POINT_EPSILON = 2.0e-3;

   @Test
   public void testCompareAgainstFramelessGJK()
   {
      Random random = new Random(34306);
      GilbertJohnsonKeerthiCollisionDetector framelessDetector = new GilbertJohnsonKeerthiCollisionDetector();
      FrameGilbertJohnsonKeerthiCollisionDetector frameDetector = new FrameGilbertJohnsonKeerthiCollisionDetector();

      for (int i = 0; i < ITERATIONS; i++)
      { // The 2 shapes in worldFrame
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

         FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, worldFrame);
         FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, worldFrame);

         EuclidShape3DCollisionResult expectedResult = new EuclidShape3DCollisionResult();
         EuclidFrameShape3DCollisionResult actualResult = new EuclidFrameShape3DCollisionResult();

         framelessDetector.evaluateCollision(shapeA, shapeB, expectedResult);
         frameDetector.evaluateCollision(shapeA, shapeB, actualResult);
         assertEquals(worldFrame, actualResult.getPointOnA().getReferenceFrame());
         assertEquals(worldFrame, actualResult.getPointOnB().getReferenceFrame());

         EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration : "
               + i, expectedResult, actualResult, DISTANCE_EPSILON, POINT_EPSILON, 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // The 2 shapes in same frame but not worldFrame
         ReferenceFrame shapeFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, shapeFrame);
         FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, shapeFrame);

         EuclidShape3DCollisionResult expectedResult = new EuclidShape3DCollisionResult();
         EuclidFrameShape3DCollisionResult actualResult = new EuclidFrameShape3DCollisionResult();

         framelessDetector.evaluateCollision(shapeA, shapeB, expectedResult);
         frameDetector.evaluateCollision(shapeA, shapeB, actualResult);
         assertEquals(shapeFrame, actualResult.getPointOnA().getReferenceFrame());
         assertEquals(shapeFrame, actualResult.getPointOnB().getReferenceFrame());

         EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration : "
               + i, expectedResult, actualResult, DISTANCE_EPSILON, POINT_EPSILON, 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // shapeA is in worldFrame and shapeB is in a random frame
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);

         FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, worldFrame);
         FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, frameB);

         FrameShape3DBasics shapeBInWorld = shapeB.copy();
         shapeBInWorld.changeFrame(worldFrame);

         EuclidShape3DCollisionResult expectedResult = new EuclidShape3DCollisionResult();
         EuclidFrameShape3DCollisionResult actualResult = new EuclidFrameShape3DCollisionResult();

         framelessDetector.evaluateCollision(shapeA, shapeBInWorld, expectedResult);
         expectedResult.setShapeB(shapeB);
         frameDetector.evaluateCollision(shapeA, shapeB, actualResult);

         assertTrue(frameB == actualResult.getPointOnA().getReferenceFrame() || worldFrame == actualResult.getPointOnA().getReferenceFrame());
         assertTrue(frameB == actualResult.getPointOnB().getReferenceFrame() || worldFrame == actualResult.getPointOnB().getReferenceFrame());

         actualResult.getPointOnA().changeFrame(worldFrame);
         actualResult.getPointOnB().changeFrame(worldFrame);

         EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration : "
               + i, expectedResult, actualResult, DISTANCE_EPSILON, POINT_EPSILON, 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // shapeA is in a random frame and shapeB is in worldFrame
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

         FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, frameA);
         FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, worldFrame);

         FrameShape3DBasics shapeAInWorld = shapeA.copy();
         shapeAInWorld.changeFrame(worldFrame);

         EuclidShape3DCollisionResult expectedResult = new EuclidShape3DCollisionResult();
         EuclidFrameShape3DCollisionResult actualResult = new EuclidFrameShape3DCollisionResult();

         framelessDetector.evaluateCollision(shapeAInWorld, shapeB, expectedResult);
         expectedResult.setShapeA(shapeA);
         frameDetector.evaluateCollision(shapeA, shapeB, actualResult);

         assertTrue(frameA == actualResult.getPointOnA().getReferenceFrame() || worldFrame == actualResult.getPointOnA().getReferenceFrame());
         assertTrue(frameA == actualResult.getPointOnB().getReferenceFrame() || worldFrame == actualResult.getPointOnB().getReferenceFrame());

         actualResult.getPointOnA().changeFrame(worldFrame);
         actualResult.getPointOnB().changeFrame(worldFrame);

         EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration : "
               + i, expectedResult, actualResult, DISTANCE_EPSILON, POINT_EPSILON, 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // The shapes are in different frames
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

         FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, frameA);
         FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, frameB);

         FrameShape3DBasics shapeBInWorld = shapeB.copy();
         shapeBInWorld.changeFrame(worldFrame);
         FrameShape3DBasics shapeAInWorld = shapeA.copy();
         shapeAInWorld.changeFrame(worldFrame);

         EuclidShape3DCollisionResult expectedResult = new EuclidShape3DCollisionResult();
         EuclidFrameShape3DCollisionResult actualResult = new EuclidFrameShape3DCollisionResult();

         framelessDetector.evaluateCollision(shapeAInWorld, shapeBInWorld, expectedResult);
         expectedResult.setShapeA(shapeA);
         expectedResult.setShapeB(shapeB);
         frameDetector.evaluateCollision(shapeA, shapeB, actualResult);

         assertTrue(frameA == actualResult.getPointOnA().getReferenceFrame() || frameB == actualResult.getPointOnA().getReferenceFrame());
         assertTrue(frameA == actualResult.getPointOnB().getReferenceFrame() || frameB == actualResult.getPointOnB().getReferenceFrame());

         actualResult.getPointOnA().changeFrame(worldFrame);
         actualResult.getPointOnB().changeFrame(worldFrame);

         EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration : "
               + i, expectedResult, actualResult, DISTANCE_EPSILON, POINT_EPSILON, 0.0);
      }
   }
}
