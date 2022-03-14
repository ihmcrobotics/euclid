package us.ihmc.euclid.referenceFrame.collision.epa;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.collision.gjk.FrameGilbertJohnsonKeerthiCollisionDetectorTest.ComparisonError;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;

public class FrameExpandingPolytopeAlgorithmTest
{
   private static final int ITERATIONS = 10000;

   private static final double DISTANCE_EPSILON = 0.0;
   private static final double POINT_TANGENTIAL_EPSILON = 0.0;

   private static final double LARGE_DISTANCE_EPSILON = 5.0e-4;
   private static final double LARGE_POINT_TANGENTIAL_EPSILON = 1.0e-2;

   private static final double DISTANCE_AVERAGE_EPSILON = 1.0e-8;
   private static final double POINT_NORMAL_ERROR_AVERAGE_EPSILON = 1.0e-8;
   private static final double POINT_TANGENTIAL_ERROR_AVERAGE_EPSILON = 1.0e-5;

   @Test
   public void testCompareAgainstFramelessEPA()
   {
      Random random = new Random(3407);
      EuclidShape3DCollisionResult expectedResult = new EuclidShape3DCollisionResult();
      EuclidFrameShape3DCollisionResult actualResult = new EuclidFrameShape3DCollisionResult();

      {
         List<ComparisonError> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // The 2 shapes in worldFrame
            ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

            FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, worldFrame);
            FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, worldFrame);
            computeResults(shapeA, shapeB, expectedResult, actualResult);

            EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration : "
                  + i, expectedResult, actualResult, DISTANCE_EPSILON, POINT_TANGENTIAL_EPSILON, 0.0);
            errors.add(ComparisonError.toComparisonError(expectedResult, actualResult));
         }

         ComparisonError average = ComparisonError.average(errors);
         assertEquals(0.0, average.distanceError, DISTANCE_AVERAGE_EPSILON, "average distance error too large: ");
         assertEquals(0.0, average.pointNormalError, POINT_NORMAL_ERROR_AVERAGE_EPSILON, "average point normal error too large: ");
         assertEquals(0.0, average.pointTangentialError, POINT_TANGENTIAL_ERROR_AVERAGE_EPSILON, "average point tangential error too large: ");

         System.out.println(getClass().getSimpleName() + ": Two shapes in worldFrame:\n\tmax error: " + ComparisonError.max(errors) + "\n\tavg error: "
               + average);
      }

      {
         List<ComparisonError> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // The 2 shapes in same frame but not worldFrame
            ReferenceFrame shapeFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

            FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, shapeFrame);
            FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, shapeFrame);
            computeResults(shapeA, shapeB, expectedResult, actualResult);

            EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration : "
                  + i, expectedResult, actualResult, DISTANCE_EPSILON, POINT_TANGENTIAL_EPSILON, 0.0);
            errors.add(ComparisonError.toComparisonError(expectedResult, actualResult));
         }

         ComparisonError average = ComparisonError.average(errors);

         assertEquals(0.0, average.distanceError, DISTANCE_AVERAGE_EPSILON, "average distance error too large: ");
         assertEquals(0.0, average.pointNormalError, POINT_NORMAL_ERROR_AVERAGE_EPSILON, "average point normal error too large: ");
         assertEquals(0.0, average.pointTangentialError, POINT_TANGENTIAL_ERROR_AVERAGE_EPSILON, "average point tangential error too large: ");

         System.out.println(getClass().getSimpleName() + ": Two shapes in same frame (not worldFrame):\n\tmax error: " + ComparisonError.max(errors)
               + "\n\tavg error: " + average);
      }

      { // Frames differ, results may occasionally differ due to numerical errors triggering different edge-cases.
         List<ComparisonError> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // shapeA is in worldFrame and shapeB is in a random frame
            ReferenceFrame frameA = ReferenceFrame.getWorldFrame();
            ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);

            FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, frameA);
            FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, frameB);
            computeResults(shapeA, shapeB, expectedResult, actualResult);

            EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration : "
                  + i, expectedResult, actualResult, LARGE_DISTANCE_EPSILON, LARGE_POINT_TANGENTIAL_EPSILON, 0.0);
            errors.add(ComparisonError.toComparisonError(expectedResult, actualResult));
         }

         ComparisonError average = ComparisonError.average(errors);

         assertEquals(0.0, average.distanceError, DISTANCE_AVERAGE_EPSILON, "average distance error too large: ");
         assertEquals(0.0, average.pointNormalError, POINT_NORMAL_ERROR_AVERAGE_EPSILON, "average point normal error too large: ");
         assertEquals(0.0, average.pointTangentialError, POINT_TANGENTIAL_ERROR_AVERAGE_EPSILON, "average point tangential error too large: ");

         System.out.println(getClass().getSimpleName() + ": shapeA in worldFrame and shape B in a random frame:\n\tmax error: " + ComparisonError.max(errors)
               + "\n\tavg error: " + average);
      }

      { // Frames differ, results may occasionally differ due to numerical errors triggering different edge-cases.
         List<ComparisonError> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // shapeA is in a random frame and shapeB is in worldFrame
            ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame frameB = ReferenceFrame.getWorldFrame();

            FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, frameA);
            FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, frameB);
            computeResults(shapeA, shapeB, expectedResult, actualResult);

            EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration : "
                  + i, expectedResult, actualResult, LARGE_DISTANCE_EPSILON, LARGE_POINT_TANGENTIAL_EPSILON, 0.0);
            errors.add(ComparisonError.toComparisonError(expectedResult, actualResult));
         }

         ComparisonError average = ComparisonError.average(errors);

         assertEquals(0.0, average.distanceError, DISTANCE_AVERAGE_EPSILON, "average distance error too large: ");
         assertEquals(0.0, average.pointNormalError, POINT_NORMAL_ERROR_AVERAGE_EPSILON, "average point normal error too large: ");
         assertEquals(0.0, average.pointTangentialError, POINT_TANGENTIAL_ERROR_AVERAGE_EPSILON, "average point tangential error too large: ");

         System.out.println(getClass().getSimpleName() + ": shapeA in random frame and shape B in worldFrame:\n\tmax error: " + ComparisonError.max(errors)
               + "\n\tavg error: " + average);
      }

      { // Frames differ, results may occasionally differ due to numerical errors triggering different edge-cases.
         List<ComparisonError> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // The shapes are in different frames
            ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);

            FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, frameA);
            FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random, frameB);
            computeResults(shapeA, shapeB, expectedResult, actualResult);

            EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration : "
                  + i, expectedResult, actualResult, LARGE_DISTANCE_EPSILON, LARGE_POINT_TANGENTIAL_EPSILON, 0.0);
            errors.add(ComparisonError.toComparisonError(expectedResult, actualResult));
         }

         ComparisonError average = ComparisonError.average(errors);

         assertEquals(0.0, average.distanceError, DISTANCE_AVERAGE_EPSILON, "average distance error too large: ");
         assertEquals(0.0, average.pointNormalError, POINT_NORMAL_ERROR_AVERAGE_EPSILON, "average point normal error too large: ");
         assertEquals(0.0, average.pointTangentialError, POINT_TANGENTIAL_ERROR_AVERAGE_EPSILON, "average point tangential error too large: ");

         System.out.println(getClass().getSimpleName() + ": Both shape in distinct random frame:\n\tmax error: " + ComparisonError.max(errors)
               + "\n\tavg error: " + average);
      }
   }

   private static void computeResults(FrameShape3DBasics shapeA,
                                      FrameShape3DBasics shapeB,
                                      EuclidShape3DCollisionResult framelessResultToPack,
                                      EuclidFrameShape3DCollisionResult frameResultToPack)
   {
      ReferenceFrame frameA = shapeA.getReferenceFrame();
      ReferenceFrame frameB = shapeB.getReferenceFrame();

      FrameExpandingPolytopeAlgorithm frameDetector = new FrameExpandingPolytopeAlgorithm();

      frameDetector.evaluateCollision(shapeA, shapeB, frameResultToPack);
      assertTrue(frameDetector.getDetectorFrame() == frameA || frameDetector.getDetectorFrame() == frameB);
      frameResultToPack.getPointOnA().checkReferenceFrameMatch(frameDetector.getDetectorFrame());
      frameResultToPack.getPointOnB().checkReferenceFrameMatch(frameDetector.getDetectorFrame());

      ExpandingPolytopeAlgorithm framelessDetector = new ExpandingPolytopeAlgorithm();

      FrameShape3DBasics shapeBInDetectorFrame = shapeB.copy();
      shapeBInDetectorFrame.changeFrame(frameDetector.getDetectorFrame());
      FrameShape3DBasics shapeAInDetectorFrame = shapeA.copy();
      shapeAInDetectorFrame.changeFrame(frameDetector.getDetectorFrame());

      framelessDetector.evaluateCollision(shapeAInDetectorFrame, shapeBInDetectorFrame, framelessResultToPack);
      framelessResultToPack.setShapeA(shapeA);
      framelessResultToPack.setShapeB(shapeB);
   }
}
