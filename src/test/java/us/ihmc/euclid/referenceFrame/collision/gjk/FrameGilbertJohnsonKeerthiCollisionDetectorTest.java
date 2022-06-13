package us.ihmc.euclid.referenceFrame.collision.gjk;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public class FrameGilbertJohnsonKeerthiCollisionDetectorTest
{
   private static final boolean VERBOSE = false;
   private static final int ITERATIONS = 10000;

   private static final double DISTANCE_EPSILON = 0.0;
   private static final double POINT_TANGENTIAL_EPSILON = 0.0;

   private static final double LARGE_DISTANCE_EPSILON = 1.0e-7;
   private static final double LARGE_POINT_TANGENTIAL_EPSILON = 1.0e-4;

   private static final double DISTANCE_AVERAGE_EPSILON = 1.0e-10;
   private static final double POINT_NORMAL_ERROR_AVERAGE_EPSILON = 1.0e-10;
   private static final double POINT_TANGENTIAL_ERROR_AVERAGE_EPSILON = 1.0e-7;

   /**
    * Tests that saving the detector's last support direction for the next evaluation, leads to
    * decreasing the number of iterations when the shapes haven't moved, while the result remains the
    * same.
    */
   @Test
   void testReusingSupportDirection()
   {
      Random random = new Random(789034504);
      ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
      FrameGilbertJohnsonKeerthiCollisionDetector detector = new FrameGilbertJohnsonKeerthiCollisionDetector();
      double distanceEpsilon = 1.0e-4;
      double pointTangentialEpsilon = 1.0e-2;

      {
         int totalIterationsWithoutHint = 0;
         int totalIterationsWithHint = 0;

         for (int i = 0; i < ITERATIONS; i++)
         {
            FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random,
                                                                                           EuclidCoreRandomTools.nextElementIn(random, referenceFrames));
            FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random,
                                                                                           EuclidCoreRandomTools.nextElementIn(random, referenceFrames));

            EuclidFrameShape3DCollisionResult expectedResult = detector.evaluateCollision(shapeA, shapeB);

            if (expectedResult.areShapesColliding())
            { // Force non-colliding cases
               i--;
               continue;
            }

            int originalNumberOfIterations = detector.getNumberOfIterations();
            FrameVector3D initialSupportDirection = new FrameVector3D(detector.getSupportDirection());

            detector.setInitialSupportDirection(initialSupportDirection);
            EuclidFrameShape3DCollisionResult actualResult = detector.evaluateCollision(shapeA, shapeB);
            int newNumberOfIterations = detector.getNumberOfIterations();

            EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration "
                  + i, expectedResult, actualResult, distanceEpsilon, pointTangentialEpsilon, 0.0);

            totalIterationsWithoutHint += originalNumberOfIterations;
            totalIterationsWithHint += newNumberOfIterations;
         }

         if (VERBOSE)
            System.out.println("Non-colliding total iterations w/o hint: " + totalIterationsWithoutHint + ", with hint: " + totalIterationsWithHint);
         assertTrue(totalIterationsWithHint < totalIterationsWithoutHint / 3);
      }

      {
         int totalIterationsWithoutHint = 0;
         int totalIterationsWithHint = 0;

         for (int i = 0; i < ITERATIONS; i++)
         {
            FrameShape3DBasics shapeA = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random,
                                                                                           EuclidCoreRandomTools.nextElementIn(random, referenceFrames));
            FrameShape3DBasics shapeB = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random,
                                                                                           EuclidCoreRandomTools.nextElementIn(random, referenceFrames));

            if (shapeA instanceof FramePointShape3DReadOnly && shapeB instanceof FramePointShape3DReadOnly)
            { // Points cannot collide, skipping
               i--;
               continue;
            }

            EuclidFrameShape3DCollisionResult expectedResult = detector.evaluateCollision(shapeA, shapeB);

            while (!expectedResult.areShapesColliding())
            { // Force colliding cases
               FramePoint3D centroidSeparation = new FramePoint3D(shapeB.getCentroid());
               centroidSeparation.changeFrame(shapeA.getReferenceFrame());
               centroidSeparation.sub(shapeA.getCentroid());
               centroidSeparation.scale(0.5);
               shapeA.applyTransform(new RigidBodyTransform(new Quaternion(), centroidSeparation));
               expectedResult = detector.evaluateCollision(shapeA, shapeB);
            }

            int originalNumberOfIterations = detector.getNumberOfIterations();
            FrameVector3D initialSupportDirection = new FrameVector3D(detector.getSupportDirection());

            detector.setInitialSupportDirection(initialSupportDirection);
            EuclidFrameShape3DCollisionResult actualResult = detector.evaluateCollision(shapeA, shapeB);
            int newNumberOfIterations = detector.getNumberOfIterations();

            EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration "
                  + i, expectedResult, actualResult, distanceEpsilon, pointTangentialEpsilon, 0.0);

            totalIterationsWithoutHint += originalNumberOfIterations;
            totalIterationsWithHint += newNumberOfIterations;
         }

         if (VERBOSE)
            System.out.println("Colliding total iterations w/o hint: " + totalIterationsWithoutHint + ", with hint: " + totalIterationsWithHint);
         /*
          * Providing the initial support direction doesn't seem to be as beneficial compared to
          * non-colliding case. The average number of iterations to converge is also by default about 4 times
          * less than the non-colliding scenario.
          */
         assertTrue(totalIterationsWithHint < totalIterationsWithoutHint);
      }
   }

   @Test
   public void testCompareAgainstFramelessGJK()
   {
      Random random = new Random(34307);
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

         if (VERBOSE)
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

         if (VERBOSE)
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

         if (VERBOSE)
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

         if (VERBOSE)
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

         if (VERBOSE)
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

      FrameGilbertJohnsonKeerthiCollisionDetector frameDetector = new FrameGilbertJohnsonKeerthiCollisionDetector();

      frameDetector.evaluateCollision(shapeA, shapeB, frameResultToPack);
      assertTrue(frameDetector.getDetectorFrame() == frameA || frameDetector.getDetectorFrame() == frameB);
      frameResultToPack.getPointOnA().checkReferenceFrameMatch(frameDetector.getDetectorFrame());
      frameResultToPack.getPointOnB().checkReferenceFrameMatch(frameDetector.getDetectorFrame());

      GilbertJohnsonKeerthiCollisionDetector framelessDetector = new GilbertJohnsonKeerthiCollisionDetector();

      FrameShape3DBasics shapeBInDetectorFrame = shapeB.copy();
      shapeBInDetectorFrame.changeFrame(frameDetector.getDetectorFrame());
      FrameShape3DBasics shapeAInDetectorFrame = shapeA.copy();
      shapeAInDetectorFrame.changeFrame(frameDetector.getDetectorFrame());

      framelessDetector.evaluateCollision(shapeAInDetectorFrame, shapeBInDetectorFrame, framelessResultToPack);
      framelessResultToPack.setShapeA(shapeA);
      framelessResultToPack.setShapeB(shapeB);
   }

   public static class ComparisonError
   {
      public double distanceError = 0.0;
      public double pointNormalError = 0.0;
      public double pointTangentialError = 0.0;

      public ComparisonError()
      {
      }

      public ComparisonError(EuclidShape3DCollisionResultReadOnly resultA, EuclidShape3DCollisionResultReadOnly resultB)
      {
         distanceError = Math.abs(resultA.getDistance() - resultB.getDistance());
         Vector3D collisionVector = new Vector3D();
         collisionVector.sub(resultA.getPointOnA(), resultA.getPointOnB());
         collisionVector.normalize();
         updatePointError(resultA.getPointOnA(), resultB.getPointOnA(), collisionVector);
         updatePointError(resultA.getPointOnB(), resultB.getPointOnB(), collisionVector);
      }

      private void updatePointError(Point3DReadOnly point1, Point3DReadOnly point2, Vector3DReadOnly normal)
      {

         Vector3D errorPointOnA = new Vector3D();
         errorPointOnA.sub(point1, point2);
         double normalError = errorPointOnA.dot(normal);

         Vector3D errorNormalVector = new Vector3D();
         errorNormalVector.setAndScale(normalError, normal);

         Vector3D errorTangentialVector = new Vector3D();
         errorTangentialVector.sub(errorPointOnA, errorNormalVector);
         pointNormalError = Math.max(pointNormalError, normalError);
         pointTangentialError = Math.max(pointTangentialError, errorTangentialVector.length());
      }

      private void add(ComparisonError other)
      {
         distanceError += other.distanceError;
         pointNormalError += other.pointNormalError;
         pointTangentialError += other.pointTangentialError;
      }

      private void scale(double scalar)
      {
         distanceError *= scalar;
         pointNormalError *= scalar;
         pointTangentialError *= scalar;
      }

      private static ComparisonError max(ComparisonError a, ComparisonError b)
      {
         ComparisonError max = new ComparisonError();
         max.distanceError = Math.max(a.distanceError, b.distanceError);
         max.pointNormalError = Math.max(a.pointNormalError, b.pointNormalError);
         max.pointTangentialError = Math.max(a.pointTangentialError, b.pointTangentialError);
         return max;
      }

      public static ComparisonError average(List<ComparisonError> errors)
      {
         ComparisonError average = new ComparisonError();
         List<ComparisonError> nonNull = errors.stream().filter(o -> o != null).collect(Collectors.toList());
         nonNull.forEach(average::add);
         average.scale(1.0 / nonNull.size());
         return average;
      }

      public static ComparisonError max(List<ComparisonError> errors)
      {
         return errors.stream().filter(o -> o != null).reduce(new ComparisonError(), ComparisonError::max);
      }

      @Override
      public String toString()
      {
         return "distance: " + distanceError + ", point (normal): " + pointNormalError + ", point (tangential): " + pointTangentialError;
      }

      public static ComparisonError toComparisonError(EuclidShape3DCollisionResult expectedResult, EuclidFrameShape3DCollisionResult actualResult)
      {
         return expectedResult.areShapesColliding() ? null : new ComparisonError(expectedResult, actualResult);
      }
   }
}
