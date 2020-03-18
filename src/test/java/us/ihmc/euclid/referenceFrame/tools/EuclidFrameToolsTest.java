package us.ihmc.euclid.referenceFrame.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.*;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class EuclidFrameToolsTest
{
   private static final Class<Point3DReadOnly> P3_RO = Point3DReadOnly.class;
   private static final Class<Point3DBasics> P3_BA = Point3DBasics.class;
   private static final Class<Point2DReadOnly> P2_RO = Point2DReadOnly.class;
   private static final Class<Point2DBasics> P2_BA = Point2DBasics.class;
   private static final Class<?> D = double.class;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testAPIIsComplete()
   {
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("orthogonalProjectionOnLine3D", P3_RO, D, D, D, D, D, D, P3_BA));
      signaturesToIgnore.add(new MethodSignature("orthogonalProjectionOnLine2D", P2_RO, D, D, D, D, P2_BA));
      signaturesToIgnore.add(new MethodSignature("orthogonalProjectionOnLineSegment2D", P2_RO, D, D, D, D, P2_BA));
      signaturesToIgnore.add(new MethodSignature("orthogonalProjectionOnLineSegment3D", P3_RO, D, D, D, D, D, D, P3_BA));
      signaturesToIgnore.add(new MethodSignature("intersectionBetweenLine3DAndBoundingBox3D", D, D, D, D, D, D, D, D, D, D, D, D, P3_BA, P3_BA));
      signaturesToIgnore.add(new MethodSignature("intersectionBetweenLine3DAndCylinder3D", D, D, D, D, D, D, D, D, D, D, D, D, D, D, P3_BA, P3_BA));
      signaturesToIgnore.add(new MethodSignature("intersectionBetweenLine3DAndEllipsoid3D", D, D, D, D, D, D, D, D, D, P3_BA, P3_BA));
      signaturesToIgnore.add(new MethodSignature("closestPoint3DsBetweenTwoLineSegment3Ds", D, D, D, D, D, D, D, D, D, D, D, D, P3_BA, P3_BA));

      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(EuclidFrameTools.class, EuclidGeometryTools.class, false, 2, methodFilter);
   }

   @Test
   public void testReferenceFrameChecked() throws Throwable
   {
      EuclidFrameAPITester.assertStaticMethodsCheckReferenceFrame(EuclidFrameTools.class);
   }

   @Test
   public void testConservedFunctionality() throws Exception
   {
      EuclidFrameAPITester.assertStaticMethodsPreserveFunctionality(EuclidFrameTools.class, EuclidGeometryTools.class);
   }

   @Test
   public void testAveragePoint2Ds() throws Exception
   {
      Random random = new Random(3245436);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test frame check
         int size = random.nextInt(50) + 2; // Making sure there are at least two elements in the list.

         for (int index = 0; index < size; index++)
         {
            List<FramePoint2D> points = new ArrayList<>(size);

            ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);

            while (points.size() < size)
            {
               ReferenceFrame referenceFrame;
               if (points.size() == index)
                  referenceFrame = frameA;
               else
                  referenceFrame = frameB;

               points.add(EuclidFrameRandomTools.nextFramePoint2D(random, referenceFrame));
            }

            try
            {
               EuclidFrameTools.averagePoint2Ds(points);
               fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
            }
            catch (ReferenceFrameMismatchException e)
            {
               // Good
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test functionality
         int size = random.nextInt(50);
         List<FramePoint2D> points = new ArrayList<>(size);

         ReferenceFrame referenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         while (points.size() < size)
         {
            points.add(EuclidFrameRandomTools.nextFramePoint2D(random, referenceFrame));
         }

         FramePoint2D actual = EuclidFrameTools.averagePoint2Ds(points);
         Point2D expected = EuclidGeometryTools.averagePoint2Ds(points);

         if (size != 0)
            assertEquals(referenceFrame, actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testAveragePoint3Ds() throws Exception
   {
      Random random = new Random(3245436);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test frame check
         int size = random.nextInt(50) + 2; // Making sure there are at least two elements in the list.

         for (int index = 0; index < size; index++)
         {
            List<FramePoint3D> points = new ArrayList<>();

            ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);

            while (points.size() < size)
            {
               ReferenceFrame referenceFrame;
               if (points.size() == index)
                  referenceFrame = frameA;
               else
                  referenceFrame = frameB;

               points.add(EuclidFrameRandomTools.nextFramePoint3D(random, referenceFrame));
            }

            try
            {
               EuclidFrameTools.averagePoint3Ds(points);
               fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
            }
            catch (ReferenceFrameMismatchException e)
            {
               // Good
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test functionality
         List<FramePoint3D> points = new ArrayList<>();
         int size = random.nextInt(50);

         ReferenceFrame referenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         while (points.size() < size)
         {
            points.add(EuclidFrameRandomTools.nextFramePoint3D(random, referenceFrame));
         }

         FramePoint3D actual = EuclidFrameTools.averagePoint3Ds(points);
         Point3D expected = EuclidGeometryTools.averagePoint3Ds(points);

         if (size != 0)
            assertEquals(referenceFrame, actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }
   }
}
