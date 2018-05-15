package us.ihmc.euclid.referenceFrame.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrameUtils;
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
   private static final Class<?> d = Double.TYPE;
   private static final int ITERATIONS = 500;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testAPIIsComplete()
   {
      Map<String, Class<?>[]> methodsToIgnore = new HashMap<>();
      methodsToIgnore.put("orthogonalProjectionOnLine3D", new Class<?>[] {Point3DReadOnly.class, d, d, d, d, d, d, Point3DBasics.class});
      methodsToIgnore.put("orthogonalProjectionOnLine2D", new Class<?>[] {Point2DReadOnly.class, d, d, d, d, Point2DBasics.class});
      methodsToIgnore.put("orthogonalProjectionOnLineSegment2D", new Class<?>[] {Point2DReadOnly.class, d, d, d, d, Point2DBasics.class});
      methodsToIgnore.put("orthogonalProjectionOnLineSegment3D", new Class<?>[] {Point3DReadOnly.class, d, d, d, d, d, d, Point3DBasics.class});
      methodsToIgnore.put("intersectionBetweenLine3DAndBoundingBox3D",
                          new Class<?>[] {d, d, d, d, d, d, d, d, d, d, d, d, Point3DBasics.class, Point3DBasics.class});
      methodsToIgnore.put("intersectionBetweenLine3DAndCylinder3D", new Class<?>[] {d, d, d, d, d, d, d, d, d, Point3DBasics.class, Point3DBasics.class});
      methodsToIgnore.put("intersectionBetweenLine3DAndEllipsoid3D", new Class<?>[] {d, d, d, d, d, d, d, d, d, Point3DBasics.class, Point3DBasics.class});

      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(EuclidFrameTools.class, EuclidGeometryTools.class, false, 2, methodsToIgnore);
   }

   @Test
   public void testReferenceFrameChecked() throws Throwable
   {
      EuclidFrameAPITestTools.assertStaticMethodsCheckReferenceFrame(EuclidFrameTools.class);
   }

   @Test
   public void testConservedFunctionality() throws Exception
   {
      EuclidFrameAPITestTools.assertStaticMethodsPreserveFunctionality(EuclidFrameTools.class, EuclidGeometryTools.class);
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
            List<FramePoint2D> points = new ArrayList<>();

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
         List<FramePoint2D> points = new ArrayList<>();
         int size = random.nextInt(50);

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

            ReferenceFrameUtils.clearWorldFrameTree();
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
