package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

public class FrameLine2DTest extends FrameLine2DReadOnlyTest<FrameLine2D>
{
   public static final double EPSILON = 1.0e-15;

   @Override
   public FrameLine2D createFrameLine(ReferenceFrame referenceFrame, Line2DReadOnly line)
   {
      return new FrameLine2D(referenceFrame, line);
   }

   @Test
   public void testConsistencyWithLine2D()
   {
      FrameTypeCopier frameTypeBuilder = (frame, line) -> createFrameLine(frame, (Line2DReadOnly) line);
      RandomFramelessTypeBuilder framelessTypeBuilder = EuclidGeometryRandomTools::nextLine2D;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                                framelessTypeBuilder,
                                                                                methodFilter,
                                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Predicate<Method> framelessMethodsToIgnore = m -> !m.getName().equals("set") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals")
            && !m.getName().equals("geometricallyEquals");
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameLine2D.class, Line2D.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      EuclidFrameAPITester.assertSetMatchingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameLine2D,
                                                                       EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      Random random = new Random(544354);

      for (int i = 0; i < 1000; i++)
      {
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameLine2DReadOnly source = EuclidFrameRandomTools.nextFrameLine2D(random, sourceFrame);
         FixedFrameLine2DBasics actual = EuclidFrameRandomTools.nextFrameLine2D(random, destinationFrame);

         actual.setMatchingFrame(source);

         FrameLine2D expected = new FrameLine2D(source);
         expected.changeFrame(destinationFrame);

         EuclidGeometryTestTools.assertLine2DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSetIncludingFrame()
   {
      EuclidFrameAPITester.assertSetIncludingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameLine2D,
                                                                        EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
