package us.ihmc.euclid.referenceFrame;

import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

public class FrameLineSegment2DTest extends FrameLineSegment2DReadOnlyTest<FrameLineSegment2D>
{
   public static final double EPSILON = 1.0e-15;

   @BeforeEach
   public void disableNameRestriction()
   {
      ReferenceFrame.getWorldFrame().setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);
   }

   @Override
   public FrameLineSegment2D createFrameLineSegment(ReferenceFrame referenceFrame, LineSegment2DReadOnly segment)
   {
      return new FrameLineSegment2D(referenceFrame, segment);
   }

   @Test
   public void testConsistencyWithLineSegment2D()
   {
      FrameTypeCopier frameTypeBuilder = (frame, quaternion) -> createFrameLineSegment(frame, (LineSegment2DReadOnly) quaternion);
      RandomFramelessTypeBuilder framelessTypeBuilder = EuclidGeometryRandomTools::nextLineSegment2D;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals") && !m.getName().equals("toString");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                  framelessTypeBuilder,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", LineSegment2D.class));
      signaturesToIgnore.add(new MethodSignature("equals", LineSegment2D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", LineSegment2D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", LineSegment2D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameLineSegment2D.class, LineSegment2D.class, true, 1, methodFilter);
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameLineSegment2D, EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameLineSegment2DReadOnly source = EuclidFrameRandomTools.nextFrameLineSegment2D(random, sourceFrame);
         FixedFrameLineSegment2DBasics actual = EuclidFrameRandomTools.nextFrameLineSegment2D(random, destinationFrame);

         actual.setMatchingFrame(source);

         FrameLineSegment2D expected = new FrameLineSegment2D(source);
         expected.changeFrame(destinationFrame);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSetIncludingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetIncludingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameLineSegment2D,
                                                          EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
