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
import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

public class FrameOrientation2DTest extends FrameOrientation2DReadOnlyTest<FrameOrientation2D>
{
   public static final double EPSILON = 1.0e-15;

   @BeforeEach
   public void disableNameRestriction()
   {
      ReferenceFrame.getWorldFrame().setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);
   }

   @Override
   public FrameOrientation2D createFrameOrientation(ReferenceFrame referenceFrame, Orientation2DReadOnly orientation)
   {
      return new FrameOrientation2D(referenceFrame, orientation);
   }

   @Test
   public void testConsistencyWithOrientation2D()
   {
      FrameTypeCopier frameTypeBuilder = (frame, pose) -> createFrameOrientation(frame, (Orientation2DReadOnly) pose);
      RandomFramelessTypeBuilder framelessTypeBuilder = EuclidCoreRandomTools::nextOrientation2D;
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
      signaturesToIgnore.add(new MethodSignature("set", Orientation2D.class));
      signaturesToIgnore.add(new MethodSignature("equals", Orientation2D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Orientation2D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Orientation2D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameOrientation2D.class, Orientation2D.class, true, 1, methodFilter);
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameOrientation2D, EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameOrientation2DReadOnly source = EuclidFrameRandomTools.nextFrameOrientation2D(random, sourceFrame);
         FixedFrameOrientation2DBasics actual = EuclidFrameRandomTools.nextFrameOrientation2D(random, destinationFrame);

         actual.setMatchingFrame(source);

         FrameOrientation2D expected = new FrameOrientation2D(source);
         expected.changeFrame(destinationFrame);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSetIncludingFrame()
   {
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("setIncludingFrame", ReferenceFrame.class, double.class));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetIncludingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameOrientation2D,
                                                          methodFilter,
                                                          EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
