package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;

public abstract class FrameOrientation2DReadOnlyTest<T extends FrameOrientation2DReadOnly>
{
   public abstract T createFrameOrientation(ReferenceFrame referenceFrame, Orientation2DReadOnly orientation);

   public final T createEmptyFramePose(ReferenceFrame referenceFrame)
   {
      return createFrameOrientation(referenceFrame, new Orientation2D());
   }

   public final T createRandomOrientation(Random random)
   {
      return createRandomFrameOrientation(random, ReferenceFrame.getWorldFrame());
   }

   public final T createRandomFrameOrientation(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameOrientation(referenceFrame, EuclidGeometryRandomTools.nextOrientation2D(random));
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFrameOrientation,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameOrientation2DReadOnly.class, Orientation2DReadOnly.class, true);
   }
}
