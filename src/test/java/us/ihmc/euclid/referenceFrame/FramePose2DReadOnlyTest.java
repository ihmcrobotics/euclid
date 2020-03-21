package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;

public abstract class FramePose2DReadOnlyTest<T extends FramePose2DReadOnly>
{
   public abstract T createFramePose(ReferenceFrame referenceFrame, Pose2DReadOnly pose);

   public final T createEmptyFramePose(ReferenceFrame referenceFrame)
   {
      return createFramePose(referenceFrame, new Pose2D());
   }

   public final T createRandomFramePose(Random random)
   {
      return createRandomFramePose(random, ReferenceFrame.getWorldFrame());
   }

   public final T createRandomFramePose(Random random, ReferenceFrame referenceFrame)
   {
      return createFramePose(referenceFrame, EuclidGeometryRandomTools.nextPose2D(random));
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().contains("IncludingFrame") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals")
            && !m.getName().equals("setMatchingFrame");
      EuclidFrameAPITester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFramePose,
                                                                                  methodFilter,
                                                                                  EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FramePose2DReadOnly.class, Pose2DReadOnly.class, true);
   }
}
