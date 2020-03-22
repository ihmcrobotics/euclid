package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;

public abstract class FrameLineSegment3DReadOnlyTest<T extends FrameLineSegment3DReadOnly>
{
   public abstract T createFrameLineSegment(ReferenceFrame referenceFrame, LineSegment3DReadOnly segment);

   public final T createRandomLineSegment(Random random)
   {
      return createRandomFrameLineSegment(random, ReferenceFrame.getWorldFrame());
   }

   public final T createRandomFrameLineSegment(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameLineSegment(referenceFrame, EuclidGeometryRandomTools.nextLineSegment3D(random));
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameLineSegment3DReadOnly.class, LineSegment3DReadOnly.class, true);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFrameLineSegment,
                                                                                  methodFilter,
                                                                                  EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }
}
