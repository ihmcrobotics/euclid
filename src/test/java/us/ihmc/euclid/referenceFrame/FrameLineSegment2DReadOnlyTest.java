package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;

public abstract class FrameLineSegment2DReadOnlyTest<T extends FrameLineSegment2DReadOnly>
{
   public abstract T createFrameLineSegment(ReferenceFrame referenceFrame, LineSegment2DReadOnly segment);

   public final T createRandomLineSegment(Random random)
   {
      return createRandomFrameLineSegment(random, ReferenceFrame.getWorldFrame());
   }

   public final T createRandomFrameLineSegment(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameLineSegment(referenceFrame, EuclidGeometryRandomTools.nextLineSegment2D(random));
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameLineSegment2DReadOnly.class, LineSegment2DReadOnly.class, true);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Random random = new Random(234);
      Predicate<Method> methodFilter = m -> !m.getName().equals("setIncludingFrame") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals") && !m.getName().equals("setMatchingFrame");
      EuclidFrameAPITestTools.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frame -> createRandomFrameLineSegment(random, frame), methodFilter);
   }
}
