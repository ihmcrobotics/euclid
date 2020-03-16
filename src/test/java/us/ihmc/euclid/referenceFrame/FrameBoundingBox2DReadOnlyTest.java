package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;

public abstract class FrameBoundingBox2DReadOnlyTest<T extends FrameBoundingBox2DReadOnly>
{
   public abstract T createFrameBoundingBox(ReferenceFrame referenceFrame, BoundingBox2DReadOnly boundingBox);

   public final T createRandomBoundingBox(Random random)
   {
      return createRandomFrameBoundingBox(random, ReferenceFrame.getWorldFrame());
   }

   public final T createRandomFrameBoundingBox(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameBoundingBox(referenceFrame, EuclidGeometryRandomTools.nextBoundingBox2D(random));
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameBoundingBox2DReadOnly.class, BoundingBox2DReadOnly.class, false);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("setIncludingFrame") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals")
            && !m.getName().equals("setMatchingFrame");
      EuclidFrameAPITestTools.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFrameBoundingBox, methodFilter);
   }
}
