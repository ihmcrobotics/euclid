package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;

public abstract class FrameBoundingBox3DReadOnlyTest<T extends FrameBoundingBox3DReadOnly>
{
   public abstract T createFrameBoundingBox(ReferenceFrame referenceFrame, BoundingBox3DReadOnly boundingBox);

   public final T createRandomBoundingBox(Random random)
   {
      return createRandomFrameBoundingBox(random, ReferenceFrame.getWorldFrame());
   }

   public final T createRandomFrameBoundingBox(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameBoundingBox(referenceFrame, EuclidGeometryRandomTools.nextBoundingBox3D(random));
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameBoundingBox3DReadOnly.class, BoundingBox3DReadOnly.class, false);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("setIncludingFrame") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals")
            && !m.getName().equals("setMatchingFrame");
      EuclidFrameAPITester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFrameBoundingBox, methodFilter);
   }
}
