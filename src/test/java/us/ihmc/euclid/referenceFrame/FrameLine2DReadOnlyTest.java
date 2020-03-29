package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;

public abstract class FrameLine2DReadOnlyTest<T extends FrameLine2DReadOnly>
{
   public abstract T createFrameLine(ReferenceFrame referenceFrame, Line2DReadOnly line);

   public final T createRandomLine(Random random)
   {
      return createRandomFrameLine(random, ReferenceFrame.getWorldFrame());
   }

   public final T createRandomFrameLine(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameLine(referenceFrame, EuclidGeometryRandomTools.nextLine2D(random));
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameLine2DReadOnly.class, Line2DReadOnly.class, true);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFrameLine, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }
}
